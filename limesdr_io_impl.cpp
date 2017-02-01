//
// Copyright (c) 2016-2017  Jiang Wei <jiangwei@jiangwei.org>
// Copyright (c) 2015-2016 Josh Blum
// Copyright 2013-2015 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//



#include "limesdr_impl.hpp"
#include <vector>
#include <map>
#include <algorithm>
#include <set>

using namespace uhd;
using namespace uhd::usrp;
using namespace lime;

#define MAX_CGEN_RATE 640e6
#define MIN_SAMP_RATE 1e5
#define MAX_SAMP_RATE 60e6

#define SOAPY_SDR_END_BURST (1 << 1)
#define SOAPY_SDR_HAS_TIME (1 << 2)
#define SOAPY_SDR_END_ABRUPT (1 << 3)
#define SOAPY_SDR_ONE_PACKET (1 << 4)
#define SOAPY_SDR_MORE_FRAGMENTS (1 << 5)
#define SOAPY_SDR_WAIT_TRIGGER (1 << 6)

#define SOAPY_SDR_TIMEOUT (-1)
#define SOAPY_SDR_STREAM_ERROR (-2)
#define SOAPY_SDR_CORRUPTION (-3)
#define SOAPY_SDR_OVERFLOW (-4)
#define SOAPY_SDR_NOT_SUPPORTED (-5)
#define SOAPY_SDR_TIME_ERROR (-6)
#define SOAPY_SDR_UNDERFLOW (-7)

long long ticksToTimeNs(const long long ticks, const double rate)
{
	const long long ratell = (long long)(rate);
	const long long full = (long long)(ticks / ratell);
	const long long err = ticks - (full*ratell);
	const double part = full*(rate - ratell);
	const double frac = ((err - part) * 1000000000) / rate;
	return (full * 1000000000) + llround(frac);
}

long long timeNsToTicks(const long long timeNs, const double rate)
{
	const long long ratell = (long long)(rate);
	const long long full = (long long)(timeNs / 1000000000);
	const long long err = timeNs - (full * 1000000000);
	const double part = full*(rate - ratell);
	const double frac = part + ((err*rate) / 1000000000);
	return (full*ratell) + llround(frac);
}

size_t formatToSize(const char *format)
{
	size_t size = 0;
	size_t isComplex = false;
	char ch = 0;
	while ((ch = *format++) != '\0')
	{
		if (ch == 'C') isComplex = true;
		if (std::isdigit(ch)) size = (size * 10) + size_t(ch - '0');
	}
	if (isComplex) size *= 2;
	return size / 8; //bits to bytes
}

static IConnectionStream *make_stream(limesdr_impl *d, const uhd::direction_t direction, const uhd::stream_args_t &args) {

	std::vector<size_t> chans = args.channels;
	if (chans.empty()) chans.push_back(0);

	//the format string
	std::string hostFormat;
	BOOST_FOREACH(const char ch, args.cpu_format)
	{
		if (ch == 'c') hostFormat = "C" + hostFormat;
		else if (ch == 'f') hostFormat += "F";
		else if (ch == 's') hostFormat += "S";
		else if (std::isdigit(ch)) hostFormat += ch;
		else throw uhd::runtime_error("OpenUSRP::setupStream(" + args.cpu_format + ") unknown format");
	}

	return d->setupStream(direction, hostFormat, chans, args);

}

IConnectionStream* limesdr_impl::setupStream(const uhd::direction_t direction, const std::string &format, const std::vector<size_t> &channels, const uhd::stream_args_t &args) {
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto stream = new IConnectionStream;
	stream->direction = direction;
	stream->elemSize = formatToSize(format.c_str());
	stream->hasCmd = false;

	StreamConfig config;
	config.isTx = (direction == TX_DIRECTION);
	config.performanceLatency = 0.5;

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (size_t i = 0; i < channelIDs.size(); ++i)
	{
		config.channelID = channelIDs[i];
		if (format == "CF32") config.format = StreamConfig::STREAM_COMPLEX_FLOAT32;
		else if (format == "CS16") config.format = StreamConfig::STREAM_12_BIT_IN_16;
		else if (format == "CS12") config.format = StreamConfig::STREAM_12_BIT_COMPRESSED;
		else throw uhd::runtime_error("OpenUSRP::setupStream(format=" + format + ") unsupported format");

		//create the stream
		size_t streamID(~0);
		const int status = _conn->SetupStream(streamID, config);
		if (status != 0)
			throw uhd::runtime_error("OpenUSRP::setupStream() failed: ");
		stream->streamID.push_back(streamID);
		stream->elemMTU = _conn->GetStreamSize(streamID);
	}
	return (IConnectionStream *)stream;


}


void limesdr_impl::closeStream(IConnectionStream* stream) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;

	for (auto i : streamID)
		_conn->CloseStream(i);

}

int limesdr_impl::deactivateStream(IConnectionStream* stream, const int flags, const long long timeNs) {
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;
	icstream->hasCmd = false;

	StreamMetadata metadata;
	metadata.timestamp = timeNsToTicks(timeNs, _conn->GetHardwareTimestampRate());
	metadata.hasTimestamp = (flags & SOAPY_SDR_HAS_TIME) != 0;
	metadata.endOfBurst = (flags & SOAPY_SDR_END_BURST) != 0;
	for (auto i : streamID)
	{
		int status = _conn->ControlStream(i, false);
		if (status != 0)
			return SOAPY_SDR_STREAM_ERROR;
	}
	return 0;
}

int limesdr_impl::activateStream(IConnectionStream* stream, const int flags, const long long timeNs, const size_t numElems) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;

	if (_conn->GetHardwareTimestampRate() == 0.0)
		throw uhd::runtime_error("OpenUSRP::activateStream() - the sample rate has not been configured!");

	//stream requests used with rx
	icstream->flags = flags;
	icstream->timeNs = timeNs;
	icstream->numElems = numElems;
	icstream->hasCmd = true;

	for (auto i : streamID)
	{
		int status = _conn->ControlStream(i, true);
		if (status != 0)
			return SOAPY_SDR_STREAM_ERROR;
	}
	return 0;


}

int limesdr_impl::readStream(IConnectionStream* stream, void * const *buffs, size_t numElems, int &flags, long long &timeNs, const long timeoutUs) {

	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;

	const auto exitTime = boost::chrono::high_resolution_clock::now() + boost::chrono::microseconds(timeoutUs);

	//wait for a command from activate stream up to the timeout specified
	if (not icstream->hasCmd)
	{
		while (boost::chrono::high_resolution_clock::now() < exitTime)
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
		}
		return SOAPY_SDR_TIMEOUT;
	}

	//handle the one packet flag by clipping
	if ((flags & SOAPY_SDR_ONE_PACKET) != 0)
	{
		numElems = std::min(numElems, icstream->elemMTU);
	}

ReadStreamAgain:
	StreamMetadata metadata;
	int status = 0;
	int bufIndex = 0;
	for (auto i : streamID)
	{
		status = _conn->ReadStream(i, buffs[bufIndex++], numElems, timeoutUs / 1000, metadata);
		if (status == 0) return SOAPY_SDR_TIMEOUT;
		if (status < 0) return SOAPY_SDR_STREAM_ERROR;
	}

	//the command had a time, so we need to compare it to received time
	if ((icstream->flags & SOAPY_SDR_HAS_TIME) != 0 and metadata.hasTimestamp)
	{
		const uint64_t cmdTicks = timeNsToTicks(icstream->timeNs, _conn->GetHardwareTimestampRate());

		//our request time is now late, clear command and return error code
		if (cmdTicks < metadata.timestamp)
		{
			icstream->hasCmd = false;
			return SOAPY_SDR_TIME_ERROR;
		}

		//our request time is not in this received buffer, try again
		if (cmdTicks >= (metadata.timestamp + status))
		{
			if (boost::chrono::high_resolution_clock::now() > exitTime) return SOAPY_SDR_TIMEOUT;
			goto ReadStreamAgain;
		}

		//otherwise our request is in this buffer, advance memory
		const size_t numOff = (cmdTicks - metadata.timestamp);
		metadata.timestamp += numOff;
		status -= numOff;
		const size_t elemSize = icstream->elemSize;
		for (size_t i = 0; i < streamID.size(); i++)
		{
			const size_t memStart = size_t(buffs[i]) + (numOff*elemSize);
			std::memmove(buffs[i], (const void *)memStart, status*elemSize);
		}
		icstream->flags &= ~SOAPY_SDR_HAS_TIME; //clear for next read
	}

	//handle finite burst request commands
	if (icstream->numElems != 0)
	{
		//Clip to within the request size when over,
		//and reduce the number of elements requested.
		status = std::min<size_t>(status, icstream->numElems);
		icstream->numElems -= status;

		//the burst completed, done with the command
		if (icstream->numElems == 0)
		{
			icstream->hasCmd = false;
			metadata.endOfBurst = true;
		}
	}

	//output metadata
	flags = 0;
	if (metadata.endOfBurst) flags |= SOAPY_SDR_END_BURST;
	if (metadata.hasTimestamp) flags |= SOAPY_SDR_HAS_TIME;
	timeNs = ticksToTimeNs(metadata.timestamp, _conn->GetHardwareTimestampRate());

	//return num read or error code
	return (status >= 0) ? status : SOAPY_SDR_STREAM_ERROR;


}

int limesdr_impl::writeStream(IConnectionStream *stream, const void * const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs) {

	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;

	//input metadata
	StreamMetadata metadata;
	metadata.timestamp = timeNsToTicks(timeNs, _conn->GetHardwareTimestampRate());
	metadata.hasTimestamp = (flags & SOAPY_SDR_HAS_TIME) != 0;
	metadata.endOfBurst = (flags & SOAPY_SDR_END_BURST) != 0;

	int ret = 0;
	int bufIndex = 0;
	for (auto i : streamID)
	{
		ret = _conn->WriteStream(i, buffs[bufIndex++], numElems, timeoutUs / 1000, metadata);
		if (ret == 0) return SOAPY_SDR_TIMEOUT;
		if (ret < 0) return SOAPY_SDR_STREAM_ERROR;
	}

	//return num written or error code
	return (ret > 0) ? ret : SOAPY_SDR_STREAM_ERROR;

}


int limesdr_impl::readStreamStatus(IConnectionStream*stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs) {

	auto icstream = (IConnectionStream *)stream;
	const auto &streamID = icstream->streamID;

	StreamMetadata metadata;
	flags = 0;
	auto start = boost::chrono::high_resolution_clock::now();
	while (1)
	{
		for (auto i : streamID)
		{
			int ret = _conn->ReadStreamStatus(i, timeoutUs / 1000, metadata);

			if (ret != 0)
			{
				//handle the default not implemented case and return not supported
				return SOAPY_SDR_TIMEOUT;
			}
		}
		//stop when event is detected
		if (metadata.endOfBurst || metadata.lateTimestamp || metadata.packetDropped)
			break;
		//check timeout
		boost::chrono::duration<double> seconds = boost::chrono::high_resolution_clock::now() - start;
		if (seconds.count() > (double)timeoutUs / 1e6)
			return SOAPY_SDR_TIMEOUT;
		//sleep to avoid high CPU load
		if (timeoutUs >= 2000)
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		else
			boost::this_thread::sleep_for(boost::chrono::microseconds(1 + timeoutUs / 2));
	}

	timeNs = ticksToTimeNs(metadata.timestamp, _conn->GetHardwareTimestampRate());
	//output metadata
	if (metadata.endOfBurst) flags |= SOAPY_SDR_END_BURST;
	if (metadata.hasTimestamp) flags |= SOAPY_SDR_HAS_TIME;

	if (metadata.lateTimestamp) return SOAPY_SDR_TIME_ERROR;
	if (metadata.packetDropped) return SOAPY_SDR_OVERFLOW;

	return 0;


}




class LimeRxStream : public uhd::rx_streamer {
public:

	LimeRxStream(limesdr_impl * d, const uhd::stream_args_t &args) :
		_device(d),
		_stream(make_stream(d, RX_DIRECTION, args)),
		_nchan(std::max<size_t>(1, args.channels.size())),
		_elemSize(uhd::convert::get_bytes_per_item(args.cpu_format)),
		_activated(false)
	{
		_offsetBuffs.resize(_nchan);

	}

	~LimeRxStream(void) {

		_device->deactivateStream(_stream, 0, 0);
		_device->closeStream(_stream);
	}

	size_t get_num_channels(void) const
	{
		return _nchan;
	}

	size_t get_max_num_samps(void) const
	{
		return _stream->elemMTU;
	}

	size_t recv(
		const buffs_type &buffs,
		const size_t nsamps_per_buff,
		uhd::rx_metadata_t &md,
		const double timeout = 1,
		const bool one_packet = false
	) {
		size_t total = 0;
		md.reset();

		while (total < nsamps_per_buff)
		{
			int flags = 0;
			if (one_packet) flags |= SOAPY_SDR_ONE_PACKET;
			long long timeNs = 0;
			size_t numElems = (nsamps_per_buff - total);
			for (size_t i = 0; i < _nchan; i++) _offsetBuffs[i] = ((char *)buffs[i]) + total*_elemSize;
			int ret = _device->readStream(_stream, &(_offsetBuffs[0]), numElems, flags, timeNs, long(timeout*1e6));

			//deal with return error codes
			switch (ret)
			{
			case SOAPY_SDR_TIMEOUT:
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
				break;

			case SOAPY_SDR_STREAM_ERROR:
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN;
				break;

			case SOAPY_SDR_CORRUPTION:
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET;
				break;

			case SOAPY_SDR_OVERFLOW:
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;
				break;

			case SOAPY_SDR_TIME_ERROR:
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND;
				break;
			}
			if (ret < 0) break;
			total += ret;

			//more fragments always over written by last recv
			md.more_fragments = (flags & SOAPY_SDR_MORE_FRAGMENTS) != 0;

			//apply time if this is the first recv
			if (total == size_t(ret))
			{
				md.has_time_spec = (flags & SOAPY_SDR_HAS_TIME) != 0;
				md.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);
			}

			//mark end of burst and exit call
			if ((flags & SOAPY_SDR_END_BURST) != 0)
			{
				md.end_of_burst = true;
				break;
			}

			//inline overflow indication
			if ((flags & SOAPY_SDR_END_ABRUPT) != 0)
			{
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;
				break;
			}

			//one pkt mode, end loop
			if (one_packet) break;
		}

		return total;

	}

	void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd) {

		int flags = 0;
		if (not stream_cmd.stream_now) flags |= SOAPY_SDR_HAS_TIME;
		long long timeNs = stream_cmd.time_spec.to_ticks(1e9);
		size_t numElems = 0;
		bool activate = true;

		switch (stream_cmd.stream_mode)
		{
		case uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
			break;

		case uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS:
			activate = false;
			break;

		case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
			flags |= SOAPY_SDR_END_BURST;
			numElems = stream_cmd.num_samps;
			break;

		case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE:
			numElems = stream_cmd.num_samps;
			break;
		}

		int ret = 0;
		if (activate) {
			if (!_activated) {
				ret = _device->activateStream(_stream, flags, timeNs, numElems);
				_activated = true;
			}

		}
		else {
			if (_activated) {
				ret = _device->deactivateStream(_stream, flags, timeNs);
				_activated = false;
			}

		}
		if (ret != 0) throw uhd::runtime_error(str(boost::format("LimeRxStream::issue_stream_cmd() = %d") % ret));


	}

private:

	limesdr_impl * _device;
	IConnectionStream * _stream;
	bool _activated;
	const size_t _nchan;
	const size_t _elemSize;
	std::vector<void *> _offsetBuffs;

};


class LimeTxStream : public uhd::tx_streamer
{
public:
	LimeTxStream(limesdr_impl * d, const uhd::stream_args_t &args) :
		_device(d),
		_stream(make_stream(d, TX_DIRECTION, args)),
		_nchan(std::max<size_t>(1, args.channels.size())),
		_elemSize(uhd::convert::get_bytes_per_item(args.cpu_format)), \
		_activated(false)
	{
		_offsetBuffs.resize(_nchan);

	}
	~LimeTxStream(void) {

		if (_activated) {
			_device->deactivateStream(_stream, 0, 0);
			_device->closeStream(_stream);
		}
	}

	size_t get_num_channels(void) const
	{
		return _nchan;
	}

	size_t get_max_num_samps(void) const
	{
		return _stream->elemMTU;
	}

	size_t send(
		const buffs_type &buffs,
		const size_t _nsamps_per_buff,
		const uhd::tx_metadata_t &md,
		const double timeout = 0.1
	) {



		size_t nsamps_per_buff = _nsamps_per_buff;
		size_t total = 0;
		if (nsamps_per_buff == 0)
			return 0;
		const long long timeNs(md.time_spec.to_ticks(1e9));

		if (!_activated) {
			_device->activateStream(_stream, 0, 0, 0);
			_activated = true;
		}

		while (total < nsamps_per_buff)
		{
			int flags = 0;
			size_t numElems = (nsamps_per_buff - total);
			if (md.has_time_spec and total == 0) flags |= SOAPY_SDR_HAS_TIME;
			if (md.end_of_burst) flags |= SOAPY_SDR_END_BURST;
			for (size_t i = 0; i < _nchan; i++) _offsetBuffs[i] = ((char *)buffs[i]) + total*_elemSize;
			int ret = _device->writeStream(_stream, &(_offsetBuffs[0]), numElems, flags, timeNs, long(timeout*1e6));
			if (ret == SOAPY_SDR_TIMEOUT) break;
			if (ret < 0) throw uhd::runtime_error(str(boost::format("LimeTxStream::send() = %d") % ret));
			total += ret;
		}

		return total;
	}

	bool recv_async_msg(uhd::async_metadata_t &md, double timeout = 0.1) {

		size_t chanMask = 0;
		int flags = 0;
		long long timeNs = 0;
		int ret = _device->readStreamStatus(_stream, chanMask, flags, timeNs, long(timeout*1e6));

		//save the first channel found in the mask
		md.channel = 0;
		for (size_t i = 0; i < _nchan; i++)
		{
			if ((chanMask & (1 << i)) == 0) continue;
			md.channel = i;
			break;
		}

		//convert the time
		md.has_time_spec = (flags & SOAPY_SDR_HAS_TIME) != 0;
		md.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);

		//check flags
		if ((flags & SOAPY_SDR_END_BURST) != 0)
		{
			md.event_code = uhd::async_metadata_t::EVENT_CODE_BURST_ACK;
		}

		//set event code based on ret
		switch (ret)
		{
		case SOAPY_SDR_TIMEOUT: return false;

		case SOAPY_SDR_STREAM_ERROR:
			md.event_code = uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR;
			break;

		case SOAPY_SDR_NOT_SUPPORTED:
			md.event_code = uhd::async_metadata_t::EVENT_CODE_USER_PAYLOAD;
			break;

		case SOAPY_SDR_TIME_ERROR:
			md.event_code = uhd::async_metadata_t::EVENT_CODE_TIME_ERROR;
			break;

		case SOAPY_SDR_UNDERFLOW:
			md.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
			break;
		}

		return true;
	}

private:
	limesdr_impl * _device;
	IConnectionStream * _stream;

	bool _activated;
	const size_t _nchan;
	const size_t _elemSize;
	std::vector<const void *> _offsetBuffs;

};

uhd::rx_streamer::sptr limesdr_impl::get_rx_stream(const uhd::stream_args_t &args)
{

	//   this->update_enables();
	uhd::rx_streamer::sptr stream(new LimeRxStream(this, args));
	BOOST_FOREACH(const size_t ch, args.channels) _rx_streamers[ch] = stream;
	if (args.channels.empty()) _rx_streamers[0] = stream;
	return stream;

}

uhd::tx_streamer::sptr limesdr_impl::get_tx_stream(const uhd::stream_args_t &args)
{
	//  this->update_enables();
	uhd::tx_streamer::sptr stream(new LimeTxStream(this, args));
	BOOST_FOREACH(const size_t ch, args.channels) _tx_streamers[ch] = stream;
	if (args.channels.empty()) _tx_streamers[0] = stream;
	return stream;
}

bool limesdr_impl::recv_async_msg(uhd::async_metadata_t &md, double timeout)
{
	uhd::tx_streamer::sptr stream = _tx_streamers[0].lock();
	if (not stream) return false;
	return stream->recv_async_msg(md, timeout);
}




uhd::meta_range_t limesdr_impl::getSampleRange(const uhd::direction_t direction, const size_t channel) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;
	//std::vector<double> rates;

	std::vector<double> rates;

	const double clockRate = this->getMasterClockRate();
	const double dacFactor = clockRate / rfic->GetReferenceClk_TSP(LMS7002M::Tx);
	const double adcFactor = clockRate / rfic->GetReferenceClk_TSP(LMS7002M::Rx);
	const double dspRate = rfic->GetReferenceClk_TSP(lmsDir);
	const bool fixedRx = _fixedRxSampRate.count(channel) != 0 and _fixedRxSampRate.at(channel);
	const bool fixedTx = _fixedTxSampRate.count(channel) != 0 and _fixedTxSampRate.at(channel);

	//clock rate is fixed, only half-band chain is configurable
	if (not _autoTickRate)
	{
		for (int i = 32; i >= 2; i /= 2) rates.push_back(dspRate / i);
	}

	//special rates when looking for rx rates and tx is fixed
	//return all rates where the tx sample rate is achievable
	else if (direction == RX_DIRECTION and fixedTx)
	{
		const double txRate = this->getSampleRate(TX_DIRECTION, channel);
		for (int iTx = 32; iTx >= 2; iTx /= 2)
		{
			const double clockRate = txRate*dacFactor*iTx;
			for (int iRx = 32; iRx >= 2; iRx /= 2)
			{
				const double rxRate = clockRate / (adcFactor*iRx);
				if (rxRate > MAX_SAMP_RATE) continue;
				if (rxRate < MIN_SAMP_RATE) continue;
				rates.push_back(rxRate);
			}
		}
	}

	//special rates when looking for tx rates and rx is fixed
	//return all rates where the rx sample rate is achievable
	else if (direction == TX_DIRECTION and fixedRx)
	{
		const double rxRate = this->getSampleRate(RX_DIRECTION, channel);
		for (int iRx = 32; iRx >= 2; iRx /= 2)
		{
			const double clockRate = rxRate*adcFactor*iRx;
			for (int iTx = 32; iTx >= 2; iTx /= 2)
			{
				const double txRate = clockRate / (dacFactor*iTx);
				if (txRate > MAX_SAMP_RATE) continue;
				if (txRate < MIN_SAMP_RATE) continue;
				rates.push_back(txRate);
			}
		}
	}

	//otherwise, the clock is the only limiting factor
	//just give a reasonable high and low
	else
	{
		rates.push_back(MIN_SAMP_RATE);
		rates.push_back(MAX_SAMP_RATE);
	}

	std::sort(rates.begin(), rates.end());

	uhd::meta_range_t out;
	for (size_t i = 0; i < rates.size(); i++)
	{
		out.push_back(uhd::range_t(rates[i]));
	}
	if (out.empty()) out.push_back(uhd::range_t(0.0));

	return out;
}


double limesdr_impl::getSampleRate(const uhd::direction_t direction, const size_t channel) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;
	return rfic->GetSampleRate(lmsDir, rfic->GetActiveChannel());

}

static double calculateClockRate(
	const int adcFactorRx,
	const int dacFactorTx,
	const double rateRx,
	const double rateTx,
	int &dspFactorRx,
	int &dspFactorTx)
{
	double bestClockRate = 0.0;



	for (int decim = 2; decim <= 32; decim *= 2)
	{
		const double rateClock = rateRx*decim*adcFactorRx;
		if (rateClock > MAX_CGEN_RATE) continue;
		if (rateClock > 450e6 && rateClock < 491.5e6) continue; //avoid range where CGEN does not lock
		if (rateClock < bestClockRate) continue;
		for (int interp = 2; interp <= 32; interp *= 2)
		{
			const double actualRateTx = rateClock / (interp*dacFactorTx);

			//good if we got the same output rate with small margin of error
			if (std::abs(actualRateTx - rateTx) < 10.0)
			{
				bestClockRate = rateClock;
				dspFactorRx = decim;
				dspFactorTx = interp;
			}
		}
	}

	//return the best possible match
	if (bestClockRate != 0.0)
		return bestClockRate;

	UHD_MSG(error) << boost::format(
		"OpenUSRP::setSampleRate(Rx %g MHz, Tx %g MHz) Failed -- no common clock rate.\n"
	) % (rateRx / 1e6) % (rateTx / 1e6) << std::endl;

	throw uhd::runtime_error("OpenUSRP::setSampleRate() -- no common clock rate");
}


void limesdr_impl::setSampleRate(const uhd::direction_t direction, const size_t channel, const double rate) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	LMS7002M_SelfCalState state(rfic);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	double clockRate = this->getMasterClockRate();
	const double dspFactor = clockRate / rfic->GetReferenceClk_TSP(lmsDir);

	//select automatic clock rate
	if (_autoTickRate)
	{
		double rxRate = rate, txRate = rate;
		if (direction != RX_DIRECTION and _fixedRxSampRate[channel]) rxRate = this->getSampleRate(RX_DIRECTION, channel);
		if (direction != RX_DIRECTION and _fixedTxSampRate[channel]) txRate = this->getSampleRate(TX_DIRECTION, channel);
		clockRate = calculateClockRate(
			clockRate / rfic->GetReferenceClk_TSP(LMS7002M::Rx),
			clockRate / rfic->GetReferenceClk_TSP(LMS7002M::Tx),
			rxRate, txRate, _decims[channel], _interps[channel]
		);
	}

	const double dspRate = clockRate / dspFactor;
	const double factor = dspRate / rate;
	int intFactor = 1 << int((std::log(factor) / std::log(2.0)) + 0.5);

	if (intFactor < 2) throw uhd::runtime_error(str(boost::format("OpenUSRP::setSampleRate(%g) -- rate too high") % (rate)));
	if (intFactor > 32) throw uhd::runtime_error(str(boost::format("OpenUSRP::setSampleRate(%g) -- rate too low") % (rate)));



	if (std::abs(factor - intFactor) > 0.01)
		UHD_MSG(warning) << boost::format(
			"OpenUSRP::setSampleRate(): not a power of two factor.\n"
			"TSP Rate = %g MHZ, Requested rate = %g MHz.\n"
		) % (dspRate / 1e6) % (rate / 1e6) << std::endl;

	if (direction == TX_DIRECTION) {
		_fixedTxSampRate[channel] = true;
		_interps[channel] = intFactor;
	}
	else {
		_fixedRxSampRate[channel] = true;
		_decims[channel] = intFactor;

	}

	int status = 0;
	status = rfic->SetInterfaceFrequency(clockRate,
		int(std::log(double(_interps[channel])) / std::log(2.0)) - 1,
		int(std::log(double(_decims[channel])) / std::log(2.0)) - 1);
	if (status != 0)
		UHD_MSG(error) << "SetInterfaceFrequency Failed." << std::endl;

	status = _conn->UpdateExternalDataRate(
		rfic->GetActiveChannelIndex(),
		rfic->GetSampleRate(LMS7002M::Tx, rfic->GetActiveChannel()),
		rfic->GetSampleRate(LMS7002M::Rx, rfic->GetActiveChannel()));
	if (status != 0)
		UHD_MSG(error) << "UpdateExternalDataRate Failed." << std::endl;
}

double limesdr_impl::getFrequency(const uhd::direction_t direction, const size_t channel, const std::string &name) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	if (name == "RF")
	{
		return rfic->GetFrequencySX(lmsDir);
	}

	if (name == "BB")
	{
		int pos = (rfic->Get_SPI_Reg_bits(LMS7_MASK, true) == 0) ? 0 : 1;
		int sign = rfic->Get_SPI_Reg_bits(lmsDir == LMS7002M::Tx ? LMS7param(CMIX_SC_TXTSP) : LMS7param(CMIX_SC_RXTSP)) == pos ? 1 : -1;
		return rfic->GetNCOFrequency(lmsDir, 0) * sign;
	}

	throw uhd::runtime_error("OpenUSRP::getFrequency(" + name + ") unknown name");
}

void limesdr_impl::setFrequency(const uhd::direction_t direction, const size_t channel, const std::string &name, const double frequency) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);

	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	if (name == "RF")
	{
		//clip the frequency into the allowed range
		double targetRfFreq = frequency;
		if (targetRfFreq < 30e6) targetRfFreq = 30e6;
		if (targetRfFreq > 3.8e9) targetRfFreq = 3.8e9;
		rfic->SetFrequencySX(lmsDir, targetRfFreq);

		//apply corrections to channel A
		rfic->SetActiveChannel(LMS7002M::ChA);
		if (rfic->ApplyDigitalCorrections(lmsDir) != 0)
		{

		}
		//success, now channel B (ignore errors)
		else
		{
			rfic->SetActiveChannel(LMS7002M::ChB);
			rfic->ApplyDigitalCorrections(lmsDir);
		}

		return;
	}

	if (name == "BB")
	{
		int pos = (rfic->Get_SPI_Reg_bits(LMS7_MASK, true) == 0) ? 0 : 1;
		int neg = pos ^ 0x1;
		if (direction == TX_DIRECTION) {

			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_BYP_TXTSP), (frequency == 0) ? 1 : 0);
			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_SC_TXTSP), (frequency < 0) ? neg : pos);
		}
		else {
			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_BYP_RXTSP), (frequency == 0) ? 1 : 0);
			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_SC_RXTSP), (frequency < 0) ? neg : pos);

		}

		if (rfic->SetNCOFrequency(lmsDir, 0, std::abs(frequency)) != 0)
		{
			//rate was out of bounds, clip to the maximum frequency
			const double dspRate = rfic->GetReferenceClk_TSP(lmsDir);
			rfic->SetNCOFrequency(lmsDir, 0, dspRate / 2);
		}
		return;
	}

	throw uhd::runtime_error("OpenUSRP::setFrequency(" + name + ") unknown name");
}

uhd::meta_range_t limesdr_impl::getFrequencyRange(const uhd::direction_t direction, const size_t channel, const std::string &name) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	uhd::meta_range_t ranges;
	if (name == "RF")
	{
		ranges.push_back(uhd::range_t(30e6, 3.8e9));
	}
	if (name == "BB")
	{
		const double dspRate = rfic->GetReferenceClk_TSP(lmsDir);
		ranges.push_back(uhd::range_t(-dspRate / 2, dspRate / 2));

	}
	return ranges;

}

void limesdr_impl::old_issue_stream_cmd(const size_t chan, const uhd::stream_cmd_t &cmd) {

	uhd::rx_streamer::sptr stream = _rx_streamers[chan].lock();
	if (stream) stream->issue_stream_cmd(cmd);
}


void limesdr_impl::setAntenna(const uhd::direction_t direction, const size_t channel, const std::string &name) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);

	auto rfic = getRFIC(channel);

	if (direction == RX_DIRECTION)
	{
		LMS7002M::PathRFE path = LMS7002M::PATH_RFE_NONE;

		if (name == "TX/RX")  path = (_rx_chan_map[channel] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
		else if (name == "RX2") path = (_rx_chan_map[channel] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
		else throw uhd::runtime_error("OpenUSRP::setAntenna(RX, " + name + ") - unknown antenna name");

		rfic->SetPathRFE(path);
	}

	if (direction == TX_DIRECTION)
	{
		int band = 0;
		if (name == "TX/RX") band = (_tx_chan_map[channel] == 0) ? LMS7002M::PATH_RFE_LB1 : LMS7002M::PATH_RFE_LB2;
		else throw uhd::runtime_error("OpenUSRP::setAntenna(TX, " + name + ") - unknown antenna name");

		rfic->SetBandTRF(band);
	}

}


double limesdr_impl::getBandwidth(const uhd::direction_t direction, const size_t channel) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	try
	{
		return _actualBw.at(direction).at(channel);
	}
	catch (...)
	{
		return 1.0;
	}
}

void limesdr_impl::setBandwidth(const uhd::direction_t direction, const size_t channel, const double bw) {

	if (bw == 0.0) return; //special ignore value

	if (bw<1000000.0 or bw >60000000.0) return;

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);

	//save dc offset mode
	auto saveDcMode = this->getDCOffsetMode(direction, channel);

	auto rfic = getRFIC(channel);
	LMS7002M_SelfCalState state(rfic);

	_actualBw[direction][channel] = bw;

	if (direction == RX_DIRECTION)
	{
		if (rfic->TuneRxFilterWithCaching(bw) != 0)
		{
			UHD_MSG(warning) << boost::format(
				"OpenUSRP::setBandwidth(Rx, %d, %g MHz) Failed .\n"
			) % (int(channel)) % (bw / 1e6) << std::endl;
		}
	}

	if (direction == TX_DIRECTION)
	{
		if (rfic->TuneTxFilterWithCaching(bw) != 0)
		{
			UHD_MSG(warning) << boost::format(
				"OpenUSRP::setBandwidth(Tx, %d, %g MHz) Failed .\n"
			) % (int(channel)) % (bw / 1e6) << std::endl;
		}
	}

	//restore dc offset mode
	this->setDCOffsetMode(direction, channel, saveDcMode);
}

void limesdr_impl::setDCOffsetMode(const uhd::direction_t direction, const size_t channel, const bool automatic) {
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);

	if (direction == RX_DIRECTION) rfic->SetRxDCRemoval(automatic);

}

bool limesdr_impl::getDCOffsetMode(const uhd::direction_t direction, const size_t channel) {
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);

	if (direction == RX_DIRECTION) return rfic->GetRxDCRemoval();

	return false;
}

void limesdr_impl::setDCOffset(const uhd::direction_t direction, const size_t channel, const std::complex<double> &offset) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);

	if (direction == TX_DIRECTION) rfic->SetTxDCOffset(offset.real(), offset.imag());

}

void limesdr_impl::setIQBalance(const uhd::direction_t direction, const size_t channel, const std::complex<double> &balance) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	double gain = std::abs(balance);
	double gainI = 1.0; if (gain < 1.0) gainI = gain / 1.0;
	double gainQ = 1.0; if (gain > 1.0) gainQ = 1.0 / gain;
	rfic->SetIQBalance(lmsDir, std::arg(balance), gainI, gainQ);

}

LMS7002M * limesdr_impl::getRFIC(const size_t channel) const {

	if (_rfics.size() <= channel / 2)
	{
		throw std::out_of_range("OpenUSRP::getRFIC(" + std::to_string(channel) + ") out of range");
	}
	auto rfic = _rfics.at(channel / 2);
	rfic->SetActiveChannel(((channel % 2) == 0) ? LMS7002M::ChA : LMS7002M::ChB);
	return rfic;

}

uhd::meta_range_t limesdr_impl::getBandwidthRange(const uhd::direction_t direction, const size_t channel) {
	meta_range_t bws;

	if (direction == RX_DIRECTION)
	{
		bws.push_back(uhd::range_t(1e6, 60e6, 1));
	}
	if (direction == TX_DIRECTION)
	{
		bws.push_back(uhd::range_t(0.8e6, 60e6, 1));
	}

	return bws;
}

double limesdr_impl::getGain(const uhd::direction_t direction, const size_t channel, const std::string &name) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);


	if (direction == TX_DIRECTION and name == "Normal") {
		double ret = this->getGain(TX_DIRECTION, channel, "PAD");
		ret /= 63.0;
		return ret*TX_GAIN_MAX;

	}

	else if (direction == RX_DIRECTION and name == "Normal") {

		const int max_gain_lna = 30;
		const int max_gain_tia = 12;
		const int max_gain_pga = 31;

		int pga = this->getGain(RX_DIRECTION, channel, "PGA");
		int tia = this->getGain(RX_DIRECTION, channel, "TIA");
		int lna = this->getGain(RX_DIRECTION, channel, "LNA");

		double ret = pga;

		if (tia == 3)
			ret += max_gain_tia;
		else if (tia == 2)
			ret += max_gain_tia - 3;

		if (lna > 8)
			ret += (max_gain_lna + lna - 15);
		else if (lna > 1)
			ret += (lna - 1) * 3;

		ret /= (max_gain_lna + max_gain_tia + max_gain_pga);

		return ret*RX_GAIN_MAX;
	}

	else if (direction == RX_DIRECTION and name == "LNA")
	{
		return rfic->GetRFELNA_dB();
	}

	else if (direction == RX_DIRECTION and name == "LB_LNA")
	{
		return rfic->GetRFELoopbackLNA_dB();
	}

	else if (direction == RX_DIRECTION and name == "TIA")
	{
		return rfic->GetRFETIA_dB();
	}

	else if (direction == RX_DIRECTION and name == "PGA")
	{
		return rfic->GetRBBPGA_dB();
	}

	else if (direction == TX_DIRECTION and name == "PAD")
	{
		return rfic->GetTRFPAD_dB();
	}

	else if (direction == TX_DIRECTION and name == "LB_PAD")
	{
		return rfic->GetTRFLoopbackPAD_dB();
	}

	else throw uhd::runtime_error("OpenUSRP::getGain(" + name + ") - unknown gain name");

}

void limesdr_impl::setGain(const uhd::direction_t direction, const size_t channel, const std::string &name, const double value) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);

	auto rfic = getRFIC(channel);

	if (direction == RX_DIRECTION and name == "Normal") {


		double gain = value / RX_GAIN_MAX;

		const int max_gain_lna = 30;
		const int max_gain_tia = 12;
		const int max_gain_pga = 31;
		const int gain_total = max_gain_lna + max_gain_tia + max_gain_pga;

		int target = gain*gain_total + 0.49;
		unsigned lna = 0, tia = 0, pga = 0;

		//adjust tia gain
		if (target >= max_gain_tia)
		{
			tia = 2;
			target -= max_gain_tia;
		}

		//adjust lna gain
		if (target >= max_gain_lna)
		{
			lna = 14;
			target -= max_gain_lna;
		}
		else if (target < max_gain_lna - 6)
		{
			lna = target / 3;
			target -= lna * 3;
		}
		else
		{
			lna = (max_gain_lna - 6) / 3;
			target -= lna * 3;
		}

		//adjust pga gain
		assert(target <= max_gain_pga);
		pga = target;

		this->setGain(RX_DIRECTION, channel, "LNA", lna + 1);
		this->setGain(RX_DIRECTION, channel, "TIA", tia + 1);
		this->setGain(RX_DIRECTION, channel, "PGA", pga);

	}

	else if (direction == TX_DIRECTION and name == "Normal") {

		double gain = value / TX_GAIN_MAX;

		int g = (63 * gain + 0.49);

		this->setGain(TX_DIRECTION, channel, "PAD", g);

	}


	else if (direction == RX_DIRECTION and name == "LNA")
	{
		rfic->SetRFELNA_dB(value);
	}

	else if (direction == RX_DIRECTION and name == "LB_LNA")
	{
		rfic->SetRFELoopbackLNA_dB(value);
	}

	else if (direction == RX_DIRECTION and name == "LB_LNA")
	{
		rfic->SetRFELoopbackLNA_dB(value);
	}

	else if (direction == RX_DIRECTION and name == "TIA")
	{
		rfic->SetRFETIA_dB(value);
	}

	else if (direction == RX_DIRECTION and name == "PGA")
	{
		rfic->SetRBBPGA_dB(value);
	}

	else if (direction == TX_DIRECTION and name == "PAD")
	{
		rfic->SetTRFPAD_dB(value);
	}

	else if (direction == TX_DIRECTION and name == "LB_PAD")
	{
		rfic->SetTRFLoopbackPAD_dB(value);
	}

	else if (direction == TX_DIRECTION and name == "LB_PAD")
	{
		rfic->SetTRFLoopbackPAD_dB(value);
	}

}
uhd::meta_range_t limesdr_impl::getGainRange(const uhd::direction_t dir, const size_t chan, const std::string &name) {


	meta_range_t out;
	if (dir == TX_DIRECTION) {
		out.push_back(uhd::range_t(0, TX_GAIN_MAX, 0.2));
	}

	if (dir == RX_DIRECTION) {

		out.push_back(uhd::range_t(0, RX_GAIN_MAX, 1));

	}
	return out;
}

uhd::sensor_value_t limesdr_impl::get_temp() {

	return uhd::sensor_value_t("temp", _rfics.front()->GetTemperature(), "C");

}

uhd::sensor_value_t limesdr_impl::get_ref_locked(void)
{
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	const bool ref_lock = _rfics.front()->GetCGENLocked();
	return sensor_value_t("Ref", ref_lock, "locked", "unlocked");
}

uhd::sensor_value_t limesdr_impl::get_lo_locked(const uhd::direction_t dir, const size_t channel) {

	return sensor_value_t("LO", true, "locked", "unlocked");

}

uhd::usrp::subdev_spec_t  limesdr_impl::get_frontend_mapping(const uhd::direction_t dir) {

	uhd::usrp::subdev_spec_t spec;

	const std::string dirName((dir == RX_DIRECTION) ? "rx" : "tx");
	std::vector<size_t> chan_to_dsp_map = _tree->access<std::vector<size_t> >("/mboards/0" / (dirName + "_chan_dsp_mapping")).get();
	subdev_spec_pair_t chan;
	for (size_t i = 0; i < chan_to_dsp_map.size(); i++) {

		chan.db_name = "A";
		chan.sd_name = (chan_to_dsp_map[i] == 0) ? "A" : "B";
		spec.push_back(chan);
	}

	return spec;
}

void limesdr_impl::set_frontend_mapping(const uhd::direction_t dir, const uhd::usrp::subdev_spec_t &spec) {


	std::vector<size_t> chan_to_dsp_map(spec.size(), 0);
	for (size_t i = 0; i < spec.size(); i++) {
		chan_to_dsp_map[i] = (spec[i].sd_name == "A") ? 0 : 1;
	}
	const std::string dirName((dir == RX_DIRECTION) ? "rx" : "tx");
	_tree->access<std::vector<size_t> >("/mboards/0" / (dirName + "_chan_dsp_mapping")).set(chan_to_dsp_map);

}


void limesdr_impl::setHardwareTime(const std::string &what, const uhd::time_spec_t &time)
{
	if (what == "NOW")
	{
		auto rate = _conn->GetHardwareTimestampRate();
		auto ticks = timeNsToTicks(time.to_ticks(1e6), rate);
		_conn->SetHardwareTimestamp(ticks);
	}
	else
	{

	}

}

uhd::time_spec_t limesdr_impl::getHardwareTime(const std::string &what) {

	uhd::time_spec_t time;
	if (what == "NOW") {
		auto ticks = _conn->GetHardwareTimestamp();
		auto rate = _conn->GetHardwareTimestampRate();
		time = uhd::time_spec_t::from_ticks(ticksToTimeNs(ticks, rate), 1e9);
	}
	else if (what == "PPS")
	{

	}

	return time;

}

double limesdr_impl::getMasterClockRate(void) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	//assume same rate for all RFIC in this wrapper
	return _rfics.front()->GetFrequencyCGEN();
}
void limesdr_impl::setMasterClockRate(const double rate) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);

	if (rate > MAX_CGEN_RATE)
		uhd::runtime_error("Master Clock out of range");

	for (auto rfic : _rfics)
	{
		//make tx rx rates equal
		rfic->Modify_SPI_Reg_bits(LMS7param(EN_ADCCLKH_CLKGN), 0);
		rfic->Modify_SPI_Reg_bits(LMS7param(CLKH_OV_CLKL_CGEN), 2);
		rfic->SetFrequencyCGEN(rate);
	}

}

uhd::filter_info_base::sptr limesdr_impl::getFilter(const uhd::direction_t dir, const size_t channel, const std::string &name) {

	return uhd::filter_info_base::sptr(new uhd::filter_info_base(uhd::filter_info_base::ANALOG_BAND_PASS, false, 0));
}

void limesdr_impl::setFilter(const uhd::direction_t dir, const size_t channel, const std::string &name, const filter_info_base::sptr filter) {

}

void limesdr_impl::setAutoTickRate(const bool enable) {

	_autoTickRate = enable;

}

std::vector<size_t>  limesdr_impl::get_chan_dsp_mapping(const uhd::direction_t dir) {

	if (dir == RX_DIRECTION) {
		return _rx_chan_map;
	}
	return _tx_chan_map;

}

void limesdr_impl::set_chan_dsp_mapping(const uhd::direction_t dir, const std::vector<size_t> & map) {

	if (map.size() > 2)
		UHD_MSG(error) << "LimeSDR only 2 channels";

	if (dir == RX_DIRECTION) {
		for (size_t i = 0; i < map.size(); i++) {
			_rx_chan_map[i] = map[i];
		}

	}

	if (dir == TX_DIRECTION) {
		for (size_t i = 0; i < map.size(); i++) {
			_tx_chan_map[i] = map[i];
		}

	}


}