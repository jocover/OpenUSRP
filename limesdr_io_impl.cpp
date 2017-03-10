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
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace uhd;
using namespace uhd::usrp;
using namespace lime;

#define MAX_CGEN_RATE 640e6
#define MIN_SAMP_RATE 1e5
#define MAX_SAMP_RATE 60e6

class LimeRxStream : public uhd::rx_streamer {
public:

	LimeRxStream(lime::IConnection * c, const uhd::stream_args_t &args) :
		_conn(c),
		_elemSize(uhd::convert::get_bytes_per_item(args.cpu_format)),
		_nchan(std::max<size_t>(1, args.channels.size())),
		_activated(false),
		_fc64(false)
	{
		StreamConfig config;
		config.isTx = false;
		config.performanceLatency = 0.5;

		//default to channel 0, if none were specified
		const std::vector<size_t> &channelIDs = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;

		for (size_t i = 0; i < channelIDs.size(); ++i)
		{
			config.channelID = (uint8_t)channelIDs[i];
			if (args.cpu_format == "fc32") config.format = StreamConfig::STREAM_COMPLEX_FLOAT32;
			else if (args.cpu_format == "sc16") config.format = StreamConfig::STREAM_12_BIT_IN_16;
			else if (args.cpu_format == "fc64") {
				config.format = StreamConfig::STREAM_COMPLEX_FLOAT32;
				_fc64 = true;
			}
			else throw uhd::runtime_error("OpenUSRP::LimeRxStream(format=" + args.cpu_format + ") unsupported format");

			//create the stream
			size_t stream_id(~0);
			const int status = _conn->SetupStream(stream_id, config);
			if (status != 0)
				throw uhd::runtime_error("OpenUSRP::LimeRxStream() failed: ");
			streamID.push_back(stream_id);
		}

	}

	~LimeRxStream(void) {

		if (_activated) {

			for (auto i : streamID)
				_conn->ControlStream(i, false);

			for (auto i : streamID)
				_conn->CloseStream(i);

			_activated = false;
		}
	}

	size_t get_num_channels(void) const
	{
		return _nchan;
	}

	size_t get_max_num_samps(void) const
	{
		return _conn->GetStreamSize(streamID.front());
	}

	size_t recv(
		const buffs_type &buffs,
		const size_t nsamps_per_buff,
		uhd::rx_metadata_t &md,
		const double timeout = 1,
		const bool one_packet = false
	) {
		size_t total = 0;

		const auto exitTime = boost::chrono::high_resolution_clock::now() + boost::chrono::microseconds(long(timeout*1e6));

		if (not _activated) {

			while (boost::chrono::high_resolution_clock::now() < exitTime) {
				boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
			}

			md.error_code = uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
			return 0;
		}

		size_t numElems = nsamps_per_buff;
		if (one_packet)
			numElems = std::min(numElems, _conn->GetStreamSize(streamID.front()));

		StreamMetadata metadata;

		if (md.has_time_spec) {
			metadata.hasTimestamp = true;
			metadata.timestamp = md.time_spec.to_ticks(_conn->GetHardwareTimestampRate());
		}

		md.reset();

		int bufIndex = 0;
		int status = 0;
		for (auto i : streamID)
		{
			if (_fc64) {

				float *buffer = new float[numElems * 2];

				status = _conn->ReadStream(i, buffer, numElems, timeout * 1000, metadata);

				double *ptr = (double*)buffs[bufIndex++];
				for (size_t i = 0; i < numElems * 2; i++)
					ptr[i] = (double)buffer[i];

				delete[] buffer;
			}
			else {
				status = _conn->ReadStream(i, buffs[bufIndex++], numElems, timeout * 1000, metadata);
			}


			if (status == 0) {
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_TIMEOUT;
				return 0;

			}

			if (status < 0) {
				md.error_code = uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN;
				return 0;
			}

		}

		if (metadata.hasTimestamp)
		{
			md.has_time_spec = true;
			md.time_spec = uhd::time_spec_t::from_ticks(metadata.timestamp, _conn->GetHardwareTimestampRate());
		}

		if (metadata.packetDropped)
			md.error_code = uhd::rx_metadata_t::ERROR_CODE_OVERFLOW;

		if (metadata.endOfBurst)
			md.end_of_burst = true;

		return status;

	}

	void issue_stream_cmd(const uhd::stream_cmd_t &stream_cmd) {

		switch (stream_cmd.stream_mode)
		{
		case uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS: {

			if (not _activated) {
				for (auto i : streamID) {
					int status = _conn->ControlStream(i, true);
				}
				_activated = true;
			}
		}
			break;

		case uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS: {

			if (_activated) {
				for (auto i : streamID)
				{
					_conn->ControlStream(i, false);
				}
				_activated = false;
			}
		}

			break;

		case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
			//TODO//
			break;

		case uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE:
			//TODO//
			break;
		}

	}

private:

	lime::IConnection *_conn;
	bool _fc64;
	std::vector<size_t> streamID;
	size_t _elemSize;
	bool _activated;
	const size_t _nchan;
};


class LimeTxStream : public uhd::tx_streamer
{
public:
	LimeTxStream(lime::IConnection * c, const uhd::stream_args_t &args) :
		_conn(c),
		_nchan(std::max<size_t>(1, args.channels.size())),
		_activated(false),
		_fc64(false)
	{

		StreamConfig config;
		config.isTx = true;
		config.performanceLatency = 0.5;

		//default to channel 0, if none were specified
		const std::vector<size_t> &channelIDs = args.channels.empty() ? std::vector<size_t>(1, 0) : args.channels;

		for (size_t i = 0; i < channelIDs.size(); ++i)
		{
			config.channelID = (uint8_t)channelIDs[i];
			if (args.cpu_format == "fc32") config.format = StreamConfig::STREAM_COMPLEX_FLOAT32;
			else if (args.cpu_format == "sc16") config.format = StreamConfig::STREAM_12_BIT_IN_16;
			else if (args.cpu_format == "fc64") {
				config.format = StreamConfig::STREAM_COMPLEX_FLOAT32;
				_fc64 = true;
			}
			else throw uhd::runtime_error("OpenUSRP::LimeTxStream(format=" + args.cpu_format + ") unsupported format");

			//create the stream
			size_t stream_id(~0);
			const int status = _conn->SetupStream(stream_id, config);
			if (status != 0)
				throw uhd::runtime_error("OpenUSRP::LimeTxStream() failed: ");

			streamID.push_back(stream_id);
		}

	}
	~LimeTxStream(void) {

		if (_activated) {
			for (auto i : streamID)
				_conn->ControlStream(i, false);

			for (auto i : streamID)
				_conn->CloseStream(i);

			_activated = false;
		}
	}

	size_t get_num_channels(void) const
	{
		return _nchan;
	}

	size_t get_max_num_samps(void) const
	{
		return _conn->GetStreamSize(streamID.front());
	}

	size_t send(
		const buffs_type &buffs,
		const size_t _nsamps_per_buff,
		const uhd::tx_metadata_t &md,
		const double timeout = 0.1
	) {

		size_t numElems = _nsamps_per_buff;
		size_t total = 0;
		if (numElems == 0)
			return 0;

		if (not _activated) {
			for (auto i : streamID)
			{
				int status = _conn->ControlStream(i, true);
			}
			_activated = true;
		}

		StreamMetadata metadata;

		if (md.has_time_spec) {
			metadata.hasTimestamp = true;
			metadata.timestamp = md.time_spec.to_ticks(_conn->GetHardwareTimestampRate());
		}

		if (md.end_of_burst) {
			metadata.endOfBurst = true;
		}

		int bufIndex = 0;
		int status = 0;
		for (auto i : streamID)
		{
			if (_fc64) {

				float *buffer = new float[numElems * 2];

				double *ptr = (double*)buffs[bufIndex++];

				for (size_t i = 0; i < numElems * 2; i++)
					buffer[i] = (float)ptr[i];

				status = _conn->WriteStream(i, buffer, numElems, timeout * 1000, metadata);

				delete[] buffer;
			}
			else {
				status = _conn->WriteStream(i, buffs[bufIndex++], numElems, timeout * 1000, metadata);
			}


		}

		return status;

	}

	bool recv_async_msg(uhd::async_metadata_t &md, double timeout = 0.1) {

		StreamMetadata metadata;
		int channel = 0;
		for (auto i : streamID) {

			md.channel = channel;
			int ret = _conn->ReadStreamStatus(i, timeout * 1000, metadata);
			if (ret != 0) {
				return false;
			}
			channel++;
		}

		if (metadata.endOfBurst)
			md.event_code = uhd::async_metadata_t::EVENT_CODE_BURST_ACK;

		if (metadata.hasTimestamp) {
			md.has_time_spec = true;
			md.time_spec = uhd::time_spec_t::from_ticks(metadata.timestamp, _conn->GetHardwareTimestampRate());
		}

		if (metadata.packetDropped) {
			md.event_code = uhd::async_metadata_t::EVENT_CODE_UNDERFLOW;
			return true;
		}

		if (metadata.lateTimestamp) {
			md.event_code = uhd::async_metadata_t::EVENT_CODE_TIME_ERROR;
			return true;
		}
		return true;
	}

private:

	lime::IConnection *_conn;
	std::vector<size_t> streamID;
	bool _fc64;
	bool _activated;
	const size_t _nchan;

};

uhd::rx_streamer::sptr limesdr_impl::get_rx_stream(const uhd::stream_args_t &args)
{
	if (not _rx_streamers[0].expired())
		return _rx_streamers[0].lock();

	uhd::rx_streamer::sptr stream(new LimeRxStream(_conn, args));
	BOOST_FOREACH(const size_t ch, args.channels) _rx_streamers[ch] = stream;
	if (args.channels.empty()) _rx_streamers[0] = stream;
	return stream;

}

uhd::tx_streamer::sptr limesdr_impl::get_tx_stream(const uhd::stream_args_t &args)
{
	if (not _tx_streamers[0].expired())
		return _tx_streamers[0].lock();

	uhd::tx_streamer::sptr stream(new LimeTxStream(_conn, args));
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

	std::vector<double> rates;

	const double clockRate = _rfics.front()->GetFrequencyCGEN();
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

	std::cout << boost::format(
		"OpenUSRP::setSampleRate(Rx %g MHz, Tx %g MHz) Failed -- no common clock rate.\n"
	) % (rateRx / 1e6) % (rateTx / 1e6) << std::endl;

	throw uhd::runtime_error("OpenUSRP::setSampleRate() -- no common clock rate");
}


void limesdr_impl::setSampleRate(const uhd::direction_t direction, const size_t channel, const double rate) {

	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	auto rfic = getRFIC(channel);
	LMS7002M_SelfCalState state(rfic);
	const auto lmsDir = (direction == TX_DIRECTION) ? LMS7002M::Tx : LMS7002M::Rx;

	double clockRate = _rfics.front()->GetFrequencyCGEN();
	const double dspFactor = clockRate / rfic->GetReferenceClk_TSP(lmsDir);

	//select automatic clock rate
	if (_autoTickRate)
	{
		double rxRate = rate, txRate = rate;
		if (direction != RX_DIRECTION and _fixedRxSampRate[channel]) rxRate = this->getSampleRate(RX_DIRECTION, channel);
		if (direction != TX_DIRECTION and _fixedTxSampRate[channel]) txRate = this->getSampleRate(TX_DIRECTION, channel);
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
		std::cout << boost::format(
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
		std::cout << "SetInterfaceFrequency Failed." << std::endl;

	status = _conn->UpdateExternalDataRate(
		rfic->GetActiveChannelIndex(),
		rfic->GetSampleRate(LMS7002M::Tx, rfic->GetActiveChannel()),
		rfic->GetSampleRate(LMS7002M::Rx, rfic->GetActiveChannel()));
	if (status != 0)
		std::cout << "UpdateExternalDataRate Failed." << std::endl;
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
		int sign = 0;
		int pos = 0, neg = 1;

		if (direction == TX_DIRECTION) {

			sign = (rfic->Get_SPI_Reg_bits(LMS7param(CMIX_SC_TXTSP)) == pos) ? 1 : -1;
		}
		else
		{
			if (rfic->Get_SPI_Reg_bits(LMS7_MASK, true) != 0) std::swap(pos, neg);
			sign = (rfic->Get_SPI_Reg_bits(LMS7param(CMIX_SC_RXTSP)) == pos) ? 1 : -1;
		}

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
		int pos = 0, neg = 1;

		if (direction == TX_DIRECTION) {

			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_BYP_TXTSP), (frequency == 0) ? 1 : 0);
			rfic->Modify_SPI_Reg_bits(LMS7param(CMIX_SC_TXTSP), (frequency < 0) ? neg : pos);
		}
		else {

			if (rfic->Get_SPI_Reg_bits(LMS7_MASK, true) != 0) std::swap(pos, neg);
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

		if (name == "TX/RX")  path = (_rx_frontend_map[channel] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
		else if (name == "RX2") path = (_rx_frontend_map[channel] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
		else throw uhd::runtime_error("OpenUSRP::setAntenna(RX, " + name + ") - unknown antenna name");

		rfic->SetPathRFE(path);
	}

	if (direction == TX_DIRECTION)
	{
		int band = 0;
		if (name == "TX/RX") band = (_tx_frontend_map[channel] == 0) ? LMS7002M::PATH_RFE_LB1 : LMS7002M::PATH_RFE_LB2;
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
			std::cout << boost::format(
				"OpenUSRP::setBandwidth(Rx, %d, %g MHz) Failed .\n"
			) % (int(channel)) % (bw / 1e6) << std::endl;
		}
	}

	if (direction == TX_DIRECTION)
	{
		if (rfic->TuneTxFilterWithCaching(bw) != 0)
		{
			std::cout << boost::format(
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

	spec.push_back(subdev_spec_pair_t("A", "A"));
	spec.push_back(subdev_spec_pair_t("A", "B"));

	return spec;
}

void limesdr_impl::set_frontend_mapping(const uhd::direction_t dir, const uhd::usrp::subdev_spec_t &spec) {

	if (spec.size() > 2) {
		std::cout << "LimeSDR only 2 channels";
		std::runtime_error("LimeSDR only 2 channels");
	}

	if (dir == RX_DIRECTION) {
		for (size_t i = 0; i < spec.size(); i++) {
			_rx_frontend_map[i] = (spec[i].sd_name == "A") ? 0 : 1;
			for (size_t i = 0; i < _rx_frontend_map.size(); i++) {
				LMS7002M::PathRFE path = (_rx_frontend_map[i] == 0) ? LMS7002M::PATH_RFE_LNAL : LMS7002M::PATH_RFE_LNAH;
				auto rfic = getRFIC(i);
				rfic->SetPathRFE(path);
			}
		}
	}

	if (dir == TX_DIRECTION) {

		for (size_t i = 0; i < spec.size(); i++) {
			_tx_frontend_map[i] = (spec[i].sd_name == "A") ? 0 : 1;
		}

		for (size_t i = 0; i < _tx_frontend_map.size(); i++) {
			for (size_t i = 0; i < _tx_frontend_map.size(); i++) {
				LMS7002M::PathRFE path = (_tx_frontend_map[i] == 0) ? LMS7002M::PATH_RFE_LB1 : LMS7002M::PATH_RFE_LB2;
				auto rfic = getRFIC(i);
				rfic->SetBandTRF(path);
			}
		}
	}
}


void limesdr_impl::setHardwareTime(const std::string &what, const uhd::time_spec_t &time)
{
	if (what == "NOW")
	{
		_conn->SetHardwareTimestamp(time.to_ticks(_conn->GetHardwareTimestampRate()));
	}
	else
	{

	}

}

uhd::time_spec_t limesdr_impl::getHardwareTime(const std::string &what) {

	uhd::time_spec_t time;
	if (what == "NOW") {
		time = uhd::time_spec_t::from_ticks(_conn->GetHardwareTimestamp(), _conn->GetHardwareTimestampRate());
	}
	else if (what == "PPS")
	{

	}

	return time;

}

double limesdr_impl::getMasterClockRate(void) {

#ifndef ENABLE_MAUNAL_CLOCK

	return fakeMasterClock;

#else
	boost::unique_lock<boost::recursive_mutex> lock(_accessMutex);
	//assume same rate for all RFIC in this wrapper
	return _rfics.front()->GetFrequencyCGEN();
#endif
}
void limesdr_impl::setMasterClockRate(const double rate) {

#ifndef ENABLE_MAUNAL_CLOCK

	fakeMasterClock = rate;

#else

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

#endif

}

uhd::filter_info_base::sptr limesdr_impl::getFilter(const uhd::direction_t dir, const size_t channel, const std::string &name) {

	return uhd::filter_info_base::sptr(new uhd::filter_info_base(uhd::filter_info_base::ANALOG_BAND_PASS, false, 0));
}

void limesdr_impl::setFilter(const uhd::direction_t dir, const size_t channel, const std::string &name, const filter_info_base::sptr filter) {

}

void limesdr_impl::setAutoTickRate(const bool enable) {

#ifdef ENABLE_MAUNAL_CLOCK
	_autoTickRate = enable;
#else
	_autoTickRate = true;
#endif // ENABLE_MAUNAL_CLOCK

}
