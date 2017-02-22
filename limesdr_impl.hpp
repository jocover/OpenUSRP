//
// Copyright (c) 2016, Jiang Wei <jiangwei@jiangwei.org>
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



#ifndef INCLUDED_LIMESDR_IMPL_HPP
#define INCLUDED_LIMESDR_IMPL_HPP

#include <IConnection.h>
#include <LMS7002M.h>
#include <LMS7002M_RegistersMap.h>
#include <ConnectionRegistry.h>

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/direction.hpp>
#include <uhd/types/clock_config.hpp>
#include <uhd/types/filters.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/utils/pimpl.hpp>
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/cast.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/paths.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/convert.hpp>



#include <boost/assign.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/bind.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define TX_GAIN_MAX 89.8
#define RX_GAIN_MAX 76.0

static const double DEFAULT_CLOCK_RATE = 80e6;

namespace lime
{
	class LMS7002M;
}

struct IConnectionStream
{
	std::vector<size_t> streamID;
	int direction;
	size_t elemSize;
	size_t elemMTU;

	//rx cmd requests
	bool hasCmd;
	int flags;
	long long timeNs;
	size_t numElems;
};


class limesdr_impl : public uhd::device {

public:
	limesdr_impl(const lime::ConnectionHandle &handle, const uhd::device_addr_t &);
	~limesdr_impl(void);

	//the io interface
	uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t &args);
	uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t &args);
	bool recv_async_msg(uhd::async_metadata_t &, double);

	IConnectionStream* setupStream(const uhd::direction_t dir, const std::string &format, const std::vector<size_t> &channels, const uhd::stream_args_t &args);

	void closeStream(IConnectionStream* stream);

	int activateStream(IConnectionStream* stream, const int flags, const long long timeNs, const size_t numElems);

	int deactivateStream(IConnectionStream* stream, const int flags, const long long timeNs);

	int readStream(IConnectionStream* stream, void * const *buffs, size_t numElems, int &flags, long long &timeNs, const long timeoutUs);

	int writeStream(IConnectionStream *stream, const void * const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs);

	int readStreamStatus(IConnectionStream *stream, size_t &chanMask, int &flags, long long &timeNs, const long timeoutUs);

private:
	lime::IConnection *_conn;
	const std::string _moduleName;

	lime::LMS7002M *getRFIC(const size_t channel) const;
	std::vector<lime::LMS7002M *> _rfics;
	boost::recursive_mutex _accessMutex;

	bool _autoTickRate;
	std::map<size_t, bool> _fixedRxSampRate;
	std::map<size_t, bool> _fixedTxSampRate;

	std::map<size_t, int> _interps;
	std::map<size_t, int> _decims;
	std::map<int, std::map<size_t, double>> _actualBw;

	std::map<size_t, boost::weak_ptr<uhd::rx_streamer> > _rx_streamers;
	std::map<size_t, boost::weak_ptr<uhd::tx_streamer> > _tx_streamers;

	std::vector<size_t> _rx_frontend_map;
	std::vector<size_t> _tx_frontend_map;

	uhd::usrp::subdev_spec_t  get_frontend_mapping(const uhd::direction_t dir);
	void set_frontend_mapping(const uhd::direction_t, const uhd::usrp::subdev_spec_t &);

	void setHardwareTime(const std::string &what, const uhd::time_spec_t &time);
	uhd::time_spec_t getHardwareTime(const std::string &what);

	double getMasterClockRate(void);
	void setMasterClockRate(const double);

	uhd::meta_range_t getSampleRange(const uhd::direction_t dir, const size_t chan);
	double getSampleRate(const uhd::direction_t dir, const size_t channel);
	void setSampleRate(const uhd::direction_t dir, const size_t channel, const double rate);

	double getGain(const uhd::direction_t dir, const size_t channel, const std::string &name);
	void setGain(const uhd::direction_t dir, const size_t channel, const std::string &name, const double gain);
	uhd::meta_range_t getGainRange(const uhd::direction_t dir, const size_t chan, const std::string &name);

	double getFrequency(const uhd::direction_t dir, const size_t channel, const std::string &name);
	void setFrequency(const uhd::direction_t direction, const size_t channel, const std::string &name, const double frequency);
	uhd::meta_range_t getFrequencyRange(const uhd::direction_t dir, const size_t chan, const std::string &name);

	void old_issue_stream_cmd(const size_t chan, const uhd::stream_cmd_t &cmd);

	void setAntenna(const uhd::direction_t, const size_t channel, const std::string &name);

	double getBandwidth(const uhd::direction_t dir, const size_t channel);
	void setBandwidth(const uhd::direction_t dir, const size_t channel, const double bw);
	uhd::meta_range_t getBandwidthRange(const uhd::direction_t dir, const size_t channel);

	void setDCOffsetMode(const uhd::direction_t direction, const size_t channel, const bool automatic);

	void setDCOffset(const uhd::direction_t direction, const size_t channel, const std::complex<double> &offset);

	void setIQBalance(const uhd::direction_t direction, const size_t channel, const std::complex<double> &balance);

	bool getDCOffsetMode(const uhd::direction_t direction, const size_t channel);

	void setAutoTickRate(const bool enable);


	uhd::filter_info_base::sptr getFilter(const uhd::direction_t dir, const size_t channel, const std::string &name);

	void setFilter(const uhd::direction_t dir, const size_t channel, const std::string &name, const uhd::filter_info_base::sptr filter);

	uhd::sensor_value_t get_temp(void);

	void setup_radio(const size_t dspno);

	uhd::sensor_value_t get_ref_locked(void);

	uhd::sensor_value_t get_lo_locked(const uhd::direction_t dir, const size_t channel);

	void error(void);

}
;
#endif /* INCLUDED_LIMESDR_IMPL_HPP */
