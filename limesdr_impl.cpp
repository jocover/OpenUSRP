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


#include "limesdr_impl.hpp"


using namespace uhd;
using namespace uhd::usrp;
using namespace lime;


/***********************************************************************
* Discovery
**********************************************************************/


static ConnectionHandle argsToHandle(const device_addr_t &args)
{
	ConnectionHandle handle;

	//load handle with key/value provided options
	if (args.has_key("module"))handle.module = args["module"];
	if (args.has_key("media"))handle.media = args["media"];
	if (args.has_key("name"))handle.name = args["name"];
	if (args.has_key("serial"))handle.serial = args["serial"];
	if (args.has_key("index"))handle.index = std::stoi(args["index"]);

	return handle;
}

static device_addr_t handleToArgs(const ConnectionHandle &handle)
{
	device_addr_t args;

	//convert the handle into key/value pairs
	args["type"] = "b200";
	args["product"] = "B210";
	if (not handle.module.empty()) args["module"] = handle.module;
	if (not handle.media.empty()) args["media"] = handle.media;
	if (not handle.name.empty()) args["name"] = handle.name;
	if (not handle.serial.empty()) args["serial"] = handle.serial;
	if (handle.index != -1) args["index"] = std::to_string(handle.index);

	return args;
}

static device_addrs_t limesdr_find(const device_addr_t &hint) {
	device_addrs_t limesdr_addrs;

	if (hint.has_key("type") and (hint["type"] != "b200")) return limesdr_addrs;

	BOOST_FOREACH(device_addr_t hint_i, separate_device_addr(hint)) {

		if (hint_i.has_key("addr") || hint_i.has_key("resource")) return limesdr_addrs;
		
	}

	for (const auto &handle : lime::ConnectionRegistry::findConnections(argsToHandle(hint)))
	{
		limesdr_addrs.push_back(handleToArgs(handle));
	}

	return limesdr_addrs;
}

/***********************************************************************
* Make
**********************************************************************/

static device::sptr limesdr_make(const device_addr_t &device_addr) {

	return device::sptr(new limesdr_impl(argsToHandle(device_addr), device_addr));

}

UHD_STATIC_BLOCK(register_limesdr_device)
{
#ifdef UHD_HAS_DEVICE_FILTER
	device::register_device(&limesdr_find, &limesdr_make, device::USRP);
#else
	device::register_device(&limesdr_find, &limesdr_make);
#endif

}


/***********************************************************************
* Structors
**********************************************************************/
limesdr_impl::limesdr_impl(const lime::ConnectionHandle &handle, const uhd::device_addr_t& device_addr) :_conn(nullptr), _moduleName(handle.module), _autoTickRate(false){


	_tree = property_tree::make();
	_type = device::USRP;
	const fs_path mb_path = "/mboards/0";



	std::cout << "Using OpenUSRP" << std::endl;


	_conn = ConnectionRegistry::makeConnection(handle);

	if (_conn == nullptr) throw uhd::runtime_error(
		"Failed to make connection with '" + handle.serialize() + "'");
	////////////////////////////////////////////////////////////////////
	// setup the mboard eeprom
	////////////////////////////////////////////////////////////////////

	const auto devInfo = _conn->GetDeviceInfo();
	const size_t numRFICs = devInfo.addrsLMS7002M.size();

	uhd::usrp::mboard_eeprom_t mb_eeprom;
	mb_eeprom["revision"] = devInfo.hardwareVersion;
	mb_eeprom["product"] = "2";
	mb_eeprom["serial"] = str(boost::format("%X") % uint32_t(devInfo.boardSerialNumber));

	_tree->create<uhd::usrp::mboard_eeprom_t>(mb_path / "eeprom").set(mb_eeprom);
	_tree->create<std::string>(mb_path / "fw_version").set(devInfo.firmwareVersion);
	_tree->create<std::string>(mb_path / "fpga_version").set(str(boost::format("%s.%s") % devInfo.gatewareVersion%devInfo.gatewareRevision));


	////////////////////////////////////////////////////////////////////
	// Create control transport
	////////////////////////////////////////////////////////////////////
	_tree->create<double>(mb_path / "link_max_rate").set(500000000);

	////////////////////////////////////////////////////////////////////
	// Initialize the properties tree
	////////////////////////////////////////////////////////////////////

	_tree->create<std::string>("/name").set("B-Series Device");

	_tree->create<std::string>(mb_path / "name").set("B210");
	_tree->create<std::string>(mb_path / "codename").set("Sasquatch");

	////////////////////////////////////////////////////////////////////
	// create codec control objects
	////////////////////////////////////////////////////////////////////
	{
		const fs_path codec_path = mb_path / ("rx_codecs") / "A";
		_tree->create<std::string>(codec_path / "name").set("B210 RX dual ADC");
		_tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
	}
	{
		const fs_path codec_path = mb_path / ("tx_codecs") / "A";
		_tree->create<std::string>(codec_path / "name").set("B210 RX dual ADC");
		_tree->create<int>(codec_path / "gains"); //empty cuz gains are in frontend
	}

	////////////////////////////////////////////////////////////////////
	// create clock control objects
	////////////////////////////////////////////////////////////////////

	_tree->create<double>(mb_path / "tick_rate")
		.publish(boost::bind(&limesdr_impl::getMasterClockRate, this))
		.subscribe(boost::bind(&limesdr_impl::setMasterClockRate, this, _1));



	_tree->create<uhd::time_spec_t>(mb_path / "time" / "cmd")
		.subscribe(boost::bind(&limesdr_impl::setHardwareTime, this, "CMD", _1));//TODO


	_tree->create<bool>(mb_path / "auto_tick_rate").set(false).subscribe(boost::bind(&limesdr_impl::setAutoTickRate, this, _1));

	////////////////////////////////////////////////////////////////////
	// and do the misc mboard sensors
	////////////////////////////////////////////////////////////////////
	_tree->create<sensor_value_t>(mb_path / "sensors" / "ref_locked")
		.publish(boost::bind(&limesdr_impl::get_ref_locked, this));


	////////////////////////////////////////////////////////////////////
	// create frontend mapping
	////////////////////////////////////////////////////////////////////


	std::vector<size_t> default_map(2, 0); default_map[1] = 1;
	_tree->create<std::vector<size_t> >(mb_path / "rx_chan_dsp_mapping").set(default_map);

	_tree->create<std::vector<size_t> >(mb_path / "tx_chan_dsp_mapping").set(default_map);

	_rx_frontend_map.resize(2, 0);
	
	_tree->create<uhd::usrp::subdev_spec_t>(mb_path / "rx_subdev_spec")
		.publish(boost::bind(&limesdr_impl::get_frontend_mapping, this, RX_DIRECTION))
		.set(subdev_spec_t())
		.subscribe(boost::bind(&limesdr_impl::set_frontend_mapping, this, RX_DIRECTION, _1));

	_tx_frontend_map.resize(2, 0);
	_tree->create<uhd::usrp::subdev_spec_t>(mb_path / "tx_subdev_spec")
		.publish(boost::bind(&limesdr_impl::get_frontend_mapping, this, TX_DIRECTION))
		.set(subdev_spec_t())
		.subscribe(boost::bind(&limesdr_impl::set_frontend_mapping, this, TX_DIRECTION, _1));

	////////////////////////////////////////////////////////////////////
	// setup radio control
	////////////////////////////////////////////////////////////////////

	for (size_t i = 0; i < numRFICs; i++) {

		_rfics.push_back(new LMS7002M());
		_rfics.back()->SetConnection(_conn, i);

		int st;

		st = _rfics.back()->ResetChip();
		if (st != 0) throw uhd::runtime_error("ResetChip() failed");

		st = _rfics.back()->SoftReset();
		if (st != 0) throw uhd::runtime_error("SoftReset() failed");

		st = _rfics.back()->UploadAll();
		if (st != 0) throw uhd::runtime_error("UploadAll() failed");

	}


	for (size_t channel = 0; channel < _rfics.size() * 2; channel++)
	{
		auto rfic = getRFIC(channel);
		rfic->EnableChannel(LMS7002M::Tx, true);
		rfic->EnableChannel(LMS7002M::Rx, true);

		setup_radio(channel);
	}

	for (auto rfic : _rfics) rfic->EnableValuesCache((not device_addr.has_key("ignore-cal-file")));

	double defaultClockRate = device_addr.cast<double>("master_clock_rate", DEFAULT_CLOCK_RATE);

#ifdef ENABLE_MAUNAL_CLOCK
	_tree->access<double>(mb_path / "tick_rate").set(defaultClockRate);
#else
	fakeMasterClock = defaultClockRate;

	for (auto rfic : _rfics)
	{
		//make tx rx rates equal
		rfic->Modify_SPI_Reg_bits(LMS7param(EN_ADCCLKH_CLKGN), 0);
		rfic->Modify_SPI_Reg_bits(LMS7param(CLKH_OV_CLKL_CGEN), 2);
		rfic->SetFrequencyCGEN(DEFAULT_CLOCK_RATE);
	}
#endif


	for (size_t channel = 0; channel < _rfics.size() * 2; channel++)
	{
		this->setFrequency(RX_DIRECTION, channel, "BB", 0.0);
		this->setFrequency(TX_DIRECTION, channel, "BB", 0.0);
		this->setAntenna(RX_DIRECTION, channel, "RX2");
		this->setAntenna(TX_DIRECTION, channel, "TX/RX");
		this->setGain(RX_DIRECTION, channel, "PGA", 0);
		this->setGain(RX_DIRECTION, channel, "LNA", 0);
		this->setGain(RX_DIRECTION, channel, "TIA", 0);
		this->setGain(TX_DIRECTION, channel, "PAD", -50);
		this->setSampleRate(RX_DIRECTION, channel, defaultClockRate / 8);
		this->setSampleRate(TX_DIRECTION, channel, defaultClockRate / 8);
		this->setBandwidth(RX_DIRECTION, channel, 30e6);
		this->setBandwidth(TX_DIRECTION, channel, 30e6);
		this->setDCOffsetMode(RX_DIRECTION, channel, true);
		this->setDCOffset(TX_DIRECTION, channel, 0.0);
		this->setIQBalance(RX_DIRECTION, channel, 1.0);
		this->setIQBalance(TX_DIRECTION, channel, 1.0);
	}

	_fixedRxSampRate.clear();
	_fixedTxSampRate.clear();
	_channelsToCal.clear();

	_tree->create<uhd::time_spec_t>(mb_path / "time" / "now")
		.publish(boost::bind(&limesdr_impl::getHardwareTime, this, "NOW"))
		.subscribe(boost::bind(&limesdr_impl::setHardwareTime, this, "NOW", _1));
	_tree->create<uhd::time_spec_t>(mb_path / "time" / "pps")
		.publish(boost::bind(&limesdr_impl::getHardwareTime, this, "PPS"))
		.subscribe(boost::bind(&limesdr_impl::setHardwareTime, this, "PPS", _1));

	//setup time source props

	static const std::vector<std::string> time_sources = boost::assign::list_of("none")("internal")("external");
	_tree->create<std::vector<std::string> >(mb_path / "time_source" / "options").set(time_sources);
	_tree->create<std::string>(mb_path / "time_source" / "value");//TODO

	static const std::vector<std::string> clock_sources = boost::assign::list_of("internal")("external");
	_tree->create<std::vector<std::string> >(mb_path / "clock_source" / "options").set(clock_sources);
	_tree->create<std::string>(mb_path / "clock_source" / "value");

	////////////////////////////////////////////////////////////////////
	// front panel gpio
	////////////////////////////////////////////////////////////////////

	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "CTRL").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "DDR").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "OUT").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "ATR_0X").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "ATR_RX").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "ATR_TX").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "ATR_XX").set(0);
	_tree->create<boost::uint32_t>(mb_path / "gpio" / "FP0" / "READBACK").set(0);

	////////////////////////////////////////////////////////////////////
	// dboard eeproms but not really
	////////////////////////////////////////////////////////////////////
	dboard_eeprom_t db_eeprom;
	_tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "rx_eeprom").set(db_eeprom);
	_tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "tx_eeprom").set(db_eeprom);
	_tree->create<dboard_eeprom_t>(mb_path / "dboards" / "A" / "gdb_eeprom").set(db_eeprom);

	////////////////////////////////////////////////////////////////////
	// do some post-init tasks
	////////////////////////////////////////////////////////////////////


	_tree->access<double>(mb_path / "tick_rate").set(getMasterClockRate());

	subdev_spec_t spec;
	spec.push_back(subdev_spec_pair_t("A", "A"));
	spec.push_back(subdev_spec_pair_t("A", "A"));
	_tree->access<subdev_spec_t>(mb_path / "rx_subdev_spec").set(spec);
	_tree->access<subdev_spec_t>(mb_path / "tx_subdev_spec").set(spec);


	//init to internal clock and time source
	_tree->access<std::string>(mb_path / "clock_source/value").set("internal");
	_tree->access<std::string>(mb_path / "time_source/value").set("internal");

	_tree->access<bool>(mb_path / "auto_tick_rate").set(not device_addr.has_key("master_clock_rate"));

}



limesdr_impl::~limesdr_impl(void)
{
	for (size_t channel = 0; channel < _rfics.size() * 2; channel++)
	{
		auto rfic = getRFIC(channel);
		rfic->EnableChannel(LMS7002M::Tx, false);
		rfic->EnableChannel(LMS7002M::Rx, false);
	}

	for (auto rfic : _rfics) delete rfic;
	ConnectionRegistry::freeConnection(_conn);

}


void limesdr_impl::setup_radio(const size_t dspno) {
	const fs_path mb_path = "/mboards/0";

	////////////////////////////////////////////////////////////////////
	// connect rx dsp control objects
	////////////////////////////////////////////////////////////////////

	const fs_path rx_dsp_path = mb_path / "rx_dsps" / dspno;

	_tree->create<uhd::meta_range_t>(rx_dsp_path / "rate" / "range")
		.publish(boost::bind(&limesdr_impl::getSampleRange, this, RX_DIRECTION, dspno));
	_tree->create<double>(rx_dsp_path / "rate" / "value")
		.publish(boost::bind(&limesdr_impl::getSampleRate, this, RX_DIRECTION, dspno))
		.subscribe(boost::bind(&limesdr_impl::setSampleRate, this, RX_DIRECTION, dspno, _1));
	//dsp freq
	_tree->create<double>(rx_dsp_path / "freq" / "value")
		.publish(boost::bind(&limesdr_impl::getFrequency, this, RX_DIRECTION, dspno, "BB"))
		.subscribe(boost::bind(&limesdr_impl::setFrequency, this, RX_DIRECTION, dspno, "BB", _1));
	_tree->create<uhd::meta_range_t>(rx_dsp_path / "freq" / "range")
		.publish(boost::bind(&limesdr_impl::getFrequencyRange, this, RX_DIRECTION, dspno, "BB"));
	_tree->create<uhd::stream_cmd_t>(rx_dsp_path / "stream_cmd")
		.subscribe(boost::bind(&limesdr_impl::old_issue_stream_cmd, this, dspno, _1));

	////////////////////////////////////////////////////////////////////
	// create tx dsp control objects
	////////////////////////////////////////////////////////////////////
	const fs_path tx_dsp_path = mb_path / "tx_dsps" / dspno;

	_tree->create<uhd::meta_range_t>(tx_dsp_path / "rate" / "range")
		.publish(boost::bind(&limesdr_impl::getSampleRange, this, TX_DIRECTION, dspno));
	_tree->create<double>(tx_dsp_path / "rate" / "value")
		.publish(boost::bind(&limesdr_impl::getSampleRate, this, TX_DIRECTION, dspno))
		.subscribe(boost::bind(&limesdr_impl::setSampleRate, this, TX_DIRECTION, dspno, _1));
	//dsp freq
	_tree->create<double>(tx_dsp_path / "freq" / "value")
		.publish(boost::bind(&limesdr_impl::getFrequency, this, TX_DIRECTION, dspno, "BB"))
		.subscribe(boost::bind(&limesdr_impl::setFrequency, this, TX_DIRECTION, dspno, "BB", _1));
	_tree->create<uhd::meta_range_t>(tx_dsp_path / "freq" / "range")
		.publish(boost::bind(&limesdr_impl::getFrequencyRange, this, TX_DIRECTION, dspno, "BB"));


	////////////////////////////////////////////////////////////////////
	// create RF frontend interfacing
	////////////////////////////////////////////////////////////////////
	static const std::vector<direction_t> dirs = boost::assign::list_of(RX_DIRECTION)(TX_DIRECTION);
	BOOST_FOREACH(direction_t dir, dirs) {

		const std::string x = (dir == RX_DIRECTION) ? "rx" : "tx";
		const std::string key = std::string(((dir == RX_DIRECTION) ? "RX" : "TX")) + std::string(((dspno == 0) ? "1" : "2"));
		const fs_path rf_fe_path = mb_path / "dboards" / "A" / (x + "_frontends") / (dspno ? "B" : "A");

		_tree->create<std::string>(rf_fe_path / "name").set("FE-" + key);
		_tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/temp").publish(boost::bind(&limesdr_impl::get_temp, this));
		_tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/lo_locked").publish(boost::bind(&limesdr_impl::get_lo_locked, this, dir, dspno));



		if (dir == RX_DIRECTION) {
			_tree->create<uhd::sensor_value_t>(rf_fe_path / "sensors/rssi");
		}

		_tree->create<meta_range_t>(rf_fe_path / "gains" / "PGA" / "range").publish(boost::bind(&limesdr_impl::getGainRange, this, dir, dspno, "Normal"));
		_tree->create<double>(rf_fe_path / "gains" / "PGA" / "value")
			.publish(boost::bind(&limesdr_impl::getGain, this, dir, dspno, "Normal"))
			.subscribe(boost::bind(&limesdr_impl::setGain, this, dir, dspno, "Normal", _1));


		_tree->create<std::string>(rf_fe_path / "connection").set("IQ");
		_tree->create<bool>(rf_fe_path / "enabled").set(true);
		_tree->create<bool>(rf_fe_path / "use_lo_offset").set(false);


		_tree->create<double>(rf_fe_path / "bandwidth" / "value")
			.publish(boost::bind(&limesdr_impl::getBandwidth, this, dir, dspno))
			.subscribe(boost::bind(&limesdr_impl::setBandwidth, this, dir, dspno, _1));
		_tree->create<uhd::meta_range_t>(rf_fe_path / "bandwidth" / "range")
			.publish(boost::bind(&limesdr_impl::getBandwidthRange, this, dir, dspno));

		_tree->create<double>(rf_fe_path / "freq" / "value")
			.publish(boost::bind(&limesdr_impl::getFrequency, this, dir, dspno, "RF"))
			.subscribe(boost::bind(&limesdr_impl::setFrequency, this, dir, dspno, "RF", _1));
		_tree->create<uhd::meta_range_t>(rf_fe_path / "freq" / "range")
			.publish(boost::bind(&limesdr_impl::getFrequencyRange, this, dir, dspno, "RF"));


		if (dir == RX_DIRECTION) {

			_tree->create<bool>(rf_fe_path / "dc_offset/enable")
				.publish(boost::bind(&limesdr_impl::getDCOffsetMode, this, dir, dspno))
				.subscribe(boost::bind(&limesdr_impl::setDCOffsetMode, this, dir, dspno, _1));

			_tree->create<bool>(rf_fe_path / "iq_balance/enable").set(true);

			const std::list<std::string> mode_strings = boost::assign::list_of("slow")("fast");
			_tree->create<bool>(rf_fe_path / "gain/agc/enable").set(false);

			_tree->create<std::string>(rf_fe_path / "gain/agc/mode/value").set(mode_strings.front());
			_tree->create< std::list<std::string> >(rf_fe_path / "gain/agc/mode/options").set(mode_strings);
		}

		// Frontend filters

		if (dir == RX_DIRECTION) {

			const std::vector<std::string> rx_filter = boost::assign::list_of("DEC_3")("FIR_1")("HB_1")("HB_2")("HB_3")("LPF_BB")("LPF_TIA");

			BOOST_FOREACH(std::string filter_name, rx_filter) {

				_tree->create<filter_info_base::sptr>(rf_fe_path / "filters" / filter_name / "value")
					.publish(boost::bind(&limesdr_impl::getFilter, this, dir, dspno, filter_name))
					.subscribe(boost::bind(&limesdr_impl::setFilter, this, dir, dspno, filter_name, _1));
			}
		}

		if (dir == TX_DIRECTION) {

			const std::vector<std::string> tx_filter = boost::assign::list_of("FIR_1")("HB_1")("HB_2")("HB_3")("INT_3")("LPF_BB")("LPF_SECONDARY");

			BOOST_FOREACH(std::string filter_name, tx_filter) {

				_tree->create<filter_info_base::sptr>(rf_fe_path / "filters" / filter_name / "value")
					.publish(boost::bind(&limesdr_impl::getFilter, this, dir, dspno, filter_name))
					.subscribe(boost::bind(&limesdr_impl::setFilter, this, dir, dspno, filter_name, _1));

			}
		}

		//TODO LimeSDR have 3 RX ant and 2 TX ant 
		if (dir == RX_DIRECTION)
		{
			static const std::vector<std::string> ants = boost::assign::list_of("TX/RX")("RX2");
			_tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
			_tree->create<std::string>(rf_fe_path / "antenna" / "value")
				.subscribe(boost::bind(&limesdr_impl::setAntenna, this, dir, dspno, _1))
				.set("RX2");

		}
		else if (dir == TX_DIRECTION)
		{
			static const std::vector<std::string> ants(1, "TX/RX");
			_tree->create<std::vector<std::string> >(rf_fe_path / "antenna" / "options").set(ants);
			_tree->create<std::string>(rf_fe_path / "antenna" / "value")
				.subscribe(boost::bind(&limesdr_impl::setAntenna, this, dir, dspno, _1))
				.set("TX/RX");
		}

	}

}

