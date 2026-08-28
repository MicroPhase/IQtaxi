#ifndef INCLUDE_IQTAXI_IMPL_HPP
#define INCLUDE_IQTAXI_IMPL_HPP

#include <uhd/device.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/direction.hpp>
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
#include "uhd/transport/bounded_buffer.hpp"
#include "uhd/utils/tasks.hpp"
#include <boost/assign.hpp>
#include "../../driver/IQTAXI/iqtaxi_udp_impl.hpp"
#include "../../driver/E100/e100_impl.hpp"
#include "../../driver/E200/e200_impl.hpp"
#include "../../driver/M300/m300_xdma_impl.hpp"
#include "../../../include/sdr/api/Device.hpp"
#include "../../../include/sdr/api/UdpDiscover.hpp"
#include "../../driver/transport/super_recv_packet_handler.hpp"
#include "../../driver/transport/super_send_packet_handler.hpp"
#include "../../driver/E100/local_e100_regs.hpp"

using namespace sdr::api;
using namespace sdr::core;
using namespace e100;

#define MICROPHASE_e100_FW_COMPAT_NUM 2

#define BUFF_SIZE 1000e3

#define E100_CTRL_LATCH_TIME_NOW (1 << 0)
#define E100_CTRL_LATCH_TIME_PPS (1 << 1)
#define E100_CTRL_LATCH_TIME_SYNC (1 << 2)

#define MICROPHASE_CHECK "MicroPhase"

#define    MICROPHASE_NAME_BR0 "e100"
#define    MICROPHASE_NAME_E100 "E100"
#define    MICROPHASE_NAME_E200 "E200"
#define    MICROPHASE_NAME_E206 "E206"
#define    MICROPHASE_NAME_M300 "M300"

#define    MICROPHASE_RX_WAZZUP_BR0 "r"

using microphase_e100_ctrl_data_t = sdr::api::IqtaxiUdpDiscoverPacket;

static const uint8_t e100_FW_COMPAT_NUM_MAJOR = 8;
static const uint8_t e100_FW_COMPAT_NUM_MINOR = 0;
static const uint16_t e100_FPGA_COMPAT_NUM    = 16;
static const double e100_BUS_CLOCK_RATE       = 100e6;
static const uint32_t e100_GPSDO_ST_NONE      = 0x83;

#define FLIP_SID(sid) (((sid) << 16) | ((sid) >> 16))

static const uint32_t e100_CTRL0_MSG_SID = 0x10;
static const uint32_t e100_RESP0_MSG_SID = e100_CTRL0_MSG_SID;

static const uint32_t e100_CTRL1_MSG_SID = 0x20;
static const uint32_t e100_RESP1_MSG_SID = e100_CTRL1_MSG_SID;

static const uint32_t e100_TX_DATA0_SID = 0x50;
static const uint32_t e100_TX_MSG0_SID  = e100_TX_DATA0_SID;

static const uint32_t e100_TX_DATA1_SID = 0x60;
static const uint32_t e100_TX_MSG1_SID  = e100_TX_DATA1_SID;

static const uint32_t e100_RX_DATA0_SID = 0xA0;
static const uint32_t e100_RX_DATA1_SID = 0xB0;

static const uint32_t e100_TX_GPS_UART_SID = 0x30;
static const uint32_t e100_RX_GPS_UART_SID = e100_TX_GPS_UART_SID;

static const uint32_t e100_LOCAL_CTRL_SID = 0x70;
static const uint32_t e100_LOCAL_RESP_SID = e100_LOCAL_CTRL_SID;

static const uint32_t e100_HW_CTRL_SID = 0x70;
static const uint32_t e100_HW_RESP_SID = e100_HW_CTRL_SID;

class iqtaxi_impl : public uhd::device
{
public:
    // structors
    iqtaxi_impl(const uhd::device_addr_t &);
    ~iqtaxi_impl(void) override;

    // the io interface
    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t& args) override;
    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t& args) override;
    bool recv_async_msg(uhd::async_metadata_t&, double) override;
    void set_mb_eeprom();

    double defaultClockRate = 30.72e6;
private:

    enum time_source_t {
        GPSDO    = 0,
        EXTERNAL = 1,
        INTERNAL = 2,
        NONE     = 3,
        UNKNOWN  = 4
    } _time_source;
    sdr::api::Device::sptr iqtaxi_device;
    const DeviceProfile* _profile = &e100_udp_profile();
    std::string _product_name = MICROPHASE_NAME_E100;
    bool _is_m300 = false;
    bool _is_e100 = true;
    bool _ignore_tx_timestamps = true;
    std::string _clock_source = "internal";
    // controllers
    std::weak_ptr<uhd::rx_streamer> _rx_streamer;
    std::weak_ptr<uhd::tx_streamer> _tx_streamer;
    local_ctrl::sptr _local_bus;
    local_ctrl::sptr _rx_stream_bus;
    local_ctrl::sptr _tx_stream_bus;

    std::mutex _transport_setup_mutex;
    std::mutex _ctrl_mutex;

    double _tick_rate;
    double get_tick_rate(void)
    {
        return _tick_rate;
    }

    double set_tick_rate(const double rate);
    double set_sample_clock_preserving_time(const double rate);

    // void set_auto_tick_rate(const double rate = 0,
    //     const uhd::fs_path& tree_dsp_path     = "",
    //     size_t num_chans                      = 0);

    void update_tick_rate(const double);


    // perifs in the radio core
    struct radio_perifs_t
    {
        local_ctrl::sptr ctrl;
        std::weak_ptr<uhd::rx_streamer> rx_streamer;
        std::weak_ptr<uhd::tx_streamer> tx_streamer;
    };
    std::vector<radio_perifs_t> _radio_perifs;

    void setup_radio(const size_t radio_index);

    uhd::sensor_value_t get_ref_locked(void);
    uhd::usrp::subdev_spec_t coerce_subdev_spec(const uhd::usrp::subdev_spec_t& spec_);
    // void update_subdev_spec(const std::string& tx_rx, const uhd::usrp::subdev_spec_t& spec);
    void set_time(const uhd::time_spec_t& t);
    void sync_times();
    void update_time_source(const std::string& source);
    void update_clock_source(const std::string& source);
    uhd::time_spec_t get_time_now(void);
    uhd::time_spec_t get_time_last_pps(void);
    void set_time_now(const uhd::time_spec_t& time);
    void set_time_sync(const uhd::time_spec_t& time);
    void set_time_next_pps(const uhd::time_spec_t& time);
    
    std::vector<size_t> _rx_frontend_map;
	std::vector<size_t> _tx_frontend_map;

	uhd::usrp::subdev_spec_t  get_frontend_mapping(const uhd::direction_t dir);
	void set_frontend_mapping(const uhd::direction_t, const uhd::usrp::subdev_spec_t &);

	uhd::meta_range_t get_sample_range();
	double getSampleRate(const uhd::direction_t dir, const size_t channel);
	void setSampleRate(const uhd::direction_t dir, const size_t channel, const double rate);

	double getGain(const uhd::direction_t dir, const size_t channel);
	void setGain(const uhd::direction_t dir, const size_t channel, const double gain);
	uhd::meta_range_t getGainRange(const uhd::direction_t dir, const size_t chan, const std::string &name);
	void setAmpEnable(bool enable);
	bool getAmpEnable();

    void update_bandsel(const std::string& which, double freq);
	double getFrequency(const uhd::direction_t dir, const size_t channel, const std::string &name);
	void set_frequency(const uhd::direction_t dir, const std::string &name, const double frequency);
	// void set_tx_frequency(const double frequency);
	
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

	// uhd::filter_info_base::sptr getFilter(const uhd::direction_t dir, const size_t channel, const std::string &name);

	// void setFilter(const uhd::direction_t dir, const size_t channel, const std::string &name, const uhd::filter_info_base::sptr filter);

	// uhd::sensor_value_t get_temp(void);

	// uhd::sensor_value_t get_ref_locked(void);

	// uhd::sensor_value_t get_rssi(void);

	// uhd::sensor_value_t get_lo_locked(const uhd::direction_t dir, const size_t channel);

    void _program_dispatcher(zero_copy_if& xport);
};

#endif
