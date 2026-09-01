#include "./iqtaxi_udp_impl.hpp"
#include "../E100/local_e100_regs.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace e100;

namespace {
constexpr double kSampleRateCommandTimeoutSec = 25.0;
/* A GC080X LO change can fall back from fastlock to a full band calibration.
 * The device server handles commands synchronously, so its ACK is intentionally
 * delayed until the synthesizers have either locked or returned a real error. */
constexpr double kLoCommandTimeoutSec = 10.0;
constexpr size_t kControlSocketBufferBytes = 1u * 1024u * 1024u;
constexpr size_t kStreamSocketBufferBytes = 64u * 1024u * 1024u;
constexpr size_t kRxStreamRecvFrames = 2048u;
constexpr size_t kTxStreamSendFrames = 64u;

void send_stream_hello(const zero_copy_if::sptr& xport, const char* payload)
{
    const size_t payload_len = std::strlen(payload);
    managed_send_buffer::sptr send_buffer = xport->get_send_buff(0.1, static_cast<uint32_t>(payload_len));
    if (!send_buffer) {
        throw std::runtime_error("failed to allocate stream hello buffer");
    }

    std::memcpy(send_buffer->cast<void*>(), payload, payload_len);
    send_buffer->commit(payload_len);
}
void check_lo_command_status(uint64_t response, const char* direction, uint64_t frequency_hz)
{
    const int32_t status = static_cast<int32_t>(static_cast<uint32_t>(response));
    if (status != 0) {
        throw std::runtime_error(
            std::string("set ") + direction + " LO failed: device status " +
            std::to_string(status) + ", target=" + std::to_string(frequency_hz) + " Hz");
    }
}

template <typename Fn>
void with_rx_epoch_restart(rx_streamer::sptr& stream, Fn&& fn)
{
    const bool restart = static_cast<bool>(stream);
    if (restart) {
        stream->prepare_for_rx_epoch_change();
    }
    try {
        fn();
    } catch (...) {
        if (restart) {
            try {
                stream->finish_rx_epoch_change();
            } catch (...) {
            }
        }
        throw;
    }
    if (restart) {
        stream->finish_rx_epoch_change();
    }
}
} // namespace

void IqtaxiUdpImpl::send_rx_hello()
{
    const std::string rx_hello = _profile.product + "-RX-HELLO";
    send_stream_hello(_udp_rx_stream, rx_hello.c_str());
}

void IqtaxiUdpImpl::send_tx_hello()
{
    const std::string tx_hello = _profile.product + "-TX-HELLO";
    send_stream_hello(_udp_tx_stream, tx_hello.c_str());
}

IqtaxiUdpImpl::IqtaxiUdpImpl(const std::string port, const DeviceProfile& profile)
    : _profile(profile)
{
    // Acquire the process-wide device lease before opening sockets or sending
    // HELLO/control packets. This also protects callers that instantiate a
    // concrete E-series backend directly instead of using Device::makeDevice.
    acquire_exclusive_access(_profile.product, port);
    if(! iqtaxi_udp_init(port))
    {
        printf("IQTAXI UDP init failed\n");
        initial_success = false;
    } else if (_profile.product == "E100") {
        // E100 firmware exposes EEPROM-backed RF limits as 64-bit readbacks.
        // Keep the static profile as a fallback for older firmware.
        try {
            const uint64_t min_hz = _local_bus->peek64(CUSTOM_RB_GET_RF_FREQ_MIN_ADDR);
            const uint64_t max_hz = _local_bus->peek64(CUSTOM_RB_GET_RF_FREQ_MAX_ADDR);
            if (min_hz > 0ull && max_hz > min_hz) {
                _profile = e100_udp_profile_with_freq(
                    static_cast<double>(min_hz),
                    static_cast<double>(max_hz));
            }
            try {
                const uint32_t board_band =
                    _local_bus->peek32(CUSTOM_RB_GET_BOARD_BAND_ADDR);
                if (board_band == 10u || board_band == 10000u) {
                    _profile.rf_band = "10G";
                } else if (board_band == 6u || board_band == 6000u) {
                    _profile.rf_band = "6G";
                }
            } catch (const std::exception&) {
                // Older firmware may not implement board-band readback.
            }
            if (_profile.rf_band.empty()) {
                _profile.rf_band = e100_rf_band_from_max_hz(
                    _profile.rx_frequency_hz.maximum);
            }
            std::cout << e100_display_name(_profile) << " RF range from device: "
                      << static_cast<uint64_t>(_profile.rx_frequency_hz.minimum)
                      << " .. "
                      << static_cast<uint64_t>(_profile.rx_frequency_hz.maximum)
                      << " Hz" << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "E100 RF range readback failed, using default profile: "
                      << ex.what() << std::endl;
        }
    }
}

bool IqtaxiUdpImpl::isInitialSuccess(){
    return initial_success;
}

IqtaxiUdpImpl::~IqtaxiUdpImpl() {
#ifdef _WIN32
    WSACleanup();
#endif
}

bool IqtaxiUdpImpl::iqtaxi_udp_init(const std::string port) {
    #ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("WSAStartup failed");
    }
    #endif
    zero_copy_xport_params default_buff_args;
    default_buff_args.send_frame_size = 1500-20-8;
    default_buff_args.recv_frame_size = 1500-20-8;
    default_buff_args.num_send_frames = 16;
    default_buff_args.num_recv_frames = 16;
    default_buff_args.send_buff_size = kControlSocketBufferBytes;
    default_buff_args.recv_buff_size = kControlSocketBufferBytes;
    _udp_ctrl =  udp_zero_copy::make(port, SDR_STRINGIZE(MICROPHASE_IQTAXI_UDP_CTRL_PORT), default_buff_args);
    _local_bus = std::make_shared<local_ctrl>(_udp_ctrl,0x70,8192);

    default_buff_args.num_send_frames = 16;
    default_buff_args.num_recv_frames = kRxStreamRecvFrames;
    default_buff_args.send_buff_size = kStreamSocketBufferBytes;
    default_buff_args.recv_buff_size = kStreamSocketBufferBytes;
    _udp_rx_stream= udp_zero_copy::make(port, SDR_STRINGIZE(MICROPHASE_IQTAXI_UDP_DATA_RX_PORT), default_buff_args);
    _rx_stream_bus = std::make_shared<local_ctrl>(_udp_rx_stream,0x71,8192);

    default_buff_args.num_send_frames = kTxStreamSendFrames;
    default_buff_args.num_recv_frames = 16;
    _udp_tx_stream= udp_zero_copy::make(port, SDR_STRINGIZE(MICROPHASE_IQTAXI_UDP_DATA_TX_PORT), default_buff_args);
    _tx_stream_bus = std::make_shared<local_ctrl>(_udp_tx_stream,0x72,8192);

    _local_bus->poke32(SET_CMD_PORT, 0);
    send_rx_hello();
    send_tx_hello();
    set_channel_enable(1);

    return true;
}

std::shared_ptr<local_ctrl> IqtaxiUdpImpl::get_local_bus(){
    return _local_bus;
}

std::shared_ptr<local_ctrl> IqtaxiUdpImpl::get_rx_stream_bus(){
    return _rx_stream_bus;
}

std::shared_ptr<local_ctrl> IqtaxiUdpImpl::get_tx_stream_bus(){
    return _tx_stream_bus;
}

std::string IqtaxiUdpImpl::get_device_name(){
    return _profile.product;
}

const DeviceProfile& IqtaxiUdpImpl::get_profile() const
{
    return _profile;
}

rx_streamer::sptr IqtaxiUdpImpl::get_rx_stream() {
    std::lock_guard<std::mutex> lock(_transport_setup_mutex);
    if (!_rx_stream) {
        _rx_stream = std::make_shared<recv_packet_streamer>(_local_bus, _rx_stream_bus);
    }
    return _rx_stream;
}

tx_streamer::sptr IqtaxiUdpImpl::get_tx_stream(){
    std::lock_guard<std::mutex> lock(_transport_setup_mutex);
    if (!_tx_stream) {
        const bool enable_dds_ctrl =
            (_profile.product == "E100" || _profile.product == "E200" ||
             _profile.product == "E206");
        _tx_stream = std::make_shared<send_packet_streamer>(
            _local_bus, _tx_stream_bus, enable_dds_ctrl);
    }
    return _tx_stream;
}

void IqtaxiUdpImpl::setTimestamp(uint64_t time_stamp, uint32_t mode) {
    _local_bus->poke64(CUSTOM_SET_VITA_TIMESTAMP_ADDR, time_stamp);
    _local_bus->poke32(CUSTOM_SET_TIME_MODE_ADDR, mode);
}

uint64_t IqtaxiUdpImpl::getTimeTicks() {
    return  _local_bus->peek64(CUSTOM_RB_GET_VITA_TIME_ADDR);
}

void IqtaxiUdpImpl::set_channel_enable(uint32_t channel_enable) {
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_channel_enable_valid && _channel_enable == channel_enable) {
        return;
    }
    _local_bus->poke32(CUSTOM_SET_CHANNEL_ENABLE_ADDR, channel_enable);
    _channel_enable = channel_enable;
    _channel_enable_valid = true;
}

uint32_t IqtaxiUdpImpl::getSampleRate(){
    return _local_bus->peek32(CUSTOM_RB_GET_SAMPLE_CLOCK_RATE_ADDR, kSampleRateCommandTimeoutSec);
}

void IqtaxiUdpImpl::setSampleRate(double rate){
    // _local_bus->poke32(CUSTOM_SET_SAMPLE_CLOCK_RATE_ADDR,uint32_t(rate));
    const uint32_t target_rate = uint32_t(rate);
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_sample_rate_valid && _sample_rate_hz == target_rate) {
        return;
    }
    try {
        _local_bus->poke32(CUSTOM_SET_SAMPLE_RATE_DY, target_rate, kSampleRateCommandTimeoutSec);
        _sample_rate_hz = target_rate;
        _sample_rate_valid = true;
    } catch (const std::runtime_error& ex) {
        std::cerr << "set sample rate command timeout, checking readback: "
                  << ex.what() << std::endl;
        const uint32_t actual_rate =
            _local_bus->peek32(CUSTOM_RB_GET_SAMPLE_CLOCK_RATE_ADDR, kSampleRateCommandTimeoutSec);
        if (actual_rate == target_rate) {
            std::cerr << "set sample rate completed despite missing ACK" << std::endl;
            _sample_rate_hz = target_rate;
            _sample_rate_valid = true;
            return;
        }
        throw;
    }
}

void IqtaxiUdpImpl::set_rx_freq(uint64_t rx_lo,size_t channel){
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_rx_freq_valid && _rx_freq == rx_lo) {
        return;
    }
    with_rx_epoch_restart(_rx_stream, [&]() {
        const uint64_t response = _local_bus->poke64_ack_value(
            CUSTOM_SET_RX_CH1_LO_FREQ_ADDR, rx_lo, kLoCommandTimeoutSec);
        check_lo_command_status(response, "RX", rx_lo);
        _rx_freq = rx_lo;
        _rx_freq_valid = true;
    });
}
void IqtaxiUdpImpl::set_tx_freq(uint64_t tx_lo,size_t channel){
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_tx_freq_valid && _tx_freq == tx_lo) {
        return;
    }
    const uint64_t response = _local_bus->poke64_ack_value(
        CUSTOM_SET_TX_CH1_LO_FREQ_ADDR, tx_lo, kLoCommandTimeoutSec);
    check_lo_command_status(response, "TX", tx_lo);
    _tx_freq = tx_lo;
    _tx_freq_valid = true;
}

uint64_t IqtaxiUdpImpl::get_rx_freq(size_t channel){
    if (_profile.product == "E206") {
        return _local_bus->peek64(CUSTOM_RB_GET_RX_CH1_LO_FREQ_ADDR);
    }
    if (_profile.product == "E100") {
        return _local_bus->peek64(CUSTOM_RB_GET_E100_RX_CH1_LO_FREQ_ADDR);
    }

    const uint32_t low = _local_bus->peek32(CUSTOM_RB_GET_RX_CH1_LO_FREQ_LOW_ADDR);
    const uint32_t high = _local_bus->peek32(CUSTOM_RB_GET_RX_CH1_LO_FREQ_HIGH_ADDR);
    return (static_cast<uint64_t>(high) << 32) | low;
}

uint64_t IqtaxiUdpImpl::get_tx_freq(size_t channel){
    if (_profile.product == "E206") {
        return _local_bus->peek64(CUSTOM_RB_GET_TX_CH1_LO_FREQ_ADDR);
    }
    if (_profile.product == "E100") {
        return _local_bus->peek64(CUSTOM_RB_GET_E100_TX_CH1_LO_FREQ_ADDR);
    }

    const uint32_t low = _local_bus->peek32(CUSTOM_RB_GET_TX_CH1_LO_FREQ_LOW_ADDR);
    const uint32_t high = _local_bus->peek32(CUSTOM_RB_GET_TX_CH1_LO_FREQ_HIGH_ADDR);
    return (static_cast<uint64_t>(high) << 32) | low;
}

uint32_t IqtaxiUdpImpl::get_rx_gain(size_t channel){
    uint32_t gain_value = 0;
    gain_value = _local_bus->peek32(CUSTOM_RB_GET_RX_CH1_GAIN_ADDR);

    return gain_value;
}

uint32_t IqtaxiUdpImpl::get_tx_atten(size_t channel){
    uint32_t atten_value = 0;
    atten_value = _local_bus->peek32(CUSTOM_RB_GET_TX_CH1_ATTEN_ADDR);
    return atten_value;
}

void IqtaxiUdpImpl::set_rx_gain(uint32_t rx_gain, size_t channel){
    uint32_t gain = (uint32_t)(rx_gain);
    const uint32_t max_gain = static_cast<uint32_t>(get_profile().rx_gain_db.maximum);
    if (gain > max_gain) {
        gain = max_gain;
    }
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_rx_gain_valid && _rx_gain == gain) {
        return;
    }
    auto apply_gain = [&]() {
        _local_bus->poke32(CUSTOM_SET_RX_CH1_GAIN_ADDR, gain);
        _rx_gain = gain;
        _rx_gain_valid = true;
    };
    if (_profile.product == "E100") {
        with_rx_epoch_restart(_rx_stream, apply_gain);
    } else {
        apply_gain();
    }
}

void IqtaxiUdpImpl::set_tx_atten(uint32_t tx_atten, size_t channel){
    const uint32_t min_atten = static_cast<uint32_t>(get_profile().tx_attenuation_db.minimum);
    const uint32_t max_atten = static_cast<uint32_t>(get_profile().tx_attenuation_db.maximum);
    tx_atten = std::clamp(tx_atten, min_atten, max_atten);
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_tx_atten_valid && _tx_atten == tx_atten) {
        return;
    }
    _local_bus->poke32(CUSTOM_SET_TX_CH1_ATTEN_ADDR, tx_atten);
    _tx_atten = tx_atten;
    _tx_atten_valid = true;
}

void IqtaxiUdpImpl::set_dma_mode(uint32_t mode){
    std::lock_guard<std::mutex> lock(_settings_mutex);
    if (_dma_mode_valid && _dma_mode == mode) {
        return;
    }
    _local_bus->poke32(CUSTOM_SET_DMA_MODE,mode);
    _dma_mode = mode;
    _dma_mode_valid = true;
}
