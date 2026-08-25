#include "m300_xdma_impl.hpp"

#include <algorithm>
#include <map>
#include <sstream>
#include <stdexcept>

using namespace sdr::core;

namespace {
constexpr double kCtrlTimeoutSec = 1.0;
constexpr size_t kDefaultRecvFrames = 16u;
constexpr size_t kDefaultSendFrames = 16u;
constexpr size_t kDefaultPacketBytes = 16384u;
constexpr size_t kDefaultRxFrameSize = 16384u;
constexpr size_t kTxPacketsPerWrite = 16u;
constexpr size_t kDefaultTxFrameSize = kDefaultPacketBytes * kTxPacketsPerWrite;
constexpr uint32_t kDefaultRxRingDepth = 256u;

size_t align_up_size(size_t value, size_t align)
{
    return ((value + align - 1) / align) * align;
}

zero_copy_xport_params make_xport_params(size_t recv_frames,
                                         size_t send_frames,
                                         size_t recv_size,
                                         size_t send_size)
{
    zero_copy_xport_params params;
    params.num_recv_frames = recv_frames;
    params.num_send_frames = send_frames;
    params.recv_frame_size = recv_size;
    params.send_frame_size = send_size;
    params.recv_buff_size = recv_frames * recv_size;
    params.send_buff_size = send_frames * send_size;
    return params;
}
}

void M300XdmaImpl::parse_device_path(const std::string& device_path)
{
    const std::string base = device_path.empty() ? "/dev/xdma0" : device_path;
    if (base.find('=') == std::string::npos) {
        _ctrl_h2c_path = base + "_h2c_0";
        _resp_c2h_path = base + "_c2h_0";
        _rx_c2h_path = base + "_c2h_1";
        _tx_h2c_path = base + "_h2c_1";
        return;
    }

    std::map<std::string, std::string> kv;
    size_t begin = 0;
    while (begin < base.size()) {
        const size_t end = base.find(',', begin);
        const std::string item = base.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        const size_t eq = item.find('=');
        if (eq != std::string::npos)
            kv[item.substr(0, eq)] = item.substr(eq + 1);
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }

    _ctrl_h2c_path = kv.count("ctrl") ? kv["ctrl"] : "/dev/xdma0_h2c_0";
    _resp_c2h_path = kv.count("resp") ? kv["resp"] : "/dev/xdma0_c2h_0";
    _rx_c2h_path = kv.count("rx") ? kv["rx"] : "/dev/xdma0_c2h_1";
    _tx_h2c_path = kv.count("tx") ? kv["tx"] : "/dev/xdma0_h2c_1";
}

M300XdmaImpl::M300XdmaImpl(const std::string& device_path)
    : M300XdmaImpl(device_path, true)
{
}

M300XdmaImpl::M300XdmaImpl(const std::string& device_path, bool open_data_channels)
    : _device_path(device_path)
{
    parse_device_path(device_path);

    // Lock before opening any XDMA channel or issuing a control request.
    acquire_exclusive_access("M300_XDMA", _device_path.empty() ? "/dev/xdma0" : _device_path);

    const auto ctrl_params = make_xport_params(kDefaultRecvFrames, kDefaultSendFrames,
                                               32u, 32u);
    _ctrl_xport = xdma_zero_copy::make(
        xdma_zero_copy_params{_ctrl_h2c_path, std::string(), _ctrl_h2c_path,
                              32u, 0u, false, false},
        ctrl_params);
    _resp_xport = xdma_zero_copy::make(
        xdma_zero_copy_params{_resp_c2h_path, _resp_c2h_path, std::string(),
                              32u, 0u, false, false},
        ctrl_params);

    if (open_data_channels) {
        const auto rx_params = make_xport_params(kDefaultRecvFrames, kDefaultSendFrames,
                                                 kDefaultRxFrameSize, kDefaultRxFrameSize);
        const auto tx_params = make_xport_params(kDefaultRecvFrames, kDefaultSendFrames,
                                                 kDefaultTxFrameSize, kDefaultTxFrameSize);

        _rx_xport = xdma_zero_copy::make(
            xdma_zero_copy_params{_rx_c2h_path, _rx_c2h_path, std::string(),
                                  kDefaultPacketBytes,
                                  align_up_size(kDefaultPacketBytes, 4096u),
                                  false, false, true, kDefaultRxRingDepth},
            rx_params);
        _tx_xport = xdma_zero_copy::make(
            xdma_zero_copy_params{_tx_h2c_path, std::string(), _tx_h2c_path,
                                  kDefaultPacketBytes, kDefaultTxFrameSize, false, false},
            tx_params);
    }

    _ctrl = std::make_shared<sdr::driver::m300_xdma_ctrl>(_ctrl_xport, _resp_xport);
    _rf_ctrl = std::make_shared<sdr::driver::m300_rf_ctrl>(_ctrl);

    try {
        const auto version = _ctrl->get_version(kCtrlTimeoutSec);
        (void)version;
        _initial_success = true;
    } catch (const std::exception& ex) {
        _initial_success = false;
        _last_error = ex.what();
    }
}

std::string M300XdmaImpl::get_device_name()
{
    return "M300_XDMA";
}

const DeviceProfile& M300XdmaImpl::get_profile() const
{
    return m300_pcie_profile();
}

rx_streamer::sptr M300XdmaImpl::get_rx_stream()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_rx_stream) {
        _rx_stream = std::make_shared<m300_rx_streamer>(_ctrl, _rx_xport,
                                                        static_cast<uint32_t>(kDefaultPacketBytes));
    }
    return _rx_stream;
}

tx_streamer::sptr M300XdmaImpl::get_tx_stream()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_tx_stream) {
        std::weak_ptr<sdr::driver::m300_rf_ctrl> weak_rf_ctrl(_rf_ctrl);
        _tx_stream = std::make_shared<m300_tx_streamer>(
            _ctrl, _tx_xport, kDefaultPacketBytes,
            [weak_rf_ctrl]() {
                if (auto rf_ctrl = weak_rf_ctrl.lock()) {
                    rf_ctrl->reapply_tx_attenuation();
                }
            });
    }
    return _tx_stream;
}

uint64_t M300XdmaImpl::getTimeTicks()
{
    std::lock_guard<std::mutex> lock(_mutex);
    try {
        return _ctrl->get_timestamp(kCtrlTimeoutSec);
    } catch (...) {
        return 0;
    }
}

void M300XdmaImpl::setTimestamp(uint64_t time_stamp, uint32_t mode)
{
    std::lock_guard<std::mutex> lock(_mutex);
    (void)mode;
    (void)_ctrl->set_timestamp(time_stamp, kCtrlTimeoutSec);
}

void M300XdmaImpl::set_channel_enable(uint32_t channel_enable)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (channel_enable) {
        (void)_ctrl->start_rx(kCtrlTimeoutSec);
    } else {
        (void)_ctrl->stop_rx(kCtrlTimeoutSec);
    }
}

uint32_t M300XdmaImpl::getSampleRate()
{
    return _rf_ctrl ? _rf_ctrl->get_sample_rate() : 0u;
}

void M300XdmaImpl::setSampleRate(double rate)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_sample_rate(static_cast<uint32_t>(rate));
    }
}

void M300XdmaImpl::set_bandwidth(uint32_t bandwidth_hz)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_bandwidth(bandwidth_hz);
    }
}

uint32_t M300XdmaImpl::get_bandwidth() const
{
    return _rf_ctrl ? _rf_ctrl->get_bandwidth() : 0u;
}

void M300XdmaImpl::set_rx_freq(uint64_t rx_lo,size_t channel)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_rx_freq(rx_lo, channel);
    }
}

void M300XdmaImpl::set_tx_freq(uint64_t tx_lo,size_t channel)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_tx_freq(tx_lo, channel);
    }
}

uint64_t M300XdmaImpl::get_rx_freq(size_t channel)
{
    return _rf_ctrl ? _rf_ctrl->get_rx_freq(channel) : 0u;
}

uint64_t M300XdmaImpl::get_tx_freq(size_t channel)
{
    return _rf_ctrl ? _rf_ctrl->get_tx_freq(channel) : 0u;
}

uint32_t M300XdmaImpl::get_rx_gain(size_t channel)
{
    return _rf_ctrl ? _rf_ctrl->get_rx_gain(channel) : 0u;
}

uint32_t M300XdmaImpl::get_tx_atten(size_t channel)
{
    return _rf_ctrl ? _rf_ctrl->get_tx_atten(channel) : 0u;
}

void M300XdmaImpl::set_rx_gain(uint32_t rx_gain, size_t channel)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_rx_gain(rx_gain, channel);
    }
}

void M300XdmaImpl::set_tx_atten(uint32_t tx_atten, size_t channel)
{
    if (_rf_ctrl) {
        _rf_ctrl->set_tx_atten(tx_atten, channel);
    }
}

void M300XdmaImpl::set_dma_mode(uint32_t mode)
{
    _dma_mode = mode;
}

bool M300XdmaImpl::isInitialSuccess() const
{
    return _initial_success;
}

std::string M300XdmaImpl::last_error() const
{
    return _last_error;
}

void M300XdmaImpl::configure_rx_packet_bytes(uint32_t packet_bytes)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_rx_xport) {
        return;
    }
    const size_t packet_stride = align_up_size(packet_bytes, 4096u);
    if (_rx_xport->get_packet_bytes() == packet_bytes &&
        _rx_xport->get_packet_stride() == packet_stride) {
        return;
    }
    const auto rx_params = make_xport_params(kDefaultRecvFrames, kDefaultSendFrames,
                                             packet_stride, packet_stride);

    _rx_xport.reset();
    _rx_xport = xdma_zero_copy::make(
        xdma_zero_copy_params{_rx_c2h_path, _rx_c2h_path, std::string(),
                              packet_bytes, packet_stride, false, false,
                              true, kDefaultRxRingDepth},
        rx_params);
    _rx_stream.reset();
}

std::shared_ptr<sdr::driver::m300_xdma_ctrl> M300XdmaImpl::get_ctrl()
{
    return _ctrl;
}

xdma_zero_copy::sptr M300XdmaImpl::get_ctrl_xport()
{
    return _ctrl_xport;
}

xdma_zero_copy::sptr M300XdmaImpl::get_resp_xport()
{
    return _resp_xport;
}

xdma_zero_copy::sptr M300XdmaImpl::get_rx_xport()
{
    return _rx_xport;
}

xdma_zero_copy::sptr M300XdmaImpl::get_tx_xport()
{
    return _tx_xport;
}
