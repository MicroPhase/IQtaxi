/* -*- c++ -*- */
#include "rx_source_impl.h"

#include "src/driver/transport/local_regs.hpp"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
constexpr uint8_t kStreamMode = 1u;
constexpr std::size_t kE100MaxPacketSamples = (1472u - 16u) / 4u;
constexpr std::size_t kDefaultOutputMultiple = 256u;
constexpr auto kSettingsFailureRetryDelay = std::chrono::milliseconds(250);

bool is_m300_device(const std::string& device)
{
    return device == "M300" || device == "M300_XDMA" ||
           device == "M300_PCIE" || device == "FNIC_XDMA";
}

std::size_t validated_channels(const std::string& device, std::size_t channels)
{
    if (channels < 1u || channels > 2u ||
        (channels == 2u && !is_m300_device(device))) {
        throw std::invalid_argument(
            "IQTAXI RX channels must be 1, or 2 when using M300");
    }
    return channels;
}

} // namespace

namespace gr {
namespace iqtaxi {

rx_source::sptr rx_source::make(const std::string& device,
                                const std::string& addr,
                                double sample_rate,
                                double center_freq,
                                double gain,
                                std::size_t samples_per_work,
                                std::size_t channels,
                                double gain_ch1)
{
    return gnuradio::get_initial_sptr(
        new rx_source_impl(device, addr, sample_rate, center_freq, gain,
                           samples_per_work, channels, gain_ch1));
}

rx_source_impl::rx_source_impl(const std::string& device,
                               const std::string& addr,
                               double sample_rate,
                               double center_freq,
                               double gain,
                               std::size_t samples_per_work,
                               std::size_t channels,
                               double gain_ch1)
    : gr::sync_block("iqtaxi_rx_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(
                         static_cast<int>(validated_channels(device, channels)),
                         static_cast<int>(validated_channels(device, channels)),
                         sizeof(gr_complex)))
    , d_device_name(device)
    , d_addr(addr)
    , d_samples_per_work(std::max<std::size_t>(samples_per_work, 1u))
    , d_channels(validated_channels(device, channels))
    , d_sample_rate(sample_rate)
    , d_center_freq(center_freq)
    , d_gain(gain)
    , d_gain_ch1(gain_ch1)
{
    set_output_multiple(static_cast<int>(std::min<std::size_t>(
        std::max<std::size_t>(d_samples_per_work, kDefaultOutputMultiple),
        32768u)));
    d_context = acquire_device_context(d_device_name, d_addr);
}

rx_source_impl::~rx_source_impl()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    try {
        stop_stream_locked();
    } catch (...) {
    }
    if (d_context && d_channel_claimed) {
        try {
            std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
            d_context->release_rx_channels_locked();
            d_channel_claimed = false;
        } catch (...) {
        }
    }
}

void rx_source_impl::ensure_device()
{
    if (d_context && d_rx_stream && d_channel_claimed) {
        return;
    }

    d_context = acquire_device_context(d_device_name, d_addr);
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    d_context->ensure_open();
    if (!d_context->rx_stream) {
        d_context->rx_stream = d_context->device->get_rx_stream();
    }
    d_rx_stream = d_context->rx_stream;
    if (!d_rx_stream) {
        throw std::runtime_error("failed to create IQTAXI RX stream");
    }
    d_context->claim_rx_channels_locked(d_channels);
    d_channel_claimed = true;
}

void rx_source_impl::apply_settings_locked()
{
    ensure_device();
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    d_context->configure_rx_locked(
        d_sample_rate, d_center_freq, d_gain, d_gain_ch1, d_channels);
    d_settings_dirty = false;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
}

void rx_source_impl::apply_pending_settings()
{
    double sample_rate = 0.0;
    double center_freq = 0.0;
    double gain = 0.0;
    double gain_ch1 = 0.0;

    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (!d_streaming || !d_context || !d_settings_dirty) {
            return;
        }
        if (d_next_settings_retry.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() < d_next_settings_retry) {
            return;
        }

        sample_rate = d_sample_rate;
        center_freq = d_center_freq;
        gain = d_gain;
        gain_ch1 = d_gain_ch1;
    }

    try {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_rx_locked(
            sample_rate, center_freq, gain, gain_ch1, d_channels);
        std::lock_guard<std::mutex> lock(d_mutex);
        if (std::llround(d_sample_rate) == std::llround(sample_rate) &&
            std::llround(d_center_freq) == std::llround(center_freq) &&
            std::llround(d_gain) == std::llround(gain) &&
            std::llround(d_gain_ch1) == std::llround(gain_ch1)) {
            d_settings_dirty = false;
            d_next_settings_retry = std::chrono::steady_clock::time_point{};
        }
    } catch (const std::exception& ex) {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_next_settings_retry = std::chrono::steady_clock::now() + kSettingsFailureRetryDelay;
        std::cerr << "iqtaxi_rx_source: settings update failed, retrying: "
                  << ex.what() << std::endl;
    }
}

bool rx_source_impl::start()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    ensure_device();
    apply_settings_locked();

    d_timestamp = 0;
    {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_rx_stream->set_rx_enable_chan(d_channels == 2u ? 0x03u : 0x01u);
        d_rx_stream->set_rx_mode(kStreamMode);
        if (!d_context->is_m300()) {
            d_rx_stream->set_max_sample_nums_per_packet(kE100MaxPacketSamples);
        }
        d_rx_stream->set_recv_param(kStreamMode, d_samples_per_work, d_timestamp, 1, 0);
    }
    d_streaming = true;
    return true;
}

void rx_source_impl::stop_stream_locked()
{
    if (!d_rx_stream || !d_streaming) {
        return;
    }

    uint64_t stop_timestamp = 0;
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    d_rx_stream->set_recv_param(kStreamMode, d_samples_per_work, stop_timestamp, 0, 1);
    d_rx_stream->set_rx_mode_exit();
    d_streaming = false;
}

bool rx_source_impl::stop()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    try {
        stop_stream_locked();
    } catch (...) {
        d_streaming = false;
    }
    return true;
}

int rx_source_impl::work(int noutput_items,
                         gr_vector_const_void_star&,
                         gr_vector_void_star& output_items)
{
    if (noutput_items <= 0) {
        return 0;
    }

    apply_pending_settings();

    std::vector<void*> buffs;
    buffs.reserve(d_channels);
    for (std::size_t channel = 0u; channel < d_channels; ++channel) {
        buffs.push_back(static_cast<gr_complex*>(output_items[channel]));
    }

    size_t received = 0;
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (!d_streaming || !d_rx_stream) {
            return 0;
        }
        received = d_rx_stream->recv(
            buffs, static_cast<std::size_t>(noutput_items), d_timestamp, MICRORF_FORMAT_FLOAT32);
    }

    if (received == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return static_cast<int>(received);
}

void rx_source_impl::set_sample_rate(double sample_rate)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_sample_rate = sample_rate;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_common_locked(d_sample_rate);
        d_settings_dirty = false;
    }
}

void rx_source_impl::set_center_freq(double center_freq)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_center_freq = center_freq;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_rx_locked(
            d_sample_rate, d_center_freq, d_gain, d_gain_ch1, d_channels);
        d_settings_dirty = false;
    }
}

void rx_source_impl::set_gain(double gain)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_gain = gain;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_rx_locked(
            d_sample_rate, d_center_freq, d_gain, d_gain_ch1, d_channels);
        d_settings_dirty = false;
    }
}

void rx_source_impl::set_gain_ch1(double gain)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_gain_ch1 = gain;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_rx_locked(
            d_sample_rate, d_center_freq, d_gain, d_gain_ch1, d_channels);
        d_settings_dirty = false;
    }
}

} // namespace iqtaxi
} // namespace gr
