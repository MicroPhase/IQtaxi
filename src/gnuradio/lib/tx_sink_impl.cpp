/* -*- c++ -*- */
#include "tx_sink_impl.h"

#include "src/driver/transport/super_send_packet_handler.hpp"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
constexpr auto kSettingsFailureRetryDelay = std::chrono::milliseconds(250);
constexpr std::size_t kM300PacketsPerWork = 16u;
constexpr std::size_t kMaxWorkSamples = 65536u;

bool is_m300_device(const std::string& device)
{
    return device == "M300" || device == "M300_XDMA" ||
           device == "M300_PCIE" || device == "FNIC" || device == "FNIC_XDMA";
}

std::size_t validated_channels(const std::string& device, std::size_t channels)
{
    if (channels < 1u || channels > 2u ||
        (channels == 2u && !is_m300_device(device))) {
        throw std::invalid_argument(
            "IQTAXI TX channels must be 1, or 2 when using M300");
    }
    return channels;
}

std::size_t work_samples(const std::string& device, std::size_t packet_samples)
{
    if (!is_m300_device(device)) {
        return packet_samples;
    }
    return std::min(kMaxWorkSamples, packet_samples * kM300PacketsPerWork);
}
} // namespace

namespace gr {
namespace iqtaxi {

tx_sink::sptr tx_sink::make(const std::string& device,
                            const std::string& addr,
                            double sample_rate,
                            double center_freq,
                            double attenuation,
                            std::size_t samples_per_packet,
                            bool timed,
                            double start_delay_ms,
                            std::size_t channels,
                            double attenuation_ch1)
{
    return gnuradio::get_initial_sptr(new tx_sink_impl(device,
                                                       addr,
                                                       sample_rate,
                                                       center_freq,
                                                       attenuation,
                                                       samples_per_packet,
                                                       timed,
                                                       start_delay_ms,
                                                       channels,
                                                       attenuation_ch1));
}

tx_sink_impl::tx_sink_impl(const std::string& device,
                           const std::string& addr,
                           double sample_rate,
                           double center_freq,
                           double attenuation,
                           std::size_t samples_per_packet,
                           bool timed,
                           double start_delay_ms,
                           std::size_t channels,
                           double attenuation_ch1)
    : gr::sync_block("iqtaxi_tx_sink",
                     gr::io_signature::make(
                         static_cast<int>(validated_channels(device, channels)),
                         static_cast<int>(validated_channels(device, channels)),
                         sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0))
    , d_device_name(device)
    , d_addr(addr)
    , d_samples_per_packet(std::max<std::size_t>(samples_per_packet, 1u))
    , d_samples_per_work(work_samples(d_device_name, d_samples_per_packet))
    , d_timed(timed)
    , d_start_delay_ms(start_delay_ms)
    , d_channels(validated_channels(device, channels))
    , d_sample_rate(sample_rate)
    , d_center_freq(center_freq)
    , d_attenuation(attenuation)
    , d_attenuation_ch1(attenuation_ch1)
{
    set_output_multiple(static_cast<int>(d_samples_per_work));
    d_context = acquire_device_context(d_device_name, d_addr);
}

tx_sink_impl::~tx_sink_impl()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    try {
        stop_stream_locked();
    } catch (...) {
    }
    if (d_context && d_channel_claimed) {
        try {
            std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
            d_context->release_tx_channels_locked();
            d_channel_claimed = false;
        } catch (...) {
        }
    }
}

void tx_sink_impl::ensure_device()
{
    if (d_context && d_tx_stream && d_channel_claimed) {
        return;
    }

    d_context = acquire_device_context(d_device_name, d_addr);
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    d_context->ensure_open();
    if (!d_context->tx_stream) {
        d_context->tx_stream = d_context->device->get_tx_stream();
    }
    d_tx_stream = d_context->tx_stream;
    if (!d_tx_stream) {
        throw std::runtime_error("failed to create IQTAXI TX stream");
    }
    d_context->claim_tx_channels_locked(d_channels);
    d_channel_claimed = true;
}

void tx_sink_impl::apply_settings_locked()
{
    ensure_device();
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    d_context->configure_tx_locked(
        d_sample_rate, d_center_freq, d_attenuation, d_attenuation_ch1,
        d_samples_per_packet, d_timed, d_channels);
    d_settings_dirty = false;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
}

void tx_sink_impl::apply_pending_settings()
{
    double sample_rate = 0.0;
    double center_freq = 0.0;
    double attenuation = 0.0;
    double attenuation_ch1 = 0.0;

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
        attenuation = d_attenuation;
        attenuation_ch1 = d_attenuation_ch1;
    }

    try {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_tx_locked(
            sample_rate, center_freq, attenuation, attenuation_ch1,
            d_samples_per_packet, d_timed, d_channels);
        std::lock_guard<std::mutex> lock(d_mutex);
        if (std::llround(d_sample_rate) == std::llround(sample_rate) &&
            std::llround(d_center_freq) == std::llround(center_freq) &&
            std::llround(d_attenuation) == std::llround(attenuation) &&
            std::llround(d_attenuation_ch1) == std::llround(attenuation_ch1)) {
            d_settings_dirty = false;
            d_next_settings_retry = std::chrono::steady_clock::time_point{};
        }
    } catch (const std::exception& ex) {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_next_settings_retry = std::chrono::steady_clock::now() + kSettingsFailureRetryDelay;
        std::cerr << "iqtaxi_tx_sink: settings update failed, retrying: "
                  << ex.what() << std::endl;
    }
}

bool tx_sink_impl::start()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    ensure_device();
    apply_settings_locked();

    {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_tx_stream->set_tx_source(1u);
        d_timestamp = d_context->device->getTimeTicks();
        if (d_timed && d_start_delay_ms > 0.0) {
            d_timestamp += static_cast<uint64_t>(
                (d_sample_rate * d_start_delay_ms) / 1000.0);
        }
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(d_tx_stream)) {
            packet_stream->begin_tx_flow_control_monitoring();
        }
        d_tx_stream->set_stream_tx_start();

        // The first attenuation write happens while the FPGA TX path is still
        // stopped. Reapply it after arming M300 so cold-start behavior matches
        // a runtime attenuation change from GNU Radio.
        if (d_context->is_m300()) {
            d_context->reapply_tx_attenuation_locked(
                d_attenuation, d_attenuation_ch1, d_channels);
        }
    }
    d_streaming = true;
    return true;
}

void tx_sink_impl::stop_stream_locked()
{
    if (!d_tx_stream || !d_streaming) {
        return;
    }

    d_streaming = false;
    std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
    if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(d_tx_stream)) {
        packet_stream->request_send_abort();
        packet_stream->end_tx_flow_control_monitoring();
    }

    d_tx_stream->set_stream_tx_stop();
}

bool tx_sink_impl::stop()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    try {
        stop_stream_locked();
    } catch (...) {
        d_streaming = false;
    }
    return true;
}

int tx_sink_impl::work(int noutput_items,
                       gr_vector_const_void_star& input_items,
                       gr_vector_void_star&)
{
    if (noutput_items <= 0) {
        return 0;
    }

    apply_pending_settings();

    size_t total_sent = 0;
    sdr::api::tx_streamer::sptr tx_stream;
    bool m300 = false;
    uint64_t timestamp = 0;

    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (!d_streaming || !d_tx_stream) {
            return 0;
        }
        tx_stream = d_tx_stream;
        m300 = d_context->is_m300();
        timestamp = d_timestamp;
    }

    const size_t to_send = std::min<std::size_t>(
        d_samples_per_work, static_cast<std::size_t>(noutput_items));
    std::vector<const void*> buffs;
    buffs.reserve(d_channels);
    for (std::size_t channel = 0u; channel < d_channels; ++channel) {
        buffs.push_back(static_cast<const gr_complex*>(input_items[channel]));
    }

    if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx_stream)) {
        if (!packet_stream->get_tx_flow_control_stats().ready_to_send) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            std::lock_guard<std::mutex> lock(d_mutex);
            if (d_streaming && tx_stream == d_tx_stream) {
                d_timestamp += to_send;
            }
            return static_cast<int>(to_send);
        }
    }

    total_sent = m300
        ? tx_stream->send(buffs, to_send, timestamp, MICRORF_FORMAT_FLOAT32)
        : tx_stream->send_nonblocking(buffs, to_send, timestamp, MICRORF_FORMAT_FLOAT32);

    if (total_sent > 0) {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_streaming && tx_stream == d_tx_stream) {
            d_timestamp = timestamp;
        }
    }

    if (total_sent == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (m300) {
            // Keep the same GNU Radio input items and retry. Consuming them
            // here created a real RF gap whenever the bounded H2C pool was
            // momentarily full.
            return 0;
        }
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_streaming && tx_stream == d_tx_stream) {
            d_timestamp += to_send;
        }
        return static_cast<int>(to_send);
    }
    return static_cast<int>(total_sent);
}

void tx_sink_impl::set_sample_rate(double sample_rate)
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

void tx_sink_impl::set_center_freq(double center_freq)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_center_freq = center_freq;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_tx_locked(
            d_sample_rate, d_center_freq, d_attenuation, d_attenuation_ch1,
            d_samples_per_packet, d_timed, d_channels);
        d_settings_dirty = false;
    }
}

void tx_sink_impl::set_attenuation(double attenuation)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_attenuation = attenuation;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_tx_locked(
            d_sample_rate, d_center_freq, d_attenuation, d_attenuation_ch1,
            d_samples_per_packet, d_timed, d_channels);
        d_settings_dirty = false;
    }
}

void tx_sink_impl::set_attenuation_ch1(double attenuation)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_attenuation_ch1 = attenuation;
    d_settings_dirty = true;
    d_next_settings_retry = std::chrono::steady_clock::time_point{};
    if (d_context && !d_streaming) {
        std::lock_guard<std::mutex> control_lock(d_context->control_mutex);
        d_context->configure_tx_locked(
            d_sample_rate, d_center_freq, d_attenuation, d_attenuation_ch1,
            d_samples_per_packet, d_timed, d_channels);
        d_settings_dirty = false;
    }
}

} // namespace iqtaxi
} // namespace gr
