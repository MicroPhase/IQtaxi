/* -*- c++ -*- */
#ifndef INCLUDED_GR_IQTAXI_TX_SINK_IMPL_H
#define INCLUDED_GR_IQTAXI_TX_SINK_IMPL_H

#include <gnuradio/iqtaxi/tx_sink.h>
#include "device_context.h"
#include <chrono>
#include <mutex>

namespace gr {
namespace iqtaxi {

class tx_sink_impl : public tx_sink
{
public:
    tx_sink_impl(const std::string& device,
                 const std::string& addr,
                 double sample_rate,
                 double center_freq,
                 double attenuation,
                 std::size_t samples_per_packet,
                 bool timed,
                 double start_delay_ms,
                 std::size_t channels,
                 double attenuation_ch1);
    ~tx_sink_impl() override;

    bool start() override;
    bool stop() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    void set_sample_rate(double sample_rate) override;
    void set_center_freq(double center_freq) override;
    void set_attenuation(double attenuation) override;
    void set_attenuation_ch1(double attenuation) override;

private:
    void ensure_device();
    void apply_settings_locked();
    void apply_pending_settings();
    void stop_stream_locked();

    const std::string d_device_name;
    const std::string d_addr;
    const std::size_t d_samples_per_packet;
    const std::size_t d_samples_per_work;
    const bool d_timed;
    const double d_start_delay_ms;
    const std::size_t d_channels;

    std::mutex d_mutex;
    device_context::sptr d_context;
    sdr::api::tx_streamer::sptr d_tx_stream;
    double d_sample_rate;
    double d_center_freq;
    double d_attenuation;
    double d_attenuation_ch1;
    bool d_channel_claimed = false;
    bool d_settings_dirty = true;
    std::chrono::steady_clock::time_point d_next_settings_retry{};
    uint64_t d_timestamp = 0;
    bool d_streaming = false;
};

} // namespace iqtaxi
} // namespace gr

#endif /* INCLUDED_GR_IQTAXI_TX_SINK_IMPL_H */
