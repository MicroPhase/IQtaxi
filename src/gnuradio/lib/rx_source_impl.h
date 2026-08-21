/* -*- c++ -*- */
#ifndef INCLUDED_GR_IQTAXI_RX_SOURCE_IMPL_H
#define INCLUDED_GR_IQTAXI_RX_SOURCE_IMPL_H

#include <gnuradio/iqtaxi/rx_source.h>
#include "device_context.h"
#include <atomic>
#include <chrono>
#include <mutex>

namespace gr {
namespace iqtaxi {

class rx_source_impl : public rx_source
{
public:
    rx_source_impl(const std::string& device,
                   const std::string& addr,
                   double sample_rate,
                   double center_freq,
                   double gain,
                   std::size_t samples_per_work,
                   std::size_t channels,
                   double gain_ch1);
    ~rx_source_impl() override;

    bool start() override;
    bool stop() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    void set_sample_rate(double sample_rate) override;
    void set_center_freq(double center_freq) override;
    void set_gain(double gain) override;
    void set_gain_ch1(double gain) override;

private:
    void ensure_device();
    void apply_settings_locked();
    void apply_pending_settings();
    void stop_stream_locked();

    const std::string d_device_name;
    const std::string d_addr;
    std::size_t d_samples_per_work;
    const std::size_t d_channels;

    std::mutex d_mutex;
    device_context::sptr d_context;
    sdr::api::rx_streamer::sptr d_rx_stream;
    double d_sample_rate;
    double d_center_freq;
    double d_gain;
    double d_gain_ch1;
    bool d_channel_claimed = false;
    bool d_settings_dirty = true;
    std::chrono::steady_clock::time_point d_next_settings_retry{};
    uint64_t d_timestamp = 0;
    bool d_streaming = false;
};

} // namespace iqtaxi
} // namespace gr

#endif /* INCLUDED_GR_IQTAXI_RX_SOURCE_IMPL_H */
