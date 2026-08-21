/* -*- c++ -*- */
#ifndef INCLUDED_GR_IQTAXI_DEVICE_CONTEXT_H
#define INCLUDED_GR_IQTAXI_DEVICE_CONTEXT_H

#include "include/sdr/api/Device.hpp"
#include "include/sdr/api/DataStream.hpp"
#include <memory>
#include <mutex>
#include <string>

namespace gr {
namespace iqtaxi {

struct device_context {
    using sptr = std::shared_ptr<device_context>;

    device_context(std::string device_name, std::string device_addr);
    bool is_m300() const;
    void ensure_open();
    void claim_rx_channels_locked(std::size_t channels);
    void claim_tx_channels_locked(std::size_t channels);
    void release_rx_channels_locked();
    void release_tx_channels_locked();
    void configure_common_locked(double sample_rate);
    void configure_rx_locked(double sample_rate,
                             double center_freq,
                             double gain0,
                             double gain1,
                             std::size_t channels);
    void configure_tx_locked(double sample_rate,
                             double center_freq,
                             double attenuation0,
                             double attenuation1,
                             std::size_t samples_per_packet,
                             bool timed,
                             std::size_t channels);
    void reapply_tx_attenuation_locked(double attenuation0,
                                       double attenuation1,
                                       std::size_t channels);

    const std::string device_name;
    const std::string addr;
    sdr::api::Device::sptr device;
    sdr::api::rx_streamer::sptr rx_stream;
    sdr::api::tx_streamer::sptr tx_stream;
    std::mutex control_mutex;
    bool common_configured = false;
    double sample_rate = 0.0;
    std::size_t rx_channels = 0u;
    std::size_t tx_channels = 0u;
};

device_context::sptr acquire_device_context(const std::string& device_name,
                                            const std::string& addr);

} // namespace iqtaxi
} // namespace gr

#endif /* INCLUDED_GR_IQTAXI_DEVICE_CONTEXT_H */
