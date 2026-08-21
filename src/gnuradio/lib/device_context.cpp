/* -*- c++ -*- */
#include "device_context.h"

#include "src/driver/E100/e100_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"
#include "src/driver/M300/m300_tx_streamer.hpp"
#include <map>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
std::mutex g_context_mutex;
std::map<std::string, std::weak_ptr<gr::iqtaxi::device_context>> g_contexts;

bool is_m300_name(const std::string& device_name)
{
    return device_name == "M300" || device_name == "M300_XDMA" ||
           device_name == "M300_PCIE" || device_name == "FNIC_XDMA";
}

std::string canonical_device_name(const std::string& device_name)
{
    return is_m300_name(device_name) ? "M300_XDMA" : device_name;
}

std::string canonical_device_addr(const std::string& device_name,
                                  const std::string& addr)
{
    return is_m300_name(device_name) && addr.empty() ? "/dev/xdma0" : addr;
}

std::string make_key(const std::string& device_name, const std::string& addr)
{
    return device_name + "@" + addr;
}

uint32_t to_u32(double value)
{
    return static_cast<uint32_t>(std::llround(std::max(0.0, value)));
}
} // namespace

namespace gr {
namespace iqtaxi {

device_context::device_context(std::string name, std::string device_addr)
    : device_name(std::move(name))
    , addr(std::move(device_addr))
{
}

bool device_context::is_m300() const
{
    return is_m300_name(device_name);
}

void device_context::ensure_open()
{
    if (device) {
        return;
    }
    device = sdr::api::Device::makeDevice(device_name, addr);
    if (!device) {
        throw std::runtime_error("failed to open IQTAXI device " + device_name + " at " + addr);
    }
}

namespace {
void validate_channel_claim(bool m300,
                            std::size_t requested,
                            std::size_t other,
                            const char* direction)
{
    if (requested < 1u || requested > 2u || (!m300 && requested != 1u)) {
        throw std::runtime_error(std::string(direction) +
            " supports two channels only on M300; select 1 or 2 channels");
    }
    if (other != 0u && other != requested) {
        throw std::runtime_error(
            "M300 RX and TX channel counts must match because the FPGA shares channel_enable");
    }
}
} // namespace

void device_context::claim_rx_channels_locked(std::size_t channels)
{
    validate_channel_claim(is_m300(), channels, tx_channels, "RX");
    rx_channels = channels;
}

void device_context::claim_tx_channels_locked(std::size_t channels)
{
    validate_channel_claim(is_m300(), channels, rx_channels, "TX");
    tx_channels = channels;
}

void device_context::release_rx_channels_locked()
{
    rx_channels = 0u;
}

void device_context::release_tx_channels_locked()
{
    tx_channels = 0u;
}

void device_context::configure_common_locked(double requested_sample_rate)
{
    ensure_open();
    if (common_configured &&
        std::llround(sample_rate) == std::llround(requested_sample_rate)) {
        return;
    }

    // M300 streaming is armed by m300_rx_streamer after all RF settings have
    // been applied. Starting RX here would run the FPGA before GNU Radio's
    // start() lifecycle reaches the stream.
    if (!is_m300()) {
        device->set_channel_enable(1u);
    }
    device->set_dma_mode(0u);
    device->setSampleRate(requested_sample_rate);
    sample_rate = requested_sample_rate;
    common_configured = true;

    if (rx_stream) {
        rx_stream->set_sampleRate(static_cast<std::size_t>(std::llround(sample_rate)));
    }
}

void device_context::configure_rx_locked(double requested_sample_rate,
                                         double center_freq,
                                         double gain0,
                                         double gain1,
                                         std::size_t channels)
{
    configure_common_locked(requested_sample_rate);
    device->set_rx_freq(static_cast<uint64_t>(std::llround(center_freq)), 1);
    device->set_rx_gain(to_u32(gain0), 1);
    if (channels == 2u) {
        device->set_rx_gain(to_u32(gain1), 2);
    }
    if (rx_stream) {
        rx_stream->set_sampleRate(static_cast<std::size_t>(std::llround(sample_rate)));
    }
}

void device_context::configure_tx_locked(double requested_sample_rate,
                                         double center_freq,
                                         double attenuation0,
                                         double attenuation1,
                                         std::size_t samples_per_packet,
                                         bool timed,
                                         std::size_t channels)
{
    configure_common_locked(requested_sample_rate);
    device->set_tx_freq(static_cast<uint64_t>(std::llround(center_freq)), 1);
    reapply_tx_attenuation_locked(attenuation0, attenuation1, channels);

    if (auto e100_dev = std::dynamic_pointer_cast<E100Impl>(device)) {
        auto local_bus = e100_dev->get_local_bus();
        local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET,
                          static_cast<uint32_t>(samples_per_packet));
        local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, timed ? 0u : 1u);
    }
    if (auto m300_tx = std::dynamic_pointer_cast<m300_tx_streamer>(tx_stream)) {
        m300_tx->configure(samples_per_packet, timed,
                           channels == 2u ? 0x03u : 0x01u);
    }
}

void device_context::reapply_tx_attenuation_locked(double attenuation0,
                                                   double attenuation1,
                                                   std::size_t channels)
{
    ensure_open();
    device->set_tx_atten(to_u32(attenuation0), 1);
    if (channels == 2u) {
        device->set_tx_atten(to_u32(attenuation1), 2);
    }
}

device_context::sptr acquire_device_context(const std::string& device_name,
                                            const std::string& addr)
{
    const std::string normalized_name = canonical_device_name(device_name);
    const std::string normalized_addr = canonical_device_addr(device_name, addr);
    const std::string key = make_key(normalized_name, normalized_addr);
    device_context::sptr ctx;

    {
        std::lock_guard<std::mutex> lock(g_context_mutex);
        auto found = g_contexts.find(key);
        if (found != g_contexts.end()) {
            ctx = found->second.lock();
        }
        if (!ctx) {
            ctx = std::make_shared<device_context>(normalized_name, normalized_addr);
            g_contexts[key] = ctx;
        }
    }

    std::lock_guard<std::mutex> control_lock(ctx->control_mutex);
    ctx->ensure_open();
    return ctx;
}

} // namespace iqtaxi
} // namespace gr
