#include "m300_rf_ctrl.hpp"

#include "m300_ad9361_ctrl.hpp"

#include <string>

using namespace sdr::driver;

namespace {
constexpr uint16_t kAd9361RegProductId = 0x037u;
constexpr uint32_t kMaxGpioNumber = 31u;

size_t channel_index(size_t channel)
{
    // Keep the driver's established convention: channel 2 selects the
    // second AD9361 chain; channel 0 or 1 selects the first chain.
    return channel == 2u ? 1u : 0u;
}
}

m300_rf_ctrl::m300_rf_ctrl(const m300_xdma_ctrl::sptr& ctrl)
    : _ctrl(ctrl)
    , _ad9361(new m300_ad9361_ctrl(ctrl))
{
    require_ctrl();
    try {
        _gpio_out = _ctrl->read_gpio_out(1.0);
        _gpio_oe = _ctrl->read_gpio_oe(1.0);
    } catch (...) {
        _gpio_out = 0x00000001u;
        _gpio_oe = 0x000001ffu;
    }
}

m300_rf_ctrl::~m300_rf_ctrl() = default;

void m300_rf_ctrl::require_ctrl() const
{
    if (!_ctrl) {
        throw std::runtime_error("M300 RF control requires a valid XDMA control transport");
    }
}

uint32_t m300_rf_ctrl::ad9361_product_id(double timeout_sec)
{
    return ad9361_spi_read(kAd9361RegProductId, timeout_sec);
}

void m300_rf_ctrl::ad9361_spi_write(uint16_t reg, uint8_t value, double timeout_sec)
{
    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    (void)_ctrl->ad9361_spi_write(reg, value, timeout_sec);
}

uint8_t m300_rf_ctrl::ad9361_spi_read(uint16_t reg, double timeout_sec)
{
    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    return _ctrl->ad9361_spi_read(reg, timeout_sec);
}

void m300_rf_ctrl::axi_write(uint32_t addr, uint32_t value, double timeout_sec)
{
    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    (void)_ctrl->write_axi(addr, value, timeout_sec);
}

uint32_t m300_rf_ctrl::axi_read(uint32_t addr, double timeout_sec)
{
    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    return _ctrl->read_axi(addr, timeout_sec);
}

void m300_rf_ctrl::gpio_set_direction(uint32_t number, bool output, double timeout_sec)
{
    if (number > kMaxGpioNumber) {
        throw std::out_of_range("M300 GPIO number out of range");
    }

    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    const uint32_t mask = (1u << number);
    if (output) {
        _gpio_oe |= mask;
    } else {
        _gpio_oe &= ~mask;
    }
    (void)_ctrl->write_gpio_oe(_gpio_oe, timeout_sec);
}

void m300_rf_ctrl::gpio_set_value(uint32_t number, bool value, double timeout_sec)
{
    if (number > kMaxGpioNumber) {
        throw std::out_of_range("M300 GPIO number out of range");
    }

    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    const uint32_t mask = (1u << number);
    if (value) {
        _gpio_out |= mask;
    } else {
        _gpio_out &= ~mask;
    }
    (void)_ctrl->write_gpio_out(_gpio_out, timeout_sec);
}

bool m300_rf_ctrl::gpio_get_value(uint32_t number, double timeout_sec)
{
    if (number > kMaxGpioNumber) {
        throw std::out_of_range("M300 GPIO number out of range");
    }

    require_ctrl();
    std::lock_guard<std::mutex> lock(_mutex);
    const uint32_t gpio_in = _ctrl->read_gpio_in(timeout_sec);
    return ((gpio_in >> number) & 0x1u) != 0u;
}

void m300_rf_ctrl::set_sample_rate(uint32_t rate_hz)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_sample_rate(rate_hz);
        _no_os_driver_attached = _ad9361->initialized();
    }
    _sample_rate = rate_hz;
}

uint32_t m300_rf_ctrl::get_sample_rate() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _sample_rate;
}

void m300_rf_ctrl::set_bandwidth(uint32_t bandwidth_hz)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_bandwidth(bandwidth_hz);
        _bandwidth = _ad9361->get_bandwidth();
        _no_os_driver_attached = _ad9361->initialized();
    }
}

uint32_t m300_rf_ctrl::get_bandwidth() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _ad9361 ? _ad9361->get_bandwidth() : _bandwidth;
}

void m300_rf_ctrl::set_rx_freq(uint64_t freq_hz, size_t channel)
{
    (void)channel;
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_rx_freq(freq_hz, channel);
        _no_os_driver_attached = _ad9361->initialized();
    }
    _rx_freq = freq_hz;
}

void m300_rf_ctrl::set_tx_freq(uint64_t freq_hz, size_t channel)
{
    (void)channel;
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_tx_freq(freq_hz, channel);
        _no_os_driver_attached = _ad9361->initialized();
    }
    _tx_freq = freq_hz;
}

uint64_t m300_rf_ctrl::get_rx_freq(size_t channel) const
{
    (void)channel;
    std::lock_guard<std::mutex> lock(_mutex);
    return _rx_freq;
}

uint64_t m300_rf_ctrl::get_tx_freq(size_t channel) const
{
    (void)channel;
    std::lock_guard<std::mutex> lock(_mutex);
    return _tx_freq;
}

void m300_rf_ctrl::set_rx_gain(uint32_t gain_db, size_t channel)
{
    const size_t index = channel_index(channel);
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_rx_gain(gain_db, channel);
        _no_os_driver_attached = _ad9361->initialized();
    }
    _rx_gain[index] = gain_db;
}

void m300_rf_ctrl::set_tx_atten(uint32_t atten_db, size_t channel)
{
    const size_t index = channel_index(channel);
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->set_tx_atten(atten_db, channel);
        _no_os_driver_attached = _ad9361->initialized();
    }
    _tx_atten[index] = atten_db;
}

void m300_rf_ctrl::reapply_tx_attenuation()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_ad9361) {
        _ad9361->reapply_tx_attenuation();
        _no_os_driver_attached = _ad9361->initialized();
    }
}

uint32_t m300_rf_ctrl::get_rx_gain(size_t channel) const
{
    const size_t index = channel_index(channel);
    std::lock_guard<std::mutex> lock(_mutex);
    return _rx_gain[index];
}

uint32_t m300_rf_ctrl::get_tx_atten(size_t channel) const
{
    const size_t index = channel_index(channel);
    std::lock_guard<std::mutex> lock(_mutex);
    return _tx_atten[index];
}

bool m300_rf_ctrl::no_os_driver_attached() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _no_os_driver_attached;
}
