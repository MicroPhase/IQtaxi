#ifndef SOAPY_M300_RF_CTRL_HPP
#define SOAPY_M300_RF_CTRL_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "m300_xdma_ctrl.hpp"

namespace sdr { namespace driver {

class m300_ad9361_ctrl;

class m300_rf_ctrl
{
public:
    typedef std::shared_ptr<m300_rf_ctrl> sptr;

    explicit m300_rf_ctrl(const m300_xdma_ctrl::sptr& ctrl);
    ~m300_rf_ctrl();

    uint32_t ad9361_product_id(double timeout_sec = 1.0);
    void ad9361_spi_write(uint16_t reg, uint8_t value, double timeout_sec = 1.0);
    uint8_t ad9361_spi_read(uint16_t reg, double timeout_sec = 1.0);
    void axi_write(uint32_t addr, uint32_t value, double timeout_sec = 1.0);
    uint32_t axi_read(uint32_t addr, double timeout_sec = 1.0);
    void gpio_set_direction(uint32_t number, bool output, double timeout_sec = 1.0);
    void gpio_set_value(uint32_t number, bool value, double timeout_sec = 1.0);
    bool gpio_get_value(uint32_t number, double timeout_sec = 1.0);

    void set_sample_rate(uint32_t rate_hz);
    uint32_t get_sample_rate() const;
    void set_bandwidth(uint32_t bandwidth_hz);
    uint32_t get_bandwidth() const;

    void set_rx_freq(uint64_t freq_hz, size_t channel);
    void set_tx_freq(uint64_t freq_hz, size_t channel);
    uint64_t get_rx_freq(size_t channel) const;
    uint64_t get_tx_freq(size_t channel) const;

    void set_rx_gain(uint32_t gain_db, size_t channel);
    void set_tx_atten(uint32_t atten_db, size_t channel);
    void reapply_tx_attenuation();
    uint32_t get_rx_gain(size_t channel) const;
    uint32_t get_tx_atten(size_t channel) const;

    bool no_os_driver_attached() const;

private:
    void require_ctrl() const;

private:
    m300_xdma_ctrl::sptr _ctrl;
    std::unique_ptr<m300_ad9361_ctrl> _ad9361;
    mutable std::mutex _mutex;
    uint32_t _sample_rate = 0;
    uint32_t _bandwidth = 0;
    uint64_t _rx_freq = 0;
    uint64_t _tx_freq = 0;
    std::array<uint32_t, 2> _rx_gain{{0u, 0u}};
    std::array<uint32_t, 2> _tx_atten{{0u, 0u}};
    uint32_t _gpio_out = 0;
    uint32_t _gpio_oe = 0;
    bool _no_os_driver_attached = false;
};

}} // namespace sdr::driver

#endif // SOAPY_M300_RF_CTRL_HPP
