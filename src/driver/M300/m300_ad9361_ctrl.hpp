#ifndef SOAPY_M300_AD9361_CTRL_HPP
#define SOAPY_M300_AD9361_CTRL_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "m300_noos_platform.hpp"
#include "m300_xdma_ctrl.hpp"

struct ad9361_rf_phy;
struct axi_dac;

namespace sdr { namespace driver {

struct m300_ad9361_init_options
{
    uint32_t reference_clk_rate_hz = 40000000u;
    uint32_t sample_rate_hz = 61440000u;
    uint32_t bandwidth_hz = 0u;
    uint64_t rx_lo_hz = 2400000000ull;
    uint64_t tx_lo_hz = 2400000000ull;
    uint32_t tx_attenuation_mdB = 10000u;
    uint32_t rx_gain_db = 20u;
    bool skip_initial_digital_tune = true;
    bool run_post_init_digital_tune = true;
};

class m300_ad9361_ctrl
{
public:
    explicit m300_ad9361_ctrl(const m300_xdma_ctrl::sptr& ctrl);
    ~m300_ad9361_ctrl();

    m300_ad9361_ctrl(const m300_ad9361_ctrl&) = delete;
    m300_ad9361_ctrl& operator=(const m300_ad9361_ctrl&) = delete;

    void init();
    void init(const m300_ad9361_init_options& options);
    bool initialized() const;

    void tune_digital_interface();

    void set_sample_rate(uint32_t rate_hz);
    void set_bandwidth(uint32_t bandwidth_hz);
    uint32_t get_bandwidth() const;
    bool bandwidth_is_auto() const;
    void set_rx_freq(uint64_t freq_hz, size_t channel);
    void set_tx_freq(uint64_t freq_hz, size_t channel);
    void set_rx_gain(uint32_t gain_db, size_t channel);
    void set_tx_atten(uint32_t atten_db, size_t channel);
    void reapply_tx_attenuation();

private:
    void wait_for_power_ready();
    void require_initialized();
    void run_digital_interface_tune(bool scan_supported_rates);
    void apply_tx_attenuation();
    void check_ready();
    uint8_t rx_channel(size_t channel) const;
    uint8_t tx_channel(size_t channel) const;

private:
    m300_xdma_ctrl::sptr _ctrl;
    m300_noos_context _context;
    ad9361_rf_phy* _phy = nullptr;
    axi_dac* _tx_dac = nullptr;
    bool _initialized = false;
    bool _bandwidth_explicit = false;
    uint32_t _sample_rate_hz = 61440000u;
    uint32_t _bandwidth_hz = 56000000u;
    uint64_t _rx_lo_hz = 2400000000ull;
    uint64_t _tx_lo_hz = 2400000000ull;
    uint32_t _tx1_attenuation_mdB = 10000u;
    uint32_t _tx2_attenuation_mdB = 10000u;
};

}} // namespace sdr::driver

#endif // SOAPY_M300_AD9361_CTRL_HPP
