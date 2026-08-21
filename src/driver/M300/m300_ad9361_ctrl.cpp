#include "m300_ad9361_ctrl.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "m300_xdma_protocol.hpp"

extern "C" {
#include "ad9361.h"
#include "ad9361_api.h"
#include "axi_dac_core.h"
#include "m300_ad9361_defaults.h"
#include "no_os_delay.h"
}

using namespace sdr::driver;

namespace {
constexpr uint32_t kM300Ad9361BaseAddr = 0x44a00000u;
constexpr uint32_t kM300Ad9361RxBaseAddr = kM300Ad9361BaseAddr;
constexpr uint32_t kM300Ad9361TxBaseAddr = kM300Ad9361BaseAddr + 0x4000u;
constexpr uint32_t kMinBandwidthHz = 200000u;
constexpr uint32_t kMaxBandwidthHz = 56000000u;
constexpr uint32_t kDigitalTuneMaxRateHz = 61440000u;
constexpr uint8_t kVcoLockMask = 0x02u;
constexpr uint32_t kAxiTxCoreOffset = 0x4000u;
constexpr uint32_t kAxiRegRstn = 0x0040u;
constexpr uint32_t kAxiRegStatus = 0x005cu;
constexpr uint32_t kAxiRstnMask = 0x00000003u;
constexpr uint32_t kAxiStatusReady = 0x00000001u;
constexpr unsigned kReadyPollCount = 50u;
constexpr unsigned kPowerReadyPollCount = 500u;
constexpr unsigned kPowerReadyStableReads = 3u;
constexpr uint32_t kPowerReadyPollIntervalMs = 10u;

class scoped_noos_context
{
public:
    explicit scoped_noos_context(m300_noos_context* context)
        : _previous(m300_noos_get_active_context())
    {
        m300_noos_set_active_context(context);
    }

    ~scoped_noos_context()
    {
        m300_noos_set_active_context(_previous);
    }

private:
    m300_noos_context* _previous;
};

void check_noos(int32_t ret, const char* what)
{
    if (ret < 0) {
        throw std::runtime_error(std::string("M300 AD9361 ") + what +
                                 " failed, ret=" + std::to_string(ret));
    }
}

uint32_t bandwidth_for_rate(uint32_t rate_hz)
{
    const uint32_t bw = std::min(rate_hz, kMaxBandwidthHz);
    return std::max(bw, kMinBandwidthHz);
}

uint32_t sanitize_bandwidth(uint32_t bandwidth_hz)
{
    return std::clamp(bandwidth_hz, kMinBandwidthHz, kMaxBandwidthHz);
}

void check_rfpll_locks(ad9361_rf_phy* phy)
{
    uint8_t rx_lock = 0u;
    uint8_t tx_lock = 0u;

    for (unsigned attempt = 0; attempt < kReadyPollCount; ++attempt) {
        rx_lock = ad9361_spi_read(phy->spi, REG_RX_CP_OVERRANGE_VCO_LOCK);
        tx_lock = ad9361_spi_read(phy->spi, REG_TX_CP_OVERRANGE_VCO_LOCK);
        if (((rx_lock & kVcoLockMask) != 0u) &&
            ((tx_lock & kVcoLockMask) != 0u)) {
            return;
        }
        no_os_mdelay(1u);
    }

    std::ostringstream oss;
    oss << "M300 AD9361 RFPLL lock timed out, rx[0x247]=0x"
        << std::hex << static_cast<unsigned>(rx_lock)
        << " tx[0x287]=0x" << static_cast<unsigned>(tx_lock);
    throw std::runtime_error(oss.str());
}
}

m300_ad9361_ctrl::m300_ad9361_ctrl(const m300_xdma_ctrl::sptr& ctrl)
    : _ctrl(ctrl), _context(ctrl)
{
}

m300_ad9361_ctrl::~m300_ad9361_ctrl()
{
    scoped_noos_context active(&_context);
    if (_tx_dac) {
        (void)axi_dac_remove(_tx_dac);
        _tx_dac = nullptr;
    }
    if (_phy) {
        _phy->tx_dac = nullptr;
        (void)ad9361_remove(_phy);
        _phy = nullptr;
    }
}

void m300_ad9361_ctrl::init()
{
    init(m300_ad9361_init_options{});
}

void m300_ad9361_ctrl::init(const m300_ad9361_init_options& options)
{
    if (_initialized) {
        return;
    }

    scoped_noos_context active(&_context);
    wait_for_power_ready();
    _sample_rate_hz = options.sample_rate_hz;
    _rx_lo_hz = options.rx_lo_hz;
    _tx_lo_hz = options.tx_lo_hz;
    _tx1_attenuation_mdB = options.tx_attenuation_mdB;
    _tx2_attenuation_mdB = options.tx_attenuation_mdB;
    _bandwidth_explicit = options.bandwidth_hz != 0u;
    _bandwidth_hz = _bandwidth_explicit ?
        sanitize_bandwidth(options.bandwidth_hz) :
        bandwidth_for_rate(_sample_rate_hz);

    m300_default_rx_adc_init.base = kM300Ad9361RxBaseAddr;
    m300_default_tx_dac_init.base = kM300Ad9361TxBaseAddr;

    static axi_dac_channel tx_channels[4] = {};
    for (auto& channel : tx_channels) {
        channel.sel = AXI_DAC_DATA_SEL_DMA;
    }
    m300_default_tx_dac_init.channels = tx_channels;

    AD9361_InitParam init_param = m300_ad9361_default_init_param_template;
    init_param.spi_param.device_id = 0;
    init_param.spi_param.chip_select = 0;
    init_param.gpio_resetb.number = M300_GPIO_AD9361_RESETB;
    init_param.gpio_resetb.platform_ops = &m300_gpio_ops;
    init_param.gpio_sync.number = -1;
    init_param.gpio_sync.platform_ops = &m300_gpio_ops;
    init_param.gpio_cal_sw1.number = -1;
    init_param.gpio_cal_sw1.platform_ops = &m300_gpio_ops;
    init_param.gpio_cal_sw2.number = -1;
    init_param.gpio_cal_sw2.platform_ops = &m300_gpio_ops;
    init_param.rx_adc_init = &m300_default_rx_adc_init;
    init_param.tx_dac_init = &m300_default_tx_dac_init;
    init_param.reference_clk_rate = options.reference_clk_rate_hz;
    init_param.aux_adc_rate = options.reference_clk_rate_hz;
    init_param.rx_synthesizer_frequency_hz = options.rx_lo_hz;
    init_param.tx_synthesizer_frequency_hz = options.tx_lo_hz;
    init_param.rf_rx_bandwidth_hz = _bandwidth_hz;
    init_param.rf_tx_bandwidth_hz = _bandwidth_hz;
    init_param.tx_attenuation_mdB = options.tx_attenuation_mdB;
    if (options.skip_initial_digital_tune) {
        init_param.digital_interface_tune_skip_mode = 2;
    }

    std::cout << "noos_init reference_clk_rate=" << init_param.reference_clk_rate
              << " xo_disable_use_ext_refclk_enable="
              << static_cast<uint32_t>(init_param.xo_disable_use_ext_refclk_enable)
              << " aux_adc_rate=" << init_param.aux_adc_rate
              << " sample_rate=" << _sample_rate_hz
              << " bandwidth=" << _bandwidth_hz
              << " bandwidth_auto=" << (_bandwidth_explicit ? 0 : 1)
              << " digital_tune_skip="
              << static_cast<uint32_t>(init_param.digital_interface_tune_skip_mode)
              << "\n";

    check_noos(ad9361_init(&_phy, &init_param), "init");
    check_noos(ad9361_set_tx_fir_config(_phy,
                                         m300_ad9361_tx_fir_config_template),
               "TX FIR config");
    check_noos(ad9361_set_rx_fir_config(_phy,
                                         m300_ad9361_rx_fir_config_template),
               "RX FIR config");

    check_noos(axi_dac_init(&_tx_dac, &m300_default_tx_dac_init), "TX DAC init");
    _phy->tx_dac = _tx_dac;
    check_noos(axi_dac_set_datasel(_tx_dac, -1, AXI_DAC_DATA_SEL_DMA),
               "TX DAC DMA select");

    // Both no-OS sampling-frequency setters calculate and program the complete
    // RX/TX clock chain. Calling both back-to-back needlessly resets the BBPLL
    // and runs the current-rate interface tune twice. Program it once, after
    // the FIR and AXI DAC are ready, then perform the final tune below.
    check_noos(ad9361_set_rx_sampling_freq(_phy, _sample_rate_hz),
               "default sample rate");
    check_noos(ad9361_set_rx_rf_bandwidth(_phy, _bandwidth_hz),
               "default RX bandwidth");
    check_noos(ad9361_set_tx_rf_bandwidth(_phy, _bandwidth_hz),
               "default TX bandwidth");
    check_noos(ad9361_set_rx_gain_control_mode(_phy, RX1, RF_GAIN_MGC),
               "RX1 MGC mode");
    check_noos(ad9361_set_rx_gain_control_mode(_phy, RX2, RF_GAIN_MGC),
               "RX2 MGC mode");
    check_noos(ad9361_set_rx_rf_gain(_phy, RX1, options.rx_gain_db),
               "default RX1 gain");
    check_noos(ad9361_set_rx_rf_gain(_phy, RX2, options.rx_gain_db),
               "default RX2 gain");
    if (options.run_post_init_digital_tune) {
        run_digital_interface_tune(true);
    }

    // Multi-rate tuning can enter a nested current-rate tune while TX is
    // muted. The no-OS mute cache is shared, so the nested tune may overwrite
    // the original attenuation with the mute value. Always apply the requested
    // attenuation after the final tune.
    apply_tx_attenuation();
    check_ready();
    _initialized = true;
}

void m300_ad9361_ctrl::wait_for_power_ready()
{
    if (!_ctrl) {
        throw std::runtime_error("M300 AD9361 power wait has no control transport");
    }

    constexpr uint32_t required = M300_GPIO_IN_LP8758_DONE_MASK |
                                  M300_GPIO_IN_AD9361_RESETB_MASK;
    uint32_t gpio_in = 0u;
    unsigned stable_reads = 0u;

    for (unsigned attempt = 0; attempt < kPowerReadyPollCount; ++attempt) {
        gpio_in = _ctrl->read_gpio_in(0.25);
        if ((gpio_in & required) == required) {
            ++stable_reads;
            if (stable_reads >= kPowerReadyStableReads) {
                // Leave a small settling interval between FPGA power/reset
                // sequencing and the first AD9361 SPI transaction.
                no_os_mdelay(kPowerReadyPollIntervalMs);
                return;
            }
        } else {
            stable_reads = 0u;
        }
        no_os_mdelay(kPowerReadyPollIntervalMs);
    }

    std::ostringstream oss;
    oss << "M300 AD9361 power-ready timed out: gpio_in=0x"
        << std::hex << gpio_in
        << " required=0x" << required
        << " (bit24=LP8758 verified, bit0=AD9361 RESETB released)";
    throw std::runtime_error(oss.str());
}

bool m300_ad9361_ctrl::initialized() const
{
    return _initialized;
}

void m300_ad9361_ctrl::tune_digital_interface()
{
    scoped_noos_context active(&_context);
    require_initialized();
    run_digital_interface_tune(true);
    apply_tx_attenuation();
    check_ready();
}

void m300_ad9361_ctrl::run_digital_interface_tune(bool scan_supported_rates)
{
    std::array<uint32_t, 6> rx_path_clks{};
    std::array<uint32_t, 6> tx_path_clks{};
    if (scan_supported_rates) {
        check_noos(ad9361_get_trx_clock_chain(
                       _phy, rx_path_clks.data(), tx_path_clks.data()),
                   "save clock chain before digital tune");
    }

    _phy->pdata->dig_interface_tune_skipmode = 0;
    check_noos(ad9361_dig_tune(
                   _phy, scan_supported_rates ? kDigitalTuneMaxRateHz : 0u,
                   static_cast<dig_tune_flags>(BE_VERBOSE | BE_MOREVERBOSE)),
               "digital interface tune");

    // Multi-rate tuning changes the entire RX/TX path clock chain and leaves
    // it at the final scan point. Restore the exact chain captured above, as
    // the E310 no-OS post-setup does. Recalculating it only from the sample
    // rate is not equivalent once the tune has changed the current dividers.
    if (scan_supported_rates) {
        check_noos(ad9361_set_trx_clock_chain(
                       _phy, rx_path_clks.data(), tx_path_clks.data()),
                   "restore clock chain after digital tune");
    }
}

void m300_ad9361_ctrl::apply_tx_attenuation()
{
    check_noos(ad9361_set_tx_attenuation(_phy, TX1, _tx1_attenuation_mdB),
               "apply TX1 attenuation");
    check_noos(ad9361_set_tx_attenuation(_phy, TX2, _tx2_attenuation_mdB),
               "apply TX2 attenuation");
}

void m300_ad9361_ctrl::check_ready()
{
    check_rfpll_locks(_phy);

    uint32_t rx_rstn = 0u;
    uint32_t rx_status = 0u;
    uint32_t tx_rstn = 0u;
    uint32_t tx_status = 0u;

    for (unsigned attempt = 0; attempt < kReadyPollCount; ++attempt) {
        check_noos(axi_adc_read(_phy->rx_adc, kAxiRegRstn, &rx_rstn),
                   "read AXI RX reset status");
        check_noos(axi_adc_read(_phy->rx_adc, kAxiRegStatus, &rx_status),
                   "read AXI RX status");
        check_noos(axi_adc_read(_phy->rx_adc,
                                kAxiTxCoreOffset + kAxiRegRstn,
                                &tx_rstn),
                   "read AXI TX reset status");
        check_noos(axi_adc_read(_phy->rx_adc,
                                kAxiTxCoreOffset + kAxiRegStatus,
                                &tx_status),
                   "read AXI TX status");

        if ((rx_rstn & kAxiRstnMask) == kAxiRstnMask &&
            (tx_rstn & kAxiRstnMask) == kAxiRstnMask &&
            (rx_status & kAxiStatusReady) != 0u &&
            (tx_status & kAxiStatusReady) != 0u) {
            return;
        }
        no_os_mdelay(1u);
    }

    std::ostringstream oss;
    oss << "M300 AXI AD9361 data path not ready"
        << " rx_rstn=0x" << std::hex << rx_rstn
        << " rx_status=0x" << rx_status
        << " tx_rstn=0x" << tx_rstn
        << " tx_status=0x" << tx_status;
    throw std::runtime_error(oss.str());
}

void m300_ad9361_ctrl::require_initialized()
{
    if (!_initialized) {
        init();
    }
}

uint8_t m300_ad9361_ctrl::rx_channel(size_t channel) const
{
    return channel == 2 ? RX2 : RX1;
}

uint8_t m300_ad9361_ctrl::tx_channel(size_t channel) const
{
    return channel == 2 ? TX2 : TX1;
}

void m300_ad9361_ctrl::set_sample_rate(uint32_t rate_hz)
{
    if (!_initialized) {
        m300_ad9361_init_options options;
        options.sample_rate_hz = rate_hz;
        init(options);
        return;
    }

    scoped_noos_context active(&_context);
    if (rate_hz == _sample_rate_hz) {
        return;
    }

    // Either public no-OS sampling-frequency API programs both sides of the
    // clock chain. One call is sufficient and avoids back-to-back BBPLL resets.
    check_noos(ad9361_set_rx_sampling_freq(_phy, rate_hz), "set sample rate");
    if (!_bandwidth_explicit) {
        _bandwidth_hz = bandwidth_for_rate(rate_hz);
        check_noos(ad9361_set_rx_rf_bandwidth(_phy, _bandwidth_hz), "set RX bandwidth");
        check_noos(ad9361_set_tx_rf_bandwidth(_phy, _bandwidth_hz), "set TX bandwidth");
    }
    // Updating the clock chain can run a digital-interface tune. no-OS mutes
    // TX during that tune, and its nested mute cache can leave the mute
    // attenuation behind. Restore the requested values after all calibrations.
    apply_tx_attenuation();
    check_ready();
    _sample_rate_hz = rate_hz;
}

void m300_ad9361_ctrl::set_bandwidth(uint32_t bandwidth_hz)
{
    scoped_noos_context active(&_context);
    require_initialized();
    _bandwidth_explicit = bandwidth_hz != 0u;
    _bandwidth_hz = _bandwidth_explicit ?
        sanitize_bandwidth(bandwidth_hz) :
        bandwidth_for_rate(_sample_rate_hz);
    check_noos(ad9361_set_rx_rf_bandwidth(_phy, _bandwidth_hz), "set RX bandwidth");
    check_noos(ad9361_set_tx_rf_bandwidth(_phy, _bandwidth_hz), "set TX bandwidth");
    apply_tx_attenuation();
}

uint32_t m300_ad9361_ctrl::get_bandwidth() const
{
    return _bandwidth_hz;
}

bool m300_ad9361_ctrl::bandwidth_is_auto() const
{
    return !_bandwidth_explicit;
}

void m300_ad9361_ctrl::set_rx_freq(uint64_t freq_hz, size_t channel)
{
    (void)channel;
    scoped_noos_context active(&_context);
    require_initialized();
    if (freq_hz == _rx_lo_hz) {
        return;
    }
    check_noos(ad9361_set_rx_lo_freq(_phy, freq_hz), "set RX LO");
    check_rfpll_locks(_phy);
    _rx_lo_hz = freq_hz;
}

void m300_ad9361_ctrl::set_tx_freq(uint64_t freq_hz, size_t channel)
{
    (void)channel;
    scoped_noos_context active(&_context);
    require_initialized();
    if (freq_hz == _tx_lo_hz) {
        return;
    }
    check_noos(ad9361_set_tx_lo_freq(_phy, freq_hz), "set TX LO");
    check_rfpll_locks(_phy);
    _tx_lo_hz = freq_hz;
}

void m300_ad9361_ctrl::set_rx_gain(uint32_t gain_db, size_t channel)
{
    scoped_noos_context active(&_context);
    require_initialized();
    const uint8_t ch = rx_channel(channel);
    check_noos(ad9361_set_rx_gain_control_mode(_phy, ch, RF_GAIN_MGC),
               "set RX gain mode");
    check_noos(ad9361_set_rx_rf_gain(_phy, ch, static_cast<int32_t>(gain_db)),
               "set RX gain");
}

void m300_ad9361_ctrl::set_tx_atten(uint32_t atten_db, size_t channel)
{
    scoped_noos_context active(&_context);
    require_initialized();
    const uint32_t attenuation_mdB = atten_db * 1000u;
    const uint8_t ch = tx_channel(channel);
    check_noos(ad9361_set_tx_attenuation(_phy, ch, attenuation_mdB),
               "set TX attenuation");
    uint32_t readback_mdB = 0u;
    check_noos(ad9361_get_tx_attenuation(_phy, ch, &readback_mdB),
               "read back TX attenuation");
    if (ch == TX2) {
        _tx2_attenuation_mdB = attenuation_mdB;
    } else {
        _tx1_attenuation_mdB = attenuation_mdB;
    }
    std::cout << "M300 TX" << (ch == TX2 ? 2 : 1)
              << " attenuation requested/readback mdB: "
              << attenuation_mdB << "/" << readback_mdB << std::endl;
}

void m300_ad9361_ctrl::reapply_tx_attenuation()
{
    scoped_noos_context active(&_context);
    require_initialized();
    apply_tx_attenuation();

    uint32_t tx1_readback_mdB = 0u;
    uint32_t tx2_readback_mdB = 0u;
    check_noos(ad9361_get_tx_attenuation(_phy, TX1, &tx1_readback_mdB),
               "read back TX1 attenuation");
    check_noos(ad9361_get_tx_attenuation(_phy, TX2, &tx2_readback_mdB),
               "read back TX2 attenuation");
    std::cout << "M300 TX attenuation requested/readback mdB: TX1="
              << _tx1_attenuation_mdB << "/" << tx1_readback_mdB
              << " TX2=" << _tx2_attenuation_mdB << "/" << tx2_readback_mdB
              << std::endl;
}
