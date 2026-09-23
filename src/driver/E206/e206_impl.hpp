#ifndef SOAPY_E206_IMPL_HPP
#define SOAPY_E206_IMPL_HPP

#include "../E100/e100_impl.hpp"

class E206Impl : public E100Impl {
public:
    enum class VcxoReferenceSource : uint32_t {
        pps = 0u,
        external_10mhz = 1u,
        manual_dac = 2u,
    };

    struct VcxoStatus {
        bool locked = false;
        bool reference_valid = false;
        bool reference_is_10mhz = false;
        bool reference_is_pps = false;
        VcxoReferenceSource selected_source = VcxoReferenceSource::pps;
        uint16_t dac_value = 0u;
        uint32_t raw = 0u;
    };

    explicit E206Impl(const std::string port);
    ~E206Impl() override = default;

    void setSampleRate(double rate) override;
    void set_vcxo_reference_source(VcxoReferenceSource source);
    // Switch and wait for lock (PPS/10 MHz) or mode confirmation (manual DAC).
    // The one-argument overload waits up to 10 s.
    void set_vcxo_reference_source(VcxoReferenceSource source, double timeout_seconds);
    void set_vcxo_manual_dac(uint16_t value);
    VcxoStatus get_vcxo_status();
};

#endif
