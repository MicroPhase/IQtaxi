#include "./e206_impl.hpp"
#include "../E100/local_e100_regs.hpp"
#include "include/sdr/api/SampleRates.hpp"

#include <algorithm>
#include <stdexcept>

namespace {
uint32_t nearest_e206_sample_rate(double rate)
{
    const uint32_t requested_rate = rate <= 0.0 ? 0u : static_cast<uint32_t>(rate + 0.5);

    return *std::min_element(
        sdr::api::kGc080xLegacySampleRatesHz.begin(),
        sdr::api::kGc080xLegacySampleRatesHz.end(),
        [requested_rate](uint32_t lhs, uint32_t rhs) {
            const uint32_t lhs_delta =
                lhs > requested_rate ? lhs - requested_rate : requested_rate - lhs;
            const uint32_t rhs_delta =
                rhs > requested_rate ? rhs - requested_rate : requested_rate - rhs;
            return lhs_delta < rhs_delta;
        });
}

void throw_if_vcxo_command_failed(uint64_t result, const char* operation)
{
    if (result != 0u) {
        throw std::runtime_error(std::string("E206 VCXO command failed: ") + operation);
    }
}
} // namespace

E206Impl::E206Impl(const std::string port)
    : E100Impl(port, sdr::api::e206_udp_profile())
{
}

void E206Impl::setSampleRate(double rate)
{
    IqtaxiUdpImpl::setSampleRate(static_cast<double>(nearest_e206_sample_rate(rate)));
}

void E206Impl::set_vcxo_reference_source(VcxoReferenceSource source)
{
    throw_if_vcxo_command_failed(
        get_local_bus()->poke32_ack_value(
            e100::CUSTOM_SET_VCXO_REF_SOURCE, static_cast<uint32_t>(source), 1.0),
        "set reference source");
}

void E206Impl::set_vcxo_manual_dac(uint16_t value)
{
    throw_if_vcxo_command_failed(
        get_local_bus()->poke32_ack_value(e100::CUSTOM_SET_VCXO_DAC_VALUE, value, 1.0),
        "set manual DAC");
}

E206Impl::VcxoStatus E206Impl::get_vcxo_status()
{
    VcxoStatus status;
    status.raw = get_local_bus()->peek32(e100::CUSTOM_RB_GET_VCXO_STATUS, 1.0);
    status.locked = (status.raw & (1u << 0)) != 0u;
    status.reference_valid = (status.raw & (1u << 1)) != 0u;
    status.reference_is_10mhz = (status.raw & (1u << 2)) != 0u;
    status.reference_is_pps = (status.raw & (1u << 3)) != 0u;
    const uint32_t source = (status.raw >> 4) & 0x3u;
    status.selected_source = source <= static_cast<uint32_t>(VcxoReferenceSource::manual_dac)
                                 ? static_cast<VcxoReferenceSource>(source)
                                 : VcxoReferenceSource::pps;
    status.dac_value = static_cast<uint16_t>(status.raw >> 16);
    return status;
}
