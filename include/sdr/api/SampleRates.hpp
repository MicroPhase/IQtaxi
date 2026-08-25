#ifndef SDR_API_SAMPLE_RATES_HPP
#define SDR_API_SAMPLE_RATES_HPP

#include <array>
#include <cstdint>

namespace sdr::api {

/* Shared by the current E100 and E206 GC080X legacy-rate firmware.  Keep host
 * capability reporting and nearest-rate coercion tied to this single table. */
inline constexpr std::array<uint32_t, 10> kGc080xLegacySampleRatesHz = {
    1920000u,
    3840000u,
    5760000u,
    7680000u,
    11520000u,
    15360000u,
    23040000u,
    30720000u,
    61440000u,
    122880000u,
};

} // namespace sdr::api

#endif
