#pragma once

#include "include/sdr/api/SampleRates.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

struct IqtaxiSettings
{
    std::string device_model = "E206"; // E100 / E200 / E206
    std::string device_addr = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t center_freq_hz = 1000000000ull;
    uint32_t rx_gain = 30u;
    uint32_t channel = 1u;

    bool is_valid() const;
};

struct IqtaxiDeviceCaps
{
    const char *model;
    const char *hardware_id; // SDRangel OriginDevice / SamplingDevice hardwareId
    const char *display_name;
    const uint32_t *sample_rates;
    std::size_t sample_rate_count;
    uint32_t default_sample_rate_hz;
    uint32_t gain_min;
    uint32_t gain_max;
};

// E200 / AD9361: keep rates at or below 61.44 Msps.
inline constexpr uint32_t kIqtaxiE200SampleRates[] = {
    1920000u,
    2000000u,
    3840000u,
    4000000u,
    5000000u,
    5760000u,
    7680000u,
    8000000u,
    10000000u,
    11520000u,
    15360000u,
    16000000u,
    20000000u,
    23040000u,
    30720000u,
    32000000u,
    40000000u,
    46080000u,
    61440000u,
};

inline constexpr IqtaxiDeviceCaps kIqtaxiDeviceCaps[] = {
    {
        "E100",
        "IQTAXI-E100",
        "IQTAXI E100",
        sdr::api::kGc080xLegacySampleRatesHz.data(),
        sdr::api::kGc080xLegacySampleRatesHz.size(),
        15360000u,
        0u,
        41u,
    },
    {
        "E200",
        "IQTAXI-E200",
        "IQTAXI E200",
        kIqtaxiE200SampleRates,
        sizeof(kIqtaxiE200SampleRates) / sizeof(kIqtaxiE200SampleRates[0]),
        30720000u,
        0u,
        75u,
    },
    {
        "E206",
        "IQTAXI-E206",
        "IQTAXI E206",
        sdr::api::kGc080xLegacySampleRatesHz.data(),
        sdr::api::kGc080xLegacySampleRatesHz.size(),
        15360000u,
        0u,
        41u,
    },
};

inline constexpr std::size_t kIqtaxiDeviceCapsCount =
    sizeof(kIqtaxiDeviceCaps) / sizeof(kIqtaxiDeviceCaps[0]);

const IqtaxiDeviceCaps *iqtaxiDeviceCapsByModel(const std::string &model);
const IqtaxiDeviceCaps *iqtaxiDeviceCapsByHardwareId(const std::string &hardware_id);
std::string iqtaxiModelFromHardwareId(const std::string &hardware_id);

const char *iqtaxiSampleRateLabel(uint32_t sample_rate_hz);
bool iqtaxiIsSupportedSampleRate(const std::string &model, uint32_t sample_rate_hz);
uint32_t iqtaxiClampGain(const std::string &model, uint32_t gain);
uint32_t iqtaxiNearestSampleRate(const std::string &model, uint32_t sample_rate_hz);
