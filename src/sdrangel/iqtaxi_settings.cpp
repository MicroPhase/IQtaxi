#include "iqtaxi_settings.h"

#include <limits>

namespace {

uint32_t abs_u32_delta(uint32_t a, uint32_t b)
{
    return a > b ? a - b : b - a;
}

} // namespace

const IqtaxiDeviceCaps *iqtaxiDeviceCapsByModel(const std::string &model)
{
    std::string resolved = model;
    if (model == "E100-6G" || model == "E100-10G" ||
        model == "E100_6G" || model == "E100_10G")
    {
        resolved = "E100";
    }
    for (std::size_t i = 0; i < kIqtaxiDeviceCapsCount; ++i)
    {
        if (resolved == kIqtaxiDeviceCaps[i].model)
        {
            return &kIqtaxiDeviceCaps[i];
        }
    }
    return nullptr;
}

const IqtaxiDeviceCaps *iqtaxiDeviceCapsByHardwareId(const std::string &hardware_id)
{
    for (std::size_t i = 0; i < kIqtaxiDeviceCapsCount; ++i)
    {
        if (hardware_id == kIqtaxiDeviceCaps[i].hardware_id)
        {
            return &kIqtaxiDeviceCaps[i];
        }
    }
    return nullptr;
}

std::string iqtaxiModelFromHardwareId(const std::string &hardware_id)
{
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByHardwareId(hardware_id))
    {
        return caps->model;
    }
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(hardware_id))
    {
        return caps->model;
    }
    if (hardware_id.find("E100") != std::string::npos ||
        hardware_id.find("e100") != std::string::npos)
    {
        return "E100";
    }
    return "E206";
}

std::string iqtaxiFactoryDeviceName(const std::string &model)
{
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(model))
    {
        return caps->factory_name;
    }
    if (model.find("E100") != std::string::npos ||
        model.find("e100") != std::string::npos)
    {
        return "E100";
    }
    return model;
}

uint64_t iqtaxiMaxCenterFreqHz(const std::string &model)
{
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(model))
    {
        return caps->max_freq_hz;
    }
    return 6000000000ull;
}

bool IqtaxiSettings::is_valid() const
{
    if (device_addr.empty())
    {
        return false;
    }
    if (!iqtaxiDeviceCapsByModel(device_model))
    {
        return false;
    }
    if (!iqtaxiIsSupportedSampleRate(device_model, sample_rate_hz))
    {
        return false;
    }
    if (center_freq_hz == 0ull)
    {
        return false;
    }
    if (channel == 0u)
    {
        return false;
    }
    return true;
}

const char *iqtaxiSampleRateLabel(uint32_t sample_rate_hz)
{
    switch (sample_rate_hz)
    {
    case 122880000u: return "122.88 MHz";
    case 61440000u:  return "61.44 MHz";
    case 46080000u:  return "46.08 MHz";
    case 30720000u:  return "30.72 MHz";
    case 23040000u:  return "23.04 MHz";
    case 15360000u:  return "15.36 MHz";
    case 11520000u:  return "11.52 MHz";
    case 7680000u:   return "7.68 MHz";
    case 5760000u:   return "5.76 MHz";
    case 3840000u:   return "3.84 MHz";
    case 1920000u:   return "1.92 MHz";
    case 80000000u:  return "80.00 MHz";
    case 64000000u:  return "64.00 MHz";
    case 40000000u:  return "40.00 MHz";
    case 32000000u:  return "32.00 MHz";
    case 20000000u:  return "20.00 MHz";
    case 16000000u:  return "16.00 MHz";
    case 10000000u:  return "10.00 MHz";
    case 8000000u:   return "8.00 MHz";
    case 5000000u:   return "5.00 MHz";
    case 4000000u:   return "4.00 MHz";
    case 2000000u:   return "2.00 MHz";
    default:         return "Unknown";
    }
}

bool iqtaxiIsSupportedSampleRate(const std::string &model, uint32_t sample_rate_hz)
{
    const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(model);
    if (!caps)
    {
        return false;
    }
    for (std::size_t i = 0; i < caps->sample_rate_count; ++i)
    {
        if (caps->sample_rates[i] == sample_rate_hz)
        {
            return true;
        }
    }
    return false;
}

uint32_t iqtaxiClampGain(const std::string &model, uint32_t gain)
{
    const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(model);
    if (!caps)
    {
        return gain;
    }
    if (gain < caps->gain_min)
    {
        return caps->gain_min;
    }
    if (gain > caps->gain_max)
    {
        return caps->gain_max;
    }
    return gain;
}

uint32_t iqtaxiNearestSampleRate(const std::string &model, uint32_t sample_rate_hz)
{
    const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(model);
    if (!caps || caps->sample_rate_count == 0u)
    {
        return sample_rate_hz;
    }

    uint32_t best = caps->default_sample_rate_hz;
    uint32_t bestDelta = std::numeric_limits<uint32_t>::max();
    for (std::size_t i = 0; i < caps->sample_rate_count; ++i)
    {
        const uint32_t delta = abs_u32_delta(caps->sample_rates[i], sample_rate_hz);
        if (delta < bestDelta)
        {
            bestDelta = delta;
            best = caps->sample_rates[i];
        }
    }
    return best;
}
