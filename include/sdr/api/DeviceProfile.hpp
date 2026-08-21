#ifndef SDR_API_DEVICE_PROFILE_HPP
#define SDR_API_DEVICE_PROFILE_HPP

#include <string>

namespace sdr::api {

struct NumericRange {
    double minimum;
    double maximum;
    double step;
};

struct DeviceProfile {
    std::string product;
    std::string transport;
    std::string rf_frontend;
    NumericRange rx_frequency_hz;
    NumericRange tx_frequency_hz;
    NumericRange rx_gain_db;
    NumericRange tx_attenuation_db;
};

inline const DeviceProfile& e100_udp_profile()
{
    static const DeviceProfile profile = {
        "E100",
        "udp",
        "MP2021",
        {70e6, 6e9, 1.0},
        {70e6, 6e9, 1.0},
        {0.0, 41.0, 1.0},
        {0.0, 50.0, 1.0},
    };
    return profile;
}

inline const DeviceProfile& e200_udp_profile()
{
    static const DeviceProfile profile = {
        "E200",
        "udp",
        "AD9361",
        {70e6, 6e9, 1.0},
        {70e6, 6e9, 1.0},
        {0.0, 75.0, 1.0},
        {0.0, 89.0, 1.0},
    };
    return profile;
}

inline const DeviceProfile& e206_udp_profile()
{
    static const DeviceProfile profile = {
        "E206",
        "udp",
        "MP2021",
        {70e6, 6e9, 1.0},
        {70e6, 6e9, 1.0},
        // E206 uses the GC080x full RX gain-table index (0..42), not dB.
        {0.0, 42.0, 1.0},
        {0.0, 50.0, 1.0},
    };
    return profile;
}

inline const DeviceProfile& m300_pcie_profile()
{
    static const DeviceProfile profile = {
        "M300",
        "pcie",
        "AD9361",
        {70e6, 6e9, 1.0},
        {70e6, 6e9, 1.0},
        {0.0, 76.0, 1.0},
        {0.0, 89.0, 1.0},
    };
    return profile;
}

} // namespace sdr::api

#endif
