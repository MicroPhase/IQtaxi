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
    std::string rf_band; // E100: "6G" or "10G"
    NumericRange rx_frequency_hz;
    NumericRange tx_frequency_hz;
    NumericRange rx_gain_db;
    NumericRange tx_attenuation_db;
};

inline std::string e100_rf_band_from_max_hz(double max_hz)
{
    return (max_hz > 8.0e9) ? "10G" : "6G";
}

inline std::string e100_rf_band_from_text(const std::string& text)
{
    if (text.find("10G") != std::string::npos || text.find("10g") != std::string::npos) {
        return "10G";
    }
    if (text.find("6G") != std::string::npos || text.find("6g") != std::string::npos) {
        return "6G";
    }
    return {};
}

inline std::string e100_display_name(const DeviceProfile& profile)
{
    if (profile.product != "E100") {
        return profile.product;
    }
    if (profile.rf_band.empty()) {
        return "E100";
    }
    return "E100 " + profile.rf_band;
}

// Device-list / combo label. E100 board_version from UDP discover is typically "6G"/"10G".
inline std::string iqtaxi_model_label(const std::string& product,
                                      const std::string& board_version = {})
{
    if (product == "E100" || product == "e100") {
        const std::string band = e100_rf_band_from_text(board_version);
        return band.empty() ? std::string("E100") : ("E100-" + band);
    }
    return product;
}

inline const DeviceProfile& e100_udp_profile()
{
    static const DeviceProfile profile = {
        "E100",
        "udp",
        "GC080x",
        "6G",
        {50e6, 6e9, 1.0},
        {50e6, 6e9, 1.0},
        {0.0, 41.0, 1.0},
        {0.0, 50.0, 1.0},
    };
    return profile;
}

inline DeviceProfile e100_udp_profile_with_freq(double min_hz, double max_hz)
{
    DeviceProfile profile = e100_udp_profile();
    if (min_hz > 0.0 && max_hz > min_hz) {
        profile.rx_frequency_hz.minimum = min_hz;
        profile.rx_frequency_hz.maximum = max_hz;
        profile.tx_frequency_hz.minimum = min_hz;
        profile.tx_frequency_hz.maximum = max_hz;
    }
    profile.rf_band = e100_rf_band_from_max_hz(profile.rx_frequency_hz.maximum);
    return profile;
}

inline const DeviceProfile& e200_udp_profile()
{
    static const DeviceProfile profile = {
        "E200",
        "udp",
        "AD9361",
        "",
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
        "",
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
        "",
        {70e6, 6e9, 1.0},
        {70e6, 6e9, 1.0},
        {0.0, 76.0, 1.0},
        {0.0, 89.0, 1.0},
    };
    return profile;
}

} // namespace sdr::api

#endif
