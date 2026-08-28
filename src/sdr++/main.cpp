#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <config.h>
#include <gui/smgui.h>
#include <utils/optionlist.h>

#include "include/sdr/api/Device.hpp"
#include "include/sdr/api/DeviceProfile.hpp"
#include "include/sdr/api/SampleRates.hpp"
#include "include/sdr/api/UdpDiscover.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

namespace {

constexpr uint8_t kStreamMode = 0x1;
constexpr size_t kMaxPacketSamples = (1472u - 16u) / 4u;
constexpr size_t kRecvChunkSamples = 8192u;

template <std::size_t N>
constexpr std::array<double, N> toDoubleRates(
    const std::array<uint32_t, N>& rates)
{
    std::array<double, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = static_cast<double>(rates[i]);
    }
    return result;
}

constexpr auto kGc080xRates =
    toDoubleRates(sdr::api::kGc080xLegacySampleRatesHz);

// E200 / AD9361: keep rates at or below 61.44 Msps.
constexpr double kE200Rates[] = {
    1920000.0,
    2000000.0,
    3840000.0,
    4000000.0,
    5000000.0,
    5760000.0,
    7680000.0,
    8000000.0,
    10000000.0,
    11520000.0,
    15360000.0,
    16000000.0,
    20000000.0,
    23040000.0,
    30720000.0,
    32000000.0,
    40000000.0,
    46080000.0,
    61440000.0,
};

struct DeviceSpec {
    const char* name;
    const char* product;
    double max_freq_hz;
    const sdr::api::DeviceProfile& (*profile)();
    const double* rates;
    size_t rateCount;
    double defaultRate;
};

const DeviceSpec kDeviceSpecs[] = {
    {"E100", "E100", 10e9, &sdr::api::e100_udp_profile, kGc080xRates.data(), kGc080xRates.size(), 15360000.0},
    {"E200", "E200", 6e9, &sdr::api::e200_udp_profile, kE200Rates, std::size(kE200Rates), 30720000.0},
    {"E206", "E206", 6e9, &sdr::api::e206_udp_profile, kGc080xRates.data(), kGc080xRates.size(), 15360000.0},
};

constexpr int kDeviceCount = static_cast<int>(std::size(kDeviceSpecs));

std::string sampleRateLabel(double rate)
{
    char buf[32];
    if (rate >= 1e6) {
        const double mhz = rate / 1e6;
        if (std::fabs(mhz - std::round(mhz)) < 1e-6) {
            std::snprintf(buf, sizeof(buf), "%.0f MHz", mhz);
        }
        else {
            std::snprintf(buf, sizeof(buf), "%.2f MHz", mhz);
        }
    }
    else {
        std::snprintf(buf, sizeof(buf), "%.0f Hz", rate);
    }
    return buf;
}

int deviceIndexByName(const std::string& name)
{
    for (int i = 0; i < kDeviceCount; i++) {
        if (name == kDeviceSpecs[i].name) {
            return i;
        }
    }
    if (name == "E100-6G" || name == "E100-10G" ||
        name == "E100_6G" || name == "E100_10G") {
        return 0;
    }
    for (int i = 0; i < kDeviceCount; i++) {
        if (name == kDeviceSpecs[i].product) {
            return i;
        }
    }
    return 2; // E206
}

double nearestRate(const DeviceSpec& spec, double rate)
{
    double best = spec.defaultRate;
    double bestDelta = std::numeric_limits<double>::max();
    for (size_t i = 0; i < spec.rateCount; i++) {
        const double delta = std::fabs(spec.rates[i] - rate);
        if (delta < bestDelta) {
            bestDelta = delta;
            best = spec.rates[i];
        }
    }
    return best;
}

} // namespace

SDRPP_MOD_INFO{
    /* Name:            */ "iqtaxi_source",
    /* Description:     */ "Microphase IQTAXI native source",
    /* Author:          */ "Microphase",
    /* Version:         */ 0, 2, 0,
    /* Max instances    */ 1
};

ConfigManager config;

class IqtaxiSourceModule : public ModuleManager::Instance {
public:
    IqtaxiSourceModule(std::string name) {
        this->name = std::move(name);

        for (int i = 0; i < kDeviceCount; i++) {
            devices.define(i, kDeviceSpecs[i].name, kDeviceSpecs[i].name);
        }

        deviceId = 2;
        std::strncpy(host, "192.168.1.10", sizeof(host) - 1);
        gain = 30.0f;

        config.acquire();
        if (config.conf.contains("host")) {
            const std::string hostStr = config.conf["host"];
            std::strncpy(host, hostStr.c_str(), sizeof(host) - 1);
            host[sizeof(host) - 1] = '\0';
        }
        if (config.conf.contains("device")) {
            deviceId = deviceIndexByName(config.conf["device"]);
        }
        if (config.conf.contains("sampleRate")) {
            sampleRate = config.conf["sampleRate"];
        }
        if (config.conf.contains("gain")) {
            gain = config.conf["gain"];
        }
        config.release();

        applyDeviceCaps(false);
        refreshFromDiscovery();

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("IQTAXI", &handler);
    }

    ~IqtaxiSourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("IQTAXI");
    }

    void postInit() {}

    void enable() { enabled = true; }

    void disable() { enabled = false; }

    bool isEnabled() { return enabled; }

private:
    const DeviceSpec& currentSpec() const {
        return kDeviceSpecs[std::clamp(deviceId, 0, kDeviceCount - 1)];
    }

    const sdr::api::DeviceProfile& currentProfile() const {
        return currentSpec().profile();
    }

    float gainMin() const {
        return static_cast<float>(currentProfile().rx_gain_db.minimum);
    }

    float gainMax() const {
        return static_cast<float>(currentProfile().rx_gain_db.maximum);
    }

    void applyDeviceCaps(bool persist) {
        const DeviceSpec& spec = currentSpec();
        samplerates.clear();
        for (size_t i = 0; i < spec.rateCount; i++) {
            samplerates.define(spec.rates[i], sampleRateLabel(spec.rates[i]), spec.rates[i]);
        }

        sampleRate = nearestRate(spec, sampleRate);
        srId = samplerates.keyId(sampleRate);
        gain = std::clamp(gain, gainMin(), gainMax());
        freq = std::clamp(freq, 1.0, spec.max_freq_hz);

        if (persist) {
            saveConfig();
        }
    }

    void rebuildDeviceLabels(const std::string& e100Label) {
        const int keepId = std::clamp(deviceId, 0, kDeviceCount - 1);
        devices.clear();
        for (int i = 0; i < kDeviceCount; i++) {
            std::string shown = kDeviceSpecs[i].name;
            if (i == 0 && !e100Label.empty()) {
                shown = e100Label;
            }
            devices.define(i, shown, std::string(kDeviceSpecs[i].product));
        }
        deviceId = keepId;
    }

    void refreshFromDiscovery() {
        const auto found = sdr::api::iqtaxi_udp_discover(250);
        std::string e100Label = "E100";
        bool matched = false;
        for (const auto& info : found) {
            flog::info("IQTAXI: found {0} serial={1} version={2} at {3}",
                       info.name, info.serial, info.board_version, info.addr);
            if (info.name == "E100") {
                e100Label = sdr::api::iqtaxi_model_label(info.name, info.board_version);
                const std::string band = sdr::api::e100_rf_band_from_text(info.board_version);
                if (!band.empty()) {
                    rfBand = band;
                    detectedMaxFreq = (band == "10G") ? 10e9 : 6e9;
                }
            }
            if (!matched && info.name == currentSpec().product) {
                std::strncpy(host, info.addr.c_str(), sizeof(host) - 1);
                host[sizeof(host) - 1] = '\0';
                matched = true;
            }
        }
        rebuildDeviceLabels(e100Label);
        if (matched) {
            saveConfig();
        }
        if (found.empty()) {
            flog::warn("IQTAXI: no UDP discovery response");
        }
    }

    void saveConfig() {
        config.acquire();
        config.conf["device"] = currentSpec().name;
        config.conf["host"] = std::string(host);
        config.conf["sampleRate"] = sampleRate;
        config.conf["gain"] = gain;
        config.release(true);
    }

    static void menuSelected(void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        core::setInputSampleRate(self->sampleRate);
        flog::info("IqtaxiSourceModule '{0}': Menu Select {1}",
                   self->name, self->currentSpec().name);
    }

    static void menuDeselected(void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        flog::info("IqtaxiSourceModule '{0}': Menu Deselect!", self->name);
    }

    static void start(void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        if (self->running.load()) {
            return;
        }

        try {
            const std::string deviceName = self->currentSpec().product;
            auto device = sdr::api::Device::makeDevice(deviceName, self->host);
            if (!device) {
                flog::error("IQTAXI: failed to open {0} at {1}", deviceName, self->host);
                return;
            }

            device->set_channel_enable(1u);
            device->set_dma_mode(0u);
            device->setSampleRate(self->sampleRate);
            self->detectedMaxFreq = self->currentSpec().max_freq_hz;
            const auto& opened_profile = device->get_profile();
            if (opened_profile.rx_frequency_hz.maximum > 1.0) {
                self->detectedMaxFreq = opened_profile.rx_frequency_hz.maximum;
            }
            self->rfBand = opened_profile.rf_band;
            if (self->rfBand.empty() && opened_profile.product == "E100") {
                self->rfBand = sdr::api::e100_rf_band_from_max_hz(
                    opened_profile.rx_frequency_hz.maximum);
            }
            if (opened_profile.product == "E100" && !self->rfBand.empty()) {
                self->rebuildDeviceLabels("E100-" + self->rfBand);
            }
            self->freq = std::clamp(self->freq, 1.0, self->detectedMaxFreq);
            device->set_rx_freq(static_cast<uint64_t>(self->freq + 0.5), 1);
            device->set_rx_gain(static_cast<uint32_t>(self->gain + 0.5f), 1);

            auto rx = device->get_rx_stream();
            if (!rx) {
                flog::error("IQTAXI: failed to create RX stream");
                self->cleanupDevice();
                return;
            }

            rx->set_rx_mode(kStreamMode);
            rx->set_max_sample_nums_per_packet(kMaxPacketSamples);

            uint64_t timestamp = 0;
            rx->set_recv_param(kStreamMode, kRecvChunkSamples, timestamp, 1, 0);

            {
                std::lock_guard<std::mutex> lock(self->deviceMutex);
                self->device = std::move(device);
                self->rxStream = std::move(rx);
            }

            self->running.store(true);
            self->workerThread = std::thread(&IqtaxiSourceModule::worker, self);
            flog::info("IqtaxiSourceModule '{0}': Start {1}{2}@{3} {4} Hz gain {5}",
                       self->name,
                       self->currentSpec().name,
                       self->rfBand.empty() ? std::string() : ("-" + self->rfBand),
                       self->host,
                       self->sampleRate,
                       self->gain);
        }
        catch (const std::exception& ex) {
            flog::error("IQTAXI start failed: {0}", ex.what());
            self->cleanupDevice();
            self->running.store(false);
        }
    }

    static void stop(void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        if (!self->running.exchange(false)) {
            return;
        }

        self->stream.stopWriter();
        if (self->workerThread.joinable()) {
            self->workerThread.join();
        }
        self->stream.clearWriteStop();

        try {
            std::lock_guard<std::mutex> lock(self->deviceMutex);
            if (self->rxStream) {
                uint64_t stopTimestamp = 0;
                self->rxStream->set_recv_param(
                    kStreamMode, kRecvChunkSamples, stopTimestamp, 0, 1);
                self->rxStream->set_rx_mode_exit();
            }
        }
        catch (const std::exception& ex) {
            flog::error("IQTAXI stop stream failed: {0}", ex.what());
        }

        self->cleanupDevice();
        flog::info("IqtaxiSourceModule '{0}': Stop!", self->name);
    }

    static void tune(double freq, void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        const double maxHz = (self->detectedMaxFreq > 1.0)
            ? self->detectedMaxFreq
            : self->currentSpec().max_freq_hz;
        self->freq = std::clamp(freq, 1.0, maxHz);
        if (!self->running.load()) {
            return;
        }

        try {
            std::lock_guard<std::mutex> lock(self->deviceMutex);
            if (self->device) {
                self->device->set_rx_freq(static_cast<uint64_t>(self->freq + 0.5), 1);
            }
        }
        catch (const std::exception& ex) {
            flog::error("IQTAXI tune failed: {0}", ex.what());
        }
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<IqtaxiSourceModule*>(ctx);
        const bool running = self->running.load();

        if (running) {
            SmGui::BeginDisabled();
        }

        SmGui::LeftLabel("Device");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_iqtaxi_dev_", self->name), &self->deviceId, self->devices.txt)) {
            self->applyDeviceCaps(true);
            core::setInputSampleRate(self->sampleRate);
        }

        SmGui::LeftLabel("Host IP");
        SmGui::FillWidth();
        if (SmGui::InputText(CONCAT("##_iqtaxi_host_", self->name), self->host, sizeof(self->host))) {
            self->saveConfig();
        }

        if (SmGui::Button(CONCAT("Scan##_iqtaxi_scan_", self->name))) {
            self->refreshFromDiscovery();
        }

        SmGui::LeftLabel("Sample Rate");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_iqtaxi_sr_", self->name), &self->srId, self->samplerates.txt)) {
            self->sampleRate = self->samplerates[self->srId];
            core::setInputSampleRate(self->sampleRate);
            self->saveConfig();
        }

        if (running) {
            SmGui::EndDisabled();
        }

        SmGui::LeftLabel("RX Gain");
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_iqtaxi_gain_", self->name),
                                        &self->gain,
                                        self->gainMin(),
                                        self->gainMax(),
                                        1.0f,
                                        SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
            self->gain = std::clamp(self->gain, self->gainMin(), self->gainMax());
            if (self->running.load()) {
                try {
                    std::lock_guard<std::mutex> lock(self->deviceMutex);
                    if (self->device) {
                        self->device->set_rx_gain(static_cast<uint32_t>(self->gain + 0.5f), 1);
                    }
                }
                catch (const std::exception& ex) {
                    flog::error("IQTAXI set gain failed: {0}", ex.what());
                }
            }
            self->saveConfig();
        }
    }

    void worker() {
        std::vector<float> scratch(kRecvChunkSamples * 2u);
        std::vector<void*> buffs{scratch.data()};

        while (running.load()) {
            sdr::api::rx_streamer::sptr rx;
            {
                std::lock_guard<std::mutex> lock(deviceMutex);
                rx = rxStream;
            }
            if (!rx) {
                break;
            }

            uint64_t timestamp = 0;
            size_t received = 0;
            try {
                received = rx->recv(buffs, kRecvChunkSamples, timestamp, MICRORF_FORMAT_FLOAT32);
            }
            catch (const std::exception& ex) {
                flog::error("IQTAXI recv failed: {0}", ex.what());
                break;
            }

            if (received == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            std::memcpy(stream.writeBuf, scratch.data(), received * sizeof(dsp::complex_t));
            if (!stream.swap(static_cast<int>(received))) {
                break;
            }
        }
    }

    void cleanupDevice() {
        std::lock_guard<std::mutex> lock(deviceMutex);
        rxStream.reset();
        device.reset();
        rfBand.clear();
        detectedMaxFreq = 0.0;
    }

    std::string name;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;

    OptionList<int, std::string> devices;
    OptionList<double, double> samplerates;
    int deviceId = 2;
    int srId = 0;
    double sampleRate = 15360000.0;
    double freq = 100e6;
    float gain = 30.0f;
    char host[256] = {};

    std::atomic<bool> running{false};
    std::thread workerThread;
    std::mutex deviceMutex;
    sdr::api::Device::sptr device;
    sdr::api::rx_streamer::sptr rxStream;
    std::string rfBand;
    double detectedMaxFreq = 0.0;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["host"] = "192.168.1.10";
    def["device"] = "E206";
    def["sampleRate"] = 15360000.0;
    def["gain"] = 30.0;
    config.setPath(core::args["root"].s() + "/iqtaxi_source_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new IqtaxiSourceModule(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete static_cast<IqtaxiSourceModule*>(instance);
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
