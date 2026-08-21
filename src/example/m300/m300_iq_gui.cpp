#define GLFW_INCLUDE_NONE

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "src/driver/M300/m300_ad9361_ctrl.hpp"
#include "src/driver/M300/m300_rx_streamer.hpp"
#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;
using namespace sdr::driver;

namespace {

constexpr std::size_t kFftSize = 1024u;
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kAd9361RxBase = 0x44a00000u;
constexpr uint8_t kAd9361ProductId = 0x0au;
constexpr uint8_t kAd9361FddState = 0x16u;
constexpr uint8_t kAd9361RxTxState = 0x1au;
constexpr uint8_t kAd9361VcoLockMask = 0x02u;

struct GuiConfig {
    char base[128] = "/dev/xdma0";
    double center_mhz = 2400.0;
    double sample_rate_msps = 61.44;
    double bandwidth_mhz = 0.0;
    int rx_gain_db = 20;
    int channel_mask = 0x3;
    int display_channel = 0;
    int request_samples = 4092;
    bool auto_init_ad9361 = true;
};

struct RuntimeStats {
    uint64_t reads = 0;
    uint64_t zero_reads = 0;
    uint64_t short_reads = 0;
    uint64_t words = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    uint64_t timestamp_gaps = 0;
    uint64_t packets = 0;
    uint64_t seq_jumps = 0;
    uint64_t lost_packets = 0;
    double elapsed_sec = 0.0;
    double aggregate_msps = 0.0;
    int16_t peak_abs = 0;
    bool have_timestamp = false;
};

struct PreviewState {
    std::atomic<bool> running{false};
    std::atomic<bool> stop_requested{false};
    std::atomic<bool> reconfigure_requested{false};
    std::thread worker;

    std::mutex config_mutex;
    GuiConfig pending_config;
    std::atomic<int> display_channel{0};

    std::mutex message_mutex;
    std::string message = "Idle";

    std::mutex spectrum_mutex;
    std::vector<float> spectrum_db = std::vector<float>(kFftSize, -120.0f);
    float peak_db = -120.0f;
    float avg_db = -120.0f;
    std::atomic<uint64_t> spectrum_generation{0};

    std::mutex stats_mutex;
    RuntimeStats stats;

    ~PreviewState()
    {
        stop_requested.store(true);
        if (worker.joinable()) {
            worker.join();
        }
    }
};

struct SpectrumDisplayState {
    static constexpr int kWaterfallRows = 280;

    GLuint waterfall_texture = 0;
    std::vector<float> smoothed_db = std::vector<float>(kFftSize, -120.0f);
    std::vector<float> peak_hold_db = std::vector<float>(kFftSize, -120.0f);
    std::vector<float> waterfall_db =
        std::vector<float>(kFftSize * static_cast<std::size_t>(kWaterfallRows), -120.0f);
    std::vector<uint8_t> waterfall_rgba =
        std::vector<uint8_t>(kFftSize * static_cast<std::size_t>(kWaterfallRows) * 4u, 0u);
    uint64_t last_generation = 0;
    uint64_t line_count = 0;
    int smooth_frames = 2;
    float min_db = -120.0f;
    float max_db = -20.0f;
    float rendered_min_db = 0.0f;
    float rendered_max_db = 0.0f;
    double displayed_center_mhz = 0.0;
    double displayed_sample_rate_msps = 0.0;
    bool have_smoothed = false;
    bool texture_dirty = true;
    bool show_peak_hold = true;

    void clear()
    {
        std::fill(smoothed_db.begin(), smoothed_db.end(), min_db);
        std::fill(peak_hold_db.begin(), peak_hold_db.end(), min_db);
        std::fill(waterfall_db.begin(), waterfall_db.end(), min_db);
        have_smoothed = false;
        line_count = 0;
        texture_dirty = true;
    }

    void shutdown()
    {
        if (waterfall_texture != 0) {
            glDeleteTextures(1, &waterfall_texture);
            waterfall_texture = 0;
        }
    }
};

struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

int enabled_channel_count(int mask)
{
    int count = 0;
    for (int bits = mask & 0xff; bits != 0; bits >>= 1) {
        count += bits & 0x1;
    }
    return std::max(count, 1);
}

void set_message(PreviewState& state, const std::string& message)
{
    std::lock_guard<std::mutex> lock(state.message_mutex);
    state.message = message;
}

std::string get_message(PreviewState& state)
{
    std::lock_guard<std::mutex> lock(state.message_mutex);
    return state.message;
}

RuntimeStats get_stats(PreviewState& state)
{
    std::lock_guard<std::mutex> lock(state.stats_mutex);
    return state.stats;
}

GuiConfig sanitized_config(GuiConfig cfg)
{
    cfg.center_mhz = std::clamp(cfg.center_mhz, 70.0, 6000.0);
    cfg.sample_rate_msps = std::clamp(cfg.sample_rate_msps, 2.083333, 61.44);
    if (cfg.bandwidth_mhz != 0.0) {
        cfg.bandwidth_mhz = std::clamp(cfg.bandwidth_mhz, 0.2, 56.0);
    }
    cfg.rx_gain_db = std::clamp(cfg.rx_gain_db, 0, 71);
    cfg.channel_mask = std::clamp(cfg.channel_mask, 1, 3);
    cfg.display_channel = std::clamp(cfg.display_channel, 0, enabled_channel_count(cfg.channel_mask) - 1);
    cfg.request_samples = std::clamp(cfg.request_samples, 1024, 65536);
    return cfg;
}

std::string format_rate(double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << value;
    return oss.str();
}

void set_double_text(char* text, std::size_t text_size, double value, int precision)
{
    std::snprintf(text, text_size, "%.*f", precision, value);
}

void set_int_text(char* text, std::size_t text_size, int value)
{
    std::snprintf(text, text_size, "%d", value);
}

bool parse_double_text(const char* text, double& value)
{
    try {
        std::size_t used = 0;
        const double parsed = std::stod(text, &used);
        if (used == 0) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_int_text(const char* text, int& value)
{
    try {
        std::size_t used = 0;
        const long parsed = std::stol(text, &used, 0);
        if (used == 0 ||
            parsed < static_cast<long>(std::numeric_limits<int>::min()) ||
            parsed > static_cast<long>(std::numeric_limits<int>::max())) {
            return false;
        }
        value = static_cast<int>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool ad9361_looks_initialized(const std::shared_ptr<m300_xdma_ctrl>& ctrl)
{
    try {
        const uint8_t product_id = ctrl->ad9361_spi_read(0x037u, 1.0);
        const uint8_t ensm_state = ctrl->ad9361_spi_read(0x017u, 1.0);
        const uint8_t rx_vco_lock = ctrl->ad9361_spi_read(0x247u, 1.0);
        const uint8_t tx_vco_lock = ctrl->ad9361_spi_read(0x287u, 1.0);
        const uint32_t rx_rstn = ctrl->read_axi(kAd9361RxBase + 0x0040u, 1.0);
        const uint32_t rx_clk_count = ctrl->read_axi(kAd9361RxBase + 0x0054u, 1.0);
        const uint32_t rx_status = ctrl->read_axi(kAd9361RxBase + 0x005cu, 1.0);
        const bool ensm_ok = (ensm_state == kAd9361FddState) ||
                             (ensm_state == kAd9361RxTxState);

        return product_id == kAd9361ProductId &&
               ensm_ok &&
               ((rx_vco_lock & kAd9361VcoLockMask) != 0u) &&
               ((tx_vco_lock & kAd9361VcoLockMask) != 0u) &&
               ((rx_rstn & 0x3u) == 0x3u) &&
               rx_clk_count != 0u &&
               rx_status != 0u;
    } catch (...) {
        return false;
    }
}

m300_ad9361_init_options make_ad9361_options(const GuiConfig& cfg)
{
    m300_ad9361_init_options options;
    options.sample_rate_hz = static_cast<uint32_t>(std::llround(cfg.sample_rate_msps * 1.0e6));
    options.bandwidth_hz = static_cast<uint32_t>(std::llround(cfg.bandwidth_mhz * 1.0e6));
    options.rx_lo_hz = static_cast<uint64_t>(std::llround(cfg.center_mhz * 1.0e6));
    options.tx_lo_hz = options.rx_lo_hz;
    options.rx_gain_db = static_cast<uint32_t>(cfg.rx_gain_db);
    return options;
}

void configure_ad9361(m300_ad9361_ctrl& ad9361, const GuiConfig& cfg, bool& context_ready)
{
    if (!context_ready) {
        ad9361.init(make_ad9361_options(cfg));
        context_ready = true;
        return;
    }

    const uint32_t sample_rate_hz = static_cast<uint32_t>(std::llround(cfg.sample_rate_msps * 1.0e6));
    const uint64_t center_hz = static_cast<uint64_t>(std::llround(cfg.center_mhz * 1.0e6));
    const uint32_t bandwidth_hz = static_cast<uint32_t>(std::llround(cfg.bandwidth_mhz * 1.0e6));
    ad9361.set_bandwidth(bandwidth_hz);
    ad9361.set_sample_rate(sample_rate_hz);
    ad9361.set_rx_freq(center_hz, 1);
    ad9361.set_rx_freq(center_hz, 2);
    ad9361.set_rx_gain(static_cast<uint32_t>(cfg.rx_gain_db), 1);
    ad9361.set_rx_gain(static_cast<uint32_t>(cfg.rx_gain_db), 2);
}

void fft_in_place(std::array<std::complex<float>, kFftSize>& data)
{
    std::size_t j = 0;
    for (std::size_t i = 1; i < kFftSize; ++i) {
        std::size_t bit = kFftSize >> 1u;
        for (; (j & bit) != 0u; bit >>= 1u) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(data[i], data[j]);
        }
    }

    for (std::size_t len = 2; len <= kFftSize; len <<= 1u) {
        const float angle = -2.0f * kPi / static_cast<float>(len);
        const std::complex<float> wlen(std::cos(angle), std::sin(angle));
        for (std::size_t i = 0; i < kFftSize; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (std::size_t k = 0; k < len / 2u; ++k) {
                const std::complex<float> u = data[i + k];
                const std::complex<float> v = data[i + k + len / 2u] * w;
                data[i + k] = u + v;
                data[i + k + len / 2u] = u - v;
                w *= wlen;
            }
        }
    }
}

void update_spectrum(PreviewState& state,
                     const int16_t* iq,
                     std::size_t words,
                     int channel_count,
                     int display_channel)
{
    channel_count = std::max(channel_count, 1);
    display_channel = std::clamp(display_channel, 0, channel_count - 1);
    if (words / static_cast<std::size_t>(channel_count) < kFftSize) {
        return;
    }

    std::array<std::complex<float>, kFftSize> fft = {};
    for (std::size_t i = 0; i < kFftSize; ++i) {
        const std::size_t word = i * static_cast<std::size_t>(channel_count) +
                                 static_cast<std::size_t>(display_channel);
        const float window = 0.5f - 0.5f * std::cos(2.0f * kPi * static_cast<float>(i) /
                                                    static_cast<float>(kFftSize - 1u));
        const float re = static_cast<float>(iq[word * 2u]) / 32768.0f;
        const float im = static_cast<float>(iq[word * 2u + 1u]) / 32768.0f;
        fft[i] = std::complex<float>(re * window, im * window);
    }

    fft_in_place(fft);

    std::vector<float> db(kFftSize);
    float peak = -120.0f;
    float sum = 0.0f;
    for (std::size_t i = 0; i < kFftSize; ++i) {
        const std::size_t src = (i + kFftSize / 2u) & (kFftSize - 1u);
        const float mag = std::abs(fft[src]) / static_cast<float>(kFftSize);
        const float value = std::clamp(20.0f * std::log10(mag + 1.0e-9f), -120.0f, 0.0f);
        db[i] = value;
        peak = std::max(peak, value);
        sum += value;
    }

    {
        std::lock_guard<std::mutex> lock(state.spectrum_mutex);
        state.spectrum_db = std::move(db);
        state.peak_db = peak;
        state.avg_db = sum / static_cast<float>(kFftSize);
        state.spectrum_generation.fetch_add(1u, std::memory_order_release);
    }
}

class RxStreamGuard {
public:
    RxStreamGuard(const rx_streamer::sptr& rx, int request_samples)
        : _rx(rx)
        , _request_samples(request_samples)
    {
    }

    ~RxStreamGuard()
    {
        stop();
    }

    void start()
    {
        if (!_rx || _started) {
            return;
        }
        uint64_t timestamp = 0;
        _rx->set_recv_param(STREAM_MODE, static_cast<size_t>(_request_samples), timestamp, 1, 0);
        _started = true;
    }

    void stop()
    {
        if (!_rx || !_started) {
            return;
        }
        uint64_t timestamp = 0;
        try {
            _rx->set_recv_param(STREAM_MODE, static_cast<size_t>(_request_samples), timestamp, 0, 1);
            _rx->set_rx_mode_exit();
        } catch (...) {
        }
        _started = false;
    }

private:
    rx_streamer::sptr _rx;
    int _request_samples = 0;
    bool _started = false;
};

void rx_worker(PreviewState& state, GuiConfig start_cfg)
{
    RuntimeStats stats;
    auto start_time = std::chrono::steady_clock::now();

    try {
        GuiConfig cfg = sanitized_config(start_cfg);
        set_message(state, "Opening M300");
        auto dev = std::make_shared<M300XdmaImpl>(cfg.base);
        if (!dev->isInitialSuccess()) {
            throw std::runtime_error("M300 XDMA open failed: " + dev->last_error());
        }

        auto ctrl = dev->get_ctrl();
        auto version = ctrl->get_version(1.0);
        {
            std::ostringstream oss;
            oss << "M300 opened, version=0x" << std::hex << version.pkt.value0
                << " build=0x" << version.pkt.value1;
            set_message(state, oss.str());
        }

        m300_ad9361_ctrl ad9361(ctrl);
        bool ad9361_context_ready = false;
        if (cfg.auto_init_ad9361) {
            set_message(state, ad9361_looks_initialized(ctrl) ?
                        "AD9361 configure begin" : "AD9361 init begin");
            configure_ad9361(ad9361, cfg, ad9361_context_ready);
            set_message(state, "AD9361 configured");
        }

        auto rx = dev->get_rx_stream();
        if (!rx) {
            throw std::runtime_error("M300 RX stream is not available");
        }
        rx->set_sampleRate(static_cast<size_t>(std::llround(cfg.sample_rate_msps * 1.0e6)));
        rx->set_rx_enable_chan(static_cast<uint8_t>(cfg.channel_mask));
        rx->set_rx_mode(STREAM_MODE);

        auto stream_guard = std::make_unique<RxStreamGuard>(rx, cfg.request_samples);
        stream_guard->start();
        set_message(state, "Streaming");

        std::vector<int16_t> buffer(static_cast<std::size_t>(cfg.request_samples) * 2u);
        std::vector<void*> buffs{buffer.data()};
        uint64_t previous_timestamp = 0;
        uint64_t previous_words = 0;
        int previous_channels = enabled_channel_count(cfg.channel_mask);

        while (!state.stop_requested.load()) {
            if (state.reconfigure_requested.exchange(false)) {
                {
                    std::lock_guard<std::mutex> lock(state.config_mutex);
                    cfg = sanitized_config(state.pending_config);
                }
                stream_guard->stop();
                set_message(state, "Applying settings");
                if (cfg.auto_init_ad9361) {
                    configure_ad9361(ad9361, cfg, ad9361_context_ready);
                }
                rx->set_sampleRate(static_cast<size_t>(std::llround(cfg.sample_rate_msps * 1.0e6)));
                rx->set_rx_enable_chan(static_cast<uint8_t>(cfg.channel_mask));
                previous_timestamp = 0;
                previous_words = 0;
                previous_channels = enabled_channel_count(cfg.channel_mask);
                buffer.assign(static_cast<std::size_t>(cfg.request_samples) * 2u, 0);
                buffs[0] = buffer.data();
                stream_guard = std::make_unique<RxStreamGuard>(rx, cfg.request_samples);
                stream_guard->start();
                set_message(state, "Streaming");
            }

            uint64_t timestamp = 0;
            const size_t got = rx->recv(rx_streamer::buffs_type(buffs),
                                        static_cast<size_t>(cfg.request_samples),
                                        timestamp,
                                        MICRORF_FORMAT_INT16);
            stats.reads++;
            if (got == 0u) {
                stats.zero_reads++;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (got != static_cast<size_t>(cfg.request_samples)) {
                stats.short_reads++;
            }

            stats.words += got;
            for (std::size_t i = 0; i < got * 2u; ++i) {
                const int value = std::abs(static_cast<int>(buffer[i]));
                stats.peak_abs = std::max<int16_t>(
                    stats.peak_abs, static_cast<int16_t>(std::min(value, 32767)));
            }

            if (!stats.have_timestamp) {
                stats.have_timestamp = true;
                stats.first_timestamp = timestamp;
            } else {
                const uint64_t expected = previous_timestamp +
                    previous_words / static_cast<uint64_t>(std::max(previous_channels, 1));
                if (timestamp != expected) {
                    stats.timestamp_gaps++;
                }
            }
            stats.last_timestamp = timestamp;
            previous_timestamp = timestamp;
            previous_words = got;
            previous_channels = enabled_channel_count(cfg.channel_mask);

            update_spectrum(state,
                            buffer.data(),
                            got,
                            previous_channels,
                            state.display_channel.load());

            if (auto m300_rx = std::dynamic_pointer_cast<m300_rx_streamer>(rx)) {
                const auto continuity = m300_rx->get_continuity_stats();
                stats.packets = continuity.packets;
                stats.seq_jumps = continuity.seq_jumps;
                stats.lost_packets = continuity.lost_packets;
            }
            const auto now = std::chrono::steady_clock::now();
            stats.elapsed_sec = std::chrono::duration<double>(now - start_time).count();
            stats.aggregate_msps = stats.elapsed_sec > 0.0 ?
                static_cast<double>(stats.words) / 1.0e6 / stats.elapsed_sec : 0.0;
            {
                std::lock_guard<std::mutex> lock(state.stats_mutex);
                state.stats = stats;
            }
        }

        stream_guard->stop();
        set_message(state, "Stopped");
    } catch (const std::exception& ex) {
        set_message(state, std::string("Error: ") + ex.what());
    }

    state.running.store(false);
}

void start_preview(PreviewState& state, GuiConfig cfg)
{
    if (state.running.load()) {
        return;
    }
    if (state.worker.joinable()) {
        state.worker.join();
    }
    cfg = sanitized_config(cfg);
    {
        std::lock_guard<std::mutex> lock(state.config_mutex);
        state.pending_config = cfg;
    }
    state.display_channel.store(cfg.display_channel);
    {
        std::lock_guard<std::mutex> lock(state.stats_mutex);
        state.stats = RuntimeStats{};
    }
    state.stop_requested.store(false);
    state.reconfigure_requested.store(false);
    state.running.store(true);
    state.worker = std::thread([&state, cfg]() { rx_worker(state, cfg); });
}

void stop_preview(PreviewState& state)
{
    state.stop_requested.store(true);
    if (state.worker.joinable()) {
        state.worker.join();
    }
}

void request_reconfigure(PreviewState& state, GuiConfig cfg)
{
    cfg = sanitized_config(cfg);
    {
        std::lock_guard<std::mutex> lock(state.config_mutex);
        state.pending_config = cfg;
    }
    state.display_channel.store(cfg.display_channel);
    state.reconfigure_requested.store(true);
}

RgbColor spectrum_color(float db, float min_db, float max_db)
{
    struct ColorStop {
        float position;
        RgbColor color;
    };
    static constexpr std::array<ColorStop, 6> stops = {{
        {0.00f, {4, 7, 16}},
        {0.18f, {18, 41, 92}},
        {0.38f, {12, 129, 168}},
        {0.58f, {43, 203, 164}},
        {0.76f, {252, 202, 92}},
        {1.00f, {245, 85, 66}},
    }};

    const float span = std::max(1.0f, max_db - min_db);
    const float value = std::clamp((db - min_db) / span, 0.0f, 1.0f);
    std::size_t upper = 1;
    while (upper + 1u < stops.size() && value > stops[upper].position) {
        ++upper;
    }
    const ColorStop& low = stops[upper - 1u];
    const ColorStop& high = stops[upper];
    const float mix = (value - low.position) / std::max(0.0001f, high.position - low.position);
    auto channel = [mix](uint8_t a, uint8_t b) {
        return static_cast<uint8_t>(std::lround(static_cast<float>(a) +
                                                (static_cast<float>(b) - static_cast<float>(a)) * mix));
    };
    return {channel(low.color.r, high.color.r),
            channel(low.color.g, high.color.g),
            channel(low.color.b, high.color.b)};
}

ImU32 imgui_color(RgbColor color, uint8_t alpha = 255)
{
    return IM_COL32(color.r, color.g, color.b, alpha);
}

void update_waterfall_texture(SpectrumDisplayState& display)
{
    if (display.waterfall_texture == 0) {
        glGenTextures(1, &display.waterfall_texture);
        glBindTexture(GL_TEXTURE_2D, display.waterfall_texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, static_cast<GLsizei>(kFftSize),
                     SpectrumDisplayState::kWaterfallRows, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        display.texture_dirty = true;
    }

    if (!display.texture_dirty &&
        display.rendered_min_db == display.min_db &&
        display.rendered_max_db == display.max_db) {
        return;
    }

    const std::size_t pixels = display.waterfall_db.size();
    for (std::size_t i = 0; i < pixels; ++i) {
        const RgbColor color = spectrum_color(display.waterfall_db[i], display.min_db, display.max_db);
        const std::size_t offset = i * 4u;
        display.waterfall_rgba[offset] = color.r;
        display.waterfall_rgba[offset + 1u] = color.g;
        display.waterfall_rgba[offset + 2u] = color.b;
        display.waterfall_rgba[offset + 3u] = 255u;
    }

    glBindTexture(GL_TEXTURE_2D, display.waterfall_texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(kFftSize),
                    SpectrumDisplayState::kWaterfallRows, GL_RGBA, GL_UNSIGNED_BYTE,
                    display.waterfall_rgba.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    display.rendered_min_db = display.min_db;
    display.rendered_max_db = display.max_db;
    display.texture_dirty = false;
}

void consume_latest_spectrum(const GuiConfig& cfg,
                             PreviewState& preview,
                             SpectrumDisplayState& display)
{
    if (display.displayed_center_mhz != cfg.center_mhz ||
        display.displayed_sample_rate_msps != cfg.sample_rate_msps) {
        display.displayed_center_mhz = cfg.center_mhz;
        display.displayed_sample_rate_msps = cfg.sample_rate_msps;
        display.clear();
    }

    const uint64_t available_generation =
        preview.spectrum_generation.load(std::memory_order_acquire);
    if (available_generation == display.last_generation) {
        return;
    }

    std::vector<float> latest;
    {
        std::lock_guard<std::mutex> lock(preview.spectrum_mutex);
        latest = preview.spectrum_db;
        display.last_generation = preview.spectrum_generation.load(std::memory_order_relaxed);
    }
    if (latest.size() != kFftSize) {
        return;
    }

    if (!display.have_smoothed) {
        display.smoothed_db = latest;
        display.peak_hold_db = latest;
        display.have_smoothed = true;
    } else {
        const float keep = static_cast<float>(display.smooth_frames - 1) /
                           static_cast<float>(display.smooth_frames);
        const float add = 1.0f - keep;
        for (std::size_t i = 0; i < kFftSize; ++i) {
            display.smoothed_db[i] = display.smoothed_db[i] * keep + latest[i] * add;
            display.peak_hold_db[i] =
                std::max(display.smoothed_db[i], display.peak_hold_db[i] - 0.18f);
        }
    }

    const std::size_t row_bytes = kFftSize * sizeof(float);
    std::memmove(display.waterfall_db.data() + kFftSize,
                 display.waterfall_db.data(),
                 row_bytes * static_cast<std::size_t>(SpectrumDisplayState::kWaterfallRows - 1));
    std::memcpy(display.waterfall_db.data(), latest.data(), row_bytes);
    ++display.line_count;
    display.texture_dirty = true;
}

void draw_plot_grid(ImDrawList* draw_list,
                    const ImVec2& plot_min,
                    const ImVec2& plot_max,
                    const GuiConfig& cfg,
                    float min_db,
                    float max_db,
                    bool show_db_axis)
{
    const ImU32 grid = IM_COL32(49, 57, 64, 180);
    const ImU32 label = IM_COL32(159, 172, 181, 255);
    for (int i = 0; i <= 5; ++i) {
        const float ratio = static_cast<float>(i) / 5.0f;
        const float y = plot_min.y + ratio * (plot_max.y - plot_min.y);
        draw_list->AddLine(ImVec2(plot_min.x, y), ImVec2(plot_max.x, y), grid);
        if (show_db_axis) {
            char text[24] = {};
            std::snprintf(text, sizeof(text), "%.0f", max_db - ratio * (max_db - min_db));
            const ImVec2 size = ImGui::CalcTextSize(text);
            draw_list->AddText(ImVec2(plot_min.x - size.x - 8.0f, y - size.y * 0.5f), label, text);
        }
    }

    const double start_mhz = cfg.center_mhz - cfg.sample_rate_msps / 2.0;
    for (int i = 0; i <= 4; ++i) {
        const float ratio = static_cast<float>(i) / 4.0f;
        const float x = plot_min.x + ratio * (plot_max.x - plot_min.x);
        draw_list->AddLine(ImVec2(x, plot_min.y), ImVec2(x, plot_max.y), grid);
        char text[32] = {};
        std::snprintf(text, sizeof(text), "%.3f", start_mhz + cfg.sample_rate_msps * ratio);
        const ImVec2 size = ImGui::CalcTextSize(text);
        const float label_x = std::clamp(x - size.x * 0.5f, plot_min.x, plot_max.x - size.x);
        draw_list->AddText(ImVec2(label_x, plot_max.y + 5.0f), label, text);
    }
}

void draw_spectrum_canvas(const GuiConfig& cfg, SpectrumDisplayState& display, float height)
{
    const float width = std::max(320.0f, ImGui::GetContentRegionAvail().x);
    ImGui::InvisibleButton("##spectrum_canvas", ImVec2(width, height));
    const ImVec2 frame_min = ImGui::GetItemRectMin();
    const ImVec2 frame_max = ImGui::GetItemRectMax();
    const ImVec2 plot_min(frame_min.x + 58.0f, frame_min.y + 8.0f);
    const ImVec2 plot_max(frame_max.x - 58.0f, frame_max.y - 28.0f);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(frame_min, frame_max, IM_COL32(5, 6, 7, 255), 3.0f);
    draw_plot_grid(draw_list, plot_min, plot_max, cfg, display.min_db, display.max_db, true);

    auto point_for = [&](std::size_t index, float db) {
        const float x_ratio = static_cast<float>(index) / static_cast<float>(kFftSize - 1u);
        const float y_ratio = std::clamp((db - display.min_db) /
                                         std::max(1.0f, display.max_db - display.min_db),
                                         0.0f, 1.0f);
        return ImVec2(plot_min.x + x_ratio * (plot_max.x - plot_min.x),
                      plot_max.y - y_ratio * (plot_max.y - plot_min.y));
    };

    if (display.have_smoothed) {
        if (display.show_peak_hold) {
            for (std::size_t i = 1; i < kFftSize; ++i) {
                draw_list->AddLine(point_for(i - 1u, display.peak_hold_db[i - 1u]),
                                   point_for(i, display.peak_hold_db[i]),
                                   IM_COL32(246, 190, 95, 170), 1.0f);
            }
        }
        for (std::size_t i = 1; i < kFftSize; ++i) {
            const float level = (display.smoothed_db[i - 1u] + display.smoothed_db[i]) * 0.5f;
            draw_list->AddLine(point_for(i - 1u, display.smoothed_db[i - 1u]),
                               point_for(i, display.smoothed_db[i]),
                               imgui_color(spectrum_color(level, display.min_db, display.max_db)), 2.0f);
        }
    }
    draw_list->AddRect(plot_min, plot_max, IM_COL32(72, 82, 90, 255));

    if (ImGui::IsItemHovered() && display.have_smoothed && ImGui::IsMousePosValid()) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        if (mouse.x >= plot_min.x && mouse.x <= plot_max.x &&
            mouse.y >= plot_min.y && mouse.y <= plot_max.y) {
            const float ratio = (mouse.x - plot_min.x) / (plot_max.x - plot_min.x);
            const std::size_t bin = std::min<std::size_t>(kFftSize - 1u,
                static_cast<std::size_t>(std::lround(ratio * static_cast<float>(kFftSize - 1u))));
            const double frequency = cfg.center_mhz - cfg.sample_rate_msps / 2.0 +
                                     cfg.sample_rate_msps * static_cast<double>(ratio);
            draw_list->AddLine(ImVec2(mouse.x, plot_min.y), ImVec2(mouse.x, plot_max.y),
                               IM_COL32(238, 243, 246, 120));
            ImGui::BeginTooltip();
            ImGui::Text("%.6f MHz", frequency);
            ImGui::Text("%.1f dBFS", display.smoothed_db[bin]);
            ImGui::EndTooltip();
        }
    }
}

void draw_waterfall_canvas(const GuiConfig& cfg, SpectrumDisplayState& display, float height)
{
    update_waterfall_texture(display);
    const float width = std::max(320.0f, ImGui::GetContentRegionAvail().x);
    ImGui::InvisibleButton("##waterfall_canvas", ImVec2(width, height));
    const ImVec2 frame_min = ImGui::GetItemRectMin();
    const ImVec2 frame_max = ImGui::GetItemRectMax();
    const ImVec2 plot_min(frame_min.x + 58.0f, frame_min.y + 8.0f);
    const ImVec2 plot_max(frame_max.x - 58.0f, frame_max.y - 28.0f);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(frame_min, frame_max, IM_COL32(5, 6, 7, 255), 3.0f);
    draw_list->AddImage(ImTextureRef(static_cast<ImTextureID>(display.waterfall_texture)),
                        plot_min, plot_max);
    draw_plot_grid(draw_list, plot_min, plot_max, cfg, display.min_db, display.max_db, false);
    draw_list->AddRect(plot_min, plot_max, IM_COL32(72, 82, 90, 255));

    const float legend_left = plot_max.x + 14.0f;
    const float legend_right = legend_left + 12.0f;
    constexpr int kLegendSteps = 48;
    for (int i = 0; i < kLegendSteps; ++i) {
        const float top = plot_min.y + (plot_max.y - plot_min.y) * static_cast<float>(i) /
                                       static_cast<float>(kLegendSteps);
        const float bottom = plot_min.y + (plot_max.y - plot_min.y) * static_cast<float>(i + 1) /
                                          static_cast<float>(kLegendSteps);
        const float db = display.max_db - (display.max_db - display.min_db) *
                                          static_cast<float>(i) / static_cast<float>(kLegendSteps - 1);
        draw_list->AddRectFilled(ImVec2(legend_left, top), ImVec2(legend_right, bottom),
                                 imgui_color(spectrum_color(db, display.min_db, display.max_db)));
    }
    char maximum[16] = {};
    char minimum[16] = {};
    std::snprintf(maximum, sizeof(maximum), "%.0f", display.max_db);
    std::snprintf(minimum, sizeof(minimum), "%.0f", display.min_db);
    draw_list->AddText(ImVec2(legend_right + 4.0f, plot_min.y - 2.0f),
                       IM_COL32(159, 172, 181, 255), maximum);
    draw_list->AddText(ImVec2(legend_right + 4.0f, plot_max.y - ImGui::GetTextLineHeight()),
                       IM_COL32(159, 172, 181, 255), minimum);
}

void draw_spectrum_display(const GuiConfig& cfg,
                           PreviewState& preview,
                           SpectrumDisplayState& display)
{
    ImGui::SeparatorText("Spectrum and Waterfall");
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::DragFloat("Floor", &display.min_db, 1.0f, -160.0f, -30.0f, "%.0f dB")) {
        display.min_db = std::min(display.min_db, display.max_db - 10.0f);
        display.texture_dirty = true;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::DragFloat("Ceiling", &display.max_db, 1.0f, -100.0f, 10.0f, "%.0f dB")) {
        display.max_db = std::max(display.max_db, display.min_db + 10.0f);
        display.texture_dirty = true;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.0f);
    ImGui::SliderInt("Smooth", &display.smooth_frames, 1, 20);
    ImGui::SameLine();
    ImGui::Checkbox("Peak hold", &display.show_peak_hold);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        display.clear();
    }

    consume_latest_spectrum(cfg, preview, display);
    float peak = display.min_db;
    float average = display.min_db;
    if (display.have_smoothed) {
        peak = *std::max_element(display.smoothed_db.begin(), display.smoothed_db.end());
        float total = 0.0f;
        for (float value : display.smoothed_db) {
            total += value;
        }
        average = total / static_cast<float>(display.smoothed_db.size());
    }
    ImGui::TextColored(preview.running.load() ? ImVec4(0.36f, 0.84f, 0.71f, 1.0f)
                                              : ImVec4(0.62f, 0.67f, 0.70f, 1.0f),
                       "%s", preview.running.load() ? "LIVE" : "IDLE");
    ImGui::SameLine();
    ImGui::Text("Peak %.1f dBFS   Average %.1f dBFS   Span %.2f MHz   %llu lines",
                peak, average, cfg.sample_rate_msps,
                static_cast<unsigned long long>(display.line_count));

    ImGui::TextUnformatted("SPECTRUM");
    draw_spectrum_canvas(cfg, display, 220.0f);
    ImGui::TextUnformatted("WATERFALL");
    draw_waterfall_canvas(cfg, display, 290.0f);
}

void apply_style()
{
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ItemSpacing = ImVec2(10.0f, 8.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.10f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.19f, 0.29f, 0.28f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.24f, 0.36f, 0.35f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.29f, 0.44f, 0.42f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.20f, 0.32f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.43f, 0.40f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.34f, 0.52f, 0.48f, 1.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.20f, 0.21f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.22f, 0.25f, 0.26f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.47f, 0.78f, 0.70f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.47f, 0.78f, 0.70f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.62f, 0.88f, 0.78f, 1.00f);
}

} // namespace

int main()
{
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1280, 900, "M300 IQ Monitor", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "glfwCreateWindow failed\n");
        glfwTerminate();
        return 1;
    }

    glfwSetWindowSizeLimits(window, 980, 720, GLFW_DONT_CARE, GLFW_DONT_CARE);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    if (glewInit() != GLEW_OK) {
        std::fprintf(stderr, "glewInit failed\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    apply_style();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    GuiConfig cfg;
    PreviewState preview;
    SpectrumDisplayState spectrum_display;
    char center_mhz_text[64] = {};
    char sample_rate_msps_text[64] = {};
    char bandwidth_mhz_text[64] = {};
    char channel_mask_text[32] = {};
    char request_samples_text[32] = {};
    set_double_text(center_mhz_text, sizeof(center_mhz_text), cfg.center_mhz, 6);
    set_double_text(sample_rate_msps_text, sizeof(sample_rate_msps_text), cfg.sample_rate_msps, 6);
    set_double_text(bandwidth_mhz_text, sizeof(bandwidth_mhz_text), cfg.bandwidth_mhz, 3);
    set_int_text(channel_mask_text, sizeof(channel_mask_text), cfg.channel_mask);
    set_int_text(request_samples_text, sizeof(request_samples_text), cfg.request_samples);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const bool running = preview.running.load();
        cfg = sanitized_config(cfg);
        const RuntimeStats stats = get_stats(preview);
        int channel_count = enabled_channel_count(cfg.channel_mask);
        const double per_channel_msps = stats.aggregate_msps / static_cast<double>(std::max(channel_count, 1));
        bool restart_device_requested = false;
        bool hardware_reconfigure_requested = false;
        bool display_channel_changed = false;
        constexpr ImGuiInputTextFlags kEnterFlags = ImGuiInputTextFlags_EnterReturnsTrue;
        auto sync_config_from_text_fields = [&]() {
            double parsed_double = 0.0;
            int parsed_int = 0;
            if (parse_double_text(center_mhz_text, parsed_double)) {
                cfg.center_mhz = parsed_double;
            }
            if (parse_double_text(sample_rate_msps_text, parsed_double)) {
                cfg.sample_rate_msps = parsed_double;
            }
            if (parse_double_text(bandwidth_mhz_text, parsed_double)) {
                cfg.bandwidth_mhz = parsed_double;
            }
            if (parse_int_text(channel_mask_text, parsed_int)) {
                cfg.channel_mask = parsed_int;
            }
            if (parse_int_text(request_samples_text, parsed_int)) {
                cfg.request_samples = parsed_int;
            }
            cfg = sanitized_config(cfg);
            set_double_text(center_mhz_text, sizeof(center_mhz_text), cfg.center_mhz, 6);
            set_double_text(sample_rate_msps_text, sizeof(sample_rate_msps_text), cfg.sample_rate_msps, 6);
            set_double_text(bandwidth_mhz_text, sizeof(bandwidth_mhz_text), cfg.bandwidth_mhz, 3);
            set_int_text(channel_mask_text, sizeof(channel_mask_text), cfg.channel_mask);
            set_int_text(request_samples_text, sizeof(request_samples_text), cfg.request_samples);
        };

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        ImGui::Begin("M300 IQ Monitor", nullptr,
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove);

        ImGui::SeparatorText("Device and RX");
        if (ImGui::BeginTable("m300_config", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            const std::string old_base = cfg.base;
            if (ImGui::InputText("XDMA Base", cfg.base, sizeof(cfg.base), kEnterFlags) &&
                old_base != cfg.base) {
                restart_device_requested = running;
            }
            ImGui::TableNextColumn();
            if (ImGui::InputText("Center Frequency (MHz)", center_mhz_text, sizeof(center_mhz_text), kEnterFlags)) {
                double parsed = 0.0;
                if (parse_double_text(center_mhz_text, parsed)) {
                    cfg.center_mhz = parsed;
                    cfg = sanitized_config(cfg);
                    hardware_reconfigure_requested = running;
                }
                set_double_text(center_mhz_text, sizeof(center_mhz_text), cfg.center_mhz, 6);
            }
            ImGui::TableNextColumn();
            if (ImGui::InputText("Sample Rate (MSPS)", sample_rate_msps_text, sizeof(sample_rate_msps_text), kEnterFlags)) {
                double parsed = 0.0;
                if (parse_double_text(sample_rate_msps_text, parsed)) {
                    cfg.sample_rate_msps = parsed;
                    cfg = sanitized_config(cfg);
                    hardware_reconfigure_requested = running;
                }
                set_double_text(sample_rate_msps_text, sizeof(sample_rate_msps_text), cfg.sample_rate_msps, 6);
            }
            ImGui::TableNextColumn();
            if (ImGui::InputText("RF Bandwidth (MHz)", bandwidth_mhz_text, sizeof(bandwidth_mhz_text), kEnterFlags)) {
                double parsed = 0.0;
                if (parse_double_text(bandwidth_mhz_text, parsed)) {
                    cfg.bandwidth_mhz = parsed;
                    cfg = sanitized_config(cfg);
                    hardware_reconfigure_requested = running;
                }
                set_double_text(bandwidth_mhz_text, sizeof(bandwidth_mhz_text), cfg.bandwidth_mhz, 3);
            }
            ImGui::TableNextColumn();
            hardware_reconfigure_requested |= ImGui::SliderInt("RX Gain (dB)", &cfg.rx_gain_db, 0, 71);
            ImGui::TableNextColumn();
            if (ImGui::InputText("Channel Mask", channel_mask_text, sizeof(channel_mask_text), kEnterFlags)) {
                int parsed = 0;
                if (parse_int_text(channel_mask_text, parsed)) {
                    cfg.channel_mask = parsed;
                    cfg = sanitized_config(cfg);
                    hardware_reconfigure_requested = running;
                }
                set_int_text(channel_mask_text, sizeof(channel_mask_text), cfg.channel_mask);
            }
            ImGui::TableNextColumn();
            cfg.channel_mask = std::clamp(cfg.channel_mask, 1, 3);
            channel_count = enabled_channel_count(cfg.channel_mask);
            cfg.display_channel = std::clamp(cfg.display_channel, 0, std::max(channel_count - 1, 0));
            display_channel_changed =
                ImGui::SliderInt("Display Channel", &cfg.display_channel, 0, std::max(channel_count - 1, 0));
            ImGui::TableNextColumn();
            if (ImGui::InputText("Read Words", request_samples_text, sizeof(request_samples_text), kEnterFlags)) {
                int parsed = 0;
                if (parse_int_text(request_samples_text, parsed)) {
                    cfg.request_samples = parsed;
                    cfg = sanitized_config(cfg);
                    hardware_reconfigure_requested = running;
                }
                set_int_text(request_samples_text, sizeof(request_samples_text), cfg.request_samples);
            }
            ImGui::EndTable();
        }
        hardware_reconfigure_requested |=
            ImGui::Checkbox("Auto initialize/configure AD9361", &cfg.auto_init_ad9361);
        cfg = sanitized_config(cfg);

        if (running) {
            if (restart_device_requested) {
                sync_config_from_text_fields();
                stop_preview(preview);
                start_preview(preview, cfg);
                spectrum_display.clear();
            } else if (hardware_reconfigure_requested) {
                request_reconfigure(preview, cfg);
                spectrum_display.clear();
            } else if (display_channel_changed) {
                preview.display_channel.store(cfg.display_channel);
                spectrum_display.clear();
            }
        }

        if (!running) {
            if (ImGui::Button("Connect")) {
                sync_config_from_text_fields();
                start_preview(preview, cfg);
            }
        } else {
            if (ImGui::Button("Disconnect")) {
                stop_preview(preview);
            }
        }
        ImGui::SameLine();
        ImGui::TextWrapped("%s", get_message(preview).c_str());

        ImGui::SeparatorText("RX Status");
        if (ImGui::BeginTable("m300_status", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Metric");
            ImGui::TableSetupColumn("Value");
            ImGui::TableSetupColumn("Metric ");
            ImGui::TableSetupColumn("Value ");
            ImGui::TableHeadersRow();
            auto row = [](const char* a, const char* b, const char* c, const char* d) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(a);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(b);
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(c);
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(d);
            };
            char reads[64] = {};
            char rate[64] = {};
            char pkts[64] = {};
            char jumps[64] = {};
            char lost[64] = {};
            char ts[64] = {};
            char peak[64] = {};
            char elapsed[64] = {};
            std::snprintf(reads, sizeof(reads), "%llu",
                          static_cast<unsigned long long>(stats.reads));
            std::snprintf(rate, sizeof(rate), "%s agg / %s per-chan",
                          format_rate(stats.aggregate_msps).c_str(),
                          format_rate(per_channel_msps).c_str());
            std::snprintf(pkts, sizeof(pkts), "%llu",
                          static_cast<unsigned long long>(stats.packets));
            std::snprintf(jumps, sizeof(jumps), "%llu",
                          static_cast<unsigned long long>(stats.seq_jumps));
            std::snprintf(lost, sizeof(lost), "%llu",
                          static_cast<unsigned long long>(stats.lost_packets));
            std::snprintf(ts, sizeof(ts), "%llu",
                          static_cast<unsigned long long>(stats.timestamp_gaps));
            std::snprintf(peak, sizeof(peak), "%d", static_cast<int>(stats.peak_abs));
            std::snprintf(elapsed, sizeof(elapsed), "%.2f s", stats.elapsed_sec);
            row("Reads", reads, "Rate (MSPS)", rate);
            row("Packets", pkts, "Seq jumps", jumps);
            row("Lost packets", lost, "Timestamp gaps", ts);
            row("Peak abs", peak, "Elapsed", elapsed);
            ImGui::EndTable();
        }

        draw_spectrum_display(cfg, preview, spectrum_display);

        ImGui::End();

        ImGui::Render();
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    stop_preview(preview);
    spectrum_display.shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
