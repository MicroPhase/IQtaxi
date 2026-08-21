#define GLFW_INCLUDE_NONE
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "src/driver/E100/e100_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <vector>

using namespace sdr::api;

#ifndef E100_IMAGE_BUILDER_ROOT
#define E100_IMAGE_BUILDER_ROOT "."
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kCs16Peak = 32767.0f;
constexpr float kCs16Norm = 32768.0f;
constexpr size_t kRxChunkSamples = 4096;
constexpr size_t kRxReadSamples = 32768;
constexpr size_t kWaveformSamples = 2048;
constexpr size_t kWaterfallRows = 160;
constexpr size_t kWaterfallBins = 384;
constexpr size_t kSpectrumBins = 512;
constexpr uint32_t kPlotUpdateIntervalMs = 33;
constexpr size_t kSpectrumHistoryMaxSamples = 65536;

enum class TxMode {
    Timed,
    Asap,
};

enum class TxSource {
    IQ,
    DDS,
};

enum class FirmwareUpdateMethod {
    Auto,
    Tcp,
    Ssh,
};

struct GuiConfig {
    char addr[64] = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t rx_lo_hz = 2450000000ull;
    uint64_t tx_lo_hz = 1000000000ull;
    uint32_t rx_gain = 30u;
    uint32_t tx_atten = 30u;
    float tx_tone_hz = 200000.0f;
    float tx_amplitude = 0.35f;
    uint32_t tx_delay_ms = 200u;
    TxMode tx_mode = TxMode::Asap;
    TxSource tx_source = TxSource::IQ;
    float dds_hz = 200000.0f;
    uint32_t dds_ctrl_word = 0u;
};

struct FirmwareUpdateConfig {
    char package_path[512] = "";
    uint32_t ssh_port = 22u;
    uint32_t tcp_port = 49312u;
    FirmwareUpdateMethod method = FirmwareUpdateMethod::Auto;
    bool reboot = true;
    bool auto_scroll = true;
};

struct RxMetrics {
    uint64_t last_timestamp = 0;
    uint64_t first_timestamp = 0;
    uint64_t expected_timestamp = 0;
    uint64_t total_samples = 0;
    uint64_t packet_count = 0;
    uint64_t zero_reads = 0;
    uint64_t short_reads = 0;
    uint64_t timestamp_gaps = 0;
    uint64_t timestamp_backwards = 0;
    int64_t last_timestamp_delta = 0;
    float peak = 0.0f;
    bool have_timestamp = false;
};

struct SpectrumTraceState {
    std::vector<float> current;
    std::vector<float> glow;
    std::vector<float> peak;
    std::vector<std::vector<float>> history;

    void reset()
    {
        current.clear();
        glow.clear();
        peak.clear();
        history.clear();
    }

    void update(const std::vector<float>& input)
    {
        if (input.empty()) {
            return;
        }
        if (current.size() != input.size()) {
            current = input;
            glow = input;
            peak = input;
            history.assign(1u, input);
            return;
        }

        for (std::size_t i = 0; i < input.size(); i++) {
            current[i] = current[i] * 0.10f + input[i] * 0.90f;
            glow[i] = std::max(current[i], glow[i] - 0.38f);
            peak[i] = std::max(current[i], peak[i] - 0.06f);
        }
        if (history.size() >= 14u) {
            history.erase(history.begin());
        }
        history.push_back(current);
    }
};

static float curve_to_y(float value, float min_v, float max_v, float y0, float y1)
{
    const float denom = std::max(1e-6f, max_v - min_v);
    const float t = std::clamp((value - min_v) / denom, 0.0f, 1.0f);
    return y1 - t * (y1 - y0);
}

static std::pair<float, float> auto_trace_bounds(const std::vector<float>& data)
{
    if (data.empty()) {
        return {-1.0f, 1.0f};
    }
    auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
    const float peak = std::max(std::abs(*min_it), std::abs(*max_it));
    const float bound = std::max(0.12f, peak * 1.15f);
    return {-bound, bound};
}

static std::vector<float> reduce_trace_for_plot(const std::vector<float>& data, std::size_t limit)
{
    if (data.size() <= limit) {
        return data;
    }
    std::vector<float> out;
    out.reserve(limit);
    for (std::size_t i = 0; i < limit; i++) {
        const std::size_t src = (i * data.size()) / limit;
        out.push_back(data[src]);
    }
    return out;
}

static std::vector<float> resample_curve(const std::vector<float>& data, std::size_t bins)
{
    if (data.empty() || bins == 0) {
        return {};
    }
    if (data.size() == bins) {
        return data;
    }
    std::vector<float> out(bins);
    for (std::size_t i = 0; i < bins; i++) {
        const std::size_t src = (i * data.size()) / bins;
        out[i] = data[std::min(src, data.size() - 1u)];
    }
    return out;
}

static std::string format_freq_label(double hz)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision((std::abs(hz) >= 1e9) ? 3 : 2)
        << ((std::abs(hz) >= 1e9) ? (hz / 1e9) : (hz / 1e6))
        << ((std::abs(hz) >= 1e9) ? " GHz" : " MHz");
    return oss.str();
}

static const std::array<uint32_t, 22> kSupportedSampleRates = {
    122880000u,
    61440000u,
    30720000u,
    15360000u,
    7680000u,
    3840000u,
    1920000u,
    46080000u,
    23040000u,
    11520000u,
    5760000u,
    80000000u,
    40000000u,
    20000000u,
    10000000u,
    5000000u,
    64000000u,
    32000000u,
    16000000u,
    8000000u,
    4000000u,
    2000000u,
};

static const std::array<size_t, 5> kSupportedSpectrumFftSizes = {
    4096u,
    8192u,
    16384u,
    32768u,
    65536u,
};

static const char* sample_rate_to_label(uint32_t sample_rate_hz)
{
    switch (sample_rate_hz) {
    case 122880000u:
        return "122.88 MHz";
    case 61440000u:
        return "61.44 MHz";
    case 46080000u:
        return "46.08 MHz";
    case 23040000u:
        return "23.04 MHz";
    case 30720000u:
        return "30.72 MHz";
    case 15360000u:
        return "15.36 MHz";
    case 11520000u:
        return "11.52 MHz";
    case 5760000u:
        return "5.76 MHz";
    case 7680000u:
        return "7.68 MHz";
    case 3840000u:
        return "3.84 MHz";
    case 1920000u:
        return "1.92 MHz";
    case 80000000u:
        return "80.00 MHz";
    case 40000000u:
        return "40.00 MHz";
    case 20000000u:
        return "20.00 MHz";
    case 10000000u:
        return "10.00 MHz";
    case 5000000u:
        return "5.00 MHz";
    case 64000000u:
        return "64.00 MHz";
    case 32000000u:
        return "32.00 MHz";
    case 16000000u:
        return "16.00 MHz";
    case 8000000u:
        return "8.00 MHz";
    case 4000000u:
        return "4.00 MHz";
    case 2000000u:
        return "2.00 MHz";
    default:
        return "Unknown";
    }
}

static const char* spectrum_fft_size_to_label(size_t fft_size)
{
    switch (fft_size) {
    case 4096u:
        return "4096";
    case 8192u:
        return "8192";
    case 16384u:
        return "16384";
    case 32768u:
        return "32768";
    case 65536u:
        return "65536";
    default:
        return "Unknown";
    }
}

static const char* tx_mode_to_label(TxMode mode)
{
    switch (mode) {
    case TxMode::Timed:
        return "Timed";
    case TxMode::Asap:
        return "ASAP";
    default:
        return "Unknown";
    }
}

static const char* tx_source_to_label(TxSource source)
{
    switch (source) {
    case TxSource::IQ:
        return "IQ";
    case TxSource::DDS:
        return "DDS";
    default:
        return "Unknown";
    }
}

static const char* firmware_method_to_label(FirmwareUpdateMethod method)
{
    switch (method) {
    case FirmwareUpdateMethod::Auto:
        return "auto";
    case FirmwareUpdateMethod::Tcp:
        return "tcp";
    case FirmwareUpdateMethod::Ssh:
        return "ssh";
    default:
        return "auto";
    }
}

static std::string default_firmware_package_path()
{
    return std::string(E100_IMAGE_BUILDER_ROOT) + "/build/antsdr-e100/qspi/e100-qspi.frm";
}

static std::string firmware_update_script_path()
{
    return std::string(E100_IMAGE_BUILDER_ROOT) + "/scripts/e100_qspi_update.sh";
}

static bool path_is_regular_file(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}

static std::string shell_quote(const std::string& value)
{
    std::string out = "'";
    for (char ch : value) {
        if (ch == '\'') {
            out += "'\\''";
        } else {
            out += ch;
        }
    }
    out += "'";
    return out;
}

static bool parse_leading_percent(const std::string& line, int& percent)
{
    std::size_t i = 0;
    while (i < line.size() && line[i] >= '0' && line[i] <= '9') {
        i++;
    }
    if (i == 0 || i >= line.size() || line[i] != '%') {
        return false;
    }
    try {
        percent = std::clamp(std::stoi(line.substr(0, i)), 0, 100);
        return true;
    } catch (...) {
        return false;
    }
}

static uint32_t dds_ctrl_word_from_freq(double fc_hz, double fs_hz)
{
    if (fs_hz <= 0.0) {
        return 0u;
    }
    const double ratio = fc_hz / fs_hz;
    const double scaled = ratio * 4294967296.0;
    const double wrapped = std::fmod(scaled, 4294967296.0);
    const double normalized = (wrapped < 0.0) ? (wrapped + 4294967296.0) : wrapped;
    return static_cast<uint32_t>(std::llround(normalized));
}

static double dds_freq_from_ctrl_word(uint32_t ctrl_word, double fs_hz)
{
    if (fs_hz <= 0.0) {
        return 0.0;
    }
    return (static_cast<double>(ctrl_word) / 4294967296.0) * fs_hz;
}

static void draw_grid(ImDrawList* draw_list, const ImVec2& min, const ImVec2& max, int vertical, int horizontal)
{
    const ImU32 grid = IM_COL32(32, 56, 74, 180);
    for (int i = 1; i < vertical; i++) {
        const float x = min.x + (max.x - min.x) * static_cast<float>(i) / static_cast<float>(vertical);
        draw_list->AddLine(ImVec2(x, min.y), ImVec2(x, max.y), grid, 1.0f);
    }
    for (int i = 1; i < horizontal; i++) {
        const float y = min.y + (max.y - min.y) * static_cast<float>(i) / static_cast<float>(horizontal);
        draw_list->AddLine(ImVec2(min.x, y), ImVec2(max.x, y), grid, 1.0f);
    }
}

static void draw_frequency_axis(
    ImDrawList* draw_list,
    const ImVec2& min,
    const ImVec2& max,
    float axis_bottom,
    uint64_t center_freq_hz,
    uint64_t sample_rate_hz)
{
    const ImU32 axis_grid = IM_COL32(42, 70, 91, 180);
    const ImU32 axis_text = IM_COL32(154, 174, 191, 220);
    const float axis_y = axis_bottom + 10.0f;
    draw_list->AddLine(ImVec2(min.x, axis_y), ImVec2(max.x, axis_y), axis_grid, 1.0f);
    for (int i = 0; i <= 4; i++) {
        const float t = static_cast<float>(i) / 4.0f;
        const float x = min.x + (max.x - min.x) * t;
        const double hz = static_cast<double>(center_freq_hz)
                        + (t - 0.5) * static_cast<double>(sample_rate_hz);
        draw_list->AddLine(ImVec2(x, min.y), ImVec2(x, axis_bottom), axis_grid, 1.0f);
        draw_list->AddLine(ImVec2(x, axis_y), ImVec2(x, axis_y + 4.0f), axis_text, 1.0f);
        const std::string label = format_freq_label(hz);
        draw_list->AddText(ImVec2(x - 24.0f, axis_y + 6.0f), axis_text, label.c_str());
    }
}

static void draw_db_range_slider(
    const char* id,
    ImDrawList* draw_list,
    const ImVec2& min,
    const ImVec2& max,
    float y0,
    float y1,
    float& top_dbfs,
    float& bottom_dbfs)
{
    enum class SliderHandle {
        None,
        Top,
        Bottom,
    };

    const float lane_width = 14.0f;
    const ImVec2 lane_min(max.x - lane_width - 6.0f, y0);
    const ImVec2 lane_max(max.x - 6.0f, y1);
    const float slider_min = -140.0f;
    const float slider_max = 0.0f;
    static ImGuiID active_slider = 0;
    static SliderHandle active_handle = SliderHandle::None;

    draw_list->AddRectFilled(lane_min, lane_max, IM_COL32(10, 18, 28, 230), 4.0f);
    draw_list->AddRect(lane_min, lane_max, IM_COL32(48, 78, 100, 220), 4.0f);

    top_dbfs = std::clamp(top_dbfs, bottom_dbfs + 10.0f, slider_max);
    bottom_dbfs = std::clamp(bottom_dbfs, slider_min, top_dbfs - 10.0f);

    const auto db_to_y = [&](float db) {
        return curve_to_y(db, slider_min, slider_max, y0, y1);
    };
    const auto y_to_db = [&](float y) {
        const float t = std::clamp((y1 - y) / std::max(1.0f, y1 - y0), 0.0f, 1.0f);
        return slider_min + t * (slider_max - slider_min);
    };

    const float top_y = db_to_y(top_dbfs);
    const float bottom_y = db_to_y(bottom_dbfs);
    draw_list->AddRectFilled(
        ImVec2(lane_min.x + 2.0f, top_y), ImVec2(lane_max.x - 2.0f, bottom_y), IM_COL32(46, 132, 186, 160), 3.0f);

    const ImVec2 top_handle_min(lane_min.x - 3.0f, top_y - 4.0f);
    const ImVec2 top_handle_max(lane_max.x + 3.0f, top_y + 4.0f);
    const ImVec2 bottom_handle_min(lane_min.x - 3.0f, bottom_y - 4.0f);
    const ImVec2 bottom_handle_max(lane_max.x + 3.0f, bottom_y + 4.0f);
    draw_list->AddRectFilled(top_handle_min, top_handle_max, IM_COL32(198, 238, 255, 245), 3.0f);
    draw_list->AddRectFilled(bottom_handle_min, bottom_handle_max, IM_COL32(198, 238, 255, 245), 3.0f);

    ImGui::PushID(id);
    ImGui::SetCursorScreenPos(ImVec2(lane_min.x - 8.0f, y0));
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("db_slider", ImVec2((lane_max.x - lane_min.x) + 16.0f, y1 - y0));
    const ImGuiID slider_id = ImGui::GetItemID();
    const bool hovered = ImGui::IsItemHovered();
    const bool clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    const float mouse_y = ImGui::GetIO().MousePos.y;

    if (clicked) {
        const float dist_top = std::abs(mouse_y - top_y);
        const float dist_bottom = std::abs(mouse_y - bottom_y);
        active_slider = slider_id;
        active_handle = (dist_top <= dist_bottom) ? SliderHandle::Top : SliderHandle::Bottom;
    }

    if (active_slider == slider_id && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const float value = y_to_db(mouse_y);
        if (active_handle == SliderHandle::Top) {
            top_dbfs = std::clamp(value, bottom_dbfs + 10.0f, slider_max);
        } else if (active_handle == SliderHandle::Bottom) {
            bottom_dbfs = std::clamp(value, slider_min, top_dbfs - 10.0f);
        }
    }

    if (active_slider == slider_id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        active_slider = 0;
        active_handle = SliderHandle::None;
    }
    ImGui::PopID();
}

static void apply_cyberether_style()
{
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 10.0f;
    style.ChildRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.ScrollbarRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.WindowPadding = ImVec2(14.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 8.0f);
    style.ItemSpacing = ImVec2(10.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.08f, 0.11f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.07f, 0.11f, 0.15f, 1.0f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.12f, 0.16f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.17f, 0.28f, 0.36f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.16f, 0.22f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.14f, 0.23f, 0.31f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.17f, 0.28f, 0.38f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.06f, 0.09f, 0.13f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.12f, 0.17f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.12f, 0.26f, 0.38f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.17f, 0.36f, 0.51f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.13f, 0.42f, 0.60f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.10f, 0.20f, 0.28f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.14f, 0.28f, 0.38f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.16f, 0.32f, 0.44f, 1.0f);
    colors[ImGuiCol_Separator] = ImVec4(0.17f, 0.28f, 0.36f, 1.0f);
    colors[ImGuiCol_Text] = ImVec4(0.86f, 0.92f, 0.95f, 1.0f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.52f, 0.61f, 0.67f, 1.0f);
    colors[ImGuiCol_PlotLines] = ImVec4(0.43f, 0.84f, 0.98f, 1.0f);
    colors[ImGuiCol_PlotHistogram] = ImVec4(0.23f, 0.77f, 0.71f, 1.0f);
}

static ImU32 color_map_db(float value_db)
{
    const float t = std::clamp((value_db + 120.0f) / 100.0f, 0.0f, 1.0f);
    const float r = std::clamp(1.7f * t - 0.2f, 0.0f, 1.0f);
    const float g = std::clamp(1.6f * (1.0f - std::abs(t - 0.55f) * 1.8f), 0.0f, 1.0f);
    const float b = std::clamp(1.3f * (1.0f - t), 0.0f, 1.0f);
    return IM_COL32(
        static_cast<int>(r * 255.0f),
        static_cast<int>(g * 255.0f),
        static_cast<int>(b * 255.0f),
        255);
}

static void draw_time_trace(const char* label, const std::vector<float>& data, float height)
{
    ImGui::TextUnformatted(label);
    const ImVec2 size(ImGui::GetContentRegionAvail().x, height);
    ImGui::InvisibleButton(label, size);
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(min, max, IM_COL32(12, 20, 29, 255), 8.0f);
    draw_list->AddRect(min, max, IM_COL32(42, 70, 91, 255), 8.0f);
    draw_grid(draw_list, min, max, 8, 4);

    const std::vector<float> plot_data = reduce_trace_for_plot(data, kWaveformSamples);
    if (plot_data.empty()) {
        draw_list->AddText(ImVec2(min.x + 10.0f, min.y + 10.0f), IM_COL32(155, 175, 190, 255), "No trace");
        return;
    }

    const auto [min_v, max_v] = auto_trace_bounds(plot_data);
    const float inner_y0 = min.y + 6.0f;
    const float inner_y1 = max.y - 6.0f;
    const float mid_y = curve_to_y(0.0f, min_v, max_v, inner_y0, inner_y1);
    draw_list->AddLine(ImVec2(min.x, mid_y), ImVec2(max.x, mid_y), IM_COL32(66, 103, 128, 185), 1.0f);

    const std::size_t columns = std::max<std::size_t>(
        1u, std::min<std::size_t>(plot_data.size(), static_cast<std::size_t>(std::max(1.0f, max.x - min.x))));
    std::vector<ImVec2> trace_pts;
    trace_pts.reserve(columns);
    for (std::size_t col = 0; col < columns; col++) {
        const std::size_t start = (col * plot_data.size()) / columns;
        const std::size_t stop = std::max(start + 1u, ((col + 1u) * plot_data.size()) / columns);
        float local_sum = plot_data[start];
        for (std::size_t i = start + 1u; i < stop; i++) {
            local_sum += plot_data[i];
        }
        const float x = min.x + (max.x - min.x) * (static_cast<float>(col) + 0.5f) / static_cast<float>(columns);
        const float mean_value = local_sum / static_cast<float>(stop - start);
        const float y_mean = curve_to_y(mean_value, min_v, max_v, inner_y0, inner_y1);
        trace_pts.emplace_back(x, y_mean);
    }

    if (columns >= 2u) {
        draw_list->AddPolyline(trace_pts.data(), static_cast<int>(trace_pts.size()), IM_COL32(214, 247, 255, 235), 0, 1.35f);
    } else if (columns == 1u) {
        draw_list->AddCircleFilled(trace_pts.front(), 2.0f, IM_COL32(214, 247, 255, 235));
    }
}

static void draw_spectrum_canvas(
    const char* title,
    const std::vector<float>& spectrum,
    SpectrumTraceState& trace_state,
    float height,
    uint64_t center_freq_hz,
    uint64_t sample_rate_hz,
    float& top_dbfs,
    float& bottom_dbfs)
{
    ImGui::TextUnformatted(title);
    const ImVec2 size(ImGui::GetContentRegionAvail().x, height);
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton(title, size);
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilledMultiColor(
        min, max, IM_COL32(5, 10, 19, 255), IM_COL32(5, 10, 19, 255), IM_COL32(11, 20, 33, 255), IM_COL32(11, 20, 33, 255));
    draw_list->AddRect(min, max, IM_COL32(42, 70, 91, 255), 8.0f);
    draw_grid(draw_list, min, max, 10, 6);

    if (spectrum.empty()) {
        draw_list->AddText(ImVec2(min.x + 10.0f, min.y + 10.0f), IM_COL32(155, 175, 190, 255), "No spectrum");
        return;
    }

    trace_state.update(spectrum);
    const float y0 = min.y + 8.0f;
    const float y1 = max.y - 30.0f;
    const float ceil_db = std::max(top_dbfs, bottom_dbfs + 10.0f);
    const float floor_db = std::min(bottom_dbfs, ceil_db - 10.0f);
    const float plot_x1 = max.x - 24.0f;

    std::vector<ImVec2> current_pts;
    std::vector<ImVec2> glow_pts;
    std::vector<ImVec2> peak_pts;
    current_pts.reserve(trace_state.current.size());
    glow_pts.reserve(trace_state.glow.size());
    peak_pts.reserve(trace_state.peak.size());
    for (std::size_t i = 0; i < trace_state.current.size(); i++) {
        const float x = min.x + (plot_x1 - min.x) * static_cast<float>(i)
                        / static_cast<float>(std::max<std::size_t>(1u, trace_state.current.size() - 1u));
        current_pts.emplace_back(x, curve_to_y(trace_state.current[i], floor_db, ceil_db, y0, y1));
        glow_pts.emplace_back(x, curve_to_y(trace_state.glow[i], floor_db, ceil_db, y0, y1));
        peak_pts.emplace_back(x, curve_to_y(trace_state.peak[i], floor_db, ceil_db, y0, y1));
    }

    const std::size_t history_count = trace_state.history.size();
    for (std::size_t h = 0; h < history_count; h++) {
        const auto& hist = trace_state.history[h];
        if (hist.size() != trace_state.current.size()) {
            continue;
        }
        std::vector<ImVec2> hist_pts;
        hist_pts.reserve(hist.size());
        for (std::size_t i = 0; i < hist.size(); i++) {
            const float x = min.x + (plot_x1 - min.x) * static_cast<float>(i)
                            / static_cast<float>(std::max<std::size_t>(1u, hist.size() - 1u));
            hist_pts.emplace_back(x, curve_to_y(hist[i], floor_db, ceil_db, y0, y1));
        }
        const float t = static_cast<float>(h + 1u) / static_cast<float>(history_count);
        const int alpha = static_cast<int>(4.0f + 40.0f * t * t);
        const float thickness = 0.7f + 0.5f * t;
        draw_list->AddPolyline(hist_pts.data(), static_cast<int>(hist_pts.size()), IM_COL32(52, 170, 255, alpha), 0, thickness);
    }

    for (float thick : {4.0f, 2.0f}) {
        const int alpha = (thick > 3.0f) ? 12 : 28;
        draw_list->AddPolyline(glow_pts.data(), static_cast<int>(glow_pts.size()), IM_COL32(72, 199, 255, alpha), 0, thick);
    }
    draw_list->AddPolyline(peak_pts.data(), static_cast<int>(peak_pts.size()), IM_COL32(255, 170, 78, 128), 0, 0.95f);
    draw_list->AddPolyline(current_pts.data(), static_cast<int>(current_pts.size()), IM_COL32(208, 248, 255, 255), 0, 1.4f);
    draw_list->AddPolyline(current_pts.data(), static_cast<int>(current_pts.size()), IM_COL32(76, 205, 255, 220), 0, 0.8f);

    for (float tick = std::floor(floor_db / 10.0f) * 10.0f; tick <= ceil_db + 0.1f; tick += 10.0f) {
        if (tick < floor_db - 0.1f) {
            continue;
        }
        const float y = curve_to_y(tick, floor_db, ceil_db, y0, y1);
        draw_list->AddLine(ImVec2(min.x, y), ImVec2(plot_x1, y), IM_COL32(46, 74, 96, 110), 1.0f);
        char label[32];
        std::snprintf(label, sizeof(label), "%.0f dBFS", tick);
        draw_list->AddText(ImVec2(min.x + 10.0f, y - 8.0f), IM_COL32(154, 174, 191, 220), label);
    }

    draw_frequency_axis(draw_list, min, ImVec2(plot_x1, max.y), y1, center_freq_hz, sample_rate_hz);
    draw_db_range_slider(title, draw_list, min, max, y0, y1, top_dbfs, bottom_dbfs);
}

static void draw_waterfall_section(
    const char* title,
    const std::deque<std::vector<float>>& waterfall,
    uint64_t center_freq_hz,
    uint64_t sample_rate_hz)
{
    ImGui::TextUnformatted(title);
    const ImVec2 size(ImGui::GetContentRegionAvail().x, 240.0f);
    ImGui::InvisibleButton(title, size);
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilledMultiColor(
        min, max, IM_COL32(5, 8, 14, 255), IM_COL32(5, 8, 14, 255), IM_COL32(12, 18, 28, 255), IM_COL32(12, 18, 28, 255));
    draw_list->AddRect(min, max, IM_COL32(42, 70, 91, 255), 8.0f);

    if (waterfall.empty()) {
        draw_list->AddText(ImVec2(min.x + 12.0f, min.y + 12.0f), IM_COL32(160, 180, 190, 255), "No spectrum history");
        return;
    }

    const float plot_bottom = max.y - 28.0f;
    const float row_h = (plot_bottom - min.y) / static_cast<float>(kWaterfallRows);
    const float bin_w = (max.x - min.x) / static_cast<float>(kWaterfallBins);
    const std::size_t rows = std::min<std::size_t>(waterfall.size(), kWaterfallRows);
    for (std::size_t row = 0; row < rows; row++) {
        const auto& raw = waterfall[waterfall.size() - 1u - row];
        const auto line = resample_curve(raw, kWaterfallBins);
        const float y0 = min.y + static_cast<float>(row) * row_h;
        const float y1 = y0 + row_h + 0.6f;
        for (std::size_t bin = 0; bin < line.size(); bin++) {
            const float x0 = min.x + static_cast<float>(bin) * bin_w;
            const float x1 = x0 + bin_w + 0.8f;
            draw_list->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), color_map_db(line[bin]));
        }
    }
    draw_list->AddText(ImVec2(min.x + 10.0f, min.y + 8.0f), IM_COL32(122, 144, 166, 220), "New");
    draw_list->AddText(ImVec2(min.x + 10.0f, plot_bottom - 18.0f), IM_COL32(122, 144, 166, 220), "Old");
    draw_frequency_axis(draw_list, min, max, plot_bottom, center_freq_hz, sample_rate_hz);
}

static std::vector<float> normalize_to_plot(const std::vector<std::complex<float>>& iq, bool imag)
{
    std::vector<float> out;
    out.reserve(iq.size());
    for (const auto& s : iq) {
        out.push_back(imag ? s.imag() : s.real());
    }
    return out;
}

static void fft_inplace(std::vector<std::complex<float>>& a)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }

    for (size_t len = 2; len <= n; len <<= 1) {
        const float ang = -2.0f * static_cast<float>(kPi) / static_cast<float>(len);
        const std::complex<float> wlen(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t j = 0; j < len / 2; ++j) {
                const auto u = a[i + j];
                const auto v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

static std::vector<float> compute_spectrum_db(const std::vector<std::complex<float>>& iq, size_t fft_size)
{
    if (fft_size == 0u) {
        return {};
    }
    std::vector<std::complex<float>> fft_buf(
        fft_size, std::complex<float>(0.0f, 0.0f));
    const size_t copy_count = std::min(iq.size(), fft_buf.size());
    for (size_t i = 0; i < copy_count; ++i) {
        const float w = 0.5f - 0.5f * std::cos(2.0f * static_cast<float>(kPi) * static_cast<float>(i) / static_cast<float>(fft_size - 1));
        fft_buf[i] = iq[i] * w;
    }
    fft_inplace(fft_buf);

    std::vector<float> out(fft_size);
    for (size_t i = 0; i < fft_size; ++i) {
        const size_t idx = (i + fft_size / 2) % fft_size;
        const float mag = std::abs(fft_buf[idx]) / static_cast<float>(fft_size);
        out[i] = 20.0f * std::log10(std::max(mag, 1e-9f));
    }
    return out;
}

static void fill_cs16_tone(
    std::vector<int16_t>& tone,
    size_t num_samples,
    uint32_t sample_rate_hz,
    float tone_hz,
    float amplitude,
    double start_phase_rad)
{
    amplitude = std::clamp(amplitude, 0.0f, 1.0f);
    const float scale = amplitude * kCs16Peak;
    tone.resize(num_samples * 2);
    const double phase_step =
        (2.0 * kPi * static_cast<double>(tone_hz)) / std::max<uint32_t>(sample_rate_hz, 1u);
    for (size_t n = 0; n < num_samples; ++n) {
        const double phase = start_phase_rad + phase_step * static_cast<double>(n);
        tone[n * 2 + 0] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
        tone[n * 2 + 1] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
    }
}

static double advance_tone_phase(double phase_rad, float tone_hz, uint32_t sample_rate_hz, size_t samples)
{
    const double phase_step =
        (2.0 * kPi * static_cast<double>(tone_hz)) / std::max<uint32_t>(sample_rate_hz, 1u);
    const double wrapped = std::fmod(phase_rad + phase_step * static_cast<double>(samples), 2.0 * kPi);
    return (wrapped < 0.0) ? (wrapped + 2.0 * kPi) : wrapped;
}

class E100GuiApp {
public:
    E100GuiApp()
    {
        set_firmware_package_path(default_firmware_package_path());
    }

    ~E100GuiApp()
    {
        wait_firmware_update();
        stop_tx();
        stop_rx();
        disconnect();
    }

    void render()
    {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin(
            "E100 Workspace",
            nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
                | ImGuiWindowFlags_NoCollapse);
        ImGui::PopStyleVar(2);

        ImGui::TextColored(ImVec4(0.40f, 0.86f, 0.98f, 1.0f), "E100 RF Console");
        ImGui::SameLine();
        ImGui::TextDisabled("Live spectrum / waterfall workspace");
        ImGui::Spacing();

        const float left_width = std::clamp(viewport->WorkSize.x * 0.24f, 430.0f, 520.0f);
        ImGui::BeginChild("ControlPane", ImVec2(left_width, 0.0f), true);
        render_connection();
        render_radio_controls();
        render_tx_controls();
        render_rx_controls();
        render_firmware_update();
        render_status();
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("WorkspacePane", ImVec2(0.0f, 0.0f), false);
        render_plots();
        ImGui::EndChild();

        ImGui::End();
    }

private:
    void commit_sample_rate_change(uint32_t sample_rate_hz)
    {
        _cfg.sample_rate_hz = std::max(sample_rate_hz, 1u);
        _cfg.dds_ctrl_word = dds_ctrl_word_from_freq(
            static_cast<double>(_cfg.dds_hz), static_cast<double>(_cfg.sample_rate_hz));
        apply_sample_rate_on_commit();
    }

    void commit_spectrum_fft_size_change(size_t fft_size)
    {
        _spectrum_fft_size = fft_size;
        std::lock_guard<std::mutex> lock(_plot_mutex);
        _spectrum_db.clear();
        _waterfall.clear();
        _spectrum_trace.reset();
        _rx_iq_history.clear();
    }

    void apply_sample_rate_on_commit()
    {
        if (!_connected) {
            return;
        }
        try {
            apply_settings();
        } catch (const std::exception& ex) {
            set_status(std::string("apply failed: ") + ex.what(), true);
        }
    }

    void apply_rx_lo_on_commit()
    {
        if (!_connected || !_device) {
            return;
        }
        try {
            _device->set_rx_freq(_cfg.rx_lo_hz, 1);
            set_status("RX LO updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("RX LO update failed: ") + ex.what(), true);
        }
    }

    void apply_tx_lo_on_commit()
    {
        if (!_connected || !_device) {
            return;
        }
        try {
            _device->set_tx_freq(_cfg.tx_lo_hz, 1);
            set_status("TX LO updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("TX LO update failed: ") + ex.what(), true);
        }
    }

    void apply_rx_gain_on_commit()
    {
        if (!_connected || !_device) {
            return;
        }
        try {
            _device->set_rx_gain(_cfg.rx_gain, 1);
            set_status("RX gain updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("RX gain update failed: ") + ex.what(), true);
        }
    }

    void apply_tx_atten_on_commit()
    {
        if (!_connected || !_device) {
            return;
        }
        try {
            _device->set_tx_atten(_cfg.tx_atten, 1);
            set_status("TX atten updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("TX atten update failed: ") + ex.what(), true);
        }
    }

    void trigger_lvds_if_reset()
    {
        if (!_connected || !_device) {
            throw std::runtime_error("device not connected");
        }

        auto local_bus = _device->get_local_bus();
        if (!local_bus) {
            throw std::runtime_error("local bus not ready");
        }

        local_bus->poke32(e100::CUSTOM_SET_LVDS_IF_RST, 1u);
        set_status("LVDS IF reset triggered", false);
    }

    void apply_tx_source_config()
    {
        if (!_connected || !_device || !_tx_stream) {
            return;
        }

        if (_tx_running.load()) {
            set_status("stop TX before changing TX source or mode", true);
            return;
        }

        try {
            configure_tx_source();
            set_status("TX source updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("TX source update failed: ") + ex.what(), true);
            return;
        }
    }

    void apply_dds_config_on_commit()
    {
        if (!_connected || !_device || !_tx_stream || _cfg.tx_source != TxSource::DDS) {
            return;
        }

        try {
            configure_tx_source();
            set_status(_tx_running.load() ? "DDS updated" : "DDS settings updated", false);
        } catch (const std::exception& ex) {
            set_status(std::string("DDS update failed: ") + ex.what(), true);
        }
    }

    void render_connection()
    {
        ImGui::SeparatorText("Connection");

        ImGui::TextUnformatted("Device IP");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputText("##DeviceIP", _cfg.addr, sizeof(_cfg.addr));
        if (!_connected) {
            if (ImGui::Button("Connect")) {
                try {
                    connect();
                } catch (const std::exception& ex) {
                    set_status(std::string("connect failed: ") + ex.what(), true);
                }
            }
        } else {
            if (ImGui::Button("Disconnect")) {
                disconnect();
            }
        }
        ImGui::Text("state: %s", _connected ? "connected" : "disconnected");
    }

    void render_radio_controls()
    {
        ImGui::SeparatorText("Radio");

        ImGui::TextUnformatted("Sample Rate");
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##SampleRate", sample_rate_to_label(_cfg.sample_rate_hz), ImGuiComboFlags_HeightLargest)) {
            for (std::size_t i = 0; i < kSupportedSampleRates.size(); ++i) {
                const bool selected = (_cfg.sample_rate_hz == kSupportedSampleRates[i]);
                if (ImGui::Selectable(sample_rate_to_label(kSupportedSampleRates[i]), selected)) {
                    commit_sample_rate_change(kSupportedSampleRates[i]);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TextUnformatted("FFT Size");
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##SpectrumFftSize", spectrum_fft_size_to_label(_spectrum_fft_size), ImGuiComboFlags_HeightLargest)) {
            for (std::size_t i = 0; i < kSupportedSpectrumFftSizes.size(); ++i) {
                const bool selected = (_spectrum_fft_size == kSupportedSpectrumFftSizes[i]);
                if (ImGui::Selectable(spectrum_fft_size_to_label(kSupportedSpectrumFftSizes[i]), selected)) {
                    commit_spectrum_fft_size_change(kSupportedSpectrumFftSizes[i]);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TextUnformatted("RX LO");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##RxLo", ImGuiDataType_U64, &_cfg.rx_lo_hz);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            apply_rx_lo_on_commit();
        }

        ImGui::TextUnformatted("TX LO");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##TxLo", ImGuiDataType_U64, &_cfg.tx_lo_hz);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            apply_tx_lo_on_commit();
        }

        ImGui::TextUnformatted("RX Gain");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##RxGain", ImGuiDataType_U32, &_cfg.rx_gain);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            apply_rx_gain_on_commit();
        }

        ImGui::TextUnformatted("TX Gain");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##TxGain", ImGuiDataType_U32, &_cfg.tx_atten);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            apply_tx_atten_on_commit();
        }

        if (ImGui::Button("LVDS IF Reset")) {
            try {
                trigger_lvds_if_reset();
            } catch (const std::exception& ex) {
                set_status(std::string("LVDS IF reset failed: ") + ex.what(), true);
            }
        }
        ImGui::TextDisabled("For the v25 server: pulse gc080x_linux_set_tx_reset().");
    }

    void render_tx_controls()
    {
        ImGui::SeparatorText("TX Tone");
        const bool tx_active = _tx_running.load();

        ImGui::TextUnformatted("TX Source");
        ImGui::SetNextItemWidth(-1.0f);
        if (tx_active) {
            ImGui::BeginDisabled();
        }
        if (ImGui::BeginCombo("##TxSource", tx_source_to_label(_cfg.tx_source), ImGuiComboFlags_HeightLargest)) {
            for (TxSource source : {TxSource::IQ, TxSource::DDS}) {
                const bool selected = (_cfg.tx_source == source);
                if (ImGui::Selectable(tx_source_to_label(source), selected)) {
                    _cfg.tx_source = source;
                    apply_tx_source_config();
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (tx_active) {
            ImGui::EndDisabled();
        }

        ImGui::TextUnformatted("TX Mode");
        ImGui::SetNextItemWidth(-1.0f);
        if (tx_active) {
            ImGui::BeginDisabled();
        }
        if (ImGui::BeginCombo("##TxMode", tx_mode_to_label(_cfg.tx_mode), ImGuiComboFlags_HeightLargest)) {
            for (TxMode mode : {TxMode::Timed, TxMode::Asap}) {
                const bool selected = (_cfg.tx_mode == mode);
                if (ImGui::Selectable(tx_mode_to_label(mode), selected)) {
                    _cfg.tx_mode = mode;
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        if (tx_active) {
            ImGui::EndDisabled();
        }
        ImGui::TextDisabled("Legacy server is generally more stable in ASAP mode.");

        if (_cfg.tx_source == TxSource::IQ) {
            ImGui::TextUnformatted("Tone Hz");
            ImGui::SetNextItemWidth(-1.0f);
            if (!_tx_tone_hz_edit_active) {
                _tx_tone_hz_edit = _cfg.tx_tone_hz;
            }
            ImGui::InputFloat("##ToneHz", &_tx_tone_hz_edit, 1000.0f, 10000.0f, "%.1f");
            _tx_tone_hz_edit_active = ImGui::IsItemActive();
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                _cfg.tx_tone_hz = _tx_tone_hz_edit;
                _tx_tone_hz_edit_active = false;
            }

            ImGui::TextUnformatted("Amplitude");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat("##Amplitude", &_cfg.tx_amplitude, 0.0f, 1.0f, "%.2f");

            ImGui::TextUnformatted("TX Delay ms");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputScalar("##TxDelayMs", ImGuiDataType_U32, &_cfg.tx_delay_ms);
            if (_cfg.tx_mode == TxMode::Asap) {
                ImGui::TextDisabled("ASAP mode ignores TX Delay ms and timestamps.");
            }
        } else {
            ImGui::TextUnformatted("DDS Hz");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputFloat("##DdsHz", &_cfg.dds_hz, 1000.0f, 10000.0f, "%.1f");
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                _cfg.dds_ctrl_word = dds_ctrl_word_from_freq(
                    static_cast<double>(_cfg.dds_hz), static_cast<double>(_cfg.sample_rate_hz));
                apply_dds_config_on_commit();
            }

            ImGui::TextUnformatted("DDS Ctrl Word");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputScalar("##DdsCtrlWord", ImGuiDataType_U32, &_cfg.dds_ctrl_word);
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                _cfg.dds_hz = static_cast<float>(dds_freq_from_ctrl_word(
                    _cfg.dds_ctrl_word, static_cast<double>(_cfg.sample_rate_hz)));
                apply_dds_config_on_commit();
            }
            ImGui::TextDisabled("DDS uses DW = fc / fs * 2^32");
        }

        if (!_tx_running.load()) {
            if (ImGui::Button(_cfg.tx_source == TxSource::DDS ? "Start DDS" : "Start Tone")) {
                try {
                    start_tx();
                } catch (const std::exception& ex) {
                    set_status(std::string("start tx failed: ") + ex.what(), true);
                }
            }
        } else {
            if (ImGui::Button(_cfg.tx_source == TxSource::DDS ? "Stop DDS" : "Stop Tone")) {
                stop_tx();
            }
        }
        ImGui::Text("tx: %s", _tx_running.load()
            ? (_cfg.tx_source == TxSource::DDS ? "DDS enabled" : "IQ streaming")
            : "stopped");
    }

    void render_rx_controls()
    {
        ImGui::SeparatorText("RX");
        if (!_rx_running.load()) {
            if (ImGui::Button("Start RX")) {
                try {
                    start_rx();
                } catch (const std::exception& ex) {
                    set_status(std::string("start rx failed: ") + ex.what(), true);
                }
            }
        } else {
            if (ImGui::Button("Stop RX")) {
                stop_rx();
            }
        }
        ImGui::Text("rx: %s", _rx_running.load() ? "running" : "stopped");
    }

    void render_firmware_update()
    {
        refresh_firmware_update_state();

        ImGui::SeparatorText("Firmware");

        ImGui::TextUnformatted("Package");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputText("##FirmwarePackage", _fw_cfg.package_path, sizeof(_fw_cfg.package_path));
        if (ImGui::Button("Default Package")) {
            set_firmware_package_path(default_firmware_package_path());
        }

        ImGui::TextUnformatted("Method");
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##FirmwareMethod", firmware_method_to_label(_fw_cfg.method))) {
            for (FirmwareUpdateMethod method : {FirmwareUpdateMethod::Auto, FirmwareUpdateMethod::Tcp, FirmwareUpdateMethod::Ssh}) {
                const bool selected = (_fw_cfg.method == method);
                if (ImGui::Selectable(firmware_method_to_label(method), selected)) {
                    _fw_cfg.method = method;
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TextUnformatted("SSH Port");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##FirmwareSshPort", ImGuiDataType_U32, &_fw_cfg.ssh_port);

        ImGui::TextUnformatted("TCP Port");
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalar("##FirmwareTcpPort", ImGuiDataType_U32, &_fw_cfg.tcp_port);

        ImGui::Checkbox("Reboot after update", &_fw_cfg.reboot);
        ImGui::Checkbox("Auto scroll log", &_fw_cfg.auto_scroll);

        const bool updating = _fw_running.load();
        if (updating) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Start Firmware Update")) {
            try {
                start_firmware_update();
            } catch (const std::exception& ex) {
                append_firmware_log(std::string("error: ") + ex.what() + "\n");
                set_status(std::string("firmware update failed: ") + ex.what(), true);
            }
        }
        if (updating) {
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Log")) {
            std::lock_guard<std::mutex> lock(_fw_mutex);
            _fw_log.clear();
            _fw_progress = 0;
        }

        std::string fw_summary;
        bool fw_error = false;
        int fw_progress = 0;
        std::string fw_log;
        {
            std::lock_guard<std::mutex> lock(_fw_mutex);
            fw_summary = _fw_status;
            fw_error = _fw_error;
            fw_progress = _fw_progress;
            fw_log = _fw_log;
        }

        const float progress = static_cast<float>(fw_progress) / 100.0f;
        ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f));
        if (fw_error) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", fw_summary.c_str());
        } else {
            ImGui::TextWrapped("%s", fw_summary.c_str());
        }

        ImGui::BeginChild("FirmwareLog", ImVec2(0.0f, 150.0f), true, ImGuiWindowFlags_HorizontalScrollbar);
        ImGui::TextUnformatted(fw_log.empty() ? "idle" : fw_log.c_str());
        if (_fw_cfg.auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) {
            ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }

    void render_status()
    {
        std::string status_text;
        bool status_error = false;
        {
            std::lock_guard<std::mutex> lock(_status_mutex);
            status_text = _status;
            status_error = _status_error;
        }
        ImGui::SeparatorText("Status");
        ImGui::TextWrapped("%s", status_text.c_str());
        if (status_error) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "error");
        }
    }

    void render_plots()
    {
        std::vector<float> plot_i;
        std::vector<float> plot_q;
        std::vector<float> spectrum;
        std::deque<std::vector<float>> waterfall;
        RxMetrics metrics;
        rx_stream_continuity_snapshot stream_stats;
        tx_flow_control_snapshot tx_stats;
        {
            std::lock_guard<std::mutex> lock(_plot_mutex);
            plot_i = _plot_i;
            plot_q = _plot_q;
            spectrum = _spectrum_db;
            waterfall = _waterfall;
            metrics = _rx_metrics;
        }
        if (_rx_stream) {
            if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(_rx_stream)) {
                stream_stats = packet_stream->get_rx_continuity_stats();
            }
        }
        if (_tx_stream) {
            if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(_tx_stream)) {
                tx_stats = packet_stream->get_tx_flow_control_stats();
            }
        }

        ImGui::BeginChild("SignalPane", ImVec2(0.0f, 0.0f), true);
        ImGui::TextDisabled("Live spectrum / waterfall");
        ImGui::Separator();

        ImGui::Text("samples: %llu  packets: %llu  last ts: %llu  peak: %.3f",
            static_cast<unsigned long long>(metrics.total_samples),
            static_cast<unsigned long long>(metrics.packet_count),
            static_cast<unsigned long long>(metrics.last_timestamp),
            metrics.peak);
        ImGui::Text("zero reads: %llu  short reads: %llu",
            static_cast<unsigned long long>(metrics.zero_reads),
            static_cast<unsigned long long>(metrics.short_reads));
        const bool stream_ok =
            stream_stats.have_last && stream_stats.seq_errors == 0 && stream_stats.timestamp_errors == 0;
        ImGui::TextColored(
            stream_ok ? ImVec4(0.45f, 0.95f, 0.62f, 1.0f) : ImVec4(1.0f, 0.58f, 0.28f, 1.0f),
            "wire continuity: %s  pkts: %llu  seq gaps: %llu  ts gaps: %llu",
            stream_stats.have_last ? (stream_ok ? "continuous" : "gap detected") : "waiting",
            static_cast<unsigned long long>(stream_stats.packet_count),
            static_cast<unsigned long long>(stream_stats.seq_errors),
            static_cast<unsigned long long>(stream_stats.timestamp_errors));
        ImGui::Text("wire last seq: %u  expect seq: %u  last pkt samples: %u  ts delta: %lld",
            static_cast<unsigned int>(stream_stats.last_seq),
            static_cast<unsigned int>(stream_stats.expected_seq),
            stream_stats.last_packet_samples,
            static_cast<long long>(stream_stats.last_timestamp_delta));
        ImGui::Text("wire last ts: %llu  expect ts: %llu  app chunk ts gaps: %llu  app delta: %lld",
            static_cast<unsigned long long>(stream_stats.last_timestamp),
            static_cast<unsigned long long>(stream_stats.expected_timestamp),
            static_cast<unsigned long long>(metrics.timestamp_gaps),
            static_cast<long long>(metrics.last_timestamp_delta));
        ImGui::Text("host queue drops: %llu  depth: %llu  peak: %llu",
            static_cast<unsigned long long>(stream_stats.host_queue_drops),
            static_cast<unsigned long long>(stream_stats.host_queue_depth),
            static_cast<unsigned long long>(stream_stats.host_queue_depth_peak));
        if (tx_stats.monitoring_active) {
            const double pause_ratio = (tx_stats.monitor_elapsed_us > 0u)
                ? static_cast<double>(tx_stats.fc_pause_total_us) / static_cast<double>(tx_stats.monitor_elapsed_us)
                : 0.0;
            const char* tx_flow_summary = "free-running";
            ImVec4 tx_flow_color(0.45f, 0.95f, 0.62f, 1.0f);
            if (tx_stats.send_buff_timeout_count > 0u) {
                tx_flow_summary = "host pressure";
                tx_flow_color = ImVec4(1.0f, 0.45f, 0.45f, 1.0f);
            } else if (pause_ratio >= 0.20 || !tx_stats.ready_to_send) {
                tx_flow_summary = "throttled";
                tx_flow_color = ImVec4(1.0f, 0.75f, 0.28f, 1.0f);
            } else if (tx_stats.fc_pause_count > 0u) {
                tx_flow_summary = "light throttling";
                tx_flow_color = ImVec4(0.55f, 0.82f, 0.98f, 1.0f);
            }

            ImGui::Text(
                "tx fc raw: %s  last fc: 0x%04x  pauses: %llu  pause us: %llu",
                tx_stats.ready_to_send ? "ready" : "paused",
                static_cast<unsigned int>(tx_stats.last_fc_word),
                static_cast<unsigned long long>(tx_stats.fc_pause_count),
                static_cast<unsigned long long>(tx_stats.fc_pause_total_us));
            ImGui::TextColored(
                tx_flow_color,
                "tx flow summary: %s  pause ratio: %.1f%%",
                tx_flow_summary,
                pause_ratio * 100.0);
            ImGui::Text("tx waits: %llu  wait timeouts: %llu  send-buff timeouts: %llu",
                static_cast<unsigned long long>(tx_stats.fc_wait_count),
                static_cast<unsigned long long>(tx_stats.fc_wait_timeout_count),
                static_cast<unsigned long long>(tx_stats.send_buff_timeout_count));
            ImGui::Text("tx pkts: %llu  samples: %llu  last seq: %u  last ts: %llu",
                static_cast<unsigned long long>(tx_stats.packets_sent),
                static_cast<unsigned long long>(tx_stats.samples_sent),
                static_cast<unsigned int>(tx_stats.last_seq),
                static_cast<unsigned long long>(tx_stats.last_timestamp));
        }

        draw_time_trace("CH0 I", plot_i, 98.0f);
        draw_time_trace("CH0 Q", plot_q, 98.0f);
        draw_spectrum_canvas(
            "CH0 Spectrum",
            resample_curve(spectrum, kSpectrumBins),
            _spectrum_trace,
            260.0f,
            _cfg.rx_lo_hz,
            _cfg.sample_rate_hz,
            _spectrum_top_dbfs,
            _spectrum_bottom_dbfs);
        draw_waterfall_section("CH0 Waterfall", waterfall, _cfg.rx_lo_hz, _cfg.sample_rate_hz);
        ImGui::EndChild();
    }

    void connect()
    {
        stop_tx();
        stop_rx();
        disconnect();

        auto device = std::make_unique<E100Impl>(std::string(_cfg.addr));
        if (!device->isInitialSuccess()) {
            throw std::runtime_error("device init failed");
        }
        _device = std::move(device);
        _connected = true;
        apply_settings();
        set_status("connected", false);
    }

    void disconnect()
    {
        stop_tx();
        stop_rx();
        _tx_stream.reset();
        _rx_stream.reset();
        _device.reset();
        _connected = false;
    }

    void apply_settings()
    {
        if (!_device) {
            throw std::runtime_error("device not connected");
        }

        const bool restart_rx = _rx_running.load();
        const bool restart_tx = _tx_running.load();
        if (restart_tx) {
            stop_tx();
        }
        if (restart_rx) {
            stop_rx();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        _device->set_channel_enable(1u);
        _device->set_dma_mode(0u);
        _device->setSampleRate(static_cast<double>(_cfg.sample_rate_hz));
        const uint32_t actual_rate = _device->getSampleRate();
        if (actual_rate != _cfg.sample_rate_hz) {
            throw std::runtime_error("sample-rate readback mismatch");
        }
        _device->set_rx_freq(_cfg.rx_lo_hz, 1);
        _device->set_tx_freq(_cfg.tx_lo_hz, 1);
        _device->set_rx_gain(_cfg.rx_gain, 1);
        _device->set_tx_atten(_cfg.tx_atten, 1);

        _rx_stream = _device->get_rx_stream();
        _tx_stream = _device->get_tx_stream();
        configure_tx_source();

        if (restart_rx) {
            start_rx();
        }
        if (restart_tx) {
            start_tx();
        }
        set_status("settings applied", false);
    }

    void start_rx()
    {
        if (!_device || !_rx_stream) {
            throw std::runtime_error("device not ready");
        }
        if (_rx_running.exchange(true)) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(_plot_mutex);
            _rx_metrics = RxMetrics{};
            _plot_i.clear();
            _plot_q.clear();
            _rx_iq_history.clear();
            _spectrum_db.clear();
            _waterfall.clear();
            _spectrum_trace.reset();
            _last_plot_update = {};
        }
        if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(_rx_stream)) {
            packet_stream->reset_rx_continuity_stats();
        }

        _rx_thread = std::thread([this]() {
            try {
                std::vector<int16_t> rx_buffer(kRxReadSamples * 2);
                std::vector<void*> buffs{rx_buffer.data()};
                uint64_t timestamp = 0;

                _rx_stream->set_rx_mode(STREAM_MODE);
                _rx_stream->set_max_sample_nums_per_packet((1472 - 16) / 4);
                _rx_stream->set_recv_param(STREAM_MODE, kRxReadSamples, timestamp, 1, 0);

                while (_rx_running.load()) {
                    const size_t received = _rx_stream->recv(buffs, kRxReadSamples, timestamp, MICRORF_FORMAT_INT16);
                    update_rx_plots(rx_buffer, received, timestamp);
                    if (received == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                }
            } catch (const std::exception& ex) {
                set_status(std::string("rx thread failed: ") + ex.what(), true);
                _rx_running.store(false);
            }
        });
        set_status("rx started", false);
    }

    void stop_rx()
    {
        _rx_running.store(false);
        if (_rx_thread.joinable()) {
            _rx_thread.join();
        }
        if (_rx_stream) {
            try {
                uint64_t stop_timestamp = 0;
                _rx_stream->set_recv_param(STREAM_MODE, kRxReadSamples, stop_timestamp, 0, 1);
                _rx_stream->set_rx_mode_exit();
            } catch (const std::exception& ex) {
                set_status(std::string("rx stop failed: ") + ex.what(), true);
            }
        }
    }

    void start_tx()
    {
        if (!_device || !_tx_stream) {
            throw std::runtime_error("device not ready");
        }
        if (_tx_running.load()) {
            return;
        }

        configure_tx_source();
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(_tx_stream)) {
            if (_cfg.tx_source == TxSource::IQ) {
                packet_stream->begin_tx_flow_control_monitoring();
            } else {
                packet_stream->end_tx_flow_control_monitoring();
            }
        }
        if (_tx_running.exchange(true)) {
            return;
        }
        _running_tx_source = _cfg.tx_source;
        if (_cfg.tx_source == TxSource::DDS) {
            set_status("DDS started", false);
            return;
        }

        _tx_thread = std::thread([this]() {
            try {
                std::vector<int16_t> tone;
                std::vector<const void*> buffs{tone.data()};
                auto local_bus = _device->get_local_bus();
                const bool timed_mode = (_cfg.tx_mode == TxMode::Timed);
                double tone_phase_rad = 0.0;
                local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET, static_cast<uint32_t>(kRxChunkSamples));
                local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, timed_mode ? 0u : 1u);

                uint64_t timestamp = _device->getTimeTicks();
                if (timed_mode) {
                    const uint64_t delay_ticks =
                        (static_cast<uint64_t>(_cfg.sample_rate_hz) * static_cast<uint64_t>(_cfg.tx_delay_ms)) / 1000ull;
                    timestamp += delay_ticks;
                }

                while (_tx_running.load()) {
                    fill_cs16_tone(
                        tone,
                        kRxChunkSamples,
                        _cfg.sample_rate_hz,
                        _cfg.tx_tone_hz,
                        _cfg.tx_amplitude,
                        tone_phase_rad);
                    buffs[0] = tone.data();
                    const size_t sent = _tx_stream->send(buffs, kRxChunkSamples, timestamp, MICRORF_FORMAT_INT16);
                    tone_phase_rad = advance_tone_phase(
                        tone_phase_rad, _cfg.tx_tone_hz, _cfg.sample_rate_hz, sent);
                    if (sent == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            } catch (const std::exception& ex) {
                set_status(std::string("tx thread failed: ") + ex.what(), true);
                _tx_running.store(false);
            }
        });
        set_status("tx started", false);
    }

    void stop_tx()
    {
        const bool was_running = _tx_running.exchange(false);
        if (was_running) {
            if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(_tx_stream)) {
                packet_stream->request_send_abort();
                packet_stream->end_tx_flow_control_monitoring();
            }
        }
        if (was_running && _tx_stream && _running_tx_source == TxSource::DDS) {
            _tx_stream->set_tx_source(1u);
        }
        if (_tx_thread.joinable()) {
            _tx_thread.join();
        }
        if (was_running && _tx_stream) {
            try {
                _tx_stream->set_stream_tx_stop();
            } catch (const std::exception& ex) {
                set_status(std::string("tx stop failed: ") + ex.what(), true);
            }
        }
    }

    void configure_tx_source()
    {
        if (!_tx_stream) {
            throw std::runtime_error("tx stream not ready");
        }

        if (_cfg.tx_source == TxSource::DDS) {
            _cfg.dds_ctrl_word = dds_ctrl_word_from_freq(
                static_cast<double>(_cfg.dds_hz), static_cast<double>(_cfg.sample_rate_hz));
            _tx_stream->dds_ctrl(_cfg.dds_ctrl_word);
            _tx_stream->set_tx_source(2u);
        } else {
            _tx_stream->set_tx_source(1u);
        }
    }

    void update_rx_plots(const std::vector<int16_t>& rx_buffer, size_t received, uint64_t timestamp)
    {
        bool refresh_plots = false;
        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(_plot_mutex);
            _rx_metrics.last_timestamp = timestamp;
            _rx_metrics.packet_count++;
            _rx_metrics.total_samples += received;
            if (received == 0) {
                _rx_metrics.zero_reads++;
                return;
            }
            if (received != kRxReadSamples) {
                _rx_metrics.short_reads++;
            }
            if (!_rx_metrics.have_timestamp) {
                _rx_metrics.have_timestamp = true;
                _rx_metrics.first_timestamp = timestamp;
                _rx_metrics.expected_timestamp = timestamp + received;
                _rx_metrics.last_timestamp_delta = 0;
            } else {
                if (timestamp != _rx_metrics.expected_timestamp) {
                    _rx_metrics.timestamp_gaps++;
                    if (timestamp < _rx_metrics.expected_timestamp) {
                        _rx_metrics.timestamp_backwards++;
                        _rx_metrics.last_timestamp_delta =
                            -static_cast<int64_t>(_rx_metrics.expected_timestamp - timestamp);
                    } else {
                        _rx_metrics.last_timestamp_delta =
                            static_cast<int64_t>(timestamp - _rx_metrics.expected_timestamp);
                    }
                } else {
                    _rx_metrics.last_timestamp_delta = 0;
                }
                _rx_metrics.expected_timestamp = timestamp + received;
            }

            if (_last_plot_update.time_since_epoch().count() == 0
                || now - _last_plot_update >= std::chrono::milliseconds(kPlotUpdateIntervalMs)) {
                _last_plot_update = now;
                refresh_plots = true;
            }
        }

        if (!refresh_plots) {
            return;
        }

        std::vector<std::complex<float>> iq;
        iq.reserve(received);
        float peak = 0.0f;
        for (size_t i = 0; i < received; ++i) {
            const float iv = static_cast<float>(rx_buffer[i * 2 + 0]) / kCs16Norm;
            const float qv = static_cast<float>(rx_buffer[i * 2 + 1]) / kCs16Norm;
            peak = std::max(peak, std::max(std::abs(iv), std::abs(qv)));
            iq.emplace_back(iv, qv);
        }

        const size_t waveform_count = std::min(iq.size(), kWaveformSamples);
        std::vector<std::complex<float>> latest_wave(iq.end() - waveform_count, iq.end());
        std::vector<float> plot_i = normalize_to_plot(latest_wave, false);
        std::vector<float> plot_q = normalize_to_plot(latest_wave, true);

        size_t spectrum_fft_size = 0u;
        std::vector<std::complex<float>> latest_fft;
        {
            std::lock_guard<std::mutex> lock(_plot_mutex);
            _rx_metrics.peak = peak;
            _rx_iq_history.insert(_rx_iq_history.end(), iq.begin(), iq.end());
            if (_rx_iq_history.size() > kSpectrumHistoryMaxSamples) {
                _rx_iq_history.erase(
                    _rx_iq_history.begin(),
                    _rx_iq_history.begin() + static_cast<std::ptrdiff_t>(_rx_iq_history.size() - kSpectrumHistoryMaxSamples));
            }
            spectrum_fft_size = std::min(_spectrum_fft_size, _rx_iq_history.size());
            latest_fft = _rx_iq_history;
            _plot_i = std::move(plot_i);
            _plot_q = std::move(plot_q);
        }

        if (spectrum_fft_size == 0u) {
            return;
        }

        if (latest_fft.size() > spectrum_fft_size) {
            latest_fft.erase(latest_fft.begin(), latest_fft.end() - static_cast<std::ptrdiff_t>(spectrum_fft_size));
        }
        std::vector<float> spectrum_db = compute_spectrum_db(latest_fft, _spectrum_fft_size);
        {
            std::lock_guard<std::mutex> lock(_plot_mutex);
            _spectrum_db = std::move(spectrum_db);
            _waterfall.push_back(_spectrum_db);
            while (_waterfall.size() > kWaterfallRows) {
                _waterfall.pop_front();
            }
        }
    }

    void set_firmware_package_path(const std::string& path)
    {
        std::snprintf(_fw_cfg.package_path, sizeof(_fw_cfg.package_path), "%s", path.c_str());
    }

    void append_firmware_log(const std::string& text)
    {
        std::lock_guard<std::mutex> lock(_fw_mutex);
        _fw_log += text;
        constexpr std::size_t max_log_size = 128u * 1024u;
        if (_fw_log.size() > max_log_size) {
            _fw_log.erase(0, _fw_log.size() - max_log_size);
        }
    }

    void set_firmware_status(std::string msg, bool is_error)
    {
        std::lock_guard<std::mutex> lock(_fw_mutex);
        _fw_status = std::move(msg);
        _fw_error = is_error;
    }

    void update_firmware_progress_from_line(const std::string& line)
    {
        int percent = 0;
        if (!parse_leading_percent(line, percent)) {
            return;
        }
        std::lock_guard<std::mutex> lock(_fw_mutex);
        _fw_progress = percent;
    }

    void refresh_firmware_update_state()
    {
        if (!_fw_finished.exchange(false)) {
            return;
        }
        wait_firmware_update();
    }

    void wait_firmware_update()
    {
        if (_fw_thread.joinable()) {
            _fw_thread.join();
        }
    }

    std::string build_firmware_update_command(const std::string& package_path) const
    {
        const std::string script = firmware_update_script_path();
        if (!path_is_regular_file(script)) {
            throw std::runtime_error("update script not found: " + script);
        }
        if (!path_is_regular_file(package_path)) {
            throw std::runtime_error("firmware package not found: " + package_path);
        }
        if (_fw_cfg.ssh_port == 0u || _fw_cfg.ssh_port > 65535u) {
            throw std::runtime_error("invalid SSH port");
        }
        if (_fw_cfg.tcp_port == 0u || _fw_cfg.tcp_port > 65535u) {
            throw std::runtime_error("invalid TCP port");
        }

        std::ostringstream cmd;
        cmd << "bash " << shell_quote(script)
            << " --host " << shell_quote(std::string(_cfg.addr))
            << " --port " << _fw_cfg.ssh_port
            << " --fw-port " << _fw_cfg.tcp_port
            << " --method " << shell_quote(firmware_method_to_label(_fw_cfg.method))
            << " --package " << shell_quote(package_path)
            << (_fw_cfg.reboot ? " --reboot" : " --no-reboot")
            << " 2>&1";
        return cmd.str();
    }

    void start_firmware_update()
    {
        refresh_firmware_update_state();
        if (_fw_running.load()) {
            throw std::runtime_error("firmware update already running");
        }

        stop_tx();
        stop_rx();

        const std::string package_path(_fw_cfg.package_path);
        const std::string command = build_firmware_update_command(package_path);
        {
            std::lock_guard<std::mutex> lock(_fw_mutex);
            _fw_log.clear();
            _fw_status = "starting firmware update";
            _fw_error = false;
            _fw_progress = 0;
        }
        set_status("firmware update started", false);
        _fw_running.store(true);
        _fw_finished.store(false);

        _fw_thread = std::thread([this, command]() {
            int exit_code = -1;
            FILE* pipe = popen(command.c_str(), "r");
            if (!pipe) {
                set_firmware_status("failed to start update script", true);
                append_firmware_log("failed to start update script\n");
                _fw_running.store(false);
                _fw_finished.store(true);
                set_status("firmware update failed", true);
                return;
            }

            append_firmware_log("$ " + command + "\n");
            char buffer[512];
            while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                const std::string line(buffer);
                append_firmware_log(line);
                update_firmware_progress_from_line(line);
            }

            const int status = pclose(pipe);
            if (WIFEXITED(status)) {
                exit_code = WEXITSTATUS(status);
            }

            if (exit_code == 0) {
                {
                    std::lock_guard<std::mutex> lock(_fw_mutex);
                    _fw_progress = 100;
                }
                set_firmware_status("firmware update complete", false);
                set_status("firmware update complete", false);
            } else {
                std::ostringstream oss;
                oss << "firmware update failed, exit code " << exit_code;
                set_firmware_status(oss.str(), true);
                set_status(oss.str(), true);
            }
            _fw_running.store(false);
            _fw_finished.store(true);
        });
    }

    void set_status(std::string msg, bool is_error)
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        _status = std::move(msg);
        _status_error = is_error;
    }

private:
    GuiConfig _cfg{};
    std::unique_ptr<E100Impl> _device;
    rx_streamer::sptr _rx_stream;
    tx_streamer::sptr _tx_stream;
    std::atomic<bool> _connected{false};
    std::atomic<bool> _rx_running{false};
    std::atomic<bool> _tx_running{false};
    TxSource _running_tx_source = TxSource::IQ;
    std::thread _rx_thread;
    std::thread _tx_thread;
    std::mutex _plot_mutex;
    std::vector<float> _plot_i;
    std::vector<float> _plot_q;
    std::vector<std::complex<float>> _rx_iq_history;
    std::vector<float> _spectrum_db;
    std::deque<std::vector<float>> _waterfall;
    SpectrumTraceState _spectrum_trace;
    std::chrono::steady_clock::time_point _last_plot_update{};
    size_t _spectrum_fft_size = 4096u;
    float _spectrum_top_dbfs = 5.0f;
    float _spectrum_bottom_dbfs = -120.0f;
    RxMetrics _rx_metrics{};
    float _tx_tone_hz_edit = 200000.0f;
    bool _tx_tone_hz_edit_active = false;
    std::mutex _status_mutex;
    std::string _status = "idle";
    bool _status_error = false;
    FirmwareUpdateConfig _fw_cfg{};
    std::atomic<bool> _fw_running{false};
    std::atomic<bool> _fw_finished{false};
    std::thread _fw_thread;
    std::mutex _fw_mutex;
    std::string _fw_log;
    std::string _fw_status = "idle";
    bool _fw_error = false;
    int _fw_progress = 0;
};

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

    GLFWwindow* window = glfwCreateWindow(1440, 900, "E100 IQ Taxi GUI", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "glfwCreateWindow failed\n");
        glfwTerminate();
        return 1;
    }

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
    apply_cyberether_style();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    E100GuiApp app;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        app.render();

        ImGui::Render();
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
