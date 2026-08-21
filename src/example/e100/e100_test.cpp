#include "src/driver/E100/e100_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"
#include "include/sdr/core/time_spec.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>
#include <inttypes.h>

using namespace sdr::api;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCs16Peak = 32767.0;

enum class TxMode {
    Timed,
    Asap,
};

struct TestConfig {
    std::string addr = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t rx_lo_hz = 2450000000ull;
    uint64_t tx_lo_hz = 1000000000ull;
    uint32_t rx_gain = 30u;
    uint32_t tx_atten = 30u;
    size_t rx_request_samples = 4096;
    size_t rx_iterations = 300;
    size_t tx_packet_samples = 4096;
    size_t tx_iterations = 30000;
    double tx_tone_hz = 2000000.0;
    double tx_amplitude = 0.35;
    uint32_t tx_start_delay_ms = 200u;
    TxMode tx_mode = TxMode::Asap;
};

struct RxStats {
    size_t iterations = 0;
    size_t total_samples = 0;
    size_t short_reads = 0;
    size_t zero_reads = 0;
    size_t discontinuities = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    double elapsed_sec = 0.0;
    double avg_iq_power = 0.0;
    int16_t peak_abs = 0;
    rx_stream_continuity_snapshot continuity{};
};

struct TxStats {
    size_t iterations = 0;
    size_t total_requested = 0;
    size_t total_sent = 0;
    size_t short_writes = 0;
    size_t zero_writes = 0;
    uint32_t tx_buffer_status = 0;
    double elapsed_sec = 0.0;
    uint64_t device_time_before = 0;
    uint64_t scheduled_start_timestamp = 0;
    uint64_t scheduled_end_timestamp = 0;
    uint64_t device_time_after_wait = 0;
    TxMode tx_mode = TxMode::Timed;
};

const char* tx_mode_to_label(TxMode mode)
{
    switch (mode) {
    case TxMode::Timed:
        return "timed";
    case TxMode::Asap:
        return "asap";
    default:
        return "unknown";
    }
}

void apply_cli_overrides(TestConfig& cfg, int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (arg == "--addr") {
            cfg.addr = require_value("--addr");
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = std::stoull(require_value("--rx-lo"));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo_hz = std::stoull(require_value("--tx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = static_cast<uint32_t>(std::stoul(require_value("--rx-gain")));
        } else if (arg == "--tx-atten") {
            cfg.tx_atten = static_cast<uint32_t>(std::stoul(require_value("--tx-atten")));
        } else if (arg == "--rx-iters") {
            cfg.rx_iterations = static_cast<size_t>(std::stoull(require_value("--rx-iters")));
        } else if (arg == "--tx-iters") {
            cfg.tx_iterations = static_cast<size_t>(std::stoull(require_value("--tx-iters")));
        } else if (arg == "--rx-request") {
            cfg.rx_request_samples = static_cast<size_t>(std::stoull(require_value("--rx-request")));
        } else if (arg == "--tx-packet") {
            cfg.tx_packet_samples = static_cast<size_t>(std::stoull(require_value("--tx-packet")));
        } else if (arg == "--tx-tone") {
            cfg.tx_tone_hz = std::stod(require_value("--tx-tone"));
        } else if (arg == "--tx-amp") {
            cfg.tx_amplitude = std::stod(require_value("--tx-amp"));
        } else if (arg == "--tx-delay-ms") {
            cfg.tx_start_delay_ms = static_cast<uint32_t>(std::stoul(require_value("--tx-delay-ms")));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(std::stoul(require_value("--sample-rate")));
        } else if (arg == "--tx-mode") {
            const std::string mode = require_value("--tx-mode");
            if (mode == "timed") {
                cfg.tx_mode = TxMode::Timed;
            } else if (mode == "asap") {
                cfg.tx_mode = TxMode::Asap;
            } else {
                throw std::runtime_error("unsupported --tx-mode: " + mode);
            }
        } else if (arg == "--help") {
            std::cout
                << "Usage: e100_test [options]\n"
                << "  --addr <ip>             device IP, default 192.168.1.10\n"
                << "  --sample-rate <hz>      sample rate, default 15360000\n"
                << "  --rx-lo <hz>            RX LO, default 2450000000\n"
                << "  --tx-lo <hz>            TX LO, default 1000000000\n"
                << "  --rx-gain <idx>         RX gain index, default 30\n"
                << "  --tx-atten <idx>         TX gain index, default 45\n"
                << "  --rx-request <samps>    RX samples per read, default 4096\n"
                << "  --rx-iters <count>      RX loop count, default 300\n"
                << "  --tx-packet <samps>     TX samples per send, default 4096\n"
                << "  --tx-iters <count>      TX loop count, default 3000\n"
                << "  --tx-tone <hz>          TX tone frequency, default 200000\n"
                << "  --tx-amp <0..0.95>      TX tone amplitude, default 0.35\n"
                << "  --tx-delay-ms <ms>      TX schedule lead time, default 200\n"
                << "  --tx-mode <timed|asap>  TX mode, default asap\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
}

void print_config(const TestConfig& cfg)
{
    std::cout << "E100 IQTAXI 15.36M self-test config\n"
              << "  addr          : " << cfg.addr << '\n'
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  rx lo         : " << cfg.rx_lo_hz << '\n'
              << "  tx lo         : " << cfg.tx_lo_hz << '\n'
              << "  rx gain       : " << cfg.rx_gain << '\n'
              << "  tx atten      : " << cfg.tx_atten << '\n'
              << "  rx request    : " << cfg.rx_request_samples << '\n'
              << "  rx iterations : " << cfg.rx_iterations << '\n'
              << "  tx packet     : " << cfg.tx_packet_samples << '\n'
              << "  tx iterations : " << cfg.tx_iterations << '\n'
              << "  tx tone       : " << cfg.tx_tone_hz << '\n'
              << "  tx amplitude  : " << cfg.tx_amplitude << '\n'
              << "  tx mode       : " << tx_mode_to_label(cfg.tx_mode) << '\n'
              << "  tx delay ms   : " << cfg.tx_start_delay_ms << "\n\n";
}

void configure_device(E100Impl& device, const TestConfig& cfg)
{
    device.set_channel_enable(1u);
    device.set_dma_mode(0u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    const uint32_t actual_rate = device.getSampleRate();
    if (actual_rate != cfg.sample_rate_hz) {
        throw std::runtime_error("sample-rate readback mismatch: requested "
            + std::to_string(cfg.sample_rate_hz) + ", got " + std::to_string(actual_rate));
    }

    device.set_rx_freq(cfg.rx_lo_hz, 1);
    device.set_tx_freq(cfg.tx_lo_hz, 1);
    device.set_rx_gain(cfg.rx_gain, 1);
    device.set_tx_atten(cfg.tx_atten, 1);
}

void fill_cs16_tone(
    std::vector<int16_t>& tone,
    size_t num_samples,
    uint32_t sample_rate_hz,
    double tone_hz,
    double amplitude,
    double start_phase_rad)
{
    amplitude = std::max(0.0, std::min(amplitude, 1.0));
    const double scale = amplitude * kCs16Peak;
    tone.resize(num_samples * 2);
    const double phase_step = (2.0 * kPi * tone_hz) / std::max<uint32_t>(sample_rate_hz, 1u);

    for (size_t n = 0; n < num_samples; ++n) {
        const double phase = start_phase_rad + phase_step * static_cast<double>(n);
        tone[n * 2 + 0] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
        tone[n * 2 + 1] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
    }
}

double advance_tone_phase(double phase_rad, double tone_hz, uint32_t sample_rate_hz, size_t samples)
{
    const double phase_step = (2.0 * kPi * tone_hz) / std::max<uint32_t>(sample_rate_hz, 1u);
    const double wrapped = std::fmod(phase_rad + phase_step * static_cast<double>(samples), 2.0 * kPi);
    return (wrapped < 0.0) ? (wrapped + 2.0 * kPi) : wrapped;
}

RxStats run_rx_stability_test(E100Impl& device, const TestConfig& cfg)
{
    RxStats stats;
    auto rx_stream = device.get_rx_stream();

    std::vector<int16_t> rx_buffer(cfg.rx_request_samples * 2);
    std::vector<void*> rx_buffs{rx_buffer.data()};
    uint64_t timestamp = 0;
    uint64_t expected_timestamp = 0;
    bool have_expected_timestamp = false;

    rx_stream->set_rx_mode(STREAM_MODE);
    rx_stream->set_max_sample_nums_per_packet((1472 - 16) / 4);
    rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, timestamp, 1, 0);

    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < cfg.rx_iterations; ++i) {
        const size_t received = rx_stream->recv(rx_buffs, cfg.rx_request_samples, timestamp, MICRORF_FORMAT_INT16);
        stats.iterations++;
        stats.total_samples += received;

        if (received == 0) {
            stats.zero_reads++;
            continue;
        }
        if (received != cfg.rx_request_samples) {
            stats.short_reads++;
        }

        if (!have_expected_timestamp) {
            stats.first_timestamp = timestamp;
            expected_timestamp = timestamp + received;
            have_expected_timestamp = true;
        } else {
            if (timestamp != expected_timestamp) {
                stats.discontinuities++;
            }
            expected_timestamp = timestamp + received;
        }
        stats.last_timestamp = timestamp;

        double power_sum = 0.0;
        for (size_t s = 0; s < received * 2; ++s) {
            const int16_t sample = rx_buffer[s];
            const int abs_sample = std::abs(static_cast<int>(sample));
            stats.peak_abs = std::max<int16_t>(stats.peak_abs, static_cast<int16_t>(abs_sample));
            power_sum += static_cast<double>(sample) * static_cast<double>(sample);
        }
        stats.avg_iq_power += power_sum;
    }
    const auto t1 = std::chrono::steady_clock::now();

    rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, timestamp, 0, 1);
    rx_stream->set_rx_mode_exit();

    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    if (stats.total_samples > 0) {
        stats.avg_iq_power /= static_cast<double>(stats.total_samples * 2);
    }
    if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(rx_stream)) {
        stats.continuity = packet_stream->get_rx_continuity_stats();
    }
    return stats;
}

TxStats run_tx_flow_test(E100Impl& device, const TestConfig& cfg)
{
    TxStats stats;
    auto tx_stream = device.get_tx_stream();
    auto local_bus = device.get_local_bus();

    std::vector<int16_t> tone;
    std::vector<const void*> tx_buffs{nullptr};
    double tone_phase_rad = 0.0;
    const uint64_t schedule_delay_ticks =
        (static_cast<uint64_t>(cfg.sample_rate_hz) * static_cast<uint64_t>(cfg.tx_start_delay_ms)) / 1000ull;
    const uint64_t guard_ticks = std::max<uint64_t>(cfg.sample_rate_hz / 20u, cfg.tx_packet_samples);
    uint64_t timestamp = 0;

    stats.tx_mode = cfg.tx_mode;
    const bool timed_mode = (cfg.tx_mode == TxMode::Timed);
    local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET, static_cast<uint32_t>(cfg.tx_packet_samples));
    local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, timed_mode ? 0u : 1u);
    tx_stream->set_tx_source(1u);
    stats.device_time_before = device.getTimeTicks();
    timestamp = stats.device_time_before + (timed_mode ? schedule_delay_ticks : 0ull);
    stats.scheduled_start_timestamp = timestamp;

    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < cfg.tx_iterations; ++i) {
        fill_cs16_tone(
            tone,
            cfg.tx_packet_samples,
            cfg.sample_rate_hz,
            cfg.tx_tone_hz,
            cfg.tx_amplitude,
            tone_phase_rad);
        tx_buffs[0] = tone.data();
        const size_t sent = tx_stream->send(tx_buffs, cfg.tx_packet_samples, timestamp, MICRORF_FORMAT_INT16);
        tone_phase_rad = advance_tone_phase(
            tone_phase_rad, cfg.tx_tone_hz, cfg.sample_rate_hz, sent);
        stats.iterations++;
        stats.total_requested += cfg.tx_packet_samples;
        stats.total_sent += sent;

        if (sent == 0) {
            stats.zero_writes++;
        } else if (sent != cfg.tx_packet_samples) {
            stats.short_writes++;
        }

        if ((i % 128) == 0) {
            stats.tx_buffer_status = local_bus->peek32(e100::CUSTOM_RB_TX_STREAM_BUFFER_STATUS);
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    stats.scheduled_end_timestamp = timestamp;

    if (timed_mode) {
        while (device.getTimeTicks() + guard_ticks < stats.scheduled_end_timestamp) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    stats.device_time_after_wait = device.getTimeTicks();

    tx_stream->set_stream_tx_stop();
    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return stats;
}

void print_rx_stats(const RxStats& stats, uint32_t sample_rate_hz)
{
    const double effective_rate = (stats.elapsed_sec > 0.0)
        ? static_cast<double>(stats.total_samples) / stats.elapsed_sec
        : 0.0;

    std::cout << "RX stability test\n"
              << "  iterations         : " << stats.iterations << '\n'
              << "  total samples      : " << stats.total_samples << '\n'
              << "  short reads        : " << stats.short_reads << '\n'
              << "  zero reads         : " << stats.zero_reads << '\n'
              << "  timestamp gaps     : " << stats.discontinuities << '\n'
              << "  first timestamp    : " << stats.first_timestamp << '\n'
              << "  last timestamp     : " << stats.last_timestamp << '\n'
              << "  avg IQ power       : " << std::fixed << std::setprecision(2) << stats.avg_iq_power << '\n'
              << "  peak abs           : " << stats.peak_abs << '\n'
              << "  elapsed            : " << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  effective rate     : " << std::setprecision(0) << effective_rate << " Sa/s\n"
              << "  expected rate      : " << sample_rate_hz << " Sa/s\n"
              << "  wire packets       : " << stats.continuity.packet_count << '\n'
              << "  wire samples       : " << stats.continuity.sample_count << '\n'
              << "  wire seq errors    : " << stats.continuity.seq_errors << '\n'
              << "  wire ts errors     : " << stats.continuity.timestamp_errors << '\n'
              << "  wire ts backwards  : " << stats.continuity.timestamp_backwards << '\n'
              << "  wire ts delta      : " << stats.continuity.last_timestamp_delta << '\n'
              << "  host queue drops   : " << stats.continuity.host_queue_drops << '\n'
              << "  host queue peak    : " << stats.continuity.host_queue_depth_peak << '\n'
              << "  result             : "
              << ((stats.zero_reads == 0 && stats.short_reads == 0 && stats.discontinuities == 0) ? "PASS" : "CHECK")
              << "\n\n";
}

void print_tx_stats(const TxStats& stats, uint32_t sample_rate_hz)
{
    const double host_enqueue_rate = (stats.elapsed_sec > 0.0)
        ? static_cast<double>(stats.total_sent) / stats.elapsed_sec
        : 0.0;
    const uint64_t scheduled_samples = stats.scheduled_end_timestamp - stats.scheduled_start_timestamp;
    const double scheduled_duration = static_cast<double>(scheduled_samples) / static_cast<double>(sample_rate_hz);
    const double schedule_lead = static_cast<double>(stats.scheduled_start_timestamp - stats.device_time_before)
        / static_cast<double>(sample_rate_hz);
    const bool timed_mode = (stats.tx_mode == TxMode::Timed);

    std::cout << "TX " << tx_mode_to_label(stats.tx_mode) << " test\n"
              << "  iterations         : " << stats.iterations << '\n'
              << "  requested samples  : " << stats.total_requested << '\n'
              << "  sent samples       : " << stats.total_sent << '\n'
              << "  short writes       : " << stats.short_writes << '\n'
              << "  zero writes        : " << stats.zero_writes << '\n'
              << "  tx buffer status   : 0x" << std::hex << stats.tx_buffer_status << std::dec << '\n'
              << "  host enqueue time  : " << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  host enqueue rate  : " << std::setprecision(0) << host_enqueue_rate << " Sa/s\n"
              << "  playback rate      : " << sample_rate_hz << " Sa/s\n";
    if (timed_mode) {
        std::cout << "  device time before : " << stats.device_time_before << '\n'
                  << "  scheduled start    : " << stats.scheduled_start_timestamp << '\n'
                  << "  scheduled end      : " << stats.scheduled_end_timestamp << '\n'
                  << "  device time after  : " << stats.device_time_after_wait << '\n'
                  << "  schedule lead      : " << std::fixed << std::setprecision(3) << schedule_lead << " s\n"
                  << "  schedule duration  : " << std::setprecision(3) << scheduled_duration << " s\n";
    }
    std::cout << "  result             : "
              << ((stats.zero_writes == 0 && stats.short_writes == 0
                    && (!timed_mode
                        || stats.device_time_after_wait + sample_rate_hz / 100u >= stats.scheduled_end_timestamp))
                      ? "PASS"
                      : "CHECK")
              << "\n\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Logger::getInstance().setOutput(Logger::TO_CONSOLE);
        Logger::getInstance().setLogLevel(Logger::DEBUG);
        Logger::getInstance().setShowDebugInfo(false);

        TestConfig cfg;
        apply_cli_overrides(cfg, argc, argv);
        print_config(cfg);

        E100Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            std::cerr << "device init failed\n";
            return 1;
        }
 
        configure_device(device, cfg);

        const RxStats rx_stats = run_rx_stability_test(device, cfg);
        print_rx_stats(rx_stats, cfg.sample_rate_hz);

        const TxStats tx_stats = run_tx_flow_test(device, cfg);
        print_tx_stats(tx_stats, cfg.sample_rate_hz);

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "e100_test failed: " << ex.what() << '\n';
        return 1;
    }
}
