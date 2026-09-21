#include "src/driver/E100/e100_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCs16Peak = 32767.0;
constexpr uint32_t kTxSourceIq = 1u;

std::atomic<bool> g_stop{false};

struct Config {
    std::string addr = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t tx_lo_hz = 80000000ull;
    uint32_t tx_atten_db = 10u;
    double tone_hz = 500000.0;
    double stage_sec = 15.0;
    double gap_sec = 1.0;
    size_t packet_samples = 4096u;
    bool amp_enable = false;
    bool sweep_tones = false;
};

void handle_signal(int)
{
    g_stop.store(true);
}

void print_usage()
{
    std::cout
        << "Usage: e100_tx_harmonic_test [options]\n"
        << "  --addr <ip>             device IP, default 192.168.1.10\n"
        << "  --sample-rate <hz>      sample rate, default 15360000\n"
        << "  --tx-lo <hz>            TX LO, default 80000000 (80 MHz)\n"
        << "  --tx-atten <db>         TX attenuation, default 10\n"
        << "  --tone <hz>             baseband tone, default 500000\n"
        << "  --sweep-tones           test 400/500/600 kHz instead of --tone only\n"
        << "  --stage-sec <sec>       duration of each dBFS level, default 15\n"
        << "  --gap-sec <sec>         silence between levels, default 1\n"
        << "  --packet <samples>      samples per TX packet, default 4096\n"
        << "  --amp                   enable the external TX amplifier\n"
        << "  --no-amp                disable the external TX amplifier (default)\n"
        << "  --help                  show this help\n";
}

void parse_cli(Config& cfg, int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (arg == "--addr") {
            cfg.addr = value("--addr");
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(std::stoul(value("--sample-rate")));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo_hz = std::stoull(value("--tx-lo"));
        } else if (arg == "--tx-atten") {
            cfg.tx_atten_db = static_cast<uint32_t>(std::stoul(value("--tx-atten")));
        } else if (arg == "--tone") {
            cfg.tone_hz = std::stod(value("--tone"));
        } else if (arg == "--stage-sec") {
            cfg.stage_sec = std::stod(value("--stage-sec"));
        } else if (arg == "--gap-sec") {
            cfg.gap_sec = std::stod(value("--gap-sec"));
        } else if (arg == "--packet") {
            cfg.packet_samples = static_cast<size_t>(std::stoull(value("--packet")));
        } else if (arg == "--sweep-tones") {
            cfg.sweep_tones = true;
        } else if (arg == "--amp") {
            cfg.amp_enable = true;
        } else if (arg == "--no-amp") {
            cfg.amp_enable = false;
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.sample_rate_hz == 0u || cfg.tone_hz <= 0.0 ||
        cfg.tone_hz >= static_cast<double>(cfg.sample_rate_hz) / 2.0) {
        throw std::runtime_error("tone must be between 0 and Nyquist");
    }
    if (cfg.tx_atten_db > 50u) {
        throw std::runtime_error("TX attenuation must be in [0, 50]");
    }
    if (cfg.stage_sec <= 0.0 || cfg.gap_sec < 0.0) {
        throw std::runtime_error("invalid stage or gap duration");
    }
    if (cfg.packet_samples == 0u) {
        throw std::runtime_error("packet sample count must be non-zero");
    }
}

void fill_tone(std::vector<int16_t>& iq,
               size_t samples,
               uint32_t sample_rate_hz,
               double tone_hz,
               double amplitude,
               double start_phase)
{
    const double scale = std::clamp(amplitude, 0.0, 1.0) * kCs16Peak;
    const double phase_step = 2.0 * kPi * tone_hz /
                              static_cast<double>(sample_rate_hz);
    iq.resize(samples * 2u);
    for (size_t n = 0; n < samples; ++n) {
        const double phase = start_phase + phase_step * static_cast<double>(n);
        iq[2u * n] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
        iq[2u * n + 1u] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
    }
}

double advance_phase(double phase,
                     double tone_hz,
                     uint32_t sample_rate_hz,
                     size_t samples)
{
    const double phase_step = 2.0 * kPi * tone_hz /
                              static_cast<double>(sample_rate_hz);
    phase = std::fmod(phase + phase_step * static_cast<double>(samples),
                      2.0 * kPi);
    return phase < 0.0 ? phase + 2.0 * kPi : phase;
}

void print_frequencies(const Config& cfg, double tone_hz, double level_dbfs)
{
    auto mhz = [](double hz) { return hz / 1.0e6; };
    const double lo_hz = static_cast<double>(cfg.tx_lo_hz);
    const double wanted_hz = lo_hz + tone_hz;
    std::cout << "\nTX stage: tone=" << std::fixed << std::setprecision(3)
              << mhz(tone_hz) << " MHz, level=" << std::setprecision(1)
              << level_dbfs << " dBFS\n"
              << "  wanted RF: " << std::setprecision(6)
              << mhz(wanted_hz) << " MHz\n"
              << "\n"
              << "  If generated after the RF output becomes nonlinear:\n"
              << "    H7  = 7*(LO+tone):  " << mhz(7.0 * wanted_hz) << " MHz\n"
              << "    H9  = 9*(LO+tone):  " << mhz(9.0 * wanted_hz) << " MHz\n"
              << "    H15 = 15*(LO+tone): " << mhz(15.0 * wanted_hz) << " MHz\n"
              << "    H17 = 17*(LO+tone): " << mhz(17.0 * wanted_hz) << " MHz\n"
              << "\n"
              << "  If generated by odd LO harmonics in the TX mixer:\n"
              << "    7*LO  +/- tone: " << mhz(7.0 * lo_hz - tone_hz)
              << " / " << mhz(7.0 * lo_hz + tone_hz) << " MHz\n"
              << "    9*LO  +/- tone: " << mhz(9.0 * lo_hz - tone_hz)
              << " / " << mhz(9.0 * lo_hz + tone_hz) << " MHz\n"
              << "    15*LO +/- tone: " << mhz(15.0 * lo_hz - tone_hz)
              << " / " << mhz(15.0 * lo_hz + tone_hz) << " MHz\n"
              << "    17*LO +/- tone: " << mhz(17.0 * lo_hz - tone_hz)
              << " / " << mhz(17.0 * lo_hz + tone_hz) << " MHz\n"
              << "  duration: " << std::setprecision(1) << cfg.stage_sec << " s\n"
              << std::flush;
}

void run_stage(const tx_streamer::sptr& tx_stream,
               E100Impl& device,
               const Config& cfg,
               double tone_hz,
               double level_dbfs)
{
    const double amplitude = std::pow(10.0, level_dbfs / 20.0);
    std::vector<int16_t> iq;
    std::vector<const void*> buffs{nullptr};
    uint64_t timestamp = device.getTimeTicks();
    double phase = 0.0;
    size_t short_writes = 0u;
    size_t total_sent = 0u;

    print_frequencies(cfg, tone_hz, level_dbfs);
    tx_stream->set_stream_tx_start();
    try {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(cfg.stage_sec));

        while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
            fill_tone(iq, cfg.packet_samples, cfg.sample_rate_hz,
                      tone_hz, amplitude, phase);
            buffs[0] = iq.data();
            const size_t sent = tx_stream->send(
                buffs, cfg.packet_samples, timestamp, MICRORF_FORMAT_INT16);
            total_sent += sent;
            if (sent != cfg.packet_samples) {
                ++short_writes;
            }
            phase = advance_phase(phase, tone_hz, cfg.sample_rate_hz, sent);
            if (sent == 0u) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        tx_stream->set_stream_tx_stop();
    } catch (...) {
        try {
            tx_stream->set_stream_tx_stop();
        } catch (...) {
        }
        throw;
    }
    std::cout << "  sent samples=" << total_sent
              << ", short writes=" << short_writes << '\n';
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_cli(cfg, argc, argv);
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::cout << "E100 TX harmonic test\n"
                  << "  device       : " << cfg.addr << '\n'
                  << "  sample rate  : " << cfg.sample_rate_hz << " Sa/s\n"
                  << "  TX LO        : " << cfg.tx_lo_hz << " Hz\n"
                  << "  TX attenuation: " << cfg.tx_atten_db << " dB\n"
                  << "  external amp : " << (cfg.amp_enable ? "on" : "off") << '\n'
                  << "  levels       : 0, -6, -12, -18 dBFS\n";

        E100Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            throw std::runtime_error("failed to initialize E100 device");
        }
        device.set_channel_enable(1u);
        device.set_dma_mode(0u);
        device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
        device.set_tx_freq(cfg.tx_lo_hz, 1u);
        device.set_tx_atten(cfg.tx_atten_db, 1u);
        device.set_amp_enable(cfg.amp_enable);
        const uint32_t actual_rate = device.getSampleRate();
        const uint64_t actual_tx_lo = device.get_tx_freq(1u);
        if (actual_rate != cfg.sample_rate_hz) {
            throw std::runtime_error("sample-rate readback mismatch");
        }
        if (actual_tx_lo != cfg.tx_lo_hz) {
            throw std::runtime_error("TX LO readback mismatch");
        }

        auto tx_stream = device.get_tx_stream();
        auto local_bus = device.get_local_bus();
        local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET,
                          static_cast<uint32_t>(cfg.packet_samples));
        local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, 1u);
        tx_stream->set_tx_source(kTxSourceIq);

        const std::vector<double> tones = cfg.sweep_tones
            ? std::vector<double>{400000.0, 500000.0, 600000.0}
            : std::vector<double>{cfg.tone_hz};
        const std::vector<double> levels_dbfs{0.0, -6.0, -12.0, -18.0};

        for (double tone_hz : tones) {
            for (double level_dbfs : levels_dbfs) {
                if (g_stop.load()) {
                    break;
                }
                run_stage(tx_stream, device, cfg, tone_hz, level_dbfs);
                if (!g_stop.load() && cfg.gap_sec > 0.0) {
                    std::cout << "  silence gap: " << cfg.gap_sec << " s\n";
                    std::this_thread::sleep_for(std::chrono::duration<double>(cfg.gap_sec));
                }
            }
        }

        std::cout << "TX harmonic test finished\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "e100_tx_harmonic_test failed: " << ex.what() << '\n';
        return 1;
    }
}
