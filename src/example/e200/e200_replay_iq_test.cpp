#include "src/driver/E100/local_e100_regs.hpp"
#if defined(IQTAXI_REPLAY_DEVICE_E206)
#include "src/driver/E206/e206_impl.hpp"
using ReplayDevice = E206Impl;
#else
#include "src/driver/E200/e200_impl.hpp"
using ReplayDevice = E200Impl;
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kMiB = 1024u * 1024u;
#if defined(IQTAXI_REPLAY_DEVICE_E206)
constexpr const char* kProgramName = "e206_replay_iq_test";
constexpr const char* kProductName = "E206";
constexpr const char* kDefaultInput = "e206_record.cs16";
constexpr uint32_t kDefaultSampleRateHz = 15360000u;
constexpr uint32_t kReplayMaxMiB = 240u;
#else
constexpr const char* kProgramName = "e200_replay_iq_test";
constexpr const char* kProductName = "E200";
constexpr const char* kDefaultInput = "e200_record.cs16";
constexpr uint32_t kDefaultSampleRateHz = 30720000u;
constexpr uint32_t kReplayMaxMiB = 256u;
#endif
constexpr uint32_t kReplayMaxBytes = kReplayMaxMiB * kMiB;
constexpr uint32_t kMaxChunkBytes = 4u * kMiB;
constexpr uint32_t kDefaultChunkBytes = 4u * kMiB;
constexpr uint32_t kDefaultPacketGapUs = 25u;
constexpr uint32_t kTxSourceIq = 1u;
constexpr uint32_t kDefaultToneLengthMiB = 128u;
constexpr double kDefaultToneHz = 1000000.0;
constexpr double kDefaultToneAmplitude = 0.25;
constexpr double kDefaultLfmStartHz = -5000000.0;
constexpr double kDefaultLfmStopHz = 5000000.0;
constexpr double kTwoPi = 6.283185307179586476925286766559;

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
    stop_requested = 1;
}

struct Config {
    std::string addr = "192.168.1.10";
    std::string input = kDefaultInput;
    uint32_t sample_rate_hz = kDefaultSampleRateHz;
    uint64_t tx_lo_hz = 2400000000ull;
    uint32_t tx_atten = 10u;
    uint32_t chunk_bytes = kDefaultChunkBytes;
    double chunk_timeout_sec = 10.0;
    uint32_t packet_gap_us = kDefaultPacketGapUs;
    bool generate_tone = false;
    bool generate_lfm = false;
    bool generate_only = false;
    uint32_t tone_length_mib = kDefaultToneLengthMiB;
    double tone_hz = kDefaultToneHz;
    double tone_amplitude = kDefaultToneAmplitude;
    double lfm_start_hz = kDefaultLfmStartHz;
    double lfm_stop_hz = kDefaultLfmStopHz;
};

struct WaveformInfo {
    bool is_lfm = false;
    uint64_t samples = 0u;
    int64_t coherent_bin = 0;
    double actual_hz = 0.0;
    double start_hz = 0.0;
    double stop_hz = 0.0;
};

uint32_t parse_u32(const char* value)
{
    return static_cast<uint32_t>(std::stoul(value));
}

uint64_t parse_u64(const char* value)
{
    return static_cast<uint64_t>(std::stoull(value));
}

void print_usage()
{
    std::cout
        << "Usage: " << kProgramName << " [options]\n"
        << "  --addr <ip>              device IP, default 192.168.1.10\n"
        << "  --input <path>           input raw cs16 IQ file, default " << kDefaultInput << "\n"
        << "  --sample-rate <hz>       sample rate, default " << kDefaultSampleRateHz << "\n"
        << "  --tx-lo <hz>             TX LO, default 2400000000\n"
        << "  --tx-atten <db>          TX attenuation, default 10\n"
        << "  --chunk <bytes>          DMA upload chunk, default 4194304, max 4194304\n"
        << "  --chunk-timeout <sec>    UDP/DMA chunk timeout, default 10\n"
        << "  --packet-gap-us <us>     delay between replay UDP IQ packets, default 25\n"
        << "  --generate-tone          generate --input as a coherent cs16 tone before upload\n"
        << "  --generate-lfm           generate --input as a periodic linear FM chirp\n"
        << "  --generate-only          generate the waveform file and exit without accessing SDR\n"
        << "  --length-mb <n>          generated waveform size in MiB, default 128, max "
        << kReplayMaxMiB << "\n"
        << "  --tone-hz <hz>           requested complex baseband tone, default 1000000\n"
        << "  --lfm-start-hz <hz>      LFM start frequency, default -5000000\n"
        << "  --lfm-stop-hz <hz>       LFM stop frequency, default 5000000\n"
        << "  --amplitude <0..1>       waveform amplitude relative to int16 full scale, default 0.25\n"
        << "  --help                   show this help\n";
}

void parse_args(Config& cfg, int argc, char** argv)
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
        } else if (arg == "--input") {
            cfg.input = value("--input");
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = parse_u32(value("--sample-rate"));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo_hz = parse_u64(value("--tx-lo"));
        } else if (arg == "--tx-atten") {
            cfg.tx_atten = parse_u32(value("--tx-atten"));
        } else if (arg == "--chunk") {
            cfg.chunk_bytes = parse_u32(value("--chunk"));
        } else if (arg == "--chunk-timeout") {
            cfg.chunk_timeout_sec = std::stod(value("--chunk-timeout"));
        } else if (arg == "--packet-gap-us") {
            cfg.packet_gap_us = parse_u32(value("--packet-gap-us"));
        } else if (arg == "--generate-tone") {
            cfg.generate_tone = true;
        } else if (arg == "--generate-lfm") {
            cfg.generate_lfm = true;
        } else if (arg == "--generate-only") {
            cfg.generate_only = true;
        } else if (arg == "--length-mb") {
            cfg.tone_length_mib = parse_u32(value("--length-mb"));
        } else if (arg == "--tone-hz") {
            cfg.tone_hz = std::stod(value("--tone-hz"));
        } else if (arg == "--lfm-start-hz") {
            cfg.lfm_start_hz = std::stod(value("--lfm-start-hz"));
        } else if (arg == "--lfm-stop-hz") {
            cfg.lfm_stop_hz = std::stod(value("--lfm-stop-hz"));
        } else if (arg == "--amplitude" || arg == "--tone-amplitude") {
            cfg.tone_amplitude = std::stod(value("--amplitude"));
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.chunk_bytes == 0u || (cfg.chunk_bytes & 0x3u) != 0u) {
        throw std::runtime_error("--chunk must be non-zero and 4-byte aligned");
    }
    if (cfg.chunk_bytes > kMaxChunkBytes) {
        throw std::runtime_error("--chunk must be <= 4194304 bytes");
    }
    if (cfg.generate_tone && cfg.generate_lfm) {
        throw std::runtime_error("--generate-tone and --generate-lfm are mutually exclusive");
    }
    const bool generate_waveform = cfg.generate_tone || cfg.generate_lfm;
    if (cfg.generate_only && !generate_waveform) {
        throw std::runtime_error("--generate-only requires a waveform generator option");
    }
    if (generate_waveform) {
        if (cfg.tone_length_mib == 0u || cfg.tone_length_mib > kReplayMaxMiB) {
            throw std::runtime_error("--length-mb must be in the range 1.." +
                                     std::to_string(kReplayMaxMiB));
        }
        if (cfg.tone_amplitude <= 0.0 || cfg.tone_amplitude > 1.0) {
            throw std::runtime_error("--amplitude must be in the range (0, 1]");
        }
    }
    if (cfg.generate_tone) {
        if (cfg.tone_hz <= 0.0 || cfg.tone_hz >= cfg.sample_rate_hz / 2.0) {
            throw std::runtime_error("--tone-hz must be between 0 and Nyquist");
        }
    }
    if (cfg.generate_lfm) {
        const double nyquist = cfg.sample_rate_hz / 2.0;
        if (cfg.lfm_start_hz >= cfg.lfm_stop_hz) {
            throw std::runtime_error("--lfm-start-hz must be less than --lfm-stop-hz");
        }
        if (cfg.lfm_start_hz <= -nyquist || cfg.lfm_stop_hz >= nyquist) {
            throw std::runtime_error("LFM endpoints must lie strictly inside Nyquist");
        }
    }
#if defined(IQTAXI_REPLAY_DEVICE_E206)
    switch (cfg.sample_rate_hz) {
    case 1920000u:
    case 7680000u:
    case 15360000u:
    case 30720000u:
    case 61440000u:
    case 122880000u:
        break;
    default:
        throw std::runtime_error(
            "E206 supports sample rates 1920000, 7680000, 15360000, 30720000, "
            "61440000 and 122880000 Hz");
    }
    if (cfg.tx_atten > 50u) {
        throw std::runtime_error("E206 --tx-atten must be in the range 0..50 dB");
    }
#endif
}

WaveformInfo generate_tone_file(const Config& cfg)
{
    const uint64_t bytes = static_cast<uint64_t>(cfg.tone_length_mib) * kMiB;
    const uint64_t samples = bytes / 4u;
    const int64_t coherent_bin = static_cast<int64_t>(std::llround(
        cfg.tone_hz * static_cast<double>(samples) /
        static_cast<double>(cfg.sample_rate_hz)));
    const double actual_hz =
        static_cast<double>(coherent_bin) * cfg.sample_rate_hz /
        static_cast<double>(samples);
    const double phase_step = kTwoPi * static_cast<double>(coherent_bin) /
                              static_cast<double>(samples);
    const double scale = cfg.tone_amplitude *
                         static_cast<double>(std::numeric_limits<int16_t>::max());
    constexpr size_t kBlockSamples = 1024u * 1024u;
    std::vector<int16_t> block(kBlockSamples * 2u);

    std::ofstream out(cfg.input, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("failed to create tone file: " + cfg.input);
    }

    std::cout << "generating coherent cs16 tone...\n"
              << "  requested Hz : " << cfg.tone_hz << '\n'
              << "  actual Hz    : " << std::setprecision(12) << actual_hz << '\n'
              << "  samples      : " << samples << '\n';

    uint64_t generated = 0u;
    double phase = 0.0;
    while (generated < samples) {
        const size_t todo = static_cast<size_t>(std::min<uint64_t>(
            kBlockSamples, samples - generated));
        for (size_t n = 0u; n < todo; ++n) {
            block[2u * n] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
            block[2u * n + 1u] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
            phase += phase_step;
            if (phase >= kTwoPi) {
                phase -= kTwoPi;
            }
        }

        out.write(reinterpret_cast<const char*>(block.data()),
                  static_cast<std::streamsize>(todo * 4u));
        if (!out) {
            throw std::runtime_error("failed while writing tone file: " + cfg.input);
        }
        generated += todo;
        std::cout << "\r  generated " << (generated * 4u) << " / " << bytes
                  << " bytes" << std::flush;
    }
    std::cout << '\n';

    WaveformInfo info;
    info.samples = samples;
    info.coherent_bin = coherent_bin;
    info.actual_hz = actual_hz;
    info.start_hz = actual_hz;
    info.stop_hz = actual_hz;
    return info;
}

WaveformInfo generate_lfm_file(const Config& cfg)
{
    const uint64_t bytes = static_cast<uint64_t>(cfg.tone_length_mib) * kMiB;
    const uint64_t samples = bytes / 4u;
    const double requested_center = (cfg.lfm_start_hz + cfg.lfm_stop_hz) * 0.5;
    const double span_hz = cfg.lfm_stop_hz - cfg.lfm_start_hz;
    const int64_t coherent_bin = static_cast<int64_t>(std::llround(
        requested_center * static_cast<double>(samples) /
        static_cast<double>(cfg.sample_rate_hz)));
    const double actual_center =
        static_cast<double>(coherent_bin) * cfg.sample_rate_hz /
        static_cast<double>(samples);
    const double center_shift = actual_center - requested_center;
    const double actual_start = cfg.lfm_start_hz + center_shift;
    const double actual_stop = cfg.lfm_stop_hz + center_shift;
    const double scale = cfg.tone_amplitude *
                         static_cast<double>(std::numeric_limits<int16_t>::max());
    const double phase_step_delta =
        kTwoPi * span_hz /
        (static_cast<double>(cfg.sample_rate_hz) * static_cast<double>(samples));
    double phase_step =
        kTwoPi * (actual_start + span_hz / (2.0 * samples)) /
        static_cast<double>(cfg.sample_rate_hz);
    constexpr size_t kBlockSamples = 1024u * 1024u;
    std::vector<int16_t> block(kBlockSamples * 2u);

    std::ofstream out(cfg.input, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("failed to create LFM file: " + cfg.input);
    }

    std::cout << "generating coherent periodic cs16 LFM...\n"
              << "  requested Hz : " << cfg.lfm_start_hz << " -> "
              << cfg.lfm_stop_hz << '\n'
              << "  actual Hz    : " << std::setprecision(12) << actual_start
              << " -> " << actual_stop << '\n'
              << "  sweep time s : " << std::setprecision(12)
              << static_cast<double>(samples) / cfg.sample_rate_hz << '\n'
              << "  samples      : " << samples << '\n';

    uint64_t generated = 0u;
    double phase = 0.0;
    while (generated < samples) {
        const size_t todo = static_cast<size_t>(std::min<uint64_t>(
            kBlockSamples, samples - generated));
        for (size_t n = 0u; n < todo; ++n) {
            block[2u * n] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
            block[2u * n + 1u] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
            phase += phase_step;
            phase_step += phase_step_delta;
            if (phase >= kTwoPi) {
                phase -= kTwoPi;
            } else if (phase < 0.0) {
                phase += kTwoPi;
            }
        }

        out.write(reinterpret_cast<const char*>(block.data()),
                  static_cast<std::streamsize>(todo * 4u));
        if (!out) {
            throw std::runtime_error("failed while writing LFM file: " + cfg.input);
        }
        generated += todo;
        std::cout << "\r  generated " << (generated * 4u) << " / " << bytes
                  << " bytes" << std::flush;
    }
    std::cout << '\n';

    WaveformInfo info;
    info.is_lfm = true;
    info.samples = samples;
    info.coherent_bin = coherent_bin;
    info.actual_hz = actual_center;
    info.start_hz = actual_start;
    info.stop_hz = actual_stop;
    return info;
}

std::vector<uint8_t> load_iq_file(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open input file: " + path);
    }

    std::vector<uint8_t> data{
        std::istreambuf_iterator<char>(in),
        std::istreambuf_iterator<char>()};

    if (data.empty()) {
        throw std::runtime_error("input file is empty");
    }
    if ((data.size() & 0x3u) != 0u) {
        throw std::runtime_error("input file size must be 4-byte aligned cs16 IQ");
    }
    if (data.size() > kReplayMaxBytes) {
        throw std::runtime_error("input file is larger than " +
                                 std::to_string(kReplayMaxMiB) + " MiB");
    }

    return data;
}

void print_config(const Config& cfg, size_t bytes, const WaveformInfo* waveform)
{
    std::cout << kProductName << " IQ replay test\n"
              << "  addr          : " << cfg.addr << '\n'
              << "  input         : " << cfg.input << '\n'
              << "  bytes         : " << bytes << '\n'
              << "  length MiB    : " << (bytes / kMiB) << '\n'
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  tx lo         : " << cfg.tx_lo_hz << '\n'
              << "  tx atten      : " << cfg.tx_atten << '\n'
              << "  chunk         : " << cfg.chunk_bytes << '\n'
              << "  packet gap us : " << cfg.packet_gap_us << '\n';
    if (waveform && waveform->is_lfm) {
        std::cout << "  LFM Hz        : " << std::setprecision(12)
                  << waveform->start_hz << " -> " << waveform->stop_hz << '\n'
                  << "  RF sweep Hz   : " << std::setprecision(15)
                  << (static_cast<double>(cfg.tx_lo_hz) + waveform->start_hz)
                  << " -> "
                  << (static_cast<double>(cfg.tx_lo_hz) + waveform->stop_hz) << '\n';
    } else if (waveform) {
        std::cout << "  tone Hz       : " << std::setprecision(12)
                  << waveform->actual_hz << '\n'
                  << "  RF output Hz  : " << std::setprecision(15)
                  << (static_cast<double>(cfg.tx_lo_hz) + waveform->actual_hz) << '\n';
    }
    std::cout << '\n';
}

void configure_tx(ReplayDevice& device, const Config& cfg)
{
    device.set_channel_enable(1u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device.set_tx_freq(cfg.tx_lo_hz, 1);
    device.set_tx_atten(cfg.tx_atten, 1);
    device.get_local_bus()->poke32(e100::CUSTOM_SET_TX_SOURCE_SEL, kTxSourceIq);

    const uint32_t actual_rate = device.getSampleRate();
    const uint64_t actual_lo = device.get_tx_freq(1);
    if (actual_rate != cfg.sample_rate_hz) {
        throw std::runtime_error(
            "sample rate readback mismatch: expected=" +
            std::to_string(cfg.sample_rate_hz) + ", actual=" +
            std::to_string(actual_rate));
    }
    if (actual_lo != cfg.tx_lo_hz) {
        throw std::runtime_error(
            "tx lo readback mismatch: expected=" +
            std::to_string(cfg.tx_lo_hz) + ", actual=" +
            std::to_string(actual_lo));
    }
}

class ReplayStopGuard {
public:
    explicit ReplayStopGuard(ReplayDevice& device) : device_(device) {}

    void arm() { armed_ = true; }
    void disarm() { armed_ = false; }

    ~ReplayStopGuard()
    {
        if (!armed_) {
            return;
        }
        try {
            device_.stop_iq_replay();
        } catch (...) {
        }
    }

private:
    ReplayDevice& device_;
    bool armed_ = false;
};

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);
        WaveformInfo waveform;
        const WaveformInfo* waveform_ptr = nullptr;
        if (cfg.generate_tone) {
            waveform = generate_tone_file(cfg);
            waveform_ptr = &waveform;
        } else if (cfg.generate_lfm) {
            waveform = generate_lfm_file(cfg);
            waveform_ptr = &waveform;
        }
        std::vector<uint8_t> iq = load_iq_file(cfg.input);
        print_config(cfg, iq.size(), waveform_ptr);
        if (cfg.generate_only) {
            std::cout << "waveform file generated; SDR was not accessed\n";
            return 0;
        }

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        ReplayDevice device(cfg.addr);
        if (!device.isInitialSuccess()) {
            throw std::runtime_error(std::string("failed to initialize ") +
                                     kProductName + " device");
        }

        configure_tx(device, cfg);
        device.set_iq_replay_packet_gap_us(cfg.packet_gap_us);
        device.configure_iq_replay(static_cast<uint32_t>(iq.size()));
        std::cout << "arming replay buffer update...\n";
        device.start_iq_replay(static_cast<uint32_t>(iq.size()));
        ReplayStopGuard stop_guard(device);
        stop_guard.arm();

        std::cout << "uploading replay IQ...\n";
        size_t uploaded = 0u;
        uint32_t chunk_index = 0u;
        while (uploaded < iq.size()) {
            if (stop_requested != 0) {
                std::cout << "upload interrupted, stopping replay\n";
                device.stop_iq_replay();
                stop_guard.disarm();
                return 130;
            }
            const size_t todo = std::min<size_t>(cfg.chunk_bytes, iq.size() - uploaded);
            ++chunk_index;
            std::cout << "  chunk " << chunk_index << " offset=" << uploaded
                      << " bytes=" << todo << '\n';
            uploaded += device.write_iq_replay_chunk(
                iq.data() + uploaded, todo, cfg.chunk_timeout_sec);
        }
        std::cout << "uploaded " << uploaded << " bytes, dma_offset="
                  << device.get_iq_replay_dma_offset() << '\n';

        std::cout << "replay running continuously; press Ctrl+C to stop\n";
        while (stop_requested == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "stopping replay...\n";
        device.stop_iq_replay();
        stop_guard.disarm();
        std::cout << "replay stopped\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << kProgramName << " failed: " << ex.what() << '\n';
        return 1;
    }
}
