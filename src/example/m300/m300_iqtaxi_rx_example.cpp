#include "include/sdr/api/Device.hpp"
#include "src/driver/M300/m300_rx_streamer.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;

namespace {

struct Config {
    std::string device = "M300_XDMA";
    std::string addr = "/dev/xdma0";
    double duration_sec = 5.0;
    uint32_t sample_rate_hz = 61440000u;
    uint64_t rx_lo_hz = 2400000000ull;
    uint32_t rx_gain = 20u;
    uint8_t channel_enable = 0x03;
    size_t request_samples = 4092;
    size_t warmup_reads = 128;
    bool separate_channels = false;
    bool analyze_samples = false;
};

struct RxStats {
    size_t channels = 1;
    size_t reads = 0;
    size_t total_samples = 0;
    size_t short_reads = 0;
    size_t zero_reads = 0;
    size_t timestamp_gaps = 0;
    uint64_t packets = 0;
    uint64_t seq_jumps = 0;
    uint64_t lost_packets = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    uint64_t timestamp_step = 0;
    double avg_iq_power = 0.0;
    int16_t peak_abs = 0;
    double elapsed_sec = 0.0;
};

class RxStreamGuard {
public:
    RxStreamGuard(const rx_streamer::sptr& rx_stream, size_t request_samples)
        : _rx_stream(rx_stream)
        , _request_samples(request_samples)
    {
    }

    ~RxStreamGuard()
    {
        stop();
    }

    void stop()
    {
        if (!_rx_stream || _stopped) {
            return;
        }
        uint64_t stop_timestamp = 0;
        try {
            _rx_stream->set_recv_param(STREAM_MODE, _request_samples, stop_timestamp, 0, 1);
            _rx_stream->set_rx_mode_exit();
        } catch (...) {
        }
        _stopped = true;
    }

private:
    rx_streamer::sptr _rx_stream;
    size_t _request_samples = 0;
    bool _stopped = false;
};

void print_usage()
{
    std::cout
        << "Usage: m300_iqtaxi_rx_example [options]\n"
        << "  --device <name>            IQTAXI device name, default M300_XDMA\n"
        << "  --addr <path>              device path, default /dev/xdma0\n"
        << "  --duration <sec>           run time, default 5\n"
        << "  --sample-rate <hz>         sample rate, default 61440000\n"
        << "  --rx-lo <hz>               RX LO, default 2400000000\n"
        << "  --rx-gain <db>             RX gain, default 20\n"
        << "  --channel-enable <mask>    RX channel mask, default 0x3\n"
        << "  --request <samples>        samples per read, default 4092\n"
        << "  --warmup-reads <N>         reads to discard before stats, default 128\n"
        << "  --separate-channels        use one output buffer per enabled channel\n"
        << "  --analyze-samples          compute simple power/peak stats while receiving\n"
        << "  --help                     show this help\n";
}

uint64_t parse_u64(const char* text)
{
    return std::stoull(text, nullptr, 0);
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

        if (arg == "--device") {
            cfg.device = value("--device");
        } else if (arg == "--addr" || arg == "--base") {
            cfg.addr = value(arg.c_str());
        } else if (arg == "--duration") {
            cfg.duration_sec = std::stod(value("--duration"));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(parse_u64(value("--sample-rate")));
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = parse_u64(value("--rx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = static_cast<uint32_t>(parse_u64(value("--rx-gain")));
        } else if (arg == "--channel-enable") {
            cfg.channel_enable = static_cast<uint8_t>(parse_u64(value("--channel-enable")));
        } else if (arg == "--request") {
            cfg.request_samples = static_cast<size_t>(parse_u64(value("--request")));
        } else if (arg == "--warmup-reads") {
            cfg.warmup_reads = static_cast<size_t>(parse_u64(value("--warmup-reads")));
        } else if (arg == "--separate-channels") {
            cfg.separate_channels = true;
        } else if (arg == "--analyze-samples") {
            cfg.analyze_samples = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.request_samples == 0) {
        throw std::runtime_error("--request must be greater than zero");
    }
    if (cfg.duration_sec < 0.0) {
        throw std::runtime_error("--duration must not be negative");
    }
}

void print_config(const Config& cfg)
{
    std::cout << "M300 IQTAXI RX example\n"
              << "  device        : " << cfg.device << '\n'
              << "  addr          : " << cfg.addr << '\n'
              << "  duration      : " << cfg.duration_sec << " s\n"
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  rx lo         : " << cfg.rx_lo_hz << '\n'
              << "  rx gain       : " << cfg.rx_gain << '\n'
              << "  channel enable: 0x" << std::hex << static_cast<unsigned>(cfg.channel_enable)
              << std::dec << '\n'
              << "  request       : " << cfg.request_samples << '\n'
              << "  warmup reads  : " << cfg.warmup_reads << '\n'
              << "  separate bufs : " << (cfg.separate_channels ? "yes" : "no") << '\n'
              << "  analyze       : " << (cfg.analyze_samples ? "yes" : "no") << "\n\n";
}

void configure_device(const Device::sptr& device, const Config& cfg)
{
    device->set_dma_mode(0u);
    device->setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device->set_rx_freq(cfg.rx_lo_hz, 1);
    device->set_rx_gain(cfg.rx_gain, 1);
}

RxStats run_rx(const rx_streamer::sptr& rx_stream, const Config& cfg)
{
    RxStats stats;
    uint64_t timestamp = 0;
    bool have_timestamp = false;
    bool have_timestamp_step = false;
    double power_sum = 0.0;

    rx_stream->set_sampleRate(cfg.sample_rate_hz);
    rx_stream->set_rx_enable_chan(cfg.channel_enable);
    stats.channels = std::max<size_t>(rx_stream->get_num_channels(), 1u);
    const size_t buffer_channels = cfg.separate_channels ? stats.channels : 1u;
    std::vector<std::vector<int16_t>> buffers(
        buffer_channels, std::vector<int16_t>(cfg.request_samples * 2u));
    std::vector<void*> buffs;
    buffs.reserve(buffer_channels);
    for (auto& buffer : buffers) {
        buffs.push_back(buffer.data());
    }
    rx_stream->set_rx_mode(STREAM_MODE);
    rx_stream->set_recv_param(STREAM_MODE, cfg.request_samples, timestamp, 1, 0);
    RxStreamGuard stream_guard(rx_stream, cfg.request_samples);

    for (size_t i = 0; i < cfg.warmup_reads; ++i) {
        (void)rx_stream->recv(buffs, cfg.request_samples, timestamp, MICRORF_FORMAT_INT16);
    }

    const auto end_time = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.duration_sec));
    const auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < end_time) {
        const size_t received =
            rx_stream->recv(buffs, cfg.request_samples, timestamp, MICRORF_FORMAT_INT16);
        stats.reads++;
        stats.total_samples += received * buffer_channels;

        if (received == 0) {
            stats.zero_reads++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (received != cfg.request_samples) {
            stats.short_reads++;
        }

        if (!have_timestamp) {
            stats.first_timestamp = timestamp;
            stats.last_timestamp = timestamp;
            have_timestamp = true;
        } else {
            const uint64_t step = timestamp - stats.last_timestamp;
            if (!have_timestamp_step) {
                stats.timestamp_step = step;
                have_timestamp_step = true;
            } else if (step != stats.timestamp_step) {
                stats.timestamp_gaps++;
            }
            stats.last_timestamp = timestamp;
        }

        if (cfg.analyze_samples) {
            for (const auto& buffer : buffers) {
                for (size_t i = 0; i < received * 2u; ++i) {
                    const int sample = buffer[i];
                    const int abs_sample = std::abs(sample);
                    stats.peak_abs = std::max<int16_t>(
                        stats.peak_abs, static_cast<int16_t>(abs_sample));
                    power_sum += static_cast<double>(sample) * static_cast<double>(sample);
                }
            }
        }
    }

    const auto stop_time = std::chrono::steady_clock::now();
    stats.elapsed_sec = std::chrono::duration<double>(stop_time - start_time).count();
    if (cfg.analyze_samples && stats.total_samples > 0) {
        stats.avg_iq_power = power_sum / static_cast<double>(stats.total_samples * 2u);
    }

    stream_guard.stop();
    if (auto m300_rx = std::dynamic_pointer_cast<m300_rx_streamer>(rx_stream)) {
        const auto continuity = m300_rx->get_continuity_stats();
        stats.packets = continuity.packets;
        stats.seq_jumps = continuity.seq_jumps;
        stats.lost_packets = continuity.lost_packets;
    }
    return stats;
}

void print_stats(const RxStats& stats)
{
    const double aggregate_rate = stats.elapsed_sec > 0.0 ?
        static_cast<double>(stats.total_samples) / stats.elapsed_sec : 0.0;
    const double per_channel_rate =
        aggregate_rate / static_cast<double>(std::max<size_t>(stats.channels, 1u));

    std::cout << "RX stats\n"
              << "  elapsed        : " << std::fixed << std::setprecision(3)
              << stats.elapsed_sec << " s\n"
              << "  channels       : " << stats.channels << '\n'
              << "  reads          : " << stats.reads << '\n'
              << "  samples        : " << stats.total_samples << '\n'
              << "  aggregate rate : " << std::setprecision(0) << aggregate_rate << " Sa/s\n"
              << "  per-chan rate  : " << std::setprecision(0) << per_channel_rate << " Sa/s\n"
              << "  short reads    : " << stats.short_reads << '\n'
              << "  zero reads     : " << stats.zero_reads << '\n'
              << "  packets        : " << stats.packets << '\n'
              << "  seq jumps      : " << stats.seq_jumps << '\n'
              << "  lost packets   : " << stats.lost_packets << '\n'
              << "  timestamp gaps : " << stats.timestamp_gaps << '\n'
              << "  timestamp step : " << stats.timestamp_step << '\n'
              << "  first timestamp: " << stats.first_timestamp << '\n'
              << "  last timestamp : " << stats.last_timestamp << '\n';
    if (stats.peak_abs > 0 || stats.avg_iq_power > 0.0) {
        std::cout << "  avg IQ power   : " << std::setprecision(2) << stats.avg_iq_power << '\n'
                  << "  peak abs       : " << stats.peak_abs << '\n';
    }
    std::cout << '\n';
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);
        print_config(cfg);

        auto device = Device::makeDevice(cfg.device, cfg.addr);
        if (!device) {
            throw std::runtime_error("failed to open IQTAXI device");
        }

        configure_device(device, cfg);
        auto rx_stream = device->get_rx_stream();
        if (!rx_stream) {
            throw std::runtime_error("device does not provide an RX stream");
        }

        const RxStats stats = run_rx(rx_stream, cfg);
        print_stats(stats);

        const bool ok = stats.total_samples > 0 && stats.seq_jumps == 0;
        std::cout << "result: " << (ok ? "PASS" : "CHECK") << '\n';
        return ok ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "m300_iqtaxi_rx_example failed: " << ex.what() << '\n';
        return 1;
    }
}
