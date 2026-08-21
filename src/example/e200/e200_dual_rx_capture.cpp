#include "src/driver/E200/e200_impl.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;

namespace {

constexpr size_t kChannels = 2;
constexpr size_t kMaxUdpPacketCs16Items = (1472u - 16u) / 4u;

struct Config {
    std::string addr = "192.168.1.10";
    std::string out_ch0 = "e200_ch0.cs16";
    std::string out_ch1 = "e200_ch1.cs16";
    double duration_sec = 5.0;
    uint32_t sample_rate_hz = 20000000u;
    uint64_t rx_lo_hz = 2400000000ull;
    uint32_t rx_gain = 20u;
    size_t frames_per_read = 4096;
};

struct CaptureStats {
    size_t reads = 0;
    size_t frames = 0;
    size_t short_reads = 0;
    size_t zero_reads = 0;
    size_t odd_items = 0;
    size_t recv_errors = 0;
    size_t timestamp_gaps = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    double elapsed_sec = 0.0;
    rx_stream_continuity_snapshot continuity{};
};

void print_usage()
{
    std::cout
        << "Usage: e200_dual_rx_capture [options]\n"
        << "  --addr <ip>               device IP, default 192.168.1.10\n"
        << "  --duration <sec>          capture duration, default 5\n"
        << "  --sample-rate <hz>        sample rate, default 20000000\n"
        << "  --rx-lo <hz>              RX LO, default 2400000000\n"
        << "  --rx-gain <idx>           RX gain, default 20\n"
        << "  --frames-per-read <n>     two-channel sample frames per read, default 4096\n"
        << "  --out0 <path>             channel 0 cs16 output, default e200_ch0.cs16\n"
        << "  --out1 <path>             channel 1 cs16 output, default e200_ch1.cs16\n"
        << "  --help                    show this help\n";
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
        } else if (arg == "--duration") {
            cfg.duration_sec = std::stod(value("--duration"));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(std::stoul(value("--sample-rate")));
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = std::stoull(value("--rx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = static_cast<uint32_t>(std::stoul(value("--rx-gain")));
        } else if (arg == "--frames-per-read") {
            cfg.frames_per_read = static_cast<size_t>(std::stoull(value("--frames-per-read")));
        } else if (arg == "--out0") {
            cfg.out_ch0 = value("--out0");
        } else if (arg == "--out1") {
            cfg.out_ch1 = value("--out1");
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.duration_sec <= 0.0) {
        throw std::runtime_error("--duration must be greater than zero");
    }
    if (cfg.frames_per_read == 0) {
        throw std::runtime_error("--frames-per-read must be greater than zero");
    }
}

void print_config(const Config& cfg)
{
    std::cout << "E200 dual RX capture\n"
              << "  addr           : " << cfg.addr << '\n'
              << "  duration       : " << cfg.duration_sec << " s\n"
              << "  sample rate    : " << cfg.sample_rate_hz << '\n'
              << "  rx lo          : " << cfg.rx_lo_hz << '\n'
              << "  rx gain        : " << cfg.rx_gain << '\n'
              << "  frames/read    : " << cfg.frames_per_read << '\n'
              << "  ch0 output     : " << cfg.out_ch0 << '\n'
              << "  ch1 output     : " << cfg.out_ch1 << "\n\n";
}

void configure_device(E200Impl& device, const Config& cfg)
{
    device.set_channel_enable(0x3u);
    device.set_dma_mode(0u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device.set_rx_freq(cfg.rx_lo_hz, 1);
    device.set_rx_freq(cfg.rx_lo_hz, 2);
    device.set_rx_gain(cfg.rx_gain, 1);
    device.set_rx_gain(cfg.rx_gain, 2);
}

void write_cs16_frame(std::ofstream& out, const int16_t i, const int16_t q)
{
    const int16_t sample[2] = {i, q};
    out.write(reinterpret_cast<const char*>(sample), sizeof(sample));
    if (!out) {
        throw std::runtime_error("failed to write output file");
    }
}

CaptureStats capture_dual_rx(const rx_streamer::sptr& rx_stream, const Config& cfg)
{
    CaptureStats stats;
    std::ofstream ch0(cfg.out_ch0, std::ios::binary | std::ios::trunc);
    std::ofstream ch1(cfg.out_ch1, std::ios::binary | std::ios::trunc);
    if (!ch0) {
        throw std::runtime_error("failed to open " + cfg.out_ch0);
    }
    if (!ch1) {
        throw std::runtime_error("failed to open " + cfg.out_ch1);
    }

    const size_t request_items = cfg.frames_per_read * kChannels;
    std::vector<int16_t> rx_buffer(request_items * 2);
    std::vector<void*> rx_buffs{rx_buffer.data()};

    uint64_t timestamp = 0;
    uint64_t expected_timestamp = 0;
    bool have_timestamp = false;

    rx_stream->set_rx_mode(STREAM_MODE);
    rx_stream->set_max_sample_nums_per_packet(static_cast<uint32_t>(kMaxUdpPacketCs16Items));
    rx_stream->set_recv_param(STREAM_MODE, request_items, timestamp, 1, 0);

    const auto t0 = std::chrono::steady_clock::now();
    const auto deadline = t0 + std::chrono::duration<double>(cfg.duration_sec);
    while (std::chrono::steady_clock::now() < deadline) {
        stats.reads++;
        size_t received_items = 0;
        try {
            received_items =
                rx_stream->recv(rx_buffs, request_items, timestamp, MICRORF_FORMAT_INT16);
        } catch (const std::exception&) {
            stats.recv_errors++;
            stats.zero_reads++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (received_items == 0) {
            stats.zero_reads++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (received_items != request_items) {
            stats.short_reads++;
        }
        if ((received_items % kChannels) != 0) {
            stats.odd_items++;
            received_items -= received_items % kChannels;
        }

        const size_t received_frames = received_items / kChannels;
        if (!have_timestamp) {
            stats.first_timestamp = timestamp;
            expected_timestamp = timestamp + received_frames;
            have_timestamp = true;
        } else if (timestamp != expected_timestamp) {
            stats.timestamp_gaps++;
            expected_timestamp = timestamp + received_frames;
        } else {
            expected_timestamp += received_frames;
        }
        stats.last_timestamp = timestamp;

        for (size_t frame = 0; frame < received_frames; ++frame) {
            const size_t ch0_base = frame * 4;
            const size_t ch1_base = ch0_base + 2;
            write_cs16_frame(ch0, rx_buffer[ch0_base + 0], rx_buffer[ch0_base + 1]);
            write_cs16_frame(ch1, rx_buffer[ch1_base + 0], rx_buffer[ch1_base + 1]);
        }

        stats.frames += received_frames;
    }

    const auto t1 = std::chrono::steady_clock::now();
    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();

    if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(rx_stream)) {
        stats.continuity = packet_stream->get_rx_continuity_stats();
    }

    uint64_t stop_timestamp = 0;
    rx_stream->set_recv_param(STREAM_MODE, request_items, stop_timestamp, 0, 1);
    rx_stream->set_rx_mode_exit();
    return stats;
}

void print_stats(const CaptureStats& stats)
{
    const double rate =
        (stats.elapsed_sec > 0.0) ? static_cast<double>(stats.frames) / stats.elapsed_sec : 0.0;

    std::cout << "RX capture stats\n"
              << "  elapsed          : " << std::fixed << std::setprecision(3)
              << stats.elapsed_sec << " s\n"
              << "  reads            : " << stats.reads << '\n'
              << "  frames/channel   : " << stats.frames << '\n'
              << "  effective rate   : " << std::setprecision(0) << rate << " Sa/s/ch\n"
              << "  short reads      : " << stats.short_reads << '\n'
              << "  zero reads       : " << stats.zero_reads << '\n'
              << "  recv errors      : " << stats.recv_errors << '\n'
              << "  odd item reads   : " << stats.odd_items << '\n'
              << "  timestamp gaps   : " << stats.timestamp_gaps << '\n'
              << "  first timestamp  : " << stats.first_timestamp << '\n'
              << "  last timestamp   : " << stats.last_timestamp << '\n'
              << "  wire packets     : " << stats.continuity.packet_count << '\n'
              << "  wire cs16 items  : " << stats.continuity.sample_count << '\n'
              << "  wire seq errors  : " << stats.continuity.seq_errors << '\n'
              << "  wire ts errors   : " << stats.continuity.timestamp_errors << '\n'
              << "  wire ts backwards: " << stats.continuity.timestamp_backwards << '\n'
              << "  wire last ts diff: " << stats.continuity.last_timestamp_delta << '\n'
              << "  host drops       : " << stats.continuity.host_queue_drops << "\n\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Logger::getInstance().setOutput(Logger::TO_CONSOLE);
        Logger::getInstance().setLogLevel(Logger::INFO);
        Logger::getInstance().setShowDebugInfo(false);

        Config cfg;
        parse_args(cfg, argc, argv);
        print_config(cfg);

        E200Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            std::cerr << "E200 init failed\n";
            return 1;
        }
        configure_device(device, cfg);

        const auto rx_stream = device.get_rx_stream();
        // recv_packet_streamer defaults to channel 0 in its constructor, so set
        // the two-channel mask again after creating the stream object.
        rx_stream->set_rx_enable_chan(0x3u);
        std::cout << "  rx stream chans: " << rx_stream->get_num_channels() << "\n\n";
        const CaptureStats stats = capture_dual_rx(rx_stream, cfg);
        print_stats(stats);

        const bool ok = stats.frames > 0 && stats.continuity.host_queue_drops == 0 &&
                        stats.recv_errors == 0 && stats.odd_items == 0;
        std::cout << "result: " << (ok ? "PASS" : "CHECK") << '\n';
        return ok ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "e200_dual_rx_capture failed: " << ex.what() << '\n';
        return 1;
    }
}
