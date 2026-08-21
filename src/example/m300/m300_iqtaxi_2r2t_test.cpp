#include "include/sdr/api/Device.hpp"
#include "src/driver/M300/m300_rx_streamer.hpp"
#include "src/driver/M300/m300_tx_streamer.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using sdr::api::Device;
using sdr::api::rx_streamer;
using sdr::api::tx_streamer;

namespace {

constexpr uint32_t kTxSourceIq = 1u;
constexpr uint32_t kTxSourceH2cSink = 7u;

struct Config {
    std::string addr = "/dev/xdma0";
    double duration_sec = 5.0;
    uint32_t sample_rate_hz = 61'440'000u;
    uint64_t center_freq_hz = 2'400'000'000ull;
    uint32_t rx_gain_db = 20u;
    uint32_t tx_attenuation_db = 30u;
    double tone_hz = 1.0e6;
    float amplitude = 0.1f;
    size_t samples_per_io = 4092u;
    size_t warmup_reads = 128u;
    bool iq_output = false;
};

struct RxStats {
    uint64_t reads = 0u;
    uint64_t samples_per_channel = 0u;
    uint64_t short_reads = 0u;
    uint64_t zero_reads = 0u;
    uint64_t timestamp_gaps = 0u;
    uint64_t packets = 0u;
    uint64_t seq_jumps = 0u;
    uint64_t lost_packets = 0u;
    double elapsed_sec = 0.0;
};

struct TxStats {
    uint64_t sends = 0u;
    uint64_t samples_per_channel = 0u;
    uint64_t short_sends = 0u;
    uint64_t zero_sends = 0u;
    double enqueue_sec = 0.0;
    double drain_sec = 0.0;
};

struct PcieLink {
    std::string current_speed;
    std::string current_width;
    std::string max_speed;
    std::string max_width;
};

std::string read_text_file(const std::string& path)
{
    std::ifstream input(path);
    std::string value;
    std::getline(input, value);
    return value;
}

PcieLink read_pcie_link(const std::string& addr)
{
    PcieLink link;
    const size_t slash = addr.find_last_of('/');
    const std::string base = slash == std::string::npos ? addr : addr.substr(slash + 1u);
    if (base.rfind("xdma", 0u) != 0u || base.find('=') != std::string::npos) {
        return link;
    }
    const std::string device =
        "/sys/class/xdma/" + base + "_control/device/";
    link.current_speed = read_text_file(device + "current_link_speed");
    link.current_width = read_text_file(device + "current_link_width");
    link.max_speed = read_text_file(device + "max_link_speed");
    link.max_width = read_text_file(device + "max_link_width");
    return link;
}

uint64_t parse_u64(const char* value)
{
    return std::stoull(value, nullptr, 0);
}

void print_usage()
{
    std::cout
        << "Usage: m300_iqtaxi_2r2t_test [options]\n"
        << "  --addr <path>              device base, default /dev/xdma0\n"
        << "  --duration <sec>           simultaneous RX/TX time, default 5\n"
        << "  --sample-rate <hz>         per-channel rate, default 61440000\n"
        << "  --center-freq <hz>         RX/TX LO, default 2400000000\n"
        << "  --rx-gain <db>             RX1/RX2 gain, default 20\n"
        << "  --attenuation <db>         TX1/TX2 attenuation, default 30\n"
        << "  --tone-hz <hz>             generated TX tone, default 1000000\n"
        << "  --amplitude <linear>       generated TX amplitude, default 0.1\n"
        << "  --samples <count>          samples per channel per call, default 4092\n"
        << "  --warmup-reads <count>     RX reads excluded from stats, default 128\n"
        << "  --iq-output                route TX data to AD9361 (default is H2C sink)\n"
        << "  --help                     show this help\n";
}

void parse_args(Config& cfg, int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char* option) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + option);
            }
            return argv[++i];
        };

        if (arg == "--addr" || arg == "--base") {
            cfg.addr = value(arg.c_str());
        } else if (arg == "--duration") {
            cfg.duration_sec = std::stod(value("--duration"));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(parse_u64(value("--sample-rate")));
        } else if (arg == "--center-freq") {
            cfg.center_freq_hz = parse_u64(value("--center-freq"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain_db = static_cast<uint32_t>(parse_u64(value("--rx-gain")));
        } else if (arg == "--attenuation") {
            cfg.tx_attenuation_db = static_cast<uint32_t>(parse_u64(value("--attenuation")));
        } else if (arg == "--tone-hz") {
            cfg.tone_hz = std::stod(value("--tone-hz"));
        } else if (arg == "--amplitude") {
            cfg.amplitude = std::stof(value("--amplitude"));
        } else if (arg == "--samples") {
            cfg.samples_per_io = static_cast<size_t>(parse_u64(value("--samples")));
        } else if (arg == "--warmup-reads") {
            cfg.warmup_reads = static_cast<size_t>(parse_u64(value("--warmup-reads")));
        } else if (arg == "--iq-output") {
            cfg.iq_output = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.duration_sec <= 0.0 || cfg.sample_rate_hz == 0u ||
        cfg.samples_per_io == 0u) {
        throw std::runtime_error("duration, sample rate and samples must be positive");
    }
    if (!(cfg.amplitude >= 0.0f && cfg.amplitude <= 1.0f)) {
        throw std::runtime_error("amplitude must be in the range 0..1");
    }
}

std::vector<int16_t> make_tone(const Config& cfg, size_t channel)
{
    constexpr double kPi = 3.14159265358979323846;
    std::vector<int16_t> output(cfg.samples_per_io * 2u);
    for (size_t sample = 0u; sample < cfg.samples_per_io; ++sample) {
        const double phase = 2.0 * kPi * cfg.tone_hz *
                                 static_cast<double>(sample) /
                                 static_cast<double>(cfg.sample_rate_hz) +
                             static_cast<double>(channel) * kPi / 2.0;
        output[sample * 2u] = static_cast<int16_t>(
            cfg.amplitude * 32767.0f * static_cast<float>(std::cos(phase)));
        output[sample * 2u + 1u] = static_cast<int16_t>(
            cfg.amplitude * 32767.0f * static_cast<float>(std::sin(phase)));
    }
    return output;
}

void configure_device(const Device::sptr& device, const Config& cfg)
{
    device->set_dma_mode(0u);
    device->setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    for (size_t channel = 1u; channel <= 2u; ++channel) {
        device->set_rx_freq(cfg.center_freq_hz, channel);
        device->set_rx_gain(cfg.rx_gain_db, channel);
        if (cfg.iq_output) {
            device->set_tx_freq(cfg.center_freq_hz, channel);
            device->set_tx_atten(cfg.tx_attenuation_db, channel);
        }
    }
}

void tx_worker(const tx_streamer::sptr& tx,
               const Config& cfg,
               std::atomic<bool>& stop,
               std::atomic<bool>& started,
               TxStats& stats,
               std::exception_ptr& error)
{
    try {
        std::vector<std::vector<int16_t>> samples;
        samples.push_back(make_tone(cfg, 0u));
        samples.push_back(make_tone(cfg, 1u));
        const std::vector<const void*> sample_ptrs{
            samples[0].data(), samples[1].data()};
        const tx_streamer::buffs_type buffs(sample_ptrs);
        uint64_t timestamp = 0u;

        tx->set_tx_source(cfg.iq_output ? kTxSourceIq : kTxSourceH2cSink);
        tx->set_stream_tx_start();
        started.store(true, std::memory_order_release);
        const auto begin = std::chrono::steady_clock::now();
        while (!stop.load(std::memory_order_acquire)) {
            const size_t sent = tx->send(
                buffs, cfg.samples_per_io, timestamp, MICRORF_FORMAT_INT16);
            ++stats.sends;
            stats.samples_per_channel += sent;
            if (sent == 0u) {
                ++stats.zero_sends;
                std::this_thread::yield();
            } else if (sent != cfg.samples_per_io) {
                ++stats.short_sends;
            }
        }
        const auto enqueue_done = std::chrono::steady_clock::now();
        stats.enqueue_sec = std::chrono::duration<double>(enqueue_done - begin).count();
        tx->set_stream_tx_stop();
        stats.drain_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - enqueue_done).count();
    } catch (...) {
        error = std::current_exception();
        started.store(true, std::memory_order_release);
    }
}

RxStats run_rx(const rx_streamer::sptr& rx,
               const Config& cfg,
               const std::atomic<bool>& tx_started)
{
    RxStats stats;
    std::vector<int16_t> rx0(cfg.samples_per_io * 2u);
    std::vector<int16_t> rx1(cfg.samples_per_io * 2u);
    const std::vector<void*> sample_ptrs{rx0.data(), rx1.data()};
    const rx_streamer::buffs_type buffs(sample_ptrs);
    uint64_t timestamp = 0u;

    rx->set_sampleRate(cfg.sample_rate_hz);
    rx->set_rx_enable_chan(0x03u);
    rx->set_rx_mode(STREAM_MODE);
    rx->set_recv_param(STREAM_MODE, cfg.samples_per_io, timestamp, 1u, 0u);

    while (!tx_started.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    for (size_t read = 0u; read < cfg.warmup_reads; ++read) {
        (void)rx->recv(buffs, cfg.samples_per_io, timestamp, MICRORF_FORMAT_INT16);
    }

    bool have_expected_timestamp = false;
    uint64_t expected_timestamp = 0u;
    const auto begin = std::chrono::steady_clock::now();
    const auto deadline = begin +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.duration_sec));
    while (std::chrono::steady_clock::now() < deadline) {
        const size_t received = rx->recv(
            buffs, cfg.samples_per_io, timestamp, MICRORF_FORMAT_INT16);
        ++stats.reads;
        stats.samples_per_channel += received;
        if (received == 0u) {
            ++stats.zero_reads;
            continue;
        }
        if (received != cfg.samples_per_io) {
            ++stats.short_reads;
        }
        if (have_expected_timestamp && timestamp != expected_timestamp) {
            ++stats.timestamp_gaps;
        }
        expected_timestamp = timestamp + received;
        have_expected_timestamp = true;
    }
    stats.elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();

    uint64_t stop_timestamp = 0u;
    rx->set_recv_param(STREAM_MODE, cfg.samples_per_io, stop_timestamp, 0u, 1u);
    rx->set_rx_mode_exit();
    if (auto m300_rx = std::dynamic_pointer_cast<m300_rx_streamer>(rx)) {
        const auto continuity = m300_rx->get_continuity_stats();
        stats.packets = continuity.packets;
        stats.seq_jumps = continuity.seq_jumps;
        stats.lost_packets = continuity.lost_packets;
    }
    return stats;
}

void print_results(const Config& cfg,
                   const PcieLink& link,
                   const RxStats& rx,
                   const TxStats& tx)
{
    const double rx_msps = static_cast<double>(rx.samples_per_channel) /
                           rx.elapsed_sec / 1.0e6;
    const double tx_msps = static_cast<double>(tx.samples_per_channel) /
                           tx.enqueue_sec / 1.0e6;
    const double rx_payload_mib = rx_msps * 1.0e6 * 2.0 * sizeof(uint32_t) /
                                  (1024.0 * 1024.0);
    const double tx_payload_mib = tx_msps * 1.0e6 * 2.0 * sizeof(uint32_t) /
                                  (1024.0 * 1024.0);
    const double required_payload_mib =
        static_cast<double>(cfg.sample_rate_hz) * 2.0 * sizeof(uint32_t) * 2.0 /
        (1024.0 * 1024.0);

    std::cout << std::fixed << std::setprecision(3)
              << "\nM300 simultaneous 2R2T results\n"
              << "  configured rate : " << static_cast<double>(cfg.sample_rate_hz) / 1.0e6
              << " MS/s/channel\n"
              << "  required payload: " << required_payload_mib << " MiB/s (RX + TX)\n"
              << "  TX destination   : " << (cfg.iq_output ? "AD9361 IQ" : "FPGA H2C sink") << '\n'
              << "  PCIe link        : "
              << (link.current_speed.empty() ? "unknown" : link.current_speed)
              << " x" << (link.current_width.empty() ? "?" : link.current_width)
              << " (endpoint max "
              << (link.max_speed.empty() ? "unknown" : link.max_speed)
              << " x" << (link.max_width.empty() ? "?" : link.max_width) << ")\n"
              << "RX\n"
              << "  per-channel rate : " << rx_msps << " MS/s\n"
              << "  payload rate     : " << rx_payload_mib << " MiB/s\n"
              << "  reads            : " << rx.reads << '\n'
              << "  short/zero       : " << rx.short_reads << '/' << rx.zero_reads << '\n'
              << "  packets          : " << rx.packets << '\n'
              << "  seq jumps/lost   : " << rx.seq_jumps << '/' << rx.lost_packets << '\n'
              << "  timestamp gaps   : " << rx.timestamp_gaps << '\n'
              << "TX\n"
              << "  per-channel rate : " << tx_msps << " MS/s\n"
              << "  payload rate     : " << tx_payload_mib << " MiB/s\n"
              << "  sends            : " << tx.sends << '\n'
              << "  short/zero       : " << tx.short_sends << '/' << tx.zero_sends << '\n'
              << "  queue drain      : " << tx.drain_sec << " s\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);
        const PcieLink pcie_link = read_pcie_link(cfg.addr);
        std::cout << "M300 IQTAXI simultaneous 2R2T test\n"
                  << "  addr            : " << cfg.addr << '\n'
                  << "  duration        : " << cfg.duration_sec << " s\n"
                  << "  sample rate     : " << cfg.sample_rate_hz << " Sa/s/channel\n"
                  << "  samples/call    : " << cfg.samples_per_io << '\n'
                  << "  TX destination  : " << (cfg.iq_output ? "AD9361 IQ" : "FPGA H2C sink")
                  << "\n\n";

        const Device::sptr device = Device::makeDevice("M300_XDMA", cfg.addr);
        if (!device) {
            throw std::runtime_error("failed to open M300_XDMA device");
        }
        configure_device(device, cfg);

        const rx_streamer::sptr rx = device->get_rx_stream();
        const tx_streamer::sptr tx = device->get_tx_stream();
        if (!rx || !tx) {
            throw std::runtime_error("M300 did not provide both RX and TX streams");
        }
        auto m300_tx = std::dynamic_pointer_cast<m300_tx_streamer>(tx);
        if (!m300_tx) {
            throw std::runtime_error("M300 TX streamer type mismatch");
        }
        m300_tx->configure(cfg.samples_per_io, false, 0x03u);

        std::atomic<bool> tx_stop{false};
        std::atomic<bool> tx_started{false};
        TxStats tx_stats;
        std::exception_ptr tx_error;
        std::thread tx_thread(tx_worker,
                              tx,
                              std::cref(cfg),
                              std::ref(tx_stop),
                              std::ref(tx_started),
                              std::ref(tx_stats),
                              std::ref(tx_error));

        RxStats rx_stats;
        try {
            rx_stats = run_rx(rx, cfg, tx_started);
        } catch (...) {
            tx_stop.store(true, std::memory_order_release);
            if (tx_thread.joinable()) {
                tx_thread.join();
            }
            throw;
        }
        tx_stop.store(true, std::memory_order_release);
        tx_thread.join();
        if (tx_error) {
            std::rethrow_exception(tx_error);
        }

        print_results(cfg, pcie_link, rx_stats, tx_stats);
        const double minimum_rate = static_cast<double>(cfg.sample_rate_hz) * 0.98;
        const bool pass = rx_stats.elapsed_sec > 0.0 && tx_stats.enqueue_sec > 0.0 &&
                          static_cast<double>(rx_stats.samples_per_channel) /
                                  rx_stats.elapsed_sec >= minimum_rate &&
                          static_cast<double>(tx_stats.samples_per_channel) /
                                  tx_stats.enqueue_sec >= minimum_rate &&
                          rx_stats.seq_jumps == 0u && rx_stats.lost_packets == 0u &&
                          tx_stats.short_sends == 0u && tx_stats.zero_sends == 0u;
        std::cout << "\nresult: " << (pass ? "PASS" : "CHECK") << '\n';
        return pass ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "m300_iqtaxi_2r2t_test failed: " << ex.what() << '\n';
        return 1;
    }
}
