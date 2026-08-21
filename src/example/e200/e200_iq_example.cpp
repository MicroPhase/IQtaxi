#include "src/driver/E200/e200_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCs16Peak = 32767.0;
constexpr size_t kMaxUdpPacketSamples = (1472u - 16u) / 4u;

struct Config {
    std::string addr = "192.168.1.10";
    std::string mode = "duplex";
    uint32_t sample_rate_hz = 30720000u;
    uint64_t rx_lo_hz = 2400000000ull;
    uint64_t tx_lo_hz = 2400000000ull;
    uint32_t rx_gain = 20u;
    uint32_t tx_atten = 10u;
    size_t rx_request_samples = 4096;
    size_t tx_packet_samples = 1024;
    double tx_tone_hz = 1000000.0;
    double tx_amplitude = 0.10;
    double duration_sec = 5.0;
};

struct RxStats {
    size_t reads = 0;
    size_t total_samples = 0;
    size_t short_reads = 0;
    size_t zero_reads = 0;
    size_t recv_errors = 0;
    size_t timestamp_gaps = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    double avg_iq_power = 0.0;
    int16_t peak_abs = 0;
    double elapsed_sec = 0.0;
    rx_stream_continuity_snapshot continuity{};
};

struct TxStats {
    size_t iterations = 0;
    size_t total_requested = 0;
    size_t total_sent = 0;
    size_t short_writes = 0;
    size_t zero_writes = 0;
    double elapsed_sec = 0.0;
    tx_flow_control_snapshot flow{};
};

void print_usage()
{
    std::cout
        << "Usage: e200_iq_example [options]\n"
        << "  --addr <ip>               device IP, default 192.168.1.10\n"
        << "  --mode <rx|tx|duplex>     test mode, default duplex\n"
        << "  --duration <sec>          run time, default 5\n"
        << "  --sample-rate <hz>        sample rate, default 30720000\n"
        << "  --rx-lo <hz>              RX LO, default 2400000000\n"
        << "  --tx-lo <hz>              TX LO, default 2400000000\n"
        << "  --rx-gain <idx>           RX gain, default 20\n"
        << "  --tx-atten <idx>          TX attenuation, default 10\n"
        << "  --rx-request <samps>      RX samples per read, default 4096\n"
        << "  --tx-packet <samps>       TX samples per send, default 1024\n"
        << "  --tx-tone <hz>            TX tone frequency, default 1000000\n"
        << "  --tx-amp <0..0.95>        TX tone amplitude, default 0.10\n"
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
        } else if (arg == "--mode") {
            cfg.mode = value("--mode");
        } else if (arg == "--duration") {
            cfg.duration_sec = std::stod(value("--duration"));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(std::stoul(value("--sample-rate")));
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = std::stoull(value("--rx-lo"));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo_hz = std::stoull(value("--tx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = static_cast<uint32_t>(std::stoul(value("--rx-gain")));
        } else if (arg == "--tx-atten") {
            cfg.tx_atten = static_cast<uint32_t>(std::stoul(value("--tx-atten")));
        } else if (arg == "--rx-request") {
            cfg.rx_request_samples = static_cast<size_t>(std::stoull(value("--rx-request")));
        } else if (arg == "--tx-packet") {
            cfg.tx_packet_samples = static_cast<size_t>(std::stoull(value("--tx-packet")));
        } else if (arg == "--tx-tone") {
            cfg.tx_tone_hz = std::stod(value("--tx-tone"));
        } else if (arg == "--tx-amp") {
            cfg.tx_amplitude = std::stod(value("--tx-amp"));
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.mode != "rx" && cfg.mode != "tx" && cfg.mode != "duplex") {
        throw std::runtime_error("--mode must be rx, tx, or duplex");
    }
    cfg.tx_amplitude = std::max(0.0, std::min(cfg.tx_amplitude, 0.95));
}

void print_config(const Config& cfg)
{
    std::cout << "E200 IQ example\n"
              << "  addr          : " << cfg.addr << '\n'
              << "  mode          : " << cfg.mode << '\n'
              << "  duration      : " << cfg.duration_sec << " s\n"
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  rx lo         : " << cfg.rx_lo_hz << '\n'
              << "  tx lo         : " << cfg.tx_lo_hz << '\n'
              << "  rx gain       : " << cfg.rx_gain << '\n'
              << "  tx atten      : " << cfg.tx_atten << '\n'
              << "  rx request    : " << cfg.rx_request_samples << '\n'
              << "  tx packet     : " << cfg.tx_packet_samples << '\n'
              << "  tx tone       : " << cfg.tx_tone_hz << '\n'
              << "  tx amplitude  : " << cfg.tx_amplitude << "\n\n";
}

void configure_device(E200Impl& device, const Config& cfg)
{
    device.set_channel_enable(1u);
    device.set_dma_mode(0u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device.set_rx_freq(cfg.rx_lo_hz, 1);
    device.set_tx_freq(cfg.tx_lo_hz, 1);
    device.set_rx_gain(cfg.rx_gain, 1);
    device.set_tx_atten(cfg.tx_atten, 1);
}

void fill_tone(std::vector<int16_t>& tone,
               size_t samples,
               uint32_t sample_rate_hz,
               double tone_hz,
               double amplitude,
               double phase_rad)
{
    tone.resize(samples * 2);
    const double scale = amplitude * kCs16Peak;
    const double phase_step = 2.0 * kPi * tone_hz / std::max<uint32_t>(sample_rate_hz, 1u);

    for (size_t n = 0; n < samples; ++n) {
        const double phase = phase_rad + phase_step * static_cast<double>(n);
        tone[n * 2 + 0] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
        tone[n * 2 + 1] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
    }
}

double advance_phase(double phase_rad, double tone_hz, uint32_t sample_rate_hz, size_t samples)
{
    const double phase_step = 2.0 * kPi * tone_hz / std::max<uint32_t>(sample_rate_hz, 1u);
    const double wrapped = std::fmod(phase_rad + phase_step * static_cast<double>(samples), 2.0 * kPi);
    return (wrapped < 0.0) ? (wrapped + 2.0 * kPi) : wrapped;
}

RxStats run_rx(const rx_streamer::sptr& rx_stream, const Config& cfg, std::atomic<bool>& stop)
{
    RxStats stats;
    std::vector<int16_t> rx_buffer(cfg.rx_request_samples * 2);
    std::vector<void*> rx_buffs{rx_buffer.data()};
    uint64_t timestamp = 0;
    uint64_t expected_timestamp = 0;
    bool have_timestamp = false;
    double power_sum = 0.0;

    rx_stream->set_rx_mode(STREAM_MODE);
    rx_stream->set_max_sample_nums_per_packet(kMaxUdpPacketSamples);
    rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, timestamp, 1, 0);

    const auto t0 = std::chrono::steady_clock::now();
    while (!stop.load()) {
        stats.reads++;
        size_t received = 0;
        try {
            received =
                rx_stream->recv(rx_buffs, cfg.rx_request_samples, timestamp, MICRORF_FORMAT_INT16);
        } catch (const std::exception&) {
            stats.recv_errors++;
            stats.zero_reads++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        stats.total_samples += received;

        if (received == 0) {
            stats.zero_reads++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (received != cfg.rx_request_samples) {
            stats.short_reads++;
        }

        if (!have_timestamp) {
            stats.first_timestamp = timestamp;
            expected_timestamp = timestamp + received;
            have_timestamp = true;
        } else if (timestamp != expected_timestamp) {
            stats.timestamp_gaps++;
            expected_timestamp = timestamp + received;
        } else {
            expected_timestamp += received;
        }
        stats.last_timestamp = timestamp;

        for (size_t i = 0; i < received * 2; ++i) {
            const int sample = rx_buffer[i];
            const int abs_sample = std::abs(sample);
            stats.peak_abs = std::max<int16_t>(stats.peak_abs, static_cast<int16_t>(abs_sample));
            power_sum += static_cast<double>(sample) * static_cast<double>(sample);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    if (stats.total_samples > 0) {
        stats.avg_iq_power = power_sum / static_cast<double>(stats.total_samples * 2);
    }
    if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(rx_stream)) {
        stats.continuity = packet_stream->get_rx_continuity_stats();
    }

    uint64_t stop_timestamp = 0;
    rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, stop_timestamp, 0, 1);
    rx_stream->set_rx_mode_exit();
    return stats;
}

TxStats run_tx(E200Impl& device,
               const tx_streamer::sptr& tx_stream,
               const Config& cfg,
               std::atomic<bool>& stop)
{
    TxStats stats;
    auto local_bus = device.get_local_bus();
    local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET,
                      static_cast<uint32_t>(cfg.tx_packet_samples));
    local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, 1u);

    tx_stream->set_tx_source(1u);
    if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx_stream)) {
        packet_stream->begin_tx_flow_control_monitoring();
    }
    tx_stream->set_stream_tx_start();

    std::vector<int16_t> tone;
    std::vector<const void*> tx_buffs{nullptr};
    uint64_t timestamp = device.getTimeTicks();
    double phase_rad = 0.0;

    const auto t0 = std::chrono::steady_clock::now();
    while (!stop.load()) {
        fill_tone(tone,
                  cfg.tx_packet_samples,
                  cfg.sample_rate_hz,
                  cfg.tx_tone_hz,
                  cfg.tx_amplitude,
                  phase_rad);
        tx_buffs[0] = tone.data();
        const size_t sent =
            tx_stream->send(tx_buffs, cfg.tx_packet_samples, timestamp, MICRORF_FORMAT_INT16);
        phase_rad = advance_phase(phase_rad, cfg.tx_tone_hz, cfg.sample_rate_hz, sent);

        stats.iterations++;
        stats.total_requested += cfg.tx_packet_samples;
        stats.total_sent += sent;
        if (sent == 0) {
            stats.zero_writes++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else if (sent != cfg.tx_packet_samples) {
            stats.short_writes++;
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();

    if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx_stream)) {
        packet_stream->request_send_abort();
        stats.flow = packet_stream->get_tx_flow_control_stats();
        packet_stream->end_tx_flow_control_monitoring();
    }
    tx_stream->set_stream_tx_stop();
    return stats;
}

void print_rx_stats(const RxStats& stats)
{
    const double rate =
        (stats.elapsed_sec > 0.0) ? static_cast<double>(stats.total_samples) / stats.elapsed_sec : 0.0;
    std::cout << "RX stats\n"
              << "  elapsed        : " << std::fixed << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  reads          : " << stats.reads << '\n'
              << "  samples        : " << stats.total_samples << '\n'
              << "  effective rate : " << std::setprecision(0) << rate << " Sa/s\n"
              << "  short reads    : " << stats.short_reads << '\n'
              << "  zero reads     : " << stats.zero_reads << '\n'
              << "  recv errors    : " << stats.recv_errors << '\n'
              << "  timestamp gaps : " << stats.timestamp_gaps << '\n'
              << "  first timestamp: " << stats.first_timestamp << '\n'
              << "  last timestamp : " << stats.last_timestamp << '\n'
              << "  avg IQ power   : " << std::setprecision(2) << stats.avg_iq_power << '\n'
              << "  peak abs       : " << stats.peak_abs << '\n'
              << "  wire packets   : " << stats.continuity.packet_count << '\n'
              << "  wire samples   : " << stats.continuity.sample_count << '\n'
              << "  wire seq errors: " << stats.continuity.seq_errors << '\n'
              << "  host drops     : " << stats.continuity.host_queue_drops << "\n\n";
}

void print_tx_stats(const TxStats& stats)
{
    const double rate =
        (stats.elapsed_sec > 0.0) ? static_cast<double>(stats.total_sent) / stats.elapsed_sec : 0.0;
    std::cout << "TX stats\n"
              << "  elapsed          : " << std::fixed << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  iterations       : " << stats.iterations << '\n'
              << "  requested samples: " << stats.total_requested << '\n'
              << "  sent samples     : " << stats.total_sent << '\n'
              << "  enqueue rate     : " << std::setprecision(0) << rate << " Sa/s\n"
              << "  short writes     : " << stats.short_writes << '\n'
              << "  zero writes      : " << stats.zero_writes << '\n'
              << "  fc ready         : " << (stats.flow.ready_to_send ? "yes" : "no") << '\n'
              << "  fc pauses        : " << stats.flow.fc_pause_count << '\n'
              << "  fc wait timeouts : " << stats.flow.fc_wait_timeout_count << "\n\n";
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

        std::atomic<bool> stop{false};
        std::exception_ptr rx_error;
        std::exception_ptr tx_error;
        RxStats rx_stats;
        TxStats tx_stats;

        std::thread rx_thread;
        std::thread tx_thread;

        if (cfg.mode == "rx" || cfg.mode == "duplex") {
            auto rx_stream = device.get_rx_stream();
            rx_thread = std::thread([&, rx_stream]() {
                try {
                    rx_stats = run_rx(rx_stream, cfg, stop);
                } catch (...) {
                    rx_error = std::current_exception();
                }
            });
        }

        if (cfg.mode == "tx" || cfg.mode == "duplex") {
            auto tx_stream = device.get_tx_stream();
            tx_thread = std::thread([&, tx_stream]() {
                try {
                    tx_stats = run_tx(device, tx_stream, cfg, stop);
                } catch (...) {
                    tx_error = std::current_exception();
                }
            });
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, cfg.duration_sec)));
        stop = true;

        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        if (rx_thread.joinable()) {
            rx_thread.join();
        }

        if (tx_error) {
            std::rethrow_exception(tx_error);
        }
        if (rx_error) {
            std::rethrow_exception(rx_error);
        }

        if (cfg.mode == "rx" || cfg.mode == "duplex") {
            print_rx_stats(rx_stats);
        }
        if (cfg.mode == "tx" || cfg.mode == "duplex") {
            print_tx_stats(tx_stats);
        }

        const bool rx_ok = (cfg.mode == "tx") ||
                           (rx_stats.total_samples > 0 && rx_stats.continuity.host_queue_drops == 0);
        const bool tx_ok = (cfg.mode == "rx") ||
                           (tx_stats.total_sent > 0 && tx_stats.zero_writes == 0);
        std::cout << "result: " << ((rx_ok && tx_ok) ? "PASS" : "CHECK") << '\n';
        return (rx_ok && tx_ok) ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "e200_iq_example failed: " << ex.what() << '\n';
        return 1;
    }
}
