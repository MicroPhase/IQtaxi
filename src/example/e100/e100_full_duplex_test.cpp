#include "src/driver/E100/e100_impl.hpp"
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

struct Config {
    std::string addr = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t rx_lo_hz = 2450000000ull;
    uint64_t tx_lo_hz = 1000000000ull;
    uint32_t rx_gain = 30u;
    uint32_t tx_atten = 30u;
    size_t rx_request_samples = 4096;
    size_t tx_packet_samples = 1024;
    double tx_tone_hz = 1000000.0;
    double tx_amplitude = 0.10;
    double pre_rx_sec = 1.0;
    double full_duplex_sec = 3.0;
    double post_rx_sec = 1.0;
};

struct RxWindowStats {
    std::string label;
    size_t reads = 0;
    size_t total_samples = 0;
    size_t short_reads = 0;
    size_t zero_reads = 0;
    size_t timestamp_gaps = 0;
    uint64_t first_timestamp = 0;
    uint64_t last_timestamp = 0;
    double elapsed_sec = 0.0;
    double avg_iq_power = 0.0;
    int16_t peak_abs = 0;
    rx_stream_continuity_snapshot wire_begin{};
    rx_stream_continuity_snapshot wire_end{};
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
        << "Usage: e100_full_duplex_test [options]\n"
        << "  --addr <ip>               device IP, default 192.168.1.10\n"
        << "  --sample-rate <hz>        sample rate, default 15360000\n"
        << "  --rx-lo <hz>              RX LO, default 2450000000\n"
        << "  --tx-lo <hz>              TX LO, default 1000000000\n"
        << "  --rx-gain <idx>           RX gain index, default 30\n"
        << "  --tx-atten <idx>          TX attenuation index, default 30\n"
        << "  --rx-request <samps>      RX samples per read, default 4096\n"
        << "  --tx-packet <samps>       TX samples per send, default 1024\n"
        << "  --tx-tone <hz>            TX tone frequency, default 1000000\n"
        << "  --tx-amp <0..0.95>        TX tone amplitude, default 0.10\n"
        << "  --pre-sec <sec>           RX-only baseline duration, default 1\n"
        << "  --fd-sec <sec>            simultaneous RX/TX duration, default 3\n"
        << "  --post-sec <sec>          RX-only recovery duration, default 1\n"
        << "  --help                    show this help\n";
}

void parse_cli(Config& cfg, int argc, char** argv)
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
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = static_cast<uint32_t>(std::stoul(require_value("--sample-rate")));
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = std::stoull(require_value("--rx-lo"));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo_hz = std::stoull(require_value("--tx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = static_cast<uint32_t>(std::stoul(require_value("--rx-gain")));
        } else if (arg == "--tx-atten") {
            cfg.tx_atten = static_cast<uint32_t>(std::stoul(require_value("--tx-atten")));
        } else if (arg == "--rx-request") {
            cfg.rx_request_samples = static_cast<size_t>(std::stoull(require_value("--rx-request")));
        } else if (arg == "--tx-packet") {
            cfg.tx_packet_samples = static_cast<size_t>(std::stoull(require_value("--tx-packet")));
        } else if (arg == "--tx-tone") {
            cfg.tx_tone_hz = std::stod(require_value("--tx-tone"));
        } else if (arg == "--tx-amp") {
            cfg.tx_amplitude = std::stod(require_value("--tx-amp"));
        } else if (arg == "--pre-sec") {
            cfg.pre_rx_sec = std::stod(require_value("--pre-sec"));
        } else if (arg == "--fd-sec") {
            cfg.full_duplex_sec = std::stod(require_value("--fd-sec"));
        } else if (arg == "--post-sec") {
            cfg.post_rx_sec = std::stod(require_value("--post-sec"));
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
}

void print_config(const Config& cfg)
{
    std::cout << "E100 IQTAXI full-duplex test\n"
              << "  addr          : " << cfg.addr << '\n'
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  rx lo         : " << cfg.rx_lo_hz << '\n'
              << "  tx lo         : " << cfg.tx_lo_hz << '\n'
              << "  rx gain       : " << cfg.rx_gain << '\n'
              << "  tx atten      : " << cfg.tx_atten << '\n'
              << "  rx request    : " << cfg.rx_request_samples << '\n'
              << "  tx packet     : " << cfg.tx_packet_samples << '\n'
              << "  tx tone       : " << cfg.tx_tone_hz << '\n'
              << "  tx amplitude  : " << cfg.tx_amplitude << '\n'
              << "  pre/fd/post   : " << cfg.pre_rx_sec << " / "
              << cfg.full_duplex_sec << " / " << cfg.post_rx_sec << " s\n\n";
}

void configure_device(E100Impl& device, const Config& cfg)
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

void fill_cs16_tone(std::vector<int16_t>& tone,
                    size_t num_samples,
                    uint32_t sample_rate_hz,
                    double tone_hz,
                    double amplitude,
                    double start_phase_rad)
{
    amplitude = std::max(0.0, std::min(amplitude, 0.95));
    const double scale = amplitude * kCs16Peak;
    const double phase_step = (2.0 * kPi * tone_hz) / std::max<uint32_t>(sample_rate_hz, 1u);
    tone.resize(num_samples * 2);
    for (size_t n = 0; n < num_samples; ++n) {
        const double phase = start_phase_rad + phase_step * static_cast<double>(n);
        tone[n * 2 + 0] = static_cast<int16_t>(std::lround(scale * std::cos(phase)));
        tone[n * 2 + 1] = static_cast<int16_t>(std::lround(scale * std::sin(phase)));
    }
}

double advance_phase(double phase_rad, double tone_hz, uint32_t sample_rate_hz, size_t samples)
{
    const double phase_step = (2.0 * kPi * tone_hz) / std::max<uint32_t>(sample_rate_hz, 1u);
    const double wrapped = std::fmod(phase_rad + phase_step * static_cast<double>(samples), 2.0 * kPi);
    return (wrapped < 0.0) ? wrapped + 2.0 * kPi : wrapped;
}

rx_stream_continuity_snapshot rx_continuity(const rx_streamer::sptr& rx_stream)
{
    if (auto packet_stream = std::dynamic_pointer_cast<recv_packet_streamer>(rx_stream)) {
        return packet_stream->get_rx_continuity_stats();
    }
    return rx_stream_continuity_snapshot{};
}

RxWindowStats run_rx_window(const rx_streamer::sptr& rx_stream,
                            const Config& cfg,
                            const char* label,
                            double seconds)
{
    RxWindowStats stats;
    stats.label = label;
    stats.wire_begin = rx_continuity(rx_stream);

    std::vector<int16_t> rx_buffer(cfg.rx_request_samples * 2);
    std::vector<void*> rx_buffs{rx_buffer.data()};
    uint64_t timestamp = 0;
    uint64_t expected_timestamp = 0;
    bool have_expected_timestamp = false;
    double power_sum = 0.0;

    const auto t0 = std::chrono::steady_clock::now();
    const auto deadline = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(std::max(seconds, 0.0)));
    while (std::chrono::steady_clock::now() < deadline) {
        const size_t received =
            rx_stream->recv(rx_buffs, cfg.rx_request_samples, timestamp, MICRORF_FORMAT_INT16);
        stats.reads++;
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
        } else if (timestamp != expected_timestamp) {
            stats.timestamp_gaps++;
            expected_timestamp = timestamp + received;
        } else {
            expected_timestamp += received;
        }
        stats.last_timestamp = timestamp;

        for (size_t s = 0; s < received * 2; ++s) {
            const int16_t sample = rx_buffer[s];
            const int abs_sample = std::abs(static_cast<int>(sample));
            stats.peak_abs = std::max<int16_t>(stats.peak_abs, static_cast<int16_t>(abs_sample));
            power_sum += static_cast<double>(sample) * static_cast<double>(sample);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    stats.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    stats.wire_end = rx_continuity(rx_stream);
    if (stats.total_samples > 0) {
        stats.avg_iq_power = power_sum / static_cast<double>(stats.total_samples * 2);
    }
    return stats;
}

void tx_worker(E100Impl& device,
               const tx_streamer::sptr& tx_stream,
               const Config& cfg,
               std::atomic<bool>& stop,
               TxStats& stats,
               std::exception_ptr& error)
{
    try {
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
        double phase_rad = 0.0;
        uint64_t timestamp = device.getTimeTicks();
        const auto t0 = std::chrono::steady_clock::now();

        while (!stop.load()) {
            fill_cs16_tone(tone,
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
    } catch (...) {
        error = std::current_exception();
    }
}

uint64_t diff_u64(uint64_t end, uint64_t begin)
{
    return (end >= begin) ? (end - begin) : 0;
}

void print_rx_stats(const RxWindowStats& stats)
{
    const double effective_rate =
        (stats.elapsed_sec > 0.0) ? static_cast<double>(stats.total_samples) / stats.elapsed_sec : 0.0;
    const uint64_t wire_packets = diff_u64(stats.wire_end.packet_count, stats.wire_begin.packet_count);
    const uint64_t wire_samples = diff_u64(stats.wire_end.sample_count, stats.wire_begin.sample_count);
    const uint64_t wire_seq_errors = diff_u64(stats.wire_end.seq_errors, stats.wire_begin.seq_errors);
    const uint64_t wire_ts_errors = diff_u64(stats.wire_end.timestamp_errors, stats.wire_begin.timestamp_errors);
    const uint64_t host_drops = diff_u64(stats.wire_end.host_queue_drops, stats.wire_begin.host_queue_drops);

    std::cout << "RX window: " << stats.label << '\n'
              << "  elapsed          : " << std::fixed << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  reads            : " << stats.reads << '\n'
              << "  total samples    : " << stats.total_samples << '\n'
              << "  effective rate   : " << std::setprecision(0) << effective_rate << " Sa/s\n"
              << "  short reads      : " << stats.short_reads << '\n'
              << "  zero reads       : " << stats.zero_reads << '\n'
              << "  timestamp gaps   : " << stats.timestamp_gaps << '\n'
              << "  first timestamp  : " << stats.first_timestamp << '\n'
              << "  last timestamp   : " << stats.last_timestamp << '\n'
              << "  avg IQ power     : " << std::setprecision(2) << stats.avg_iq_power << '\n'
              << "  peak abs         : " << stats.peak_abs << '\n'
              << "  wire packets     : " << wire_packets << '\n'
              << "  wire samples     : " << wire_samples << '\n'
              << "  wire seq errors  : " << wire_seq_errors << '\n'
              << "  wire ts errors   : " << wire_ts_errors << '\n'
              << "  host drops       : " << host_drops << '\n'
              << "  host queue peak  : " << stats.wire_end.host_queue_depth_peak << "\n\n";
}

void print_tx_stats(const TxStats& stats)
{
    const double enqueue_rate =
        (stats.elapsed_sec > 0.0) ? static_cast<double>(stats.total_sent) / stats.elapsed_sec : 0.0;
    std::cout << "TX window\n"
              << "  elapsed          : " << std::fixed << std::setprecision(3) << stats.elapsed_sec << " s\n"
              << "  iterations       : " << stats.iterations << '\n'
              << "  requested        : " << stats.total_requested << '\n'
              << "  sent             : " << stats.total_sent << '\n'
              << "  enqueue rate     : " << std::setprecision(0) << enqueue_rate << " Sa/s\n"
              << "  short writes     : " << stats.short_writes << '\n'
              << "  zero writes      : " << stats.zero_writes << '\n'
              << "  fc ready         : " << (stats.flow.ready_to_send ? "yes" : "no") << '\n'
              << "  fc pauses        : " << stats.flow.fc_pause_count << '\n'
              << "  fc wait timeouts : " << stats.flow.fc_wait_timeout_count << '\n'
              << "  send buff timeouts: " << stats.flow.send_buff_timeout_count << "\n\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Logger::getInstance().setOutput(Logger::TO_CONSOLE);
        Logger::getInstance().setLogLevel(Logger::DEBUG);
        Logger::getInstance().setShowDebugInfo(false);

        Config cfg;
        parse_cli(cfg, argc, argv);
        print_config(cfg);

        E100Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            std::cerr << "device init failed\n";
            return 1;
        }
        configure_device(device, cfg);

        auto rx_stream = device.get_rx_stream();
        auto tx_stream = device.get_tx_stream();
        uint64_t rx_timestamp = 0;
        rx_stream->set_rx_mode(STREAM_MODE);
        rx_stream->set_max_sample_nums_per_packet((1472 - 16) / 4);
        rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, rx_timestamp, 1, 0);

        const RxWindowStats pre = run_rx_window(rx_stream, cfg, "rx-only before TX", cfg.pre_rx_sec);

        std::atomic<bool> tx_stop{false};
        TxStats tx_stats;
        std::exception_ptr tx_error;
        std::thread tx_thread(tx_worker,
                              std::ref(device),
                              tx_stream,
                              std::cref(cfg),
                              std::ref(tx_stop),
                              std::ref(tx_stats),
                              std::ref(tx_error));

        const RxWindowStats full_duplex =
            run_rx_window(rx_stream, cfg, "simultaneous RX + TX", cfg.full_duplex_sec);

        tx_stop = true;
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx_stream)) {
            packet_stream->request_send_abort();
        }
        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx_stream)) {
            tx_stats.flow = packet_stream->get_tx_flow_control_stats();
            packet_stream->end_tx_flow_control_monitoring();
        }
        tx_stream->set_stream_tx_stop();
        if (tx_error) {
            std::rethrow_exception(tx_error);
        }

        const RxWindowStats post = run_rx_window(rx_stream, cfg, "rx-only after TX", cfg.post_rx_sec);

        uint64_t stop_timestamp = 0;
        rx_stream->set_recv_param(STREAM_MODE, cfg.rx_request_samples, stop_timestamp, 0, 1);
        rx_stream->set_rx_mode_exit();

        print_rx_stats(pre);
        print_rx_stats(full_duplex);
        print_tx_stats(tx_stats);
        print_rx_stats(post);

        const bool rx_ok = (pre.zero_reads == 0) &&
                           (full_duplex.zero_reads == 0) &&
                           (post.zero_reads == 0) &&
                           (diff_u64(full_duplex.wire_end.host_queue_drops,
                                     full_duplex.wire_begin.host_queue_drops) == 0);
        const bool tx_ok = (tx_stats.zero_writes == 0) && (tx_stats.short_writes == 0);
        std::cout << "result: " << ((rx_ok && tx_ok) ? "PASS" : "CHECK") << '\n';
        return (rx_ok && tx_ok) ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "e100_full_duplex_test failed: " << ex.what() << '\n';
        return 1;
    }
}
