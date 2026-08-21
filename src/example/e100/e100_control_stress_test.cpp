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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::api;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCs16Peak = 32767.0;

struct Config {
    std::string addr = "192.168.1.10";
    uint32_t sample_rate_hz = 7680000u;
    uint64_t base_lo_hz = 1000000000ull;
    uint32_t rx_gain = 10u;
    uint32_t tx_atten = 20u;
    size_t rx_request_samples = 4096;
    size_t tx_packet_samples = 1024;
    double seconds = 5.0;
    int tune_iters = 200;
    int tune_step_us = 5000;
};

struct RxStats {
    size_t reads = 0;
    size_t samples = 0;
    size_t zero_reads = 0;
    size_t short_reads = 0;
    size_t timestamp_gaps = 0;
};

struct TxStats {
    size_t iters = 0;
    size_t requested = 0;
    size_t sent = 0;
    size_t zero_writes = 0;
    size_t short_writes = 0;
};

struct CtrlStats {
    size_t attempts = 0;
    size_t success = 0;
    size_t failures = 0;
    std::string first_error;
};

void usage()
{
    std::cout
        << "Usage: e100_control_stress_test [options]\n"
        << "  --addr <ip>               default 192.168.1.10\n"
        << "  --sample-rate <hz>        default 7680000\n"
        << "  --base-lo <hz>            default 1000000000\n"
        << "  --seconds <sec>           default 5\n"
        << "  --tune-iters <count>      default 200\n"
        << "  --tune-step-us <usec>     default 5000\n"
        << "  --rx-request <samps>      default 4096\n"
        << "  --tx-packet <samps>       default 1024\n";
}

void parse(Config& cfg, int argc, char** argv)
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
        } else if (arg == "--base-lo") {
            cfg.base_lo_hz = std::stoull(value("--base-lo"));
        } else if (arg == "--seconds") {
            cfg.seconds = std::stod(value("--seconds"));
        } else if (arg == "--tune-iters") {
            cfg.tune_iters = std::stoi(value("--tune-iters"));
        } else if (arg == "--tune-step-us") {
            cfg.tune_step_us = std::stoi(value("--tune-step-us"));
        } else if (arg == "--rx-request") {
            cfg.rx_request_samples = static_cast<size_t>(std::stoull(value("--rx-request")));
        } else if (arg == "--tx-packet") {
            cfg.tx_packet_samples = static_cast<size_t>(std::stoull(value("--tx-packet")));
        } else if (arg == "--help") {
            usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
}

void fill_tone(std::vector<int16_t>& tone,
               size_t samples,
               uint32_t sample_rate_hz,
               double& phase)
{
    tone.resize(samples * 2);
    const double step = 2.0 * kPi * 100000.0 / std::max<uint32_t>(sample_rate_hz, 1u);
    const double amp = 0.05 * kCs16Peak;
    for (size_t i = 0; i < samples; ++i) {
        tone[2 * i + 0] = static_cast<int16_t>(std::lround(amp * std::cos(phase)));
        tone[2 * i + 1] = static_cast<int16_t>(std::lround(amp * std::sin(phase)));
        phase += step;
        if (phase >= 2.0 * kPi) {
            phase -= 2.0 * kPi;
        }
    }
}

void configure(E100Impl& dev, const Config& cfg)
{
    dev.set_channel_enable(1u);
    dev.set_dma_mode(0u);
    dev.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    dev.set_rx_freq(cfg.base_lo_hz, 1);
    dev.set_tx_freq(cfg.base_lo_hz, 1);
    dev.set_rx_gain(cfg.rx_gain, 1);
    dev.set_tx_atten(cfg.tx_atten, 1);
}

void rx_worker(const rx_streamer::sptr& rx,
               const Config& cfg,
               std::atomic<bool>& stop,
               RxStats& stats,
               std::exception_ptr& error)
{
    try {
        std::vector<int16_t> buff(cfg.rx_request_samples * 2);
        std::vector<void*> buffs{buff.data()};
        uint64_t ts = 0;
        uint64_t expected = 0;
        bool have_expected = false;
        while (!stop.load()) {
            const size_t got = rx->recv(buffs, cfg.rx_request_samples, ts, MICRORF_FORMAT_INT16);
            stats.reads++;
            stats.samples += got;
            if (got == 0) {
                stats.zero_reads++;
                continue;
            }
            if (got != cfg.rx_request_samples) {
                stats.short_reads++;
            }
            if (!have_expected) {
                expected = ts + got;
                have_expected = true;
            } else if (ts != expected) {
                stats.timestamp_gaps++;
                expected = ts + got;
            } else {
                expected += got;
            }
        }
    } catch (...) {
        error = std::current_exception();
    }
}

void tx_worker(E100Impl& dev,
               const tx_streamer::sptr& tx,
               const Config& cfg,
               std::atomic<bool>& stop,
               TxStats& stats,
               std::exception_ptr& error)
{
    try {
        auto local_bus = dev.get_local_bus();
        local_bus->poke32(e100::CUSTOM_SET_TX_SAMPLES_PER_PACKET,
                          static_cast<uint32_t>(cfg.tx_packet_samples));
        local_bus->poke32(e100::CUSTOM_SET_TX_IGNORE_TIMESTAMPS, 1u);
        tx->set_tx_source(1u);
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx)) {
            packet_stream->begin_tx_flow_control_monitoring();
        }
        tx->set_stream_tx_start();

        std::vector<int16_t> tone;
        std::vector<const void*> buffs{nullptr};
        double phase = 0.0;
        uint64_t timestamp = dev.getTimeTicks();
        while (!stop.load()) {
            fill_tone(tone, cfg.tx_packet_samples, cfg.sample_rate_hz, phase);
            buffs[0] = tone.data();
            const size_t sent = tx->send_nonblocking(
                buffs, cfg.tx_packet_samples, timestamp, MICRORF_FORMAT_INT16);
            stats.iters++;
            stats.requested += cfg.tx_packet_samples;
            stats.sent += sent;
            if (sent == 0) {
                stats.zero_writes++;
                timestamp += cfg.tx_packet_samples;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else if (sent != cfg.tx_packet_samples) {
                stats.short_writes++;
            }
        }
    } catch (...) {
        error = std::current_exception();
    }
}

void ctrl_worker(E100Impl& dev,
                 const Config& cfg,
                 CtrlStats& stats)
{
    for (int i = 0; i < cfg.tune_iters; ++i) {
        const uint64_t lo = cfg.base_lo_hz + static_cast<uint64_t>(i % 20) * 1000000ull;
        try {
            dev.set_rx_freq(lo, 1);
            dev.set_tx_freq(lo, 1);
            if ((i % 5) == 0) {
                dev.set_rx_gain(5u + static_cast<uint32_t>(i % 30), 1);
                dev.set_tx_atten(10u + static_cast<uint32_t>(i % 20), 1);
            }
            stats.success++;
        } catch (const std::exception& ex) {
            if (stats.first_error.empty()) {
                stats.first_error = ex.what();
            }
            stats.failures++;
        }
        stats.attempts++;
        if (cfg.tune_step_us > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(cfg.tune_step_us));
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Logger::getInstance().setOutput(Logger::TO_CONSOLE);
        Logger::getInstance().setLogLevel(Logger::DEBUG);
        Logger::getInstance().setShowDebugInfo(false);

        Config cfg;
        parse(cfg, argc, argv);
        std::cout << "E100 IQTAXI control stress test\n"
                  << "  addr          : " << cfg.addr << '\n'
                  << "  sample rate   : " << cfg.sample_rate_hz << '\n'
                  << "  base lo       : " << cfg.base_lo_hz << '\n'
                  << "  seconds       : " << cfg.seconds << '\n'
                  << "  tune iters    : " << cfg.tune_iters << '\n'
                  << "  tune step us  : " << cfg.tune_step_us << "\n\n";

        E100Impl dev(cfg.addr);
        if (!dev.isInitialSuccess()) {
            throw std::runtime_error("device init failed");
        }
        configure(dev, cfg);

        auto rx = dev.get_rx_stream();
        auto tx = dev.get_tx_stream();
        uint64_t rx_timestamp = 0;
        rx->set_rx_mode(STREAM_MODE);
        rx->set_max_sample_nums_per_packet((1472 - 16) / 4);
        rx->set_recv_param(STREAM_MODE, cfg.rx_request_samples, rx_timestamp, 1, 0);

        std::atomic<bool> stop{false};
        RxStats rx_stats;
        TxStats tx_stats;
        CtrlStats ctrl_stats;
        std::exception_ptr rx_error;
        std::exception_ptr tx_error;

        std::thread rx_thread(rx_worker, rx, std::cref(cfg), std::ref(stop),
                              std::ref(rx_stats), std::ref(rx_error));
        std::thread tx_thread(tx_worker, std::ref(dev), tx, std::cref(cfg), std::ref(stop),
                              std::ref(tx_stats), std::ref(tx_error));

        const auto ctrl_start = std::chrono::steady_clock::now();
        ctrl_worker(dev, cfg, ctrl_stats);
        const auto min_end = ctrl_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<double>(cfg.seconds));
        while (std::chrono::steady_clock::now() < min_end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        stop = true;
        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx)) {
            packet_stream->request_send_abort();
        }
        if (rx_thread.joinable()) {
            rx_thread.join();
        }
        if (tx_thread.joinable()) {
            tx_thread.join();
        }
        if (rx_error) {
            std::rethrow_exception(rx_error);
        }
        if (tx_error) {
            std::rethrow_exception(tx_error);
        }

        if (auto packet_stream = std::dynamic_pointer_cast<send_packet_streamer>(tx)) {
            packet_stream->end_tx_flow_control_monitoring();
        }
        tx->set_stream_tx_stop();
        uint64_t stop_timestamp = 0;
        rx->set_recv_param(STREAM_MODE, cfg.rx_request_samples, stop_timestamp, 0, 1);
        rx->set_rx_mode_exit();

        std::cout << "RX\n"
                  << "  reads          : " << rx_stats.reads << '\n'
                  << "  samples        : " << rx_stats.samples << '\n'
                  << "  zero reads     : " << rx_stats.zero_reads << '\n'
                  << "  short reads    : " << rx_stats.short_reads << '\n'
                  << "  timestamp gaps : " << rx_stats.timestamp_gaps << "\n\n";
        std::cout << "TX\n"
                  << "  iters          : " << tx_stats.iters << '\n'
                  << "  requested      : " << tx_stats.requested << '\n'
                  << "  sent           : " << tx_stats.sent << '\n'
                  << "  zero writes    : " << tx_stats.zero_writes << '\n'
                  << "  short writes   : " << tx_stats.short_writes << "\n\n";
        std::cout << "CONTROL\n"
                  << "  attempts       : " << ctrl_stats.attempts << '\n'
                  << "  success        : " << ctrl_stats.success << '\n'
                  << "  failures       : " << ctrl_stats.failures << '\n';
        if (!ctrl_stats.first_error.empty()) {
            std::cout << "  first error    : " << ctrl_stats.first_error << '\n';
        }

        const bool ok = ctrl_stats.failures == 0 && rx_stats.samples > 0;
        std::cout << "\nresult: " << (ok ? "PASS" : "CHECK") << '\n';
        return ok ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "e100_control_stress_test failed: " << ex.what() << '\n';
        return 1;
    }
}
