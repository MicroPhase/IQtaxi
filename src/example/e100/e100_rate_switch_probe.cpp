#include "src/driver/E100/e100_impl.hpp"
#include "src/driver/E100/local_e100_regs.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

using namespace e100;

namespace {

struct ProbeConfig {
    std::string addr = "192.168.1.10";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t rx_lo_hz = 2450000000ull;
    uint64_t tx_lo_hz = 1000000000ull;
    uint32_t rx_gain = 30u;
    uint32_t tx_atten = 30u;
    double set_timeout_sec = 15.0;
    double read_timeout_sec = 15.0;
    bool run_post_config = true;
};

void print_usage()
{
    std::cout
        << "Usage: e100_rate_switch_probe [options]\n"
        << "  --addr <ip>               device IP, default 192.168.1.10\n"
        << "  --sample-rate <hz>        target sample rate, default 15360000\n"
        << "                            supported: 1920000, 3840000, 5760000,\n"
        << "                            7680000, 11520000, 15360000, 23040000,\n"
        << "                            30720000, 61440000, 122880000\n"
        << "  --rx-lo <hz>              RX LO for post-config step\n"
        << "  --tx-lo <hz>              TX LO for post-config step\n"
        << "  --rx-gain <idx>           RX gain for post-config step\n"
        << "  --tx-gain <idx>           TX gain for post-config step\n"
        << "  --set-timeout-sec <sec>   ACK timeout used by set sample-rate command\n"
        << "  --read-timeout-sec <sec>  long timeout used by sample-rate readback\n"
        << "  --no-post-config          skip RX/TX LO and gain restore steps\n"
        << "  --help                    show this help\n";
}

void parse_cli(ProbeConfig& cfg, int argc, char** argv)
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
        } else if (arg == "--tx-gain") {
            cfg.tx_atten = static_cast<uint32_t>(std::stoul(require_value("--tx-gain")));
        } else if (arg == "--set-timeout-sec") {
            cfg.set_timeout_sec = std::stod(require_value("--set-timeout-sec"));
        } else if (arg == "--read-timeout-sec") {
            cfg.read_timeout_sec = std::stod(require_value("--read-timeout-sec"));
        } else if (arg == "--no-post-config") {
            cfg.run_post_config = false;
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
}

double elapsed_ms(const std::chrono::steady_clock::time_point& start,
                  const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

template <typename Fn>
bool timed_step(const char* label, Fn&& fn)
{
    const auto t0 = std::chrono::steady_clock::now();
    try {
        fn();
    } catch (const std::exception& ex) {
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << "[FAIL] " << label << " (" << std::fixed << std::setprecision(3)
                  << elapsed_ms(t0, t1) << " ms): " << ex.what() << '\n';
        return false;
    }

    const auto t1 = std::chrono::steady_clock::now();
    std::cout << "[ OK ] " << label << " (" << std::fixed << std::setprecision(3)
              << elapsed_ms(t0, t1) << " ms)" << '\n';
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    ProbeConfig cfg;
    parse_cli(cfg, argc, argv);

    std::cout << "E100 rate-switch probe\n"
              << "  addr             : " << cfg.addr << '\n'
              << "  target rate      : " << cfg.sample_rate_hz << '\n'
              << "  set timeout sec  : " << cfg.set_timeout_sec << '\n'
              << "  read timeout sec : " << cfg.read_timeout_sec << '\n'
              << "  post config      : " << (cfg.run_post_config ? "on" : "off") << "\n\n";

    E100Impl device(cfg.addr);
    if (!device.isInitialSuccess()) {
        std::cerr << "device init failed\n";
        return 1;
    }

    auto local_bus = device.get_local_bus();
    if (!local_bus) {
        std::cerr << "local bus not ready\n";
        return 1;
    }

    if (!timed_step("set channel enable", [&]() {
            device.set_channel_enable(1u);
        })) {
        return 2;
    }

    if (!timed_step("set dma mode", [&]() {
            device.set_dma_mode(0u);
        })) {
        return 3;
    }

    if (!timed_step("set sample rate", [&]() {
            local_bus->poke32(CUSTOM_SET_SAMPLE_RATE_DY, cfg.sample_rate_hz, cfg.set_timeout_sec);
        })) {
        return 4;
    }

    {
        const auto t0 = std::chrono::steady_clock::now();
        const uint32_t short_rate = local_bus->peek32(CUSTOM_RB_GET_SAMPLE_CLOCK_RATE_ADDR);
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << "[INFO] readback(1.0s default) = " << short_rate
                  << " (" << std::fixed << std::setprecision(3)
                  << elapsed_ms(t0, t1) << " ms)" << '\n';
    }

    {
        const auto t0 = std::chrono::steady_clock::now();
        const uint32_t long_rate =
            local_bus->peek32(CUSTOM_RB_GET_SAMPLE_CLOCK_RATE_ADDR, cfg.read_timeout_sec);
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << "[INFO] readback(" << cfg.read_timeout_sec << "s) = " << long_rate
                  << " (" << std::fixed << std::setprecision(3)
                  << elapsed_ms(t0, t1) << " ms)" << '\n';
        if (long_rate != cfg.sample_rate_hz) {
            std::cout << "[WARN] sample-rate readback mismatch: requested " << cfg.sample_rate_hz
                      << ", got " << long_rate << '\n';
        }
    }

    if (!cfg.run_post_config) {
        return 0;
    }

    if (!timed_step("set RX LO", [&]() {
            device.set_rx_freq(cfg.rx_lo_hz, 1);
        })) {
        return 5;
    }

    if (!timed_step("set TX LO", [&]() {
            device.set_tx_freq(cfg.tx_lo_hz, 1);
        })) {
        return 6;
    }

    if (!timed_step("set RX gain", [&]() {
            device.set_rx_gain(cfg.rx_gain, 1);
        })) {
        return 7;
    }

    if (!timed_step("set TX gain", [&]() {
            device.set_tx_atten(cfg.tx_atten, 1);
        })) {
        return 8;
    }

    return 0;
}
