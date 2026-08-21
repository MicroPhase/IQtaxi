#include "src/driver/E200/e200_impl.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kMiB = 1024u * 1024u;
constexpr uint32_t kRecordLengthGranularityBytes = 4u * kMiB;
constexpr uint32_t kRecordLengthMinBytes = kRecordLengthGranularityBytes;
constexpr uint32_t kRecordLengthMaxBytes = 256u * kMiB;

struct Config {
    std::string addr = "192.168.1.10";
    std::string output = "e200_record.cs16";
    uint32_t sample_rate_hz = 20000000u;
    uint64_t rx_lo_hz = 2400000000ull;
    uint32_t rx_gain = 20u;
    uint32_t record_bytes = 16u * 1024u * 1024u;
    uint32_t dma_block_bytes = kRecordLengthGranularityBytes;
    double record_timeout_sec = 10.0;
    double chunk_timeout_sec = 2.0;
    uint32_t poll_ms = 20u;
    uint32_t settle_ms = 200u;
    uint32_t warmup_bytes = kRecordLengthGranularityBytes;
    uint32_t iterations = 1u;
    bool overwrite_output = false;
};

uint32_t parse_u32(const char* value)
{
    return static_cast<uint32_t>(std::stoul(value));
}

uint64_t parse_u64(const char* value)
{
    return static_cast<uint64_t>(std::stoull(value));
}

uint32_t parse_length_mb(const char* value)
{
    const uint64_t length_mb = parse_u64(value);
    const uint64_t length_bytes = length_mb * kMiB;
    if (length_bytes > UINT32_MAX) {
        throw std::runtime_error("--length-mb is too large");
    }
    return static_cast<uint32_t>(length_bytes);
}

std::string hex_u32(uint32_t value)
{
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(8) << std::setfill('0') << value;
    return oss.str();
}

void print_usage()
{
    std::cout
        << "Usage: e200_record_iq_test [options]\n"
        << "  --addr <ip>              device IP, default 192.168.1.10\n"
        << "  --output <path>          output cs16 file, default e200_record.cs16\n"
        << "  --length-mb <n>          record length in MiB, 4..256 and multiple of 4, default 16\n"
        << "  --bytes <n>              record length in bytes, 4 MiB aligned, default 16777216\n"
        << "  --block <n>              DMA read chunk bytes, default 4194304\n"
        << "  --sample-rate <hz>       sample rate, default 20000000\n"
        << "  --rx-lo <hz>             RX LO, default 2400000000\n"
        << "  --rx-gain <idx>          RX gain, default 20\n"
        << "  --record-timeout <sec>   wait timeout for FPGA record, default 10\n"
        << "  --chunk-timeout <sec>    UDP chunk timeout, default 2\n"
        << "  --poll-ms <ms>           record status poll period, default 20\n"
        << "  --settle-ms <ms>         wait after RF tune before capture, default 200\n"
        << "  --warmup-mb <n>          discard warmup capture before record, 0 disables, default 4\n"
        << "  --iterations <n>         repeated records on one connection, default 1\n"
        << "  --overwrite              overwrite the same output on every iteration\n"
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
        } else if (arg == "--output") {
            cfg.output = value("--output");
        } else if (arg == "--length-mb") {
            cfg.record_bytes = parse_length_mb(value("--length-mb"));
        } else if (arg == "--bytes") {
            cfg.record_bytes = parse_u32(value("--bytes"));
        } else if (arg == "--block") {
            cfg.dma_block_bytes = parse_u32(value("--block"));
        } else if (arg == "--sample-rate") {
            cfg.sample_rate_hz = parse_u32(value("--sample-rate"));
        } else if (arg == "--rx-lo") {
            cfg.rx_lo_hz = parse_u64(value("--rx-lo"));
        } else if (arg == "--rx-gain") {
            cfg.rx_gain = parse_u32(value("--rx-gain"));
        } else if (arg == "--record-timeout") {
            cfg.record_timeout_sec = std::stod(value("--record-timeout"));
        } else if (arg == "--chunk-timeout") {
            cfg.chunk_timeout_sec = std::stod(value("--chunk-timeout"));
        } else if (arg == "--poll-ms") {
            cfg.poll_ms = parse_u32(value("--poll-ms"));
        } else if (arg == "--settle-ms") {
            cfg.settle_ms = parse_u32(value("--settle-ms"));
        } else if (arg == "--warmup-mb") {
            cfg.warmup_bytes = parse_length_mb(value("--warmup-mb"));
        } else if (arg == "--iterations") {
            cfg.iterations = parse_u32(value("--iterations"));
        } else if (arg == "--overwrite") {
            cfg.overwrite_output = true;
        } else if (arg == "--help") {
            print_usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.record_bytes == 0u || cfg.dma_block_bytes == 0u) {
        throw std::runtime_error("record length and --block must be non-zero");
    }
    if (cfg.iterations == 0u) {
        throw std::runtime_error("--iterations must be non-zero");
    }
    if (cfg.record_bytes < kRecordLengthMinBytes ||
        cfg.record_bytes > kRecordLengthMaxBytes ||
        (cfg.record_bytes % kRecordLengthGranularityBytes) != 0u) {
        throw std::runtime_error("E200 record length must be 4..256 MiB and a multiple of 4 MiB");
    }
    if ((cfg.dma_block_bytes & 0x3u) != 0u) {
        throw std::runtime_error("--block must be 4-byte aligned");
    }
    if (cfg.warmup_bytes != 0u &&
        (cfg.warmup_bytes < kRecordLengthMinBytes ||
         cfg.warmup_bytes > kRecordLengthMaxBytes ||
         (cfg.warmup_bytes % kRecordLengthGranularityBytes) != 0u)) {
        throw std::runtime_error("--warmup-mb must be 0 or 4..256 MiB and a multiple of 4 MiB");
    }
}

void print_config(const Config& cfg)
{
    std::cout << "E200 IQ record test\n"
              << "  addr           : " << cfg.addr << '\n'
              << "  output         : " << cfg.output << '\n'
              << "  bytes          : " << cfg.record_bytes << '\n'
              << "  length MiB     : " << (cfg.record_bytes / kMiB) << '\n'
              << "  DMA block      : " << cfg.dma_block_bytes << '\n'
              << "  sample rate    : " << cfg.sample_rate_hz << '\n'
              << "  rx lo          : " << cfg.rx_lo_hz << '\n'
              << "  rx gain        : " << cfg.rx_gain << '\n'
              << "  record timeout : " << cfg.record_timeout_sec << " s\n"
              << "  chunk timeout  : " << cfg.chunk_timeout_sec << " s\n"
              << "  settle         : " << cfg.settle_ms << " ms\n"
              << "  warmup MiB     : " << (cfg.warmup_bytes / kMiB) << '\n'
              << "  iterations     : " << cfg.iterations << '\n'
              << "  overwrite      : " << (cfg.overwrite_output ? "yes" : "no")
              << "\n\n";
}

void require_u32(const char* name, uint32_t expected, uint32_t actual)
{
    if (actual != expected) {
        std::ostringstream oss;
        oss << name << " readback mismatch, expected " << expected
            << " got " << actual;
        throw std::runtime_error(oss.str());
    }
}

void require_u64(const char* name, uint64_t expected, uint64_t actual)
{
    if (actual != expected) {
        std::ostringstream oss;
        oss << name << " readback mismatch, expected " << expected
            << " got " << actual;
        throw std::runtime_error(oss.str());
    }
}

void configure_rf(E200Impl& device, const Config& cfg)
{
    device.set_channel_enable(1u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device.set_rx_freq(cfg.rx_lo_hz, 1);
    device.set_rx_gain(cfg.rx_gain, 1);

    require_u32("sample rate", cfg.sample_rate_hz, device.getSampleRate());
    require_u64("rx lo", cfg.rx_lo_hz, device.get_rx_freq(1));
    require_u32("rx gain", cfg.rx_gain, device.get_rx_gain(1));

    if (cfg.settle_ms != 0u) {
        std::cout << "waiting RF settle for " << cfg.settle_ms << " ms...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.settle_ms));
    }
}

double wait_record_done(E200Impl& device, const Config& cfg)
{
    const auto started = std::chrono::steady_clock::now();
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.record_timeout_sec));

    while (std::chrono::steady_clock::now() < deadline) {
        const uint32_t status = device.get_iq_record_status();
        if ((status & E200Impl::RECORD_STATUS_ERROR) != 0u) {
            throw std::runtime_error("FPGA IQ record reported ERROR");
        }
        if ((status & E200Impl::RECORD_STATUS_DONE) != 0u) {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.poll_ms));
    }

    throw std::runtime_error(
        "timeout waiting for FPGA IQ record completion, last status=" +
        hex_u32(device.get_iq_record_status()));
}

std::string iteration_output_path(const Config& cfg, uint32_t iteration)
{
    if (cfg.iterations == 1u || cfg.overwrite_output) {
        return cfg.output;
    }

    std::ostringstream oss;
    oss << cfg.output << '.' << std::setw(4) << std::setfill('0') << iteration;
    return oss.str();
}

void read_record_to_file(E200Impl& device, const Config& cfg,
                         const std::string& output_path)
{
    std::ofstream out(output_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("failed to open output file: " + output_path);
    }

    std::vector<uint8_t> chunk(cfg.dma_block_bytes);
    uint32_t total = 0u;

    while (total < cfg.record_bytes) {
        const size_t got = device.read_iq_record_chunk(
            chunk.data(), chunk.size(), cfg.chunk_timeout_sec);
        if (got == 0u) {
            break;
        }

        out.write(reinterpret_cast<const char*>(chunk.data()),
                  static_cast<std::streamsize>(got));
        if (!out) {
            throw std::runtime_error("failed while writing output file");
        }

        total += static_cast<uint32_t>(got);
        std::cout << "\r  read " << total << " / " << cfg.record_bytes << " bytes" << std::flush;
    }

    std::cout << '\n';
    if (total != cfg.record_bytes) {
        throw std::runtime_error("record read length mismatch");
    }
}

void run_warmup_record(E200Impl& device, const Config& cfg)
{
    if (cfg.warmup_bytes == 0u) {
        return;
    }

    std::cout << "running warmup capture (" << (cfg.warmup_bytes / kMiB)
              << " MiB discard)...\n";
    device.configure_iq_record(cfg.warmup_bytes, cfg.dma_block_bytes);
    device.start_iq_record();
    const double done_ms = wait_record_done(device, cfg);
    std::cout << "warmup done in " << done_ms << " ms, transfered_len="
              << device.get_iq_record_transfered_len() << '\n';

    std::vector<uint8_t> chunk(cfg.dma_block_bytes);
    uint32_t discarded = 0u;
    while (discarded < cfg.warmup_bytes) {
        const size_t got = device.read_iq_record_chunk(
            chunk.data(), chunk.size(), cfg.chunk_timeout_sec);
        if (got == 0u) {
            break;
        }
        discarded += static_cast<uint32_t>(got);
    }
    if (discarded != cfg.warmup_bytes) {
        throw std::runtime_error("warmup record read length mismatch");
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);
        print_config(cfg);

        E200Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            throw std::runtime_error("failed to initialize E200 device");
        }

        configure_rf(device, cfg);
        run_warmup_record(device, cfg);
        device.configure_iq_record(cfg.record_bytes, cfg.dma_block_bytes);

        for (uint32_t iteration = 1u; iteration <= cfg.iterations; ++iteration) {
            const std::string output_path = iteration_output_path(cfg, iteration);
            std::cout << "starting FPGA IQ record " << iteration << " / "
                      << cfg.iterations << "...\n";
            device.start_iq_record();
            const double done_ms = wait_record_done(device, cfg);
            const uint32_t before_read = device.get_iq_record_transfered_len();
            std::cout << "record done in " << done_ms
                      << " ms, readback_bytes=" << before_read << '\n';

            read_record_to_file(device, cfg, output_path);
            require_u32("readback progress before DMA read", 0u, before_read);
            require_u32("readback progress after DMA read", cfg.record_bytes,
                        device.get_iq_record_transfered_len());
            std::cout << "saved " << output_path << '\n';
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "e200_record_iq_test failed: " << ex.what() << '\n';
        return 1;
    }
}
