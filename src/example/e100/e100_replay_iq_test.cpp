#include "src/driver/E100/local_e100_regs.hpp"
#include "src/driver/E100/e100_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kMiB = 1024u * 1024u;
constexpr uint32_t kReplayMaxBytes = 240u * kMiB;
constexpr uint32_t kMaxChunkBytes = 4u * kMiB;
constexpr uint32_t kDefaultChunkBytes = 4u * kMiB;
constexpr uint32_t kDefaultPacketGapUs = 25u;
constexpr uint32_t kTxSourceIq = 1u;

struct Config {
    std::string addr = "192.168.1.10";
    std::string input = "e100_record.cs16";
    uint32_t sample_rate_hz = 15360000u;
    uint64_t tx_lo_hz = 1000000000ull;
    uint32_t tx_atten = 10u;
    uint32_t chunk_bytes = kDefaultChunkBytes;
    double chunk_timeout_sec = 10.0;
    uint32_t packet_gap_us = kDefaultPacketGapUs;
    uint32_t repeat = 1u;
    uint32_t gap_ms = 0u;
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
        << "Usage: e100_replay_iq_test [options]\n"
        << "  --addr <ip>              device IP, default 192.168.1.10\n"
        << "  --input <path>           input raw cs16 IQ file, default e100_record.cs16\n"
        << "  --sample-rate <hz>       sample rate, default 15360000\n"
        << "  --tx-lo <hz>             TX LO, default 1000000000\n"
        << "  --tx-atten <db>          TX attenuation, default 10\n"
        << "  --chunk <bytes>          DMA upload chunk, default 4194304, max 4194304\n"
        << "  --chunk-timeout <sec>    UDP/DMA chunk timeout, default 10\n"
        << "  --packet-gap-us <us>     delay between replay UDP IQ packets, default 25\n"
        << "  --repeat <n>             replay trigger count after upload, default 1\n"
        << "  --gap-ms <ms>            delay between repeats, default 0\n"
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
        } else if (arg == "--repeat") {
            cfg.repeat = parse_u32(value("--repeat"));
        } else if (arg == "--gap-ms") {
            cfg.gap_ms = parse_u32(value("--gap-ms"));
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
    if (cfg.repeat == 0u) {
        throw std::runtime_error("--repeat must be non-zero");
    }
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
        throw std::runtime_error("input file is larger than 240 MiB");
    }

    return data;
}

void print_config(const Config& cfg, size_t bytes)
{
    std::cout << "E100 IQ replay test\n"
              << "  addr          : " << cfg.addr << '\n'
              << "  input         : " << cfg.input << '\n'
              << "  bytes         : " << bytes << '\n'
              << "  length MiB    : " << (bytes / kMiB) << '\n'
              << "  sample rate   : " << cfg.sample_rate_hz << '\n'
              << "  tx lo         : " << cfg.tx_lo_hz << '\n'
              << "  tx atten      : " << cfg.tx_atten << '\n'
              << "  chunk         : " << cfg.chunk_bytes << '\n'
              << "  packet gap us : " << cfg.packet_gap_us << '\n'
              << "  repeat        : " << cfg.repeat << "\n\n";
}

void configure_tx(E100Impl& device, const Config& cfg)
{
    device.set_channel_enable(1u);
    device.setSampleRate(static_cast<double>(cfg.sample_rate_hz));
    device.set_tx_freq(cfg.tx_lo_hz, 1);
    device.set_tx_atten(cfg.tx_atten, 1);
    device.get_local_bus()->poke32(e100::CUSTOM_SET_TX_SOURCE_SEL, kTxSourceIq);

    const uint32_t actual_rate = device.getSampleRate();
    const uint64_t actual_lo = device.get_tx_freq(1);
    if (actual_rate != cfg.sample_rate_hz) {
        throw std::runtime_error("sample rate readback mismatch");
    }
    if (actual_lo != cfg.tx_lo_hz) {
        throw std::runtime_error("tx lo readback mismatch");
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);
        std::vector<uint8_t> iq = load_iq_file(cfg.input);
        print_config(cfg, iq.size());

        E100Impl device(cfg.addr);
        if (!device.isInitialSuccess()) {
            throw std::runtime_error("failed to initialize E100 device");
        }

        configure_tx(device, cfg);
        device.set_iq_replay_packet_gap_us(cfg.packet_gap_us);
        device.configure_iq_replay(static_cast<uint32_t>(iq.size()));
        std::cout << "arming replay buffer update...\n";
        device.start_iq_replay(static_cast<uint32_t>(iq.size()));

        std::cout << "uploading replay IQ...\n";
        size_t uploaded = 0u;
        uint32_t chunk_index = 0u;
        while (uploaded < iq.size()) {
            const size_t todo = std::min<size_t>(cfg.chunk_bytes, iq.size() - uploaded);
            ++chunk_index;
            std::cout << "  chunk " << chunk_index << " offset=" << uploaded
                      << " bytes=" << todo << '\n';
            uploaded += device.write_iq_replay_chunk(
                iq.data() + uploaded, todo, cfg.chunk_timeout_sec);
        }
        std::cout << "uploaded " << uploaded << " bytes, dma_offset="
                  << device.get_iq_replay_dma_offset() << '\n';

        for (uint32_t n = 1; n < cfg.repeat; ++n) {
            if (cfg.gap_ms != 0u) {
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg.gap_ms));
            }
            device.start_iq_replay();
            std::cout << "replay trigger " << (n + 1u) << " / " << cfg.repeat << '\n';
        }

        std::cout << "replay running\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "e100_replay_iq_test failed: " << ex.what() << '\n';
        return 1;
    }
}
