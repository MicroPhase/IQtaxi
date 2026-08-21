#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/M300/m300_xdma_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

using sdr::core::managed_send_buffer;
using sdr::driver::M300_MAGIC_TX;
using sdr::driver::m300_header;
using sdr::driver::store_le64;
using sdr::driver::write_header;

namespace {

constexpr uint32_t kCenterCtrlBase = 0x44a10000u;
constexpr uint32_t kRegChannelEnable = 18u * 4u;
constexpr uint32_t kRegTxSamplesPerPacket = 20u * 4u;
constexpr uint32_t kRegTxSourceSel = 21u * 4u;
constexpr uint32_t kRegIgnoreTxTimestamps = 22u * 4u;
constexpr uint32_t kRegFcWindow = 28u * 4u;
constexpr uint32_t kAd9361RxBase = 0x44a00000u;
constexpr uint32_t kTxSourceIq = 1u;
constexpr uint32_t kTxSourceH2cSink = 7u;
constexpr uint8_t kAd9361ProductId = 0x0au;
constexpr uint8_t kAd9361FddState = 0x16u;
constexpr uint8_t kAd9361RxTxState = 0x1au;
constexpr uint8_t kAd9361VcoLockMask = 0x02u;
constexpr size_t kHeaderBytes = 16u;
constexpr size_t kAxisBytes = 16u;
constexpr size_t kXdmaAlignment = 4096u;

struct Config {
    std::string base = "/dev/xdma0";
    std::string tx_device;
    uint32_t sample_rate = 61440000u;
    uint64_t tx_lo = 2400000000ull;
    uint32_t tx_atten_db = 10u;
    uint8_t channel_enable = 0x03u;
    size_t samples_per_channel = 1024u;
    size_t packet_bytes = 0u;
    size_t packets = 64u;
    size_t bulk_packets = 1u;
    double tone_hz = 1000000.0;
    double amplitude = 0.20;
    bool ramp = false;
    bool skip_init_ad9361 = false;
    bool force_init_ad9361 = false;
    bool direct_write = false;
    bool reuse_payload = false;
    bool h2c_sink = false;
};

void usage()
{
    std::cout
        << "Usage: m300_tx_iq_probe [--base /dev/xdma0]\n"
        << "                         [--sample-rate 61440000]\n"
        << "                         [--tx-lo 2400000000]\n"
        << "                         [--tx-atten-db 10]\n"
        << "                         [--tx-atten-mdB 10000]\n"
        << "                         [--channels 0x3]\n"
        << "                         [--samples-per-channel 1024]\n"
        << "                         [--packet-bytes 262144]\n"
        << "                         [--packets 64]\n"
        << "                         [--bulk-packets 16]\n"
        << "                         [--tone-hz 1000000]\n"
        << "                         [--amp 0.20]\n"
        << "                         [--ramp]\n"
        << "                         [--force-init-ad9361]\n"
        << "                         [--direct-write]\n"
        << "                         [--reuse-payload]\n"
        << "                         [--h2c-sink]\n"
        << "                         [--tx-device /dev/xdma0_h2c_1]\n";
}

uint32_t parse_u32(const char* s)
{
    return static_cast<uint32_t>(std::stoul(s, nullptr, 0));
}

uint64_t parse_u64(const char* s)
{
    return static_cast<uint64_t>(std::stoull(s, nullptr, 0));
}

void parse_args(Config& cfg, int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char* name) -> const char* {
            if (i + 1 >= argc)
                throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };

        if (arg == "--base") {
            cfg.base = value("--base");
        } else if (arg == "--tx-device") {
            cfg.tx_device = value("--tx-device");
        } else if (arg == "--sample-rate") {
            cfg.sample_rate = parse_u32(value("--sample-rate"));
        } else if (arg == "--tx-lo") {
            cfg.tx_lo = parse_u64(value("--tx-lo"));
        } else if (arg == "--tx-atten-db" || arg == "--tx-atten") {
            cfg.tx_atten_db = parse_u32(value(arg.c_str()));
        } else if (arg == "--tx-atten-mdB") {
            cfg.tx_atten_db = parse_u32(value("--tx-atten-mdB")) / 1000u;
        } else if (arg == "--channels") {
            cfg.channel_enable = static_cast<uint8_t>(parse_u32(value("--channels")) & 0xffu);
        } else if (arg == "--samples-per-channel") {
            cfg.samples_per_channel = static_cast<size_t>(parse_u64(value("--samples-per-channel")));
        } else if (arg == "--packet-bytes") {
            cfg.packet_bytes = static_cast<size_t>(parse_u64(value("--packet-bytes")));
        } else if (arg == "--packets") {
            cfg.packets = static_cast<size_t>(parse_u64(value("--packets")));
        } else if (arg == "--bulk-packets") {
            cfg.bulk_packets = static_cast<size_t>(parse_u64(value("--bulk-packets")));
        } else if (arg == "--tone-hz") {
            cfg.tone_hz = std::stod(value("--tone-hz"));
        } else if (arg == "--amp") {
            cfg.amplitude = std::stod(value("--amp"));
        } else if (arg == "--ramp") {
            cfg.ramp = true;
        } else if (arg == "--skip-init-ad9361") {
            cfg.skip_init_ad9361 = true;
        } else if (arg == "--force-init-ad9361") {
            cfg.force_init_ad9361 = true;
        } else if (arg == "--direct-write") {
            cfg.direct_write = true;
        } else if (arg == "--reuse-payload") {
            cfg.reuse_payload = true;
        } else if (arg == "--h2c-sink") {
            cfg.h2c_sink = true;
        } else if (arg == "--help") {
            usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.channel_enable != 0x01u && cfg.channel_enable != 0x03u) {
        throw std::runtime_error("only --channels 0x1 or 0x3 are supported by this probe");
    }
    if (cfg.samples_per_channel == 0u || cfg.packets == 0u || cfg.bulk_packets == 0u) {
        throw std::runtime_error("samples, packets, and bulk-packets must be non-zero");
    }
    cfg.amplitude = std::max(0.0, std::min(cfg.amplitude, 0.95));
}

uint32_t iq_word(int16_t i, int16_t q)
{
    return (static_cast<uint32_t>(static_cast<uint16_t>(i)) << 16) |
           static_cast<uint16_t>(q);
}

void store_le32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

void make_sample(const Config& cfg, size_t n, unsigned channel, int16_t& i, int16_t& q)
{
    if (cfg.ramp) {
        const int16_t v = static_cast<int16_t>((static_cast<int>(n) * 257) & 0x7fff);
        const int16_t cv = (channel == 0u)
            ? v
            : static_cast<int16_t>((static_cast<int>(n) * 257 + 0x2000) & 0x7fff);
        i = cv;
        q = static_cast<int16_t>(-cv);
    } else {
        constexpr double pi = 3.14159265358979323846;
        const double channel_phase = (channel == 0u) ? 0.0 : pi / 2.0;
        const double phase = 2.0 * pi * cfg.tone_hz *
                             static_cast<double>(n) /
                             static_cast<double>(cfg.sample_rate) +
                             channel_phase;
        const double scale = 32767.0 * cfg.amplitude;
        i = static_cast<int16_t>(std::cos(phase) * scale);
        q = static_cast<int16_t>(std::sin(phase) * scale);
    }
}

void store_sample_slot(uint8_t* p, const Config& cfg, size_t n, unsigned channel)
{
    int16_t i = 0;
    int16_t q = 0;
    make_sample(cfg, n, channel, i, q);
    store_le32(p, iq_word(i, q));
}

void fill_body_word(uint8_t* p, const Config& cfg, size_t base_sample)
{
    if (cfg.channel_enable == 0x03u) {
        store_sample_slot(p + 0, cfg, base_sample + 0u, 0u);
        store_sample_slot(p + 4, cfg, base_sample + 0u, 1u);
        store_sample_slot(p + 8, cfg, base_sample + 1u, 0u);
        store_sample_slot(p + 12, cfg, base_sample + 1u, 1u);
        return;
    }

    store_sample_slot(p + 0, cfg, base_sample + 0u, 0u);
    store_sample_slot(p + 4, cfg, base_sample + 1u, 0u);
    store_sample_slot(p + 8, cfg, base_sample + 2u, 0u);
    store_sample_slot(p + 12, cfg, base_sample + 3u, 0u);
}

std::string resolve_tx_device(const Config& cfg)
{
    if (!cfg.tx_device.empty())
        return cfg.tx_device;

    if (cfg.base.find('=') == std::string::npos)
        return cfg.base + "_h2c_1";

    size_t begin = 0;
    while (begin < cfg.base.size()) {
        const size_t end = cfg.base.find(',', begin);
        const std::string item =
            cfg.base.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        const size_t eq = item.find('=');
        if (eq != std::string::npos && item.substr(0, eq) == "tx")
            return item.substr(eq + 1);
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    return "/dev/xdma0_h2c_1";
}

ssize_t write_all(int fd, const void* data, size_t bytes)
{
    size_t done = 0;
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (done < bytes) {
        const ssize_t rc = ::write(fd, p + done, bytes - done);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return rc;
        }
        if (rc == 0)
            break;
        done += static_cast<size_t>(rc);
    }
    return static_cast<ssize_t>(done);
}

void fill_packet(uint8_t* p,
                 const Config& cfg,
                 uint16_t seq,
                 size_t packet_index,
                 size_t packet_bytes,
                 size_t body_words,
                 size_t samples_per_word)
{
    std::memset(p, 0, packet_bytes);

    m300_header hdr;
    hdr.magic_type = M300_MAGIC_TX;
    hdr.seq = seq;
    hdr.sid = 0;
    hdr.length = static_cast<uint32_t>(packet_bytes);
    write_header(p, hdr);
    store_le64(p + 8, 0u);

    for (size_t w = 0; w < body_words; ++w) {
        fill_body_word(p + kHeaderBytes + w * kAxisBytes,
                       cfg,
                       packet_index * cfg.samples_per_channel + w * samples_per_word);
    }
}

void update_packet_header(uint8_t* p, uint16_t seq, size_t packet_bytes)
{
    m300_header hdr;
    hdr.magic_type = M300_MAGIC_TX;
    hdr.seq = seq;
    hdr.sid = 0;
    hdr.length = static_cast<uint32_t>(packet_bytes);
    write_header(p, hdr);
    store_le64(p + 8, 0u);
}

bool ad9361_looks_initialized(const std::shared_ptr<sdr::driver::m300_xdma_ctrl>& ctrl,
                              bool verbose)
{
    try {
        const uint8_t product_id = ctrl->ad9361_spi_read(0x037u, 1.0);
        const uint8_t ensm_state = ctrl->ad9361_spi_read(0x017u, 1.0);
        const uint8_t rx_vco_lock = ctrl->ad9361_spi_read(0x247u, 1.0);
        const uint8_t tx_vco_lock = ctrl->ad9361_spi_read(0x287u, 1.0);
        const uint32_t rx_rstn = ctrl->read_axi(kAd9361RxBase + 0x0040u, 1.0);
        const uint32_t rx_clk_count = ctrl->read_axi(kAd9361RxBase + 0x0054u, 1.0);
        const uint32_t rx_status = ctrl->read_axi(kAd9361RxBase + 0x005cu, 1.0);

        const bool ensm_ok = (ensm_state == kAd9361FddState) ||
                             (ensm_state == kAd9361RxTxState);
        const bool ok = (product_id == kAd9361ProductId) &&
                        ensm_ok &&
                        ((rx_vco_lock & kAd9361VcoLockMask) != 0u) &&
                        ((tx_vco_lock & kAd9361VcoLockMask) != 0u) &&
                        ((rx_rstn & 0x3u) == 0x3u) &&
                        (rx_clk_count != 0u) &&
                        (rx_status != 0u);

        if (verbose) {
            std::cout << "ad9361_auto_check"
                      << " product_id=0x" << std::hex << static_cast<unsigned>(product_id)
                      << " state=0x" << static_cast<unsigned>(ensm_state)
                      << " rx_vco=0x" << static_cast<unsigned>(rx_vco_lock)
                      << " tx_vco=0x" << static_cast<unsigned>(tx_vco_lock)
                      << " axi_rx_rstn=0x" << rx_rstn
                      << " axi_rx_clk=0x" << rx_clk_count
                      << " axi_rx_status=0x" << rx_status
                      << std::dec
                      << " initialized=" << (ok ? 1 : 0)
                      << "\n";
        }
        return ok;
    } catch (const std::exception& ex) {
        if (verbose) {
            std::cout << "ad9361_auto_check failed: " << ex.what() << "\n";
        }
        return false;
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);

        M300XdmaImpl dev(cfg.base);
        if (!dev.isInitialSuccess()) {
            std::cerr << "M300_XDMA device init failed: " << dev.last_error() << "\n";
            return 1;
        }

        auto ctrl = dev.get_ctrl();
        auto tx_xport = dev.get_tx_xport();
        if (!ctrl || !tx_xport) {
            throw std::runtime_error("M300 TX transport is not available");
        }

        const auto version = ctrl->get_version();
        std::cout << "version=0x" << std::hex << version.pkt.value0
                  << " build=0x" << version.pkt.value1 << std::dec << "\n";

        const bool ad9361_ready =
            cfg.skip_init_ad9361 ? true : ad9361_looks_initialized(ctrl, true);

        if (cfg.skip_init_ad9361) {
            std::cout << "ad9361_config skipped by compatibility option\n";
        } else if (ad9361_ready && !cfg.force_init_ad9361) {
            std::cout << "ad9361_config skipped: existing configuration looks valid\n";
        } else {
            std::cout << "ad9361_config begin"
                      << (cfg.force_init_ad9361 ? " (forced)" : " (auto)")
                      << "\n";
            dev.setSampleRate(cfg.sample_rate);
            dev.set_tx_freq(cfg.tx_lo, 1);
            dev.set_tx_atten(cfg.tx_atten_db, 1);
            std::cout << "ad9361_config done\n";
        }

        const size_t enabled_channels = (cfg.channel_enable == 0x03u) ? 2u : 1u;
        const size_t samples_per_word = (cfg.channel_enable == 0x03u) ? 2u : 4u;
        if (cfg.packet_bytes != 0u) {
            if (cfg.packet_bytes <= kHeaderBytes ||
                ((cfg.packet_bytes - kHeaderBytes) % kAxisBytes) != 0u) {
                throw std::runtime_error("--packet-bytes must be 16-byte aligned and larger than 16");
            }
            cfg.samples_per_channel =
                ((cfg.packet_bytes - kHeaderBytes) / kAxisBytes) * samples_per_word;
        }
        if ((cfg.samples_per_channel % samples_per_word) != 0u) {
            throw std::runtime_error("samples-per-channel must be a multiple of the samples carried by one 128-bit word");
        }
        const size_t body_words = cfg.samples_per_channel / samples_per_word;
        const size_t packet_bytes = kHeaderBytes + body_words * kAxisBytes;
        const uint32_t tx_samples_per_packet =
            static_cast<uint32_t>(cfg.samples_per_channel * enabled_channels);

        ctrl->write_axi(kCenterCtrlBase + kRegChannelEnable,
                        cfg.channel_enable, 1.0);
        ctrl->write_axi(kCenterCtrlBase + kRegTxSamplesPerPacket,
                        tx_samples_per_packet, 1.0);
        ctrl->write_axi(kCenterCtrlBase + kRegTxSourceSel,
                        cfg.h2c_sink ? kTxSourceH2cSink : kTxSourceIq, 1.0);
        ctrl->write_axi(kCenterCtrlBase + kRegIgnoreTxTimestamps,
                        1u, 1.0);
        ctrl->write_axi(kCenterCtrlBase + kRegFcWindow,
                        0u, 1.0);

        std::cout << "tx_config"
                  << " channels=0x" << std::hex << static_cast<unsigned>(cfg.channel_enable)
                  << std::dec
                  << " samples_per_channel=" << cfg.samples_per_channel
                  << " samples_per_word=" << samples_per_word
                  << " body_words=" << body_words
                  << " tx_samples_per_packet=" << tx_samples_per_packet
                  << " packet_bytes=" << packet_bytes
                  << " packets=" << cfg.packets
                  << " bulk_packets=" << cfg.bulk_packets
                  << " direct_write=" << (cfg.direct_write ? 1 : 0)
                  << " reuse_payload=" << (cfg.reuse_payload ? 1 : 0)
                  << " h2c_sink=" << (cfg.h2c_sink ? 1 : 0)
                  << "\n";

        const auto start = std::chrono::steady_clock::now();
        uint16_t seq = 0;
        uint64_t total_bytes = 0;
        if (cfg.direct_write) {
            const std::string tx_device = resolve_tx_device(cfg);
            const int fd = ::open(tx_device.c_str(), O_WRONLY);
            if (fd < 0) {
                throw std::runtime_error("failed to open TX H2C device " + tx_device + ": " +
                                         std::strerror(errno));
            }

            const size_t bulk_bytes = packet_bytes * cfg.bulk_packets;
            if (cfg.bulk_packets != 0u && bulk_bytes / cfg.bulk_packets != packet_bytes) {
                ::close(fd);
                throw std::runtime_error("bulk TX buffer size overflow");
            }

            void* raw = nullptr;
            if (::posix_memalign(&raw, kXdmaAlignment, bulk_bytes) != 0 || raw == nullptr) {
                ::close(fd);
                throw std::runtime_error("failed to allocate aligned TX bulk buffer");
            }
            std::unique_ptr<void, decltype(&std::free)> bulk(raw, &std::free);
            auto* p = static_cast<uint8_t*>(bulk.get());
            if (cfg.reuse_payload) {
                for (size_t i = 0; i < cfg.bulk_packets; ++i) {
                    fill_packet(p + i * packet_bytes,
                                cfg,
                                0u,
                                i,
                                packet_bytes,
                                body_words,
                                samples_per_word);
                }
            }

            std::cout << "direct_tx_write device=" << tx_device
                      << " one_write_per_packet=" << (cfg.bulk_packets == 1u ? 1 : 0)
                      << " packet_bytes=" << packet_bytes
                      << " bulk_packets=" << cfg.bulk_packets
                      << " bulk_bytes=" << bulk_bytes
                      << (cfg.bulk_packets > 1u ? " requires_fpga_h2c_packet_splitter=1" : "")
                      << (cfg.h2c_sink ? " fpga_h2c_sink=1" : "")
                      << "\n";

            for (size_t pidx = 0; pidx < cfg.packets;) {
                const size_t batch_packets = std::min(cfg.bulk_packets, cfg.packets - pidx);
                for (size_t i = 0; i < batch_packets; ++i) {
                    uint8_t* packet = p + i * packet_bytes;
                    if (cfg.reuse_payload) {
                        update_packet_header(packet, seq++, packet_bytes);
                    } else {
                        fill_packet(packet,
                                    cfg,
                                    seq++,
                                    pidx + i,
                                    packet_bytes,
                                    body_words,
                                    samples_per_word);
                    }
                }

                const size_t write_bytes = batch_packets * packet_bytes;
                const ssize_t rc = write_all(fd, p, write_bytes);
                if (rc < 0) {
                    const int saved_errno = errno;
                    ::close(fd);
                    throw std::runtime_error("TX write failed on " + tx_device + ": " +
                                             std::strerror(saved_errno));
                }
                if (static_cast<size_t>(rc) != write_bytes) {
                    ::close(fd);
                    throw std::runtime_error("short TX write");
                }
                total_bytes += write_bytes;
                pidx += batch_packets;
            }
            ::close(fd);
        } else {
            for (size_t pidx = 0; pidx < cfg.packets; ++pidx) {
                managed_send_buffer::sptr buff =
                    tx_xport->get_send_buff(1.0, static_cast<uint32_t>(packet_bytes));
                if (!buff) {
                    throw std::runtime_error("failed to allocate TX send buffer");
                }

                auto* p = buff->cast<uint8_t*>();
                fill_packet(p, cfg, seq++, pidx, packet_bytes, body_words, samples_per_word);

                buff->commit(packet_bytes);
                total_bytes += packet_bytes;
            }
        }
        const auto end = std::chrono::steady_clock::now();
        const double sec = std::chrono::duration<double>(end - start).count();
        const double mib_s = static_cast<double>(total_bytes) / (1024.0 * 1024.0) / sec;

        std::cout << "m300_tx_iq_probe=PASS"
                  << " bytes=" << total_bytes
                  << " elapsed=" << sec
                  << " sec rate=" << mib_s << " MiB/s\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_tx_iq_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
