#include "src/driver/M300/m300_ad9361_ctrl.hpp"
#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/M300/m300_xdma_protocol.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace sdr::driver;

namespace {

constexpr uint32_t kCenterCtrlBase = 0x44a10000u;
constexpr uint32_t kRegModeExit = 16u * 4u;
constexpr uint32_t kRegStreamStart = 17u * 4u;
constexpr uint32_t kRegRxSampleBytes = 10u * 4u;
constexpr uint32_t kRegRxMode = 14u * 4u;
constexpr uint32_t kRegRxModeStrobe = 15u * 4u;
constexpr uint32_t kRegChannelEnable = 18u * 4u;
constexpr uint32_t kRegDmaPktPerBurst = 19u * 4u;
constexpr uint32_t kRegMaxSampleBytesPerPacket = 27u * 4u;
constexpr uint32_t kRegXdmaStatus = 0x00030000u;
constexpr uint32_t kAd9361RxBase = 0x44a00000u;
constexpr uint32_t kRxSourceIq = 0u;
constexpr uint32_t kRxSourceC2hTest = 1u;

constexpr uint32_t kDefaultPacketBytes = 16384u;
constexpr uint32_t kPacketPrefixBytes = M300_HDR_BYTES;
constexpr uint64_t kMaxSeqJumpLogs = 16u;
constexpr uint16_t kMaxReasonableSeqLoss = 4096u;
constexpr uint8_t kAd9361ProductId = 0x0au;
constexpr uint8_t kAd9361FddState = 0x16u;
constexpr uint8_t kAd9361RxTxState = 0x1au;
constexpr uint8_t kAd9361VcoLockMask = 0x02u;

constexpr uint32_t kM300C2hRingVersion = 1u;

struct xdma_m300_c2h_ring_start
{
    uint32_t version;
    uint32_t packet_bytes;
    uint32_t depth;
    uint32_t flags;
};

struct xdma_m300_c2h_ring_get
{
    uint64_t buffer;
    uint32_t buffer_bytes;
    uint32_t timeout_ms;
    uint32_t bytes;
    uint32_t status;
    uint32_t index;
    uint32_t reserved;
    uint64_t sequence;
    uint64_t completed;
    uint64_t errors;
};

#define IOCTL_XDMA_M300_C2H_RING_START _IOW('q', 20, struct xdma_m300_c2h_ring_start *)
#define IOCTL_XDMA_M300_C2H_RING_STOP  _IO('q', 21)
#define IOCTL_XDMA_M300_C2H_RING_GET   _IOWR('q', 22, struct xdma_m300_c2h_ring_get *)

struct sample_stats
{
    uint64_t count = 0;
    uint64_t nonzero = 0;
    int16_t min_i = std::numeric_limits<int16_t>::max();
    int16_t max_i = std::numeric_limits<int16_t>::min();
    int16_t min_q = std::numeric_limits<int16_t>::max();
    int16_t max_q = std::numeric_limits<int16_t>::min();
    long double sum_i = 0.0;
    long double sum_q = 0.0;
    long double sum_i2 = 0.0;
    long double sum_q2 = 0.0;
};

struct iq_sample
{
    int16_t i = 0;
    int16_t q = 0;
    uint32_t raw = 0;
};

struct rx_process_state
{
    std::vector<sample_stats> stats;
    uint16_t expected_seq = 0;
    bool have_seq = false;
    uint64_t bytes = 0;
    uint64_t payload_bytes = 0;
    uint64_t seq_jumps = 0;
    uint64_t lost_packets = 0;
    uint64_t reordered_or_wrapped = 0;
    uint64_t short_packets = 0;
    uint64_t timestamp_gaps = 0;
    uint64_t timestamp_gap_logs = 0;
    uint64_t last_timestamp = 0;
    uint64_t timestamp_step = 0;
    bool have_timestamp = false;
    bool have_timestamp_step = false;
    uint64_t seq_jump_logs = 0;
    size_t dumped = 0;
};

struct direct_read_block
{
    std::vector<uint8_t> data;
    size_t index = 0;
    size_t bytes = 0;
};

struct direct_read_queue
{
    std::mutex mutex;
    std::condition_variable cv_not_empty;
    std::condition_variable cv_not_full;
    std::map<size_t, direct_read_block> blocks;
    bool done = false;
    std::string error;
};

enum class ad9361_init_policy
{
    auto_detect,
    force,
    skip
};

std::string default_base(const std::string& base)
{
    return base.empty() ? std::string("/dev/xdma0") : base;
}

uint64_t parse_u64(const std::string& text)
{
    return std::stoull(text, nullptr, 0);
}

uint32_t parse_u32(const std::string& text)
{
    return static_cast<uint32_t>(parse_u64(text));
}

std::string errno_text(const char* prefix, int err)
{
    std::string msg = std::string(prefix) + ": errno=" + std::to_string(err) +
                      " (" + std::strerror(err) + ")";
    if (err == 512) {
        msg += "; Linux internal ERESTARTSYS leaked from XDMA read. "
               "Check for stale D-state m300_rx_iq_probe tasks holding /dev/xdma0_*; "
               "a reboot is usually required to recover the XDMA char device cleanly.";
    }
    return msg;
}

size_t count_enabled_channels(uint32_t channel_mask)
{
    size_t count = 0;
    for (uint32_t mask = channel_mask; mask != 0u; mask >>= 1u)
        count += mask & 0x1u;
    return std::max<size_t>(count, 1u);
}

uint32_t load_le32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

size_t align_up_size(size_t value, size_t align)
{
    return ((value + align - 1u) / align) * align;
}

std::string channel_path(const std::string& base, const char* suffix)
{
    return default_base(base) + suffix;
}

std::string rx_path_from_device_arg(const std::string& base)
{
    if (base.find('=') == std::string::npos)
        return channel_path(base, "_c2h_1");

    size_t begin = 0;
    while (begin < base.size()) {
        const size_t end = base.find(',', begin);
        const std::string item = base.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        const size_t eq = item.find('=');
        if (eq != std::string::npos && item.substr(0, eq) == "rx")
            return item.substr(eq + 1);
        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    return "/dev/xdma0_c2h_1";
}

ssize_t read_retry(int fd, void* buf, size_t len)
{
    ssize_t rc;
    do {
        rc = ::read(fd, buf, len);
    } while (rc < 0 && errno == EINTR);
    return rc;
}

ssize_t read_full_retry(int fd, void* buf, size_t len)
{
    size_t total = 0;
    uint8_t* p = static_cast<uint8_t*>(buf);
    while (total < len) {
        const ssize_t rc = read_retry(fd, p + total, len - total);
        if (rc < 0)
            return rc;
        if (rc == 0)
            break;
        total += static_cast<size_t>(rc);
    }
    return static_cast<ssize_t>(total);
}

iq_sample parse_iq_word(const uint8_t* p)
{
    const uint32_t raw = load_le32(p);
    iq_sample s;
    s.raw = raw;
    s.i = static_cast<int16_t>(raw & 0xffffu);
    s.q = static_cast<int16_t>((raw >> 16) & 0xffffu);
    return s;
}

void update_stats(sample_stats& stats, const iq_sample& sample)
{
    stats.count++;
    if (sample.i != 0 || sample.q != 0)
        stats.nonzero++;
    stats.min_i = std::min(stats.min_i, sample.i);
    stats.max_i = std::max(stats.max_i, sample.i);
    stats.min_q = std::min(stats.min_q, sample.q);
    stats.max_q = std::max(stats.max_q, sample.q);
    stats.sum_i += sample.i;
    stats.sum_q += sample.q;
    stats.sum_i2 += static_cast<long double>(sample.i) * sample.i;
    stats.sum_q2 += static_cast<long double>(sample.q) * sample.q;
}

void print_stats(const std::vector<sample_stats>& stats)
{
    std::cout << "iq_stats\n";
    for (size_t lane = 0; lane < stats.size(); ++lane) {
        const auto& s = stats[lane];
        if (s.count == 0) {
            std::cout << "  lane" << lane << ": no samples\n";
            continue;
        }
        const long double denom = static_cast<long double>(s.count);
        const long double rms_i = std::sqrt(s.sum_i2 / denom);
        const long double rms_q = std::sqrt(s.sum_q2 / denom);
        std::cout << "  lane" << lane
                  << ": count=" << s.count
                  << " nonzero=" << s.nonzero
                  << " I[min,max,mean,rms]=" << s.min_i << "," << s.max_i
                  << "," << static_cast<double>(s.sum_i / denom)
                  << "," << static_cast<double>(rms_i)
                  << " Q[min,max,mean,rms]=" << s.min_q << "," << s.max_q
                  << "," << static_cast<double>(s.sum_q / denom)
                  << "," << static_cast<double>(rms_q)
                  << "\n";
    }
}

void process_rx_packet(const uint8_t* p,
                       size_t available_len,
                       size_t packet_index,
                       uint32_t packet_bytes,
                       uint32_t lanes,
                       size_t dump_samples,
                       size_t packet_log,
                       bool collect_stats,
                       bool measure_packet,
                       std::ofstream* save_cs16,
                       rx_process_state& state)
{
    if (available_len < kPacketPrefixBytes) {
        throw std::runtime_error("RX packet shorter than header");
    }

    const m300_header hdr = parse_header(p);
    if (hdr.magic_type != M300_MAGIC_RX) {
        std::cerr << "bad magic at packet " << packet_index
                  << ": 0x" << std::hex << hdr.magic_type << std::dec << "\n";
        return;
    }
    if (hdr.length < kPacketPrefixBytes || hdr.length > packet_bytes) {
        throw std::runtime_error("invalid RX packet length: " + std::to_string(hdr.length));
    }

    const size_t actual_len = std::min<size_t>(hdr.length, available_len);
    const bool short_packet = actual_len < hdr.length;
    const uint64_t timestamp = load_le64(p + 8);
    const size_t payload_bytes = actual_len - kPacketPrefixBytes;
    const size_t words = payload_bytes / sizeof(uint32_t);

    if (!measure_packet)
        return;

    if (short_packet)
        state.short_packets++;

    if (!state.have_seq) {
        state.expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
        state.have_seq = true;
    } else {
        const uint16_t lost = static_cast<uint16_t>(hdr.seq - state.expected_seq);
        if (lost != 0u) {
            state.seq_jumps++;
            if (lost <= kMaxReasonableSeqLoss) {
                state.lost_packets += lost;
            } else {
                state.reordered_or_wrapped++;
            }
            if (state.seq_jump_logs < kMaxSeqJumpLogs) {
                std::cerr << "seq jump expected=" << state.expected_seq
                          << " got=" << hdr.seq
                          << " lost=" << lost << "\n";
                state.seq_jump_logs++;
            }
        }
        state.expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
    }

    state.bytes += actual_len;
    state.payload_bytes += payload_bytes;

    if (!state.have_timestamp) {
        state.last_timestamp = timestamp;
        state.have_timestamp = true;
    } else {
        const uint64_t step = timestamp - state.last_timestamp;
        if (!state.have_timestamp_step) {
            state.timestamp_step = step;
            state.have_timestamp_step = true;
        } else if (step != state.timestamp_step) {
            state.timestamp_gaps++;
            if (state.timestamp_gap_logs < kMaxSeqJumpLogs) {
                std::cerr << "timestamp step changed expected=" << state.timestamp_step
                          << " got=" << timestamp
                          << " previous=" << state.last_timestamp
                          << " step=" << step
                          << "\n";
                state.timestamp_gap_logs++;
            }
        }
        state.last_timestamp = timestamp;
    }

    if (packet_index < packet_log || short_packet) {
        std::cout << "rx[" << packet_index << "] seq=" << hdr.seq
                  << " sid=0x" << std::hex << static_cast<unsigned>(hdr.sid)
                  << " timestamp=0x" << timestamp << std::dec
                  << " len=" << hdr.length
                  << " actual=" << actual_len
                  << " payload=" << payload_bytes
                  << " words=" << words
                  << (short_packet ? " SHORT" : "")
                  << "\n";
    }

    if (!collect_stats && dump_samples == 0u && (!save_cs16 || !save_cs16->is_open()))
        return;

    const uint8_t* payload = p + kPacketPrefixBytes;
    if (save_cs16 && save_cs16->is_open()) {
        save_cs16->write(reinterpret_cast<const char*>(payload),
                         static_cast<std::streamsize>(words * sizeof(uint32_t)));
    }

    for (size_t w = 0; w < words; ++w) {
        const iq_sample s = parse_iq_word(payload + w * sizeof(uint32_t));
        const size_t lane = w % lanes;
        if (collect_stats)
            update_stats(state.stats[lane], s);
        if (state.dumped < dump_samples) {
            std::cout << "  sample[" << state.dumped << "]"
                      << " lane" << lane
                      << " raw=0x" << std::hex << std::setw(8)
                      << std::setfill('0') << s.raw
                      << std::dec << std::setfill(' ')
                      << " i=" << s.i
                      << " q=" << s.q << "\n";
            state.dumped++;
        }
    }
}

size_t find_next_rx_header(const uint8_t* p, size_t begin, size_t end)
{
    for (size_t off = begin; off + kPacketPrefixBytes <= end; off += 16u) {
        const m300_header h = parse_header(p + off);
        if (h.magic_type == M300_MAGIC_RX &&
            h.length >= kPacketPrefixBytes &&
            h.length <= (1u << 20)) {
            return off;
        }
    }
    return end;
}

void dump_runtime_status(const std::shared_ptr<m300_xdma_ctrl>& ctrl,
                         const char* title)
{
    const uint32_t stream_enable = ctrl->read_reg(M300_REG_STREAM_ENABLE, 1.0);
    const uint32_t rx_packet_bytes = ctrl->read_reg(M300_REG_RX_PACKET_BYTES, 1.0);
    const uint32_t rx_source_sel = ctrl->read_reg(M300_REG_RX_SOURCE_SEL, 1.0);
    const uint32_t xdma_status = ctrl->read_reg(kRegXdmaStatus, 1.0);

    std::cout << title << "\n";
    std::cout << "  local.stream_enable=0x" << std::hex << stream_enable
              << " local.rx_packet_bytes=0x" << rx_packet_bytes
              << " local.rx_source_sel=0x" << rx_source_sel
              << " local.xdma_status=0x" << xdma_status << std::dec << "\n";
    std::cout << "  xdma_status_bits:"
              << " bus_rst=" << ((xdma_status >> 0) & 1u)
              << " user_lnk_up=" << ((xdma_status >> 1) & 1u)
              << " iq_rx128_tready=" << ((xdma_status >> 2) & 1u)
              << " iq_rx128_tvalid=" << ((xdma_status >> 3) & 1u)
              << " rx_tready=" << ((xdma_status >> 4) & 1u)
              << " rx_tvalid=" << ((xdma_status >> 5) & 1u)
              << " tx_tready=" << ((xdma_status >> 6) & 1u)
              << " tx_tvalid=" << ((xdma_status >> 7) & 1u)
              << "\n";
    std::cout << "  center.rx_sample_bytes=0x" << std::hex
              << ctrl->read_axi(kCenterCtrlBase + kRegRxSampleBytes, 1.0)
              << " center.max_sample_bytes_per_packet=0x"
              << ctrl->read_axi(kCenterCtrlBase + kRegMaxSampleBytesPerPacket, 1.0)
              << " center.rx_mode=0x"
              << ctrl->read_axi(kCenterCtrlBase + kRegRxMode, 1.0)
              << " center.channel_enable=0x"
              << ctrl->read_axi(kCenterCtrlBase + kRegChannelEnable, 1.0)
              << " center.dma_s2mm_pkt_per_burst=0x"
              << ctrl->read_axi(kCenterCtrlBase + kRegDmaPktPerBurst, 1.0)
              << std::dec << "\n";
}

bool rx_iq_path_has_data(const std::shared_ptr<m300_xdma_ctrl>& ctrl)
{
    const uint32_t xdma_status = ctrl->read_reg(kRegXdmaStatus, 1.0);
    const bool iq_rx128_tvalid = ((xdma_status >> 3) & 1u) != 0u;
    const bool rx_tvalid = ((xdma_status >> 5) & 1u) != 0u;
    return iq_rx128_tvalid || rx_tvalid;
}

void configure_iq_framer(const std::shared_ptr<m300_xdma_ctrl>& ctrl,
                         uint32_t packet_bytes,
                         uint32_t channel_enable,
                         uint32_t rx_mode,
                         uint32_t dma_pkt_per_burst,
                         bool trace)
{
    const uint32_t payload_bytes = packet_bytes - kPacketPrefixBytes;
    auto write = [&](uint32_t offset, uint32_t value, const char* name) {
        if (trace) {
            std::cout << "center_axi_write " << name
                      << " addr=0x" << std::hex << (kCenterCtrlBase + offset)
                      << " value=0x" << value << std::dec << "\n";
        }
    ctrl->write_axi(kCenterCtrlBase + offset, value, 1.0);
    };

    write(kRegModeExit, 1u, "mode_exit");
    write(kRegRxSampleBytes, payload_bytes, "rx_sample_bytes");
    write(kRegMaxSampleBytesPerPacket, payload_bytes, "max_sample_bytes_per_packet");
    write(kRegChannelEnable, channel_enable, "channel_enable");
    write(kRegDmaPktPerBurst, dma_pkt_per_burst, "dma_s2mm_pkt_per_burst");
    write(kRegRxMode, rx_mode, "rx_mode");
    write(kRegRxModeStrobe, 1u, "rx_mode_strobe");
    write(kRegStreamStart, 1u, "stream_start");
}

bool ad9361_looks_initialized(const std::shared_ptr<m300_xdma_ctrl>& ctrl,
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

void process_direct_read_bytes(const uint8_t* data,
                               size_t got_bytes,
                               size_t& pkt_i,
                               size_t target_packets,
                               size_t warmup_packets,
                               uint32_t packet_bytes,
                               uint32_t lanes,
                               size_t dump_samples,
                               size_t packet_log,
                               bool collect_stats,
                               bool trace_config,
                               std::ofstream& save_cs16,
                               rx_process_state& state,
                               const std::function<void()>& start_measurement)
{
    size_t offset = 0;
    while (offset + kPacketPrefixBytes <= got_bytes && pkt_i < target_packets) {
        const m300_header hdr0 = parse_header(data + offset);
        if (hdr0.magic_type != M300_MAGIC_RX ||
            hdr0.length < kPacketPrefixBytes ||
            hdr0.length > packet_bytes) {
            const size_t next = find_next_rx_header(data, offset + 16u, got_bytes);
            if (next >= got_bytes)
                break;
            if (trace_config) {
                std::cerr << "skip padding/noise bytes=" << (next - offset)
                          << " next_header_offset=" << next << "\n";
            }
            offset = next;
            continue;
        }

        if (offset + hdr0.length > got_bytes)
            break;

        const bool measure_packet = pkt_i >= warmup_packets;
        if (measure_packet)
            start_measurement();
        process_rx_packet(data + offset, hdr0.length, pkt_i,
                          packet_bytes, lanes, dump_samples, packet_log,
                          collect_stats, measure_packet,
                          save_cs16.is_open() ? &save_cs16 : nullptr, state);
        offset += hdr0.length;
        pkt_i++;
    }
}

void usage()
{
    std::cout << "Usage: m300_rx_iq_probe [--base /dev/xdma0]\n"
              << "                         [--reads N]\n"
              << "                         [--packet-bytes N]\n"
              << "                         [--channel-enable MASK]\n"
              << "                         [--rx-mode N]\n"
              << "                         [--lanes N]\n"
              << "                         [--sample-rate HZ]\n"
              << "                         [--bandwidth HZ]\n"
              << "                         [--refclk HZ]\n"
              << "                         [--dump-samples N]\n"
              << "                         [--packet-log N]\n"
              << "                         [--bulk-packets N]\n"
              << "                         [--warmup-packets N]\n"
              << "                         [--bandwidth-only]\n"
              << "                         [--no-stats]\n"
              << "                         [--reader-thread]\n"
              << "                         [--reader-threads N]\n"
              << "                         [--single-thread-read]\n"
              << "                         [--driver-ring-read]\n"
              << "                         [--driver-ring-depth N]\n"
              << "                         [--readv-read]      (disabled: XDMA read_iter uses unsafe AIO path)\n"
              << "                         [--readv-depth N]   (disabled)\n"
              << "                         [--aio-read]        (disabled: XDMA io_destroy can hang)\n"
              << "                         [--aio-depth N]     (disabled)\n"
              << "                         [--direct-read]\n"
              << "                         [--transport-read]\n"
              << "                         [--rx-device /dev/xdma0_c2h_1]\n"
              << "                         [--save-cs16 FILE]\n"
              << "                         [--timeout SEC]\n"
              << "                         [--auto-init-ad9361]\n"
              << "                         [--init-ad9361]\n"
              << "                         [--skip-init-ad9361]\n"
              << "                         [--no-configure-iq]\n"
              << "                         [--c2h-test-source]\n"
              << "                         [--iq-source]\n"
              << "                         [--keep-running]\n"
              << "                         [--trace-config]\n";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        std::string base = "/dev/xdma0";
        size_t reads = 8;
        uint32_t packet_bytes = kDefaultPacketBytes;
        uint32_t channel_enable = 0x03u;
        uint32_t rx_mode = 1u;
        uint32_t dma_pkt_per_burst = 1u;
        uint32_t lanes = 4u;
        uint32_t refclk_hz = 40000000u;
        uint32_t sample_rate_hz = 61440000u;
        uint32_t bandwidth_hz = 0u;
        size_t dump_samples = 32;
        size_t packet_log = 8;
        size_t bulk_packets = 1;
        size_t warmup_packets = 0;
        double timeout_sec = 1.0;
        ad9361_init_policy init_policy = ad9361_init_policy::auto_detect;
        bool configure_iq = true;
        bool keep_running = false;
        bool trace_config = false;
        bool c2h_test_source = false;
        bool direct_read = false;
        bool force_transport_read = false;
        bool collect_stats = true;
        bool bandwidth_only = false;
        bool use_reader_thread = false;
        bool use_driver_ring_read = false;
        size_t reader_threads = 1;
        uint32_t driver_ring_depth = 256u;
        std::string rx_device;
        std::string save_cs16_path;

        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--base" && i + 1 < argc) {
                base = argv[++i];
            } else if (arg == "--reads" && i + 1 < argc) {
                reads = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--packet-bytes" && i + 1 < argc) {
                packet_bytes = parse_u32(argv[++i]);
            } else if (arg == "--channel-enable" && i + 1 < argc) {
                channel_enable = parse_u32(argv[++i]);
            } else if (arg == "--rx-mode" && i + 1 < argc) {
                rx_mode = parse_u32(argv[++i]);
            } else if (arg == "--dma-pkt-per-burst" && i + 1 < argc) {
                dma_pkt_per_burst = parse_u32(argv[++i]);
            } else if (arg == "--lanes" && i + 1 < argc) {
                lanes = parse_u32(argv[++i]);
            } else if (arg == "--sample-rate" && i + 1 < argc) {
                sample_rate_hz = parse_u32(argv[++i]);
            } else if (arg == "--bandwidth" && i + 1 < argc) {
                bandwidth_hz = parse_u32(argv[++i]);
            } else if (arg == "--refclk" && i + 1 < argc) {
                refclk_hz = parse_u32(argv[++i]);
            } else if (arg == "--dump-samples" && i + 1 < argc) {
                dump_samples = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--packet-log" && i + 1 < argc) {
                packet_log = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--bulk-packets" && i + 1 < argc) {
                bulk_packets = static_cast<size_t>(parse_u64(argv[++i]));
                direct_read = true;
            } else if (arg == "--warmup-packets" && i + 1 < argc) {
                warmup_packets = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--bandwidth-only") {
                bandwidth_only = true;
                collect_stats = false;
                dump_samples = 0;
                packet_log = 0;
                direct_read = true;
                use_reader_thread = true;
            } else if (arg == "--no-stats") {
                collect_stats = false;
            } else if (arg == "--reader-thread") {
                use_reader_thread = true;
                direct_read = true;
            } else if (arg == "--reader-threads" && i + 1 < argc) {
                reader_threads = static_cast<size_t>(parse_u64(argv[++i]));
                use_reader_thread = reader_threads != 0u;
                direct_read = true;
            } else if (arg == "--single-thread-read") {
                use_reader_thread = false;
                use_driver_ring_read = false;
            } else if (arg == "--driver-ring-read") {
                use_driver_ring_read = true;
                use_reader_thread = false;
                direct_read = true;
            } else if (arg == "--driver-ring-depth" && i + 1 < argc) {
                driver_ring_depth = parse_u32(argv[++i]);
                use_driver_ring_read = true;
                use_reader_thread = false;
                direct_read = true;
            } else if (arg == "--readv-read") {
                throw std::runtime_error("--readv-read is disabled because XDMA read_iter uses the same unsafe async path as native AIO");
            } else if (arg == "--readv-depth" && i + 1 < argc) {
                ++i;
                throw std::runtime_error("--readv-depth is disabled with --readv-read");
            } else if (arg == "--aio-read") {
                throw std::runtime_error("--aio-read is disabled because Linux native AIO can hang in io_destroy() on the XDMA char device");
            } else if (arg == "--aio-depth" && i + 1 < argc) {
                ++i;
                throw std::runtime_error("--aio-depth is disabled with --aio-read");
            } else if (arg == "--timeout" && i + 1 < argc) {
                timeout_sec = std::stod(argv[++i]);
            } else if (arg == "--direct-read") {
                direct_read = true;
                force_transport_read = false;
            } else if (arg == "--transport-read") {
                force_transport_read = true;
                direct_read = false;
                use_reader_thread = false;
                use_driver_ring_read = false;
            } else if (arg == "--rx-device" && i + 1 < argc) {
                rx_device = argv[++i];
                direct_read = true;
                force_transport_read = false;
            } else if (arg == "--save-cs16" && i + 1 < argc) {
                save_cs16_path = argv[++i];
            } else if (arg == "--auto-init-ad9361") {
                init_policy = ad9361_init_policy::auto_detect;
            } else if (arg == "--init-ad9361") {
                init_policy = ad9361_init_policy::force;
            } else if (arg == "--skip-init-ad9361") {
                init_policy = ad9361_init_policy::skip;
            } else if (arg == "--no-configure-iq") {
                configure_iq = false;
            } else if (arg == "--c2h-test-source") {
                c2h_test_source = true;
                init_policy = ad9361_init_policy::skip;
                configure_iq = false;
                direct_read = true;
            } else if (arg == "--iq-source") {
                c2h_test_source = false;
            } else if (arg == "--keep-running") {
                keep_running = true;
            } else if (arg == "--trace-config") {
                trace_config = true;
            } else if (arg == "--help" || arg == "-h") {
                usage();
                return 0;
            } else {
                usage();
                return 1;
            }
        }

        if (c2h_test_source) {
            init_policy = ad9361_init_policy::skip;
            configure_iq = false;
            if (!force_transport_read)
                direct_read = true;
        }
        if (force_transport_read) {
            direct_read = false;
            use_reader_thread = false;
            use_driver_ring_read = false;
        }
        if (use_driver_ring_read) {
            use_reader_thread = false;
        }

        if (packet_bytes <= kPacketPrefixBytes || (packet_bytes & 0xfu) != 0u) {
            throw std::runtime_error("packet_bytes must be greater than 16 and 16-byte aligned");
        }
        if (lanes == 0u || lanes > 4u) {
            throw std::runtime_error("lanes must be 1..4");
        }
        if (bulk_packets == 0u) {
            throw std::runtime_error("bulk_packets must be greater than 0");
        }
        if (driver_ring_depth < 2u) {
            throw std::runtime_error("driver_ring_depth must be at least 2");
        }
        if (bandwidth_only && warmup_packets == 0u)
            warmup_packets = 128u;

        if (direct_read && rx_device.empty())
            rx_device = rx_path_from_device_arg(base);

        int direct_rx_fd = -1;
        if (direct_read) {
            direct_rx_fd = ::open(rx_device.c_str(), O_RDONLY);
            if (direct_rx_fd < 0) {
                throw std::runtime_error("failed to open " + rx_device + ": " +
                                         std::strerror(errno));
            }
        }

        auto dev = std::make_shared<M300XdmaImpl>(default_base(base), !direct_read);
        if (!dev->isInitialSuccess()) {
            std::cerr << "M300_XDMA device init failed: " << dev->last_error() << "\n";
            return 1;
        }

        auto ctrl = dev->get_ctrl();
        const auto version = ctrl->get_version(1.0);
        std::cout << "version=0x" << std::hex << std::setw(8) << std::setfill('0')
                  << version.pkt.value0
                  << " build=0x" << std::setw(8) << version.pkt.value1
                  << std::dec << std::setfill(' ') << "\n";

        ctrl->stop_rx(1.0);
        if (!direct_read)
            dev->configure_rx_packet_bytes(packet_bytes);
        ctrl->clear_counters();
        ctrl->write_reg(M300_REG_RX_SOURCE_SEL,
                        c2h_test_source ? kRxSourceC2hTest : kRxSourceIq, 1.0);
        ctrl->set_rx_packet_bytes(packet_bytes, 1.0);

        bool need_ad9361_init = init_policy == ad9361_init_policy::force;
        if (init_policy == ad9361_init_policy::auto_detect) {
            need_ad9361_init = !ad9361_looks_initialized(ctrl, true);
        } else if (init_policy == ad9361_init_policy::skip) {
            std::cout << "ad9361_init skipped by option\n";
        }

        if (need_ad9361_init) {
            std::cout << "ad9361_init begin"
                      << (init_policy == ad9361_init_policy::auto_detect ? " (auto)" : "")
                      << "\n";
            m300_ad9361_ctrl ad9361(ctrl);
            m300_ad9361_init_options options;
            options.reference_clk_rate_hz = refclk_hz;
            options.sample_rate_hz = sample_rate_hz;
            options.bandwidth_hz = bandwidth_hz;
            std::cout << "init_options refclk=" << options.reference_clk_rate_hz
                      << " sample_rate=" << options.sample_rate_hz
                      << " bandwidth=" << options.bandwidth_hz
                      << " rx_lo=" << options.rx_lo_hz
                      << " tx_lo=" << options.tx_lo_hz
                      << "\n";
            ad9361.init(options);
            std::cout << "ad9361_init done\n";
        } else if (init_policy == ad9361_init_policy::auto_detect) {
            std::cout << "ad9361_init skipped: existing configuration looks valid\n";
        }

        ctrl->start_rx(1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (configure_iq) {
            configure_iq_framer(ctrl, packet_bytes, channel_enable, rx_mode,
                                dma_pkt_per_burst, trace_config);
        }
        dump_runtime_status(ctrl, "status_after_start");
        if (!c2h_test_source && init_policy == ad9361_init_policy::skip &&
            !rx_iq_path_has_data(ctrl)) {
            throw std::runtime_error("RX IQ path has no valid data; AD9361/axi_ad9361 is probably not initialized. "
                                     "Run without --skip-init-ad9361 or use --auto-init-ad9361.");
        }

        rx_process_state state;
        state.stats.resize(lanes);
        std::ofstream save_cs16;
        if (!save_cs16_path.empty()) {
            save_cs16.open(save_cs16_path, std::ios::binary);
            if (!save_cs16) {
                throw std::runtime_error("failed to open --save-cs16 output: " + save_cs16_path);
            }
        }
        std::chrono::steady_clock::time_point start;
        bool measurement_started = false;
        const size_t target_packets = warmup_packets + reads;
        const auto maybe_start_measurement = [&]() {
            if (!measurement_started) {
                start = std::chrono::steady_clock::now();
                measurement_started = true;
            }
        };

        if (direct_read) {
            const size_t max_xdma_read = 255u * 4096u;
            const size_t packet_stride = align_up_size(packet_bytes, 4096u);
            bulk_packets = std::min(bulk_packets, std::max<size_t>(1u, max_xdma_read / packet_stride));
            const size_t bulk_bytes = packet_stride * bulk_packets;
            std::vector<uint8_t> bulk_buffer(bulk_bytes);
            std::cout << "direct_rx_read device=" << rx_device
                      << " packet_bytes=" << packet_bytes
                      << " packet_stride=" << packet_stride
                      << " bulk_packets=" << bulk_packets
                      << " bulk_bytes=" << bulk_bytes
                      << " warmup_packets=" << warmup_packets
                      << " reader_thread=" << (use_reader_thread ? 1 : 0)
                      << " reader_threads=" << (use_reader_thread ? reader_threads : 0)
                      << " driver_ring_read=" << (use_driver_ring_read ? 1 : 0)
                      << " driver_ring_depth=" << (use_driver_ring_read ? driver_ring_depth : 0)
                      << "\n";

            size_t pkt_i = 0;
            if (use_driver_ring_read) {
                xdma_m300_c2h_ring_start ring_cfg {};
                ring_cfg.version = kM300C2hRingVersion;
                ring_cfg.packet_bytes = static_cast<uint32_t>(packet_stride);
                ring_cfg.depth = driver_ring_depth;
                std::vector<uint8_t> ring_packet(packet_stride);

                if (::ioctl(direct_rx_fd, IOCTL_XDMA_M300_C2H_RING_START, &ring_cfg) != 0) {
                    const int ioctl_errno = errno;
                    ::close(direct_rx_fd);
                    throw std::runtime_error(errno_text("M300 C2H ring start failed", ioctl_errno));
                }

                try {
                    while (pkt_i < target_packets) {
                        xdma_m300_c2h_ring_get get {};
                        get.buffer = reinterpret_cast<uint64_t>(ring_packet.data());
                        get.buffer_bytes = static_cast<uint32_t>(ring_packet.size());
                        get.timeout_ms = static_cast<uint32_t>(timeout_sec * 1000.0);
                        if (get.timeout_ms == 0u)
                            get.timeout_ms = 1000u;

                        if (::ioctl(direct_rx_fd, IOCTL_XDMA_M300_C2H_RING_GET, &get) != 0) {
                            const int ioctl_errno = errno;
                            throw std::runtime_error(errno_text("M300 C2H ring get failed", ioctl_errno));
                        }
                        if (get.status != 0u) {
                            throw std::runtime_error("M300 C2H ring packet error status=" +
                                                     std::to_string(get.status));
                        }
                        if (get.bytes == 0u) {
                            throw std::runtime_error("M300 C2H ring returned 0 bytes");
                        }

                        process_direct_read_bytes(ring_packet.data(), get.bytes, pkt_i,
                                                  target_packets, warmup_packets,
                                                  packet_bytes, lanes, dump_samples, packet_log,
                                                  collect_stats, trace_config, save_cs16,
                                                  state, maybe_start_measurement);

                        if (trace_config && get.bytes != packet_stride) {
                            std::cout << "driver_ring_packet got=" << get.bytes
                                      << " packet_stride=" << packet_stride
                                      << " ring_index=" << get.index
                                      << " ring_sequence=" << get.sequence
                                      << " ring_completed=" << get.completed
                                      << " ring_errors=" << get.errors << "\n";
                        }
                    }
                } catch (...) {
                    (void)::ioctl(direct_rx_fd, IOCTL_XDMA_M300_C2H_RING_STOP);
                    throw;
                }

                if (::ioctl(direct_rx_fd, IOCTL_XDMA_M300_C2H_RING_STOP) != 0) {
                    const int ioctl_errno = errno;
                    ::close(direct_rx_fd);
                    throw std::runtime_error(errno_text("M300 C2H ring stop failed", ioctl_errno));
                }
            } else if (use_reader_thread) {
                constexpr size_t kQueueDepth = 8u;
                direct_read_queue queue;
                const size_t total_blocks = (target_packets + bulk_packets - 1u) / bulk_packets;
                std::atomic<size_t> next_block{0};
                const size_t active_reader_threads = std::max<size_t>(1u, reader_threads);

                auto reader_func = [&](size_t reader_id) {
                    try {
                        int reader_fd = direct_rx_fd;
                        if (active_reader_threads > 1u && reader_id != 0u) {
                            reader_fd = ::open(rx_device.c_str(), O_RDONLY);
                            if (reader_fd < 0) {
                                const int open_errno = errno;
                                std::lock_guard<std::mutex> lock(queue.mutex);
                                queue.error = "reader " + std::to_string(reader_id) +
                                              " failed to open " + rx_device + ": errno=" +
                                              std::to_string(open_errno) + " (" +
                                              std::strerror(open_errno) + ")" +
                                              ". XDMA char C2H appears to allow only one open/read stream.";
                                queue.done = true;
                                queue.cv_not_empty.notify_all();
                                queue.cv_not_full.notify_all();
                                return;
                            }
                        }

                        for (;;) {
                            const size_t block_i = next_block.fetch_add(1u);
                            if (block_i >= total_blocks)
                                break;
                            const size_t packets_done = block_i * bulk_packets;
                            const size_t want_packets = std::min(bulk_packets, target_packets - packets_done);
                            const size_t want_bytes = want_packets * packet_stride;
                            direct_read_block block;
                            block.index = block_i;
                            block.data.resize(want_bytes);

                            const ssize_t rc = read_full_retry(reader_fd, block.data.data(), want_bytes);
                            if (rc < 0) {
                                const int read_errno = errno;
                                if (active_reader_threads > 1u && reader_id != 0u)
                                    ::close(reader_fd);
                                std::lock_guard<std::mutex> lock(queue.mutex);
                                queue.error = errno_text("RX read failed", read_errno);
                                queue.done = true;
                                queue.cv_not_empty.notify_all();
                                queue.cv_not_full.notify_all();
                                return;
                            }
                            if (rc == 0) {
                                if (active_reader_threads > 1u && reader_id != 0u)
                                    ::close(reader_fd);
                                std::lock_guard<std::mutex> lock(queue.mutex);
                                queue.error = "RX read returned 0 bytes";
                                queue.done = true;
                                queue.cv_not_empty.notify_all();
                                queue.cv_not_full.notify_all();
                                return;
                            }
                            block.bytes = static_cast<size_t>(rc);

                            std::unique_lock<std::mutex> lock(queue.mutex);
                            queue.cv_not_full.wait(lock, [&]() {
                                return queue.blocks.size() < kQueueDepth * active_reader_threads || queue.done;
                            });
                            if (queue.done) {
                                if (active_reader_threads > 1u && reader_id != 0u)
                                    ::close(reader_fd);
                                return;
                            }
                            queue.blocks.emplace(block.index, std::move(block));
                            queue.cv_not_empty.notify_one();
                        }

                        if (active_reader_threads > 1u && reader_id != 0u)
                            ::close(reader_fd);
                        std::lock_guard<std::mutex> lock(queue.mutex);
                        if (next_block.load() >= total_blocks) {
                            queue.done = true;
                            queue.cv_not_empty.notify_all();
                        }
                    } catch (const std::exception& ex) {
                        std::lock_guard<std::mutex> lock(queue.mutex);
                        queue.error = ex.what();
                        queue.done = true;
                        queue.cv_not_empty.notify_all();
                        queue.cv_not_full.notify_all();
                    }
                };

                std::vector<std::thread> readers;
                readers.reserve(active_reader_threads);
                for (size_t i = 0; i < active_reader_threads; ++i)
                    readers.emplace_back(reader_func, i);

                try {
                    size_t next_consume_block = 0;
                    while (pkt_i < target_packets) {
                        direct_read_block block;
                        {
                            std::unique_lock<std::mutex> lock(queue.mutex);
                            queue.cv_not_empty.wait(lock, [&]() {
                                return queue.blocks.count(next_consume_block) != 0u ||
                                       queue.done || !queue.error.empty();
                            });
                            const auto it = queue.blocks.find(next_consume_block);
                            if (it == queue.blocks.end()) {
                                if (!queue.error.empty())
                                    throw std::runtime_error(queue.error);
                                break;
                            }
                            block = std::move(it->second);
                            queue.blocks.erase(it);
                            next_consume_block++;
                            queue.cv_not_full.notify_one();
                        }

                        process_direct_read_bytes(block.data.data(), block.bytes, pkt_i,
                                                  target_packets, warmup_packets,
                                                  packet_bytes, lanes, dump_samples, packet_log,
                                                  collect_stats, trace_config, save_cs16,
                                                  state, maybe_start_measurement);

                        if (trace_config && (block.bytes % packet_stride) != 0u) {
                            std::cout << "bulk_read_partial got=" << block.bytes
                                      << " packet_stride=" << packet_stride << "\n";
                        }
                    }
                } catch (...) {
                    {
                        std::lock_guard<std::mutex> lock(queue.mutex);
                        queue.done = true;
                        queue.cv_not_full.notify_all();
                    }
                    for (auto& reader : readers)
                        reader.join();
                    throw;
                }
                for (auto& reader : readers)
                    reader.join();
            } else {
                while (pkt_i < target_packets) {
                    const size_t want_packets = std::min(bulk_packets, target_packets - pkt_i);
                    const size_t want_bytes = want_packets * packet_stride;
                    const ssize_t rc = read_full_retry(direct_rx_fd, bulk_buffer.data(), want_bytes);
                    if (rc < 0) {
                        const int read_errno = errno;
                        ::close(direct_rx_fd);
                        throw std::runtime_error(errno_text("RX read failed", read_errno));
                    }
                    if (rc == 0) {
                        ::close(direct_rx_fd);
                        throw std::runtime_error("RX read returned 0 bytes");
                    }

                    const size_t got_bytes = static_cast<size_t>(rc);
                    process_direct_read_bytes(bulk_buffer.data(), got_bytes, pkt_i,
                                              target_packets, warmup_packets,
                                              packet_bytes, lanes, dump_samples, packet_log,
                                              collect_stats, trace_config, save_cs16,
                                              state, maybe_start_measurement);

                    if (trace_config && (got_bytes % packet_stride) != 0u) {
                        std::cout << "bulk_read_partial got=" << got_bytes
                                  << " want=" << want_bytes
                                  << " packet_stride=" << packet_stride << "\n";
                    }
                }
            }
            ::close(direct_rx_fd);
        } else {
            auto rx_xport = dev->get_rx_xport();
            for (size_t pkt_i = 0; pkt_i < target_packets; ++pkt_i) {
                auto buff = rx_xport->get_recv_buff(timeout_sec);
                if (!buff) {
                    dump_runtime_status(ctrl, "status_after_rx_timeout");
                    throw std::runtime_error("timeout waiting for RX IQ packet");
                }

                const auto* p = static_cast<const uint8_t*>(buff->cast<const void*>());
                const bool measure_packet = pkt_i >= warmup_packets;
                if (measure_packet)
                    maybe_start_measurement();
                process_rx_packet(p, buff->size(), pkt_i, packet_bytes, lanes,
                                  dump_samples, packet_log,
                                  collect_stats, measure_packet,
                                  save_cs16.is_open() ? &save_cs16 : nullptr, state);
            }
        }

        const auto end = std::chrono::steady_clock::now();
        const double sec = measurement_started ?
            std::chrono::duration<double>(end - start).count() : 0.0;
        const double mib_s = sec > 0.0 ? static_cast<double>(state.bytes) / (1024.0 * 1024.0) / sec : 0.0;
        const double payload_mib_s = sec > 0.0 ? static_cast<double>(state.payload_bytes) / (1024.0 * 1024.0) / sec : 0.0;
        const double payload_gbps = sec > 0.0 ? static_cast<double>(state.payload_bytes) * 8.0 / 1.0e9 / sec : 0.0;
        const double expected_payload_mib_s =
            static_cast<double>(sample_rate_hz) *
            static_cast<double>(count_enabled_channels(channel_enable)) *
            sizeof(uint32_t) / (1024.0 * 1024.0);
        std::cout << "done: packets=" << reads
                  << " warmup_packets=" << warmup_packets
                  << " bytes=" << state.bytes
                  << " payload_bytes=" << state.payload_bytes
                  << " elapsed=" << sec
                  << " sec rate=" << mib_s << " MiB/s"
                  << " payload_rate=" << payload_mib_s << " MiB/s"
                  << " payload_gbps=" << payload_gbps
                  << " expected_payload_at_config=" << expected_payload_mib_s << " MiB/s"
                  << " seq_jumps=" << state.seq_jumps
                  << " lost_packets=" << state.lost_packets
                  << " reordered_or_wrapped=" << state.reordered_or_wrapped
                  << " timestamp_gaps=" << state.timestamp_gaps
                  << " timestamp_step=" << state.timestamp_step
                  << " short_packets=" << state.short_packets << "\n";
        dump_runtime_status(ctrl, "status_after_read");
        if (collect_stats)
            print_stats(state.stats);

        if (!keep_running)
            ctrl->stop_rx(1.0);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_rx_iq_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
