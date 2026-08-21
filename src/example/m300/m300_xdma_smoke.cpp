#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/M300/m300_xdma_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace sdr::driver;

namespace {

std::string default_base(const std::string& base)
{
    return base.empty() ? std::string("/dev/xdma0") : base;
}

}

int main(int argc, char** argv)
{
    std::string base = "/dev/xdma0";
    size_t rx_reads = 8;
    uint32_t packet_bytes = 16384u;
    uint8_t sid = 0u;
    bool start_only = false;
    bool keep_running = false;
    bool bandwidth = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--base" && i + 1 < argc) {
            base = argv[++i];
        } else if (arg == "--reads" && i + 1 < argc) {
            rx_reads = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--packet-bytes" && i + 1 < argc) {
            packet_bytes = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--sid" && i + 1 < argc) {
            sid = static_cast<uint8_t>(std::stoul(argv[++i]));
        } else if (arg == "--start-only") {
            start_only = true;
        } else if (arg == "--keep-running") {
            keep_running = true;
        } else if (arg == "--bandwidth") {
            bandwidth = true;
        } else if (arg == "--help") {
            std::cout << "Usage: m300_xdma_smoke [--base /dev/xdma0] [--reads N] [--packet-bytes N] [--sid N] [--start-only] [--keep-running] [--bandwidth]\n";
            return 0;
        }
    }

    auto dev = std::make_shared<M300XdmaImpl>(default_base(base));
    if (!dev->isInitialSuccess()) {
        std::cerr << "M300_XDMA device init failed: " << dev->last_error() << "\n";
        return 1;
    }

    auto ctrl = dev->get_ctrl();
    const auto version = ctrl->get_version();
    std::cout << "version=0x" << std::hex << version.pkt.value0
              << " build=0x" << version.pkt.value1 << std::dec << "\n";

    ctrl->stop_rx();
    dev->configure_rx_packet_bytes(packet_bytes);
    ctrl->clear_counters();
    ctrl->set_rx_packet_bytes(packet_bytes);
    ctrl->set_rx_sid(sid);
    ctrl->start_rx();

    if (start_only) {
        std::cout << "rx started packet_bytes=" << packet_bytes
                  << " sid=" << unsigned(sid) << "\n";
        return 0;
    }

    auto rx_xport = dev->get_rx_xport();
    uint16_t expected_seq = 0;
    bool have_seq = false;
    uint64_t bytes = 0;
    uint64_t seq_jumps = 0;
    uint64_t lost_packets = 0;
    const auto start = std::chrono::steady_clock::now();

    for (size_t i = 0; i < rx_reads; ++i) {
        auto buff = rx_xport->get_recv_buff(1.0);
        if (!buff) {
            throw std::runtime_error("timeout waiting for RX packet");
        }

        const auto* p = static_cast<const uint8_t*>(buff->cast<const void*>());
        const m300_header hdr = parse_header(p);
        if (hdr.magic_type != M300_MAGIC_RX) {
            std::cerr << "bad magic: 0x" << std::hex << hdr.magic_type << std::dec << "\n";
            continue;
        }
        if (hdr.length != packet_bytes) {
            ctrl->stop_rx();
            throw std::runtime_error("RX packet length mismatch: expected " +
                                     std::to_string(packet_bytes) +
                                     " got " + std::to_string(hdr.length) +
                                     ". Rebuild/reload the FPGA bitstream if the RX framer was changed.");
        }
        if (!have_seq) {
            expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
            have_seq = true;
        } else {
            const uint16_t lost = static_cast<uint16_t>(hdr.seq - expected_seq);
            if (lost != 0) {
                seq_jumps++;
                lost_packets += lost;
                if (!bandwidth) {
                    std::cerr << "seq jump expected=" << expected_seq
                              << " got=" << hdr.seq
                              << " lost=" << lost << "\n";
                }
            }
            expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
        }

        bytes += hdr.length;

        if (!bandwidth) {
            std::cout << "rx[" << i << "] seq=" << hdr.seq
                      << " sid=" << unsigned(hdr.sid)
                      << " len=" << hdr.length << "\n";
        }
    }
    const auto end = std::chrono::steady_clock::now();

    if (bandwidth) {
        const double sec = std::chrono::duration<double>(end - start).count();
        const double mib_s = static_cast<double>(bytes) / (1024.0 * 1024.0) / sec;
        const double gbit_s = static_cast<double>(bytes) * 8.0 / 1000000000.0 / sec;

        std::cout << "done: packets=" << rx_reads
                  << " bytes=" << bytes
                  << " seq_jumps=" << seq_jumps
                  << " lost_packets=" << lost_packets << "\n";
        std::cout << "bandwidth: elapsed=" << sec
                  << " sec payload=" << mib_s << " MiB/s "
                  << gbit_s << " Gbit/s\n";
    }

    if (!keep_running)
        ctrl->stop_rx();
    return 0;
}
