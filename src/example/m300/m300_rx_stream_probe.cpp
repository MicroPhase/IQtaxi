#include "src/driver/M300/m300_ad9361_ctrl.hpp"
#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/transport/local_regs.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

std::string default_base(const std::string& base)
{
    return base.empty() ? std::string("/dev/xdma0") : base;
}

uint64_t parse_u64(const std::string& text)
{
    return std::stoull(text, nullptr, 0);
}

void usage()
{
    std::cout << "Usage: m300_rx_stream_probe [--base /dev/xdma0]\n"
              << "                            [--reads N]\n"
              << "                            [--samples N]\n"
              << "                            [--channel-enable MASK]\n"
              << "                            [--skip-init-ad9361]\n";
}

}

int main(int argc, char** argv)
{
    try {
        std::string base = "/dev/xdma0";
        size_t reads = 8;
        size_t samples = 4096;
        uint8_t channel_enable = 0x03;
        bool init_ad9361 = true;

        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--base" && i + 1 < argc) {
                base = argv[++i];
            } else if (arg == "--reads" && i + 1 < argc) {
                reads = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = static_cast<size_t>(parse_u64(argv[++i]));
            } else if (arg == "--channel-enable" && i + 1 < argc) {
                channel_enable = static_cast<uint8_t>(parse_u64(argv[++i]));
            } else if (arg == "--init-ad9361") {
                init_ad9361 = true;
            } else if (arg == "--skip-init-ad9361") {
                init_ad9361 = false;
            } else if (arg == "--help" || arg == "-h") {
                usage();
                return 0;
            } else {
                usage();
                return 1;
            }
        }

        auto dev = std::make_shared<M300XdmaImpl>(default_base(base));
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

        if (init_ad9361) {
            sdr::driver::m300_ad9361_ctrl ad9361(ctrl);
            ad9361.init(sdr::driver::m300_ad9361_init_options{});
        }

        auto rx = dev->get_rx_stream();
        rx->set_rx_enable_chan(channel_enable);
        uint64_t ts = 0;
        rx->set_recv_param(STREAM_MODE, samples, ts, 1, 0);

        std::vector<int16_t> buffer(samples * 2u);
        uint64_t total = 0;
        uint64_t nonzero = 0;
        int16_t min_i = std::numeric_limits<int16_t>::max();
        int16_t max_i = std::numeric_limits<int16_t>::min();
        int16_t min_q = std::numeric_limits<int16_t>::max();
        int16_t max_q = std::numeric_limits<int16_t>::min();
        const auto start = std::chrono::steady_clock::now();

        for (size_t r = 0; r < reads; ++r) {
            void* ptr = buffer.data();
            sdr::api::rx_streamer::buffs_type buffs(&ptr, 1);
            uint64_t timestamp = 0;
            const size_t got = rx->recv(buffs, samples, timestamp, MICRORF_FORMAT_INT16);
            std::cout << "recv[" << r << "] samples=" << got
                      << " timestamp=0x" << std::hex << timestamp << std::dec << "\n";

            for (size_t i = 0; i < got; ++i) {
                const int16_t i_sample = buffer[2u * i + 0u];
                const int16_t q_sample = buffer[2u * i + 1u];
                if (i_sample != 0 || q_sample != 0)
                    nonzero++;
                min_i = std::min(min_i, i_sample);
                max_i = std::max(max_i, i_sample);
                min_q = std::min(min_q, q_sample);
                max_q = std::max(max_q, q_sample);
            }
            total += got;
        }

        const auto end = std::chrono::steady_clock::now();
        const double sec = std::chrono::duration<double>(end - start).count();
        const double msps = sec > 0.0 ? static_cast<double>(total) / 1.0e6 / sec : 0.0;
        std::cout << "done: samples=" << total
                  << " nonzero=" << nonzero
                  << " elapsed=" << sec
                  << " sec rate=" << msps << " MSps"
                  << " I[min,max]=" << min_i << "," << max_i
                  << " Q[min,max]=" << min_q << "," << max_q << "\n";

        rx->set_recv_param(STREAM_MODE, samples, ts, 0, 1);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_rx_stream_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
