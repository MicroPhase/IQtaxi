#include "src/driver/M300/m300_ad9361_ctrl.hpp"
#include "src/driver/M300/m300_xdma_impl.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using sdr::driver::m300_ad9361_ctrl;

namespace {
constexpr uint16_t kAd9361RegProductId = 0x037u;

std::string default_base(const std::string& base)
{
    return base.empty() ? std::string("/dev/xdma0") : base;
}

void usage()
{
    std::cout << "Usage: m300_ad9361_init_probe [--base /dev/xdma0]\n"
              << "                              [--refclk HZ]\n"
              << "                              [--rx-lo HZ]\n"
              << "                              [--tx-lo HZ]\n"
              << "                              [--sample-rate HZ]\n"
              << "                              [--bandwidth HZ]\n"
              << "                              [--skip-digital-tune]\n";
}

uint64_t parse_u64(const std::string& text)
{
    return std::stoull(text, nullptr, 0);
}
}

int main(int argc, char** argv)
{
    try {
        std::string base = "/dev/xdma0";
        sdr::driver::m300_ad9361_init_options options;

        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--base" && i + 1 < argc) {
                base = argv[++i];
            } else if (arg == "--refclk" && i + 1 < argc) {
                options.reference_clk_rate_hz =
                    static_cast<uint32_t>(parse_u64(argv[++i]));
            } else if (arg == "--rx-lo" && i + 1 < argc) {
                options.rx_lo_hz = parse_u64(argv[++i]);
            } else if (arg == "--tx-lo" && i + 1 < argc) {
                options.tx_lo_hz = parse_u64(argv[++i]);
            } else if (arg == "--sample-rate" && i + 1 < argc) {
                options.sample_rate_hz = static_cast<uint32_t>(parse_u64(argv[++i]));
            } else if (arg == "--bandwidth" && i + 1 < argc) {
                options.bandwidth_hz = static_cast<uint32_t>(parse_u64(argv[++i]));
            } else if (arg == "--skip-digital-tune") {
                options.run_post_init_digital_tune = false;
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

        const auto ctrl = dev->get_ctrl();
        const auto version = ctrl->get_version(1.0);
        std::cout << "version=0x" << std::hex << std::setw(8) << std::setfill('0')
                  << version.pkt.value0
                  << " build=0x" << std::setw(8) << version.pkt.value1
                  << std::dec << "\n";

        const uint32_t gpio_out = ctrl->read_gpio_out(1.0);
        const uint32_t gpio_oe = ctrl->read_gpio_oe(1.0);
        const uint32_t gpio_in = ctrl->read_gpio_in(1.0);
        std::cout << "gpio_out=0x" << std::hex << std::setw(8) << gpio_out
                  << " gpio_oe=0x" << std::setw(8) << gpio_oe
                  << " gpio_in=0x" << std::setw(8) << gpio_in << std::dec << "\n";

        const uint8_t product_id_before = ctrl->ad9361_spi_read(kAd9361RegProductId, 1.0);
        std::cout << "ad9361_product_id_before=0x" << std::hex << std::setw(2)
                  << static_cast<uint32_t>(product_id_before) << std::dec << "\n";
        std::cout << "init_options refclk=" << options.reference_clk_rate_hz
                  << " sample_rate=" << options.sample_rate_hz
                  << " bandwidth=" << options.bandwidth_hz
                  << " rx_lo=" << options.rx_lo_hz
                  << " tx_lo=" << options.tx_lo_hz
                  << " post_digital_tune=" << options.run_post_init_digital_tune
                  << "\n";

        m300_ad9361_ctrl ad9361(ctrl);
        ad9361.init(options);
        std::cout << "resolved_bandwidth=" << ad9361.get_bandwidth()
                  << " bandwidth_auto=" << (ad9361.bandwidth_is_auto() ? 1 : 0)
                  << "\n";

        const uint8_t product_id_after = ctrl->ad9361_spi_read(kAd9361RegProductId, 1.0);
        std::cout << "ad9361_product_id_after=0x" << std::hex << std::setw(2)
                  << static_cast<uint32_t>(product_id_after) << std::dec << "\n";
        std::cout << "m300_ad9361_init=PASS\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_ad9361_init_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
