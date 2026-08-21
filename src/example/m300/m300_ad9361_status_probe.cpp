#include "src/driver/M300/m300_xdma_impl.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

using namespace sdr::driver;

namespace {
constexpr uint32_t kAd9361RxBase = 0x44a00000u;
constexpr uint32_t kAd9361TxBase = 0x44a04000u;
constexpr uint64_t kM300AxiAd9361UpClkHz = 125000000ull;

std::string default_base(const std::string& base)
{
    return base.empty() ? std::string("/dev/xdma0") : base;
}

void print_hex8(const char* name, uint8_t value)
{
    std::cout << name << "=0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(value) << std::dec << "\n";
}

void print_hex32(const char* name, uint32_t value)
{
    std::cout << name << "=0x" << std::hex << std::setw(8) << std::setfill('0')
              << value << std::dec << "\n";
}

uint64_t clock_hz_from_monitor(uint32_t count, uint32_t ratio)
{
    return (static_cast<uint64_t>(count) * ratio * kM300AxiAd9361UpClkHz) >> 16;
}

void dump_ad9361_regs(const m300_xdma_ctrl::sptr& ctrl,
                      const char* title,
                      const uint16_t* regs,
                      size_t count)
{
    std::cout << title << "\n";
    for (size_t i = 0; i < count; ++i) {
        std::cout << "  [0x" << std::hex << std::setw(3) << std::setfill('0')
                  << regs[i] << "]=0x" << std::setw(2)
                  << static_cast<unsigned>(ctrl->ad9361_spi_read(regs[i], 1.0))
                  << std::dec << "\n";
    }
}

void dump_axi_core(const m300_xdma_ctrl::sptr& ctrl, const char* name, uint32_t base)
{
    std::cout << name << " base=0x" << std::hex << base << std::dec << "\n";
    print_hex32("  version", ctrl->read_axi(base + 0x0000u, 1.0));
    print_hex32("  rstn", ctrl->read_axi(base + 0x0040u, 1.0));
    print_hex32("  cntrl", ctrl->read_axi(base + 0x0044u, 1.0));
    const uint32_t clk_count = ctrl->read_axi(base + 0x0054u, 1.0);
    const uint32_t clk_ratio = ctrl->read_axi(base + 0x0058u, 1.0);
    print_hex32("  clk_freq", clk_count);
    print_hex32("  clk_ratio", clk_ratio);
    std::cout << "  clk_hz_125m=" << clock_hz_from_monitor(clk_count, clk_ratio) << "\n";
    print_hex32("  status", ctrl->read_axi(base + 0x005cu, 1.0));
}

void dump_axi_adc_channels(const m300_xdma_ctrl::sptr& ctrl, uint32_t base)
{
    std::cout << "axi_ad9361_rx_channels\n";
    for (uint32_t ch = 0; ch < 4; ++ch) {
        const uint32_t ch_base = base + 0x0400u + ch * 0x40u;
        std::cout << "  ch" << ch << "\n";
        print_hex32("    cntrl", ctrl->read_axi(ch_base + 0x00u, 1.0));
        print_hex32("    status", ctrl->read_axi(ch_base + 0x04u, 1.0));
        print_hex32("    cntrl_1", ctrl->read_axi(ch_base + 0x10u, 1.0));
        print_hex32("    cntrl_2", ctrl->read_axi(ch_base + 0x14u, 1.0));
        print_hex32("    cntrl_3", ctrl->read_axi(ch_base + 0x18u, 1.0));
    }
}
}

int main(int argc, char** argv)
{
    try {
        std::string base = "/dev/xdma0";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--base" && i + 1 < argc) {
                base = argv[++i];
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: m300_ad9361_status_probe [--base /dev/xdma0]\n";
                return 0;
            } else {
                std::cout << "Usage: m300_ad9361_status_probe [--base /dev/xdma0]\n";
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
        print_hex32("fnic_version", version.pkt.value0);
        print_hex32("fnic_build", version.pkt.value1);
        print_hex32("gpio_out", ctrl->read_gpio_out(1.0));
        print_hex32("gpio_oe", ctrl->read_gpio_oe(1.0));
        print_hex32("gpio_in", ctrl->read_gpio_in(1.0));

        print_hex8("ad9361_product_id", ctrl->ad9361_spi_read(0x037u, 1.0));
        print_hex8("ad9361_state", ctrl->ad9361_spi_read(0x017u, 1.0));
        print_hex8("ad9361_cal_ctrl", ctrl->ad9361_spi_read(0x016u, 1.0));
        print_hex8("ad9361_rx_cal_status", ctrl->ad9361_spi_read(0x244u, 1.0));
        print_hex8("ad9361_rx_vco_lock", ctrl->ad9361_spi_read(0x247u, 1.0));
        print_hex8("ad9361_tx_cal_status", ctrl->ad9361_spi_read(0x284u, 1.0));
        print_hex8("ad9361_tx_vco_lock", ctrl->ad9361_spi_read(0x287u, 1.0));
        print_hex8("ad9361_rx_clk_data_delay", ctrl->ad9361_spi_read(0x006u, 1.0));
        print_hex8("ad9361_tx_clk_data_delay", ctrl->ad9361_spi_read(0x007u, 1.0));

        const uint16_t clock_regs[] = {
            0x004u, 0x005u, 0x006u, 0x007u, 0x009u, 0x00au,
            0x010u, 0x011u, 0x012u, 0x013u, 0x014u, 0x015u,
            0x016u, 0x017u, 0x03au, 0x045u, 0x04bu, 0x04cu,
            0x04du, 0x050u, 0x051u, 0x2abu, 0x2acu
        };
        const uint16_t rfpll_regs[] = {
            0x230u, 0x231u, 0x232u, 0x233u, 0x234u, 0x235u,
            0x237u, 0x238u, 0x23au, 0x23bu, 0x23du, 0x245u,
            0x247u, 0x249u, 0x250u, 0x251u,
            0x270u, 0x271u, 0x272u, 0x273u, 0x274u, 0x275u,
            0x277u, 0x278u, 0x27au, 0x27bu, 0x27du, 0x285u,
            0x287u, 0x289u, 0x290u, 0x291u
        };
        dump_ad9361_regs(ctrl, "ad9361_clock_port_regs", clock_regs,
                         sizeof(clock_regs) / sizeof(clock_regs[0]));
        dump_ad9361_regs(ctrl, "ad9361_rfpll_regs", rfpll_regs,
                         sizeof(rfpll_regs) / sizeof(rfpll_regs[0]));

        dump_axi_core(ctrl, "axi_ad9361_rx", kAd9361RxBase);
        dump_axi_adc_channels(ctrl, kAd9361RxBase);
        dump_axi_core(ctrl, "axi_ad9361_tx", kAd9361TxBase);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_ad9361_status_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
