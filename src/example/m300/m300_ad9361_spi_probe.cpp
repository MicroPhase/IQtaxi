#include "src/driver/M300/m300_xdma_ctrl.hpp"
#include "include/sdr/core/xdma_zero_copy.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using sdr::core::xdma_zero_copy;
using sdr::core::xdma_zero_copy_params;
using sdr::core::zero_copy_xport_params;
using sdr::driver::m300_xdma_ctrl;

namespace {
constexpr uint16_t kAd9361RegProductId = 0x037u;

std::string channel_path(const std::string& base, const std::string& suffix)
{
    return base.empty() ? std::string("/dev/xdma0") + suffix : base + suffix;
}

uint32_t parse_u32(const std::string& text)
{
    return static_cast<uint32_t>(std::stoul(text, nullptr, 0));
}

void validate_gpio_number(int number)
{
    if (number < 0 || number > 31) {
        throw std::out_of_range("GPIO number must be 0..31");
    }
}

std::shared_ptr<m300_xdma_ctrl> make_ctrl(const std::string& base)
{
    zero_copy_xport_params params;
    params.num_recv_frames = 16;
    params.num_send_frames = 16;
    params.recv_frame_size = 32;
    params.send_frame_size = 32;
    params.recv_buff_size = params.num_recv_frames * params.recv_frame_size;
    params.send_buff_size = params.num_send_frames * params.send_frame_size;

    const auto ctrl_xport = xdma_zero_copy::make(
        xdma_zero_copy_params{channel_path(base, "_h2c_0"), std::string(),
                              channel_path(base, "_h2c_0"), 32u, 0u, false, false},
        params);
    const auto resp_xport = xdma_zero_copy::make(
        xdma_zero_copy_params{channel_path(base, "_c2h_0"), channel_path(base, "_c2h_0"),
                              std::string(), 32u, 0u, false, false},
        params);
    return std::make_shared<m300_xdma_ctrl>(ctrl_xport, resp_xport);
}

void usage()
{
    std::cout
        << "Usage: m300_ad9361_spi_probe [--base /dev/xdma0]\n"
        << "                             [--read-reg REG]\n"
        << "                             [--write-reg REG --value VALUE]\n"
        << "                             [--gpio-read]\n"
        << "                             [--gpio-dir NUMBER 0|1]\n"
        << "                             [--gpio-set NUMBER 0|1]\n";
}
}

int main(int argc, char** argv)
{
    try {
        std::string base = "/dev/xdma0";
        bool do_read = false;
        bool do_write = false;
        bool gpio_read = false;
        uint16_t reg = kAd9361RegProductId;
        uint8_t value = 0;
        int gpio_dir_number = -1;
        int gpio_set_number = -1;
        uint32_t gpio_dir_value = 0;
        uint32_t gpio_set_value = 0;

        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--base" && i + 1 < argc) {
                base = argv[++i];
            } else if (arg == "--read-reg" && i + 1 < argc) {
                reg = static_cast<uint16_t>(parse_u32(argv[++i]));
                do_read = true;
            } else if (arg == "--write-reg" && i + 1 < argc) {
                reg = static_cast<uint16_t>(parse_u32(argv[++i]));
                do_write = true;
            } else if (arg == "--value" && i + 1 < argc) {
                value = static_cast<uint8_t>(parse_u32(argv[++i]));
            } else if (arg == "--gpio-read") {
                gpio_read = true;
            } else if (arg == "--gpio-dir" && i + 2 < argc) {
                gpio_dir_number = static_cast<int>(parse_u32(argv[++i]));
                gpio_dir_value = parse_u32(argv[++i]) ? 1u : 0u;
            } else if (arg == "--gpio-set" && i + 2 < argc) {
                gpio_set_number = static_cast<int>(parse_u32(argv[++i]));
                gpio_set_value = parse_u32(argv[++i]) ? 1u : 0u;
            } else if (arg == "--help" || arg == "-h") {
                usage();
                return 0;
            } else {
                usage();
                return 1;
            }
        }

        const auto ctrl = make_ctrl(base);
        (void)ctrl->get_version(1.0);

        if (gpio_dir_number >= 0) {
            validate_gpio_number(gpio_dir_number);
            uint32_t gpio_oe = ctrl->read_gpio_oe(1.0);
            const uint32_t mask = 1u << static_cast<uint32_t>(gpio_dir_number);
            gpio_oe = gpio_dir_value ? (gpio_oe | mask) : (gpio_oe & ~mask);
            (void)ctrl->write_gpio_oe(gpio_oe, 1.0);
        }

        if (gpio_set_number >= 0) {
            validate_gpio_number(gpio_set_number);
            uint32_t gpio_out = ctrl->read_gpio_out(1.0);
            const uint32_t mask = 1u << static_cast<uint32_t>(gpio_set_number);
            gpio_out = gpio_set_value ? (gpio_out | mask) : (gpio_out & ~mask);
            (void)ctrl->write_gpio_out(gpio_out, 1.0);
        }

        if (gpio_read || gpio_dir_number >= 0 || gpio_set_number >= 0) {
            const uint32_t gpio_out = ctrl->read_gpio_out(1.0);
            const uint32_t gpio_oe = ctrl->read_gpio_oe(1.0);
            const uint32_t gpio_in = ctrl->read_gpio_in(1.0);
            std::cout << "gpio_out=0x" << std::hex << std::setw(8) << std::setfill('0')
                      << gpio_out << " gpio_oe=0x" << std::setw(8) << gpio_oe
                      << " gpio_in=0x" << std::setw(8) << gpio_in << std::dec << "\n";
        }

        if (do_write) {
            (void)ctrl->ad9361_spi_write(reg, value, 1.0);
            std::cout << "write AD9361[0x" << std::hex << std::setw(3)
                      << std::setfill('0') << (reg & 0x03ffu)
                      << "] = 0x" << std::setw(2)
                      << static_cast<uint32_t>(value) << std::dec << "\n";
        }

        const uint8_t read_value = ctrl->ad9361_spi_read(reg, 1.0);
        std::cout << "read AD9361[0x" << std::hex << std::setw(3)
                  << std::setfill('0') << (reg & 0x03ffu)
                  << "] = 0x" << std::setw(2)
                  << static_cast<uint32_t>(read_value) << std::dec << "\n";

        if (!do_read && !do_write && !gpio_read &&
            gpio_dir_number < 0 && gpio_set_number < 0) {
            std::cout << "default probe: product-id register 0x037\n";
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_ad9361_spi_probe_error: " << ex.what() << "\n";
        return 1;
    }
}
