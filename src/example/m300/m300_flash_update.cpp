#include "src/driver/M300/m300_xdma_impl.hpp"
#include "src/driver/M300/m300_xdma_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace sdr::driver;

namespace {

constexpr std::size_t kHandshakeBytes = 4096u;
constexpr std::size_t kEraseBlockBytes = 65536u;
constexpr uint8_t kFlashReadSid = 0xf1u;
constexpr auto kPollInterval = std::chrono::milliseconds(20);

struct Config
{
    std::string base = "/dev/xdma0";
    std::string bin_path;
    std::string readback_path;
    bool assume_yes = false;
    bool reboot = true;
    bool verify_only = false;
};

void usage(const char* program)
{
    std::cout
        << "Usage: " << program << " --bin system_top.bin [options]\n"
        << "Options:\n"
        << "  --base /dev/xdma0  XDMA device base (default /dev/xdma0)\n"
        << "  --readback PATH     Save Flash readback (default BIN.readback.bin)\n"
        << "  --verify-only       Read and verify the existing Flash without erasing\n"
        << "  --yes              Skip the destructive-operation prompt\n"
        << "  --no-reboot        Do not reload the image at Flash address 0\n"
        << "  --help              Show this help\n";
}

Config parse_args(int argc, char** argv)
{
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char* option) -> std::string {
            if (i + 1 >= argc)
                throw std::runtime_error(std::string("missing value for ") + option);
            return argv[++i];
        };

        if (arg == "--bin") {
            cfg.bin_path = value("--bin");
        } else if (arg == "--readback") {
            cfg.readback_path = value("--readback");
        } else if (arg == "--base") {
            cfg.base = value("--base");
        } else if (arg == "--yes") {
            cfg.assume_yes = true;
        } else if (arg == "--verify-only") {
            cfg.verify_only = true;
        } else if (arg == "--no-reboot") {
            cfg.reboot = false;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.bin_path.empty())
        throw std::runtime_error("--bin is required");
    if (cfg.readback_path.empty())
        cfg.readback_path = cfg.bin_path + ".readback.bin";
    if (cfg.verify_only)
        cfg.reboot = false;
    return cfg;
}

std::vector<uint8_t> read_bin(const std::string& path)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        throw std::runtime_error("cannot open BIN: " + path);

    const std::streamoff end = input.tellg();
    if (end <= 0)
        throw std::runtime_error("BIN is empty: " + path);
    if (static_cast<uint64_t>(end) >
        static_cast<uint64_t>(M300_FLASH_ONLINE_BYTES)) {
        throw std::runtime_error("BIN does not fit below the three-byte-address 16 MiB boundary");
    }

    std::vector<uint8_t> data(static_cast<std::size_t>(end));
    input.seekg(0, std::ios::beg);
    if (!input.read(reinterpret_cast<char*>(data.data()), end))
        throw std::runtime_error("failed to read BIN: " + path);
    return data;
}

uint32_t load_be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

void validate_single_image_bin(const std::vector<uint8_t>& data)
{
    const std::size_t header_bytes = std::min<std::size_t>(data.size(), 4096u);
    bool sync_found = false;
    bool wbstar_found = false;
    uint32_t wbstar = 0xffffffffu;

    for (std::size_t i = 0; i + 8u <= header_bytes; ++i) {
        if (data[i] == 0xaau && data[i + 1u] == 0x99u &&
            data[i + 2u] == 0x55u && data[i + 3u] == 0x66u) {
            sync_found = true;
        }
        if (data[i] == 0x30u && data[i + 1u] == 0x02u &&
            data[i + 2u] == 0x00u && data[i + 3u] == 0x01u) {
            wbstar = load_be32(data.data() + i + 4u);
            wbstar_found = true;
            break;
        }
    }

    if (!sync_found)
        throw std::runtime_error("BIN has no Xilinx sync word; use m300_golden system_top.bin");
    if (wbstar_found && wbstar != 0u) {
        throw std::runtime_error(
            "BIN WBSTAR is 0x" + [&] {
                std::ostringstream out;
                out << std::hex << std::setw(8) << std::setfill('0') << wbstar;
                return out.str();
            }() + "; refusing an image that redirects to another Flash slot");
    }
}

template <typename Predicate>
uint32_t wait_status(const std::shared_ptr<m300_xdma_ctrl>& ctrl,
                     Predicate done,
                     std::chrono::steady_clock::duration timeout,
                     const char* operation)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        const uint32_t status = ctrl->flash_status();
        if (done(status))
            return status;
        if (std::chrono::steady_clock::now() >= deadline) {
            std::ostringstream message;
            message << operation << " timed out: status=0x"
                    << std::hex << std::setw(8) << std::setfill('0') << status
                    << " last_rdsr=0x" << std::setw(2) << ((status >> 24) & 0xffu)
                    << " spi_state=" << std::dec << ((status >> 20) & 0x0fu)
                    << " controller_state=" << ((status >> 16) & 0x0fu);
            throw std::runtime_error(message.str());
        }
        std::this_thread::sleep_for(kPollInterval);
    }
}

void send_chunk(const sdr::core::xdma_zero_copy::sptr& tx,
                const uint8_t* data,
                std::size_t bytes)
{
    auto buffer = tx->get_send_buff(5.0, static_cast<uint32_t>(bytes));
    if (!buffer)
        throw std::runtime_error("timed out acquiring h2c_1 send buffer");
    if (buffer->size() < bytes)
        throw std::runtime_error("h2c_1 send buffer is smaller than the update chunk");

    std::memcpy(buffer->cast<void*>(), data, bytes);
    buffer->commit(bytes);
    buffer.reset(); // Releasing the managed buffer performs the XDMA write.
}

void write_bin(const std::string& path, const std::vector<uint8_t>& data)
{
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("cannot create Flash readback: " + path);
    if (!output.write(reinterpret_cast<const char*>(data.data()),
                      static_cast<std::streamsize>(data.size()))) {
        throw std::runtime_error("failed to write Flash readback: " + path);
    }
}

std::vector<uint8_t> readback_flash(
    const std::shared_ptr<m300_xdma_ctrl>& ctrl,
    const sdr::core::xdma_zero_copy::sptr& rx,
    std::size_t bytes)
{
    if (!rx)
        throw std::runtime_error("M300 c2h_1 transport is unavailable");

    // Stop and restart the driver ring so packets left by an earlier IQ
    // receive session cannot be mistaken for Flash data.
    rx->stop_recv();
    if (!rx->start_recv(1.0))
        throw std::runtime_error("failed to start the c2h_1 Flash readback ring");

    std::vector<uint8_t> data;
    data.reserve(bytes);
    unsigned last_percent = 0u;

    std::cout << "Reading:       0%" << std::flush;

    try {
        while (data.size() < bytes) {
            // A long 03h transaction cannot be paused when the host C2H path
            // is briefly descheduled.  Limit each transaction to 4 KiB so
            // the complete response always fits in the FPGA's 8 KiB FIFO.
            const std::size_t command_offset = data.size();
            const std::size_t command_bytes =
                std::min<std::size_t>(kHandshakeBytes, bytes - command_offset);
            ctrl->start_flash_read(
                M300_FLASH_IMAGE_ADDR + static_cast<uint32_t>(command_offset),
                static_cast<uint32_t>(command_bytes));

            auto buffer = rx->get_recv_buff(5.0);
            if (!buffer) {
                const uint32_t status = ctrl->flash_status();
                if ((status & M300_FLASH_STATUS_READ_OVERFLOW) != 0u)
                    throw std::runtime_error("Flash readback FIFO overflowed");

                std::ostringstream message;
                message << "Flash readback packet timed out: received="
                        << data.size() << "/" << bytes << " status=0x"
                        << std::hex << std::setw(8) << std::setfill('0')
                        << status;
                throw std::runtime_error(message.str());
            }

            if (buffer->size() < M300_HDR_BYTES)
                throw std::runtime_error("Flash readback packet is shorter than its header");

            const auto* packet =
                static_cast<const uint8_t*>(buffer->cast<const void*>());
            const m300_header header = parse_header(packet);
            const std::size_t expected_packet = M300_HDR_BYTES + command_bytes;

            if (header.magic_type != M300_MAGIC_RX ||
                header.sid != kFlashReadSid ||
                header.seq != 0u ||
                header.length != expected_packet ||
                buffer->size() != expected_packet) {
                std::ostringstream message;
                message << "invalid Flash readback packet: magic=0x"
                        << std::hex << std::setw(4) << std::setfill('0')
                        << header.magic_type << " sid=0x" << std::setw(2)
                        << static_cast<unsigned>(header.sid) << std::dec
                        << " seq=" << header.seq << " expected_seq=0"
                        << " length=" << header.length
                        << " buffer=" << buffer->size()
                        << " expected_length=" << expected_packet;
                throw std::runtime_error(message.str());
            }

            const uint64_t flash_address = load_le64(packet + 8u);
            const uint64_t expected_address =
                static_cast<uint64_t>(M300_FLASH_IMAGE_ADDR) + data.size();
            if (flash_address != expected_address) {
                std::ostringstream message;
                message << "Flash readback address mismatch: got=0x"
                        << std::hex << std::setw(8) << std::setfill('0')
                        << flash_address << " expected=0x" << std::setw(8)
                        << expected_address;
                throw std::runtime_error(message.str());
            }

            data.insert(data.end(), packet + M300_HDR_BYTES,
                        packet + expected_packet);

            const uint32_t status = wait_status(
                ctrl,
                [](uint32_t value) {
                    return (value & M300_FLASH_STATUS_READ_DONE) != 0u &&
                           (value & M300_FLASH_STATUS_BUSY) == 0u;
                },
                std::chrono::seconds(5), "4 KiB Flash readback");
            if ((status & M300_FLASH_STATUS_READ_OVERFLOW) != 0u)
                throw std::runtime_error("Flash readback FIFO overflowed");

            const unsigned percent = static_cast<unsigned>(
                static_cast<uint64_t>(data.size()) * 100u / bytes);
            if (percent != last_percent) {
                std::cout << "\rReading:     " << std::setw(3) << percent << "%"
                          << std::flush;
                last_percent = percent;
            }
        }

    } catch (...) {
        rx->stop_recv();
        throw;
    }

    rx->stop_recv();
    std::cout << "\rReading:     100%\n";
    return data;
}

void verify_readback(const std::vector<uint8_t>& expected,
                     const std::vector<uint8_t>& actual)
{
    if (actual.size() != expected.size()) {
        throw std::runtime_error("Flash readback size mismatch: expected " +
                                 std::to_string(expected.size()) + ", got " +
                                 std::to_string(actual.size()));
    }

    std::cout << "Verifying:     0%" << std::flush;
    unsigned last_percent = 0u;
    for (std::size_t offset = 0; offset < expected.size(); ++offset) {
        if (actual[offset] != expected[offset]) {
            std::ostringstream message;
            message << "Flash verification failed at offset 0x" << std::hex
                    << std::setw(8) << std::setfill('0') << offset
                    << ": expected=0x" << std::setw(2)
                    << static_cast<unsigned>(expected[offset])
                    << " actual=0x" << std::setw(2)
                    << static_cast<unsigned>(actual[offset]);
            throw std::runtime_error(message.str());
        }

        const unsigned percent = static_cast<unsigned>(
            static_cast<uint64_t>(offset + 1u) * 100u / expected.size());
        if (percent != last_percent) {
            std::cout << "\rVerifying:   " << std::setw(3) << percent << "%"
                      << std::flush;
            last_percent = percent;
        }
    }
    std::cout << "\rVerifying:   100%\n";
}

void save_and_verify(const Config& cfg,
                     const std::shared_ptr<m300_xdma_ctrl>& ctrl,
                     const sdr::core::xdma_zero_copy::sptr& rx,
                     const std::vector<uint8_t>& image)
{
    const std::vector<uint8_t> readback =
        readback_flash(ctrl, rx, image.size());
    write_bin(cfg.readback_path, readback);
    std::cout << "Readback saved: " << cfg.readback_path << "\n";
    verify_readback(image, readback);
    std::cout << "Flash verification passed: " << image.size()
              << " bytes match byte-for-byte.\n";
}

void confirm_program(const Config& cfg,
                    std::size_t file_bytes,
                    std::size_t transfer_bytes,
                    uint32_t erase_blocks)
{
    std::cout << "M300 single-image Flash programming plan:\n"
              << "  BIN:          " << cfg.bin_path << "\n"
              << "  file bytes:   " << file_bytes << "\n"
              << "  Flash bytes:  " << transfer_bytes
              << " (0xFF padded to 4 KiB)\n"
              << "  Flash start:  0x" << std::hex << std::setw(8)
              << std::setfill('0') << M300_FLASH_IMAGE_ADDR << std::dec
              << std::setfill(' ') << "\n"
              << "  erase blocks: " << erase_blocks << " x 64 KiB\n"
              << "  readback:     " << cfg.readback_path << "\n"
              << "  verify:       byte-for-byte\n"
              << "  reboot:       " << (cfg.reboot ? "Flash address 0" : "disabled") << "\n"
              << "WARNING: this erases the only boot image; there is no fallback image.\n";

    if (cfg.assume_yes)
        return;

    std::cout << "Type PROGRAM to erase and replace the boot image: " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "PROGRAM")
        throw std::runtime_error("programming cancelled");
}

void run_program(const Config& cfg, const std::vector<uint8_t>& image)
{
    const std::size_t transfer_bytes =
        ((image.size() + kHandshakeBytes - 1u) / kHandshakeBytes) *
        kHandshakeBytes;
    const std::size_t image_region_bytes =
        static_cast<std::size_t>(M300_FLASH_ONLINE_BYTES);
    if (transfer_bytes > image_region_bytes)
        throw std::runtime_error("0xFF-padded BIN does not fit below the 16 MiB boundary");

    std::vector<uint8_t> transfer_image = image;
    transfer_image.resize(transfer_bytes, 0xffu);

    const uint32_t erase_blocks = static_cast<uint32_t>(
        (transfer_image.size() + kEraseBlockBytes - 1u) / kEraseBlockBytes);
    if (!cfg.verify_only) {
        confirm_program(cfg, image.size(), transfer_image.size(), erase_blocks);
    } else {
        std::cout << "M300 Flash readback-only plan:\n"
                  << "  BIN:          " << cfg.bin_path << "\n"
                  << "  bytes:        " << image.size() << "\n"
                  << "  Flash start:  0x00000000\n"
                  << "  readback:     " << cfg.readback_path << "\n"
                  << "  verify:       byte-for-byte\n"
                  << "  erase/write:  disabled\n"
                  << "  reboot:       disabled\n";
    }

    auto device = std::make_shared<M300XdmaImpl>(cfg.base, true);
    if (!device->isInitialSuccess())
        throw std::runtime_error("M300 device initialization failed: " + device->last_error());

    const auto ctrl = device->get_ctrl();
    const auto tx = device->get_tx_xport();
    const auto rx = device->get_rx_xport();
    if (!cfg.verify_only && !tx)
        throw std::runtime_error("M300 h2c_1 transport is unavailable");
    if (!rx)
        throw std::runtime_error("M300 c2h_1 transport is unavailable");

    bool update_mode = false;
    bool programming = false;
    try {
        (void)ctrl->get_version();
        // Prevent normal IQ packets from entering c2h_1 while the driver ring
        // is prepared for Flash readback.
        ctrl->stop_rx();
        ctrl->set_flash_update_mode(true);
        update_mode = true;

        const uint32_t mode_status = ctrl->flash_status();
        if ((mode_status & M300_FLASH_STATUS_MODE) == 0u)
            throw std::runtime_error("FPGA did not enter Flash update mode");

        if ((mode_status & M300_FLASH_STATUS_BUSY) != 0u) {
            std::cout << "Waiting for an earlier Flash operation to become idle..."
                      << std::flush;
            wait_status(ctrl,
                        [](uint32_t status) {
                            return (status & M300_FLASH_STATUS_BUSY) == 0u;
                        },
                        std::chrono::minutes(10), "existing Flash operation");
            std::cout << " done\n";
        }

        if (cfg.verify_only) {
            save_and_verify(cfg, ctrl, rx, image);
            ctrl->set_flash_update_mode(false);
            update_mode = false;
            return;
        }

        std::cout << "Erasing:     0% (0/" << erase_blocks << " blocks)" << std::flush;
        for (uint32_t block = 0; block < erase_blocks; ++block) {
            const uint32_t block_addr =
                M300_FLASH_IMAGE_ADDR + block * static_cast<uint32_t>(kEraseBlockBytes);
            ctrl->configure_flash_erase(block_addr, 1u);
            ctrl->start_flash_erase();

            // Wait for the new command to leave IDLE before accepting the
            // erase-done bit.  That bit remains high from the preceding
            // block until the next command has actually started.
            wait_status(ctrl,
                        [](uint32_t status) {
                            return (status & M300_FLASH_STATUS_BUSY) != 0u;
                        },
                        std::chrono::seconds(5), "64 KiB block erase start");
            wait_status(ctrl,
                        [](uint32_t status) {
                            return (status & M300_FLASH_STATUS_ERASE_DONE) != 0u &&
                                   (status & M300_FLASH_STATUS_BUSY) == 0u;
                        },
                        std::chrono::minutes(2), "64 KiB block erase");

            const unsigned percent = static_cast<unsigned>(
                (static_cast<uint64_t>(block + 1u) * 100u) / erase_blocks);
            std::cout << "\rErasing:   " << std::setw(3) << percent << "% ("
                      << (block + 1u) << "/" << erase_blocks << " blocks)"
                      << std::flush;
        }
        std::cout << "\n";

        ctrl->start_flash_program(M300_FLASH_IMAGE_ADDR);
        programming = true;

        std::size_t sent = 0;
        unsigned last_percent = 0;
        while (sent < transfer_image.size()) {
            const std::size_t bytes = kHandshakeBytes;
            send_chunk(tx, transfer_image.data() + sent, bytes);
            sent += bytes;

            wait_status(ctrl,
                        [](uint32_t status) {
                            return (status & M300_FLASH_STATUS_4K_DONE) != 0u;
                        },
                        std::chrono::seconds(30), "4 KiB page programming");
            ctrl->ack_flash_4k();

            const unsigned percent = std::min(
                99u,
                static_cast<unsigned>(sent * 100u / transfer_image.size()));
            if (percent != last_percent) {
                std::cout << "\rProgramming: " << std::setw(3) << percent << "%" << std::flush;
                last_percent = percent;
            }
        }

        const auto fifo_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        uint32_t fifo_level = ctrl->flash_fifo_level();
        while (fifo_level != 0u) {
            if (std::chrono::steady_clock::now() >= fifo_deadline) {
                const uint32_t status = ctrl->flash_status();
                std::ostringstream message;
                message << "Flash FIFO drain timed out: fifo_level=" << fifo_level
                        << " status=0x" << std::hex << std::setw(8)
                        << std::setfill('0') << status;
                throw std::runtime_error(message.str());
            }
            std::this_thread::sleep_for(kPollInterval);
            fifo_level = ctrl->flash_fifo_level();
        }

        ctrl->stop_flash_program();
        wait_status(ctrl,
                    [](uint32_t status) {
                        return (status & M300_FLASH_STATUS_BUSY) == 0u;
                    },
                    std::chrono::seconds(30), "final page programming");
        programming = false;
        std::cout << "\rProgramming: 100%\nBoot BIN written; starting Flash readback.\n";

        save_and_verify(cfg, ctrl, rx, image);

        ctrl->set_flash_update_mode(false);
        update_mode = false;

        if (cfg.reboot) {
            std::cout << "Reloading the image at Flash address 0; PCIe will disconnect...\n";
            ctrl->multiboot(0u);
        }
    } catch (...) {
        if (programming) {
            try {
                ctrl->stop_flash_program();
            } catch (...) {
            }
        }
        if (update_mode) {
            try {
                ctrl->set_flash_update_mode(false);
            } catch (...) {
            }
        }
        throw;
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const Config cfg = parse_args(argc, argv);
        const std::vector<uint8_t> image = read_bin(cfg.bin_path);
        validate_single_image_bin(image);
        run_program(cfg, image);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_flash_update: " << ex.what() << "\n";
        return 1;
    }
}
