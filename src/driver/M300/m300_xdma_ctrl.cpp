#include "m300_xdma_ctrl.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>

using namespace sdr::driver;
using namespace sdr::core;

namespace {
constexpr double kDefaultTimeoutSec = 1.0;
}

m300_xdma_ctrl::m300_xdma_ctrl(const xdma_zero_copy::sptr& ctrl_xport,
                               const xdma_zero_copy::sptr& resp_xport)
    : _ctrl_xport(ctrl_xport)
    , _resp_xport(resp_xport)
{
    if (!_ctrl_xport || !_resp_xport) {
        throw std::runtime_error("m300_xdma_ctrl requires valid ctrl/resp transports");
    }
}

m300_ctrl_response m300_xdma_ctrl::send_command(uint16_t cmd_id,
                                                uint8_t flags,
                                                uint8_t target,
                                                uint32_t arg0,
                                                uint32_t arg1,
                                                uint32_t arg2,
                                                uint64_t timestamp,
                                                uint8_t sid,
                                                double timeout_sec)
{
    std::lock_guard<std::mutex> command_lock(_command_mutex);
    managed_send_buffer::sptr send_buff = _ctrl_xport->get_send_buff(timeout_sec, M300_CTRL_BYTES);
    if (!send_buff) {
        throw std::runtime_error("failed to allocate M300 CTRL send buffer");
    }

    uint8_t* p = static_cast<uint8_t*>(send_buff->cast<void*>());
    m300_ctrl_packet pkt;
    pkt.hdr.magic_type = M300_MAGIC_CTRL;
    pkt.hdr.seq = _seq;
    pkt.hdr.sid = sid;
    pkt.hdr.length = M300_CTRL_BYTES;
    pkt.timestamp = timestamp;
    pkt.cmd_id = cmd_id;
    pkt.flags = flags;
    pkt.target = target;
    pkt.arg0 = arg0;
    pkt.arg1 = arg1;
    pkt.arg2 = arg2;
    write_ctrl_packet(p, pkt);
    send_buff->commit(M300_CTRL_BYTES);
    send_buff.reset();

    m300_ctrl_response resp = wait_for_response(_seq, sid, timeout_sec);
    _seq = static_cast<uint16_t>(_seq + 1u);
    return resp;
}

m300_ctrl_response m300_xdma_ctrl::wait_for_response(uint16_t expected_seq,
                                                     uint8_t expected_sid,
                                                     double timeout_sec)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(std::max(timeout_sec, kDefaultTimeoutSec));

    for (;;) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            throw std::runtime_error("M300 CTRL response timeout");
        }

        const double remaining = std::chrono::duration<double>(deadline - now).count();
        managed_recv_buffer::sptr recv_buff = _resp_xport->get_recv_buff(remaining);
        if (!recv_buff) {
            throw std::runtime_error("M300 CTRL response timeout");
        }
        if (recv_buff->size() < M300_RESP_BYTES) {
            continue;
        }

        m300_ctrl_response response;
        response.pkt = parse_resp_packet(static_cast<const uint8_t*>(recv_buff->cast<const void*>()));
        if (response.pkt.hdr.magic_type != M300_MAGIC_RESP) {
            continue;
        }
        if (response.pkt.hdr.length != M300_RESP_BYTES) {
            continue;
        }
        if (response.pkt.hdr.seq != expected_seq || response.pkt.hdr.sid != expected_sid) {
            continue;
        }
        if (response.pkt.status != M300_STATUS_OK) {
            throw std::runtime_error("M300 CTRL command failed, status=" +
                                     std::to_string(response.pkt.status));
        }
        return response;
    }
}

m300_ctrl_response m300_xdma_ctrl::nop(double timeout_sec)
{
    return send_command(M300_CMD_NOP, 0x00, M300_TARGET_LOCAL, 0, 0, 0, 0, 0, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::get_version(double timeout_sec)
{
    return send_command(M300_CMD_GET_VERSION, 0x00, M300_TARGET_LOCAL, 0, 0, 0, 0, 0, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::write_reg(uint32_t addr, uint32_t value, double timeout_sec)
{
    return send_command(M300_CMD_WRITE_REG, 0x02, M300_TARGET_LOCAL, addr, value, 0, 0, 0, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::write_reg64(uint32_t addr, uint64_t value, double timeout_sec)
{
    return send_command(M300_CMD_WRITE_REG, 0x02, M300_TARGET_LOCAL, addr,
                        static_cast<uint32_t>(value),
                        static_cast<uint32_t>(value >> 32),
                        0, 0, timeout_sec);
}

uint32_t m300_xdma_ctrl::read_reg(uint32_t addr, double timeout_sec)
{
    const m300_ctrl_response resp =
        send_command(M300_CMD_READ_REG, 0x01, M300_TARGET_LOCAL, addr, 0, 0, 0, 0, timeout_sec);
    return resp.pkt.value0;
}

uint64_t m300_xdma_ctrl::read_reg64(uint32_t addr, double timeout_sec)
{
    const m300_ctrl_response resp =
        send_command(M300_CMD_READ_REG, 0x01, M300_TARGET_LOCAL, addr, 0, 0, 0, 0, timeout_sec);
    return (static_cast<uint64_t>(resp.pkt.value1) << 32) | resp.pkt.value0;
}

m300_ctrl_response m300_xdma_ctrl::write_axi(uint32_t addr, uint32_t value, double timeout_sec)
{
    return send_command(M300_CMD_WRITE_REG, 0x02, M300_TARGET_AXI_LITE,
                        addr, value, 0, 0, 0, timeout_sec);
}

uint32_t m300_xdma_ctrl::read_axi(uint32_t addr, double timeout_sec)
{
    const m300_ctrl_response resp =
        send_command(M300_CMD_READ_REG, 0x01, M300_TARGET_AXI_LITE,
                     addr, 0, 0, 0, 0, timeout_sec);
    return resp.pkt.value0;
}

m300_ctrl_response m300_xdma_ctrl::ad9361_spi_write(uint16_t reg, uint8_t value, double timeout_sec)
{
    return send_command(M300_CMD_WRITE_REG, 0x02, M300_TARGET_AD9361_SPI,
                        static_cast<uint32_t>(reg & 0x03ffu),
                        static_cast<uint32_t>(value),
                        0, 0, 0, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::ad9361_spi_write_burst(uint16_t reg,
                                                          const uint8_t* data,
                                                          uint32_t count,
                                                          double timeout_sec)
{
    if (!data || count == 0u || count > 8u) {
        throw std::runtime_error("AD9361 SPI burst count must be 1..8");
    }

    uint32_t arg1 = 0;
    uint32_t arg2 = 0;
    for (uint32_t i = 0; i < count; ++i) {
        if (i < 4u) {
            arg1 |= static_cast<uint32_t>(data[i]) << (8u * i);
        } else {
            arg2 |= static_cast<uint32_t>(data[i]) << (8u * (i - 4u));
        }
    }

    return send_command(M300_CMD_WRITE_REG,
                        static_cast<uint8_t>(0x10u | (count & 0x0fu)),
                        M300_TARGET_AD9361_SPI,
                        static_cast<uint32_t>(reg & 0x03ffu),
                        arg1,
                        arg2,
                        0, 0, timeout_sec);
}

uint8_t m300_xdma_ctrl::ad9361_spi_read(uint16_t reg, double timeout_sec)
{
    const m300_ctrl_response resp =
        send_command(M300_CMD_READ_REG, 0x01, M300_TARGET_AD9361_SPI,
                     static_cast<uint32_t>(reg & 0x03ffu), 0, 0, 0, 0, timeout_sec);
    return static_cast<uint8_t>(resp.pkt.value0 & 0xffu);
}

m300_ctrl_response m300_xdma_ctrl::write_gpio_out(uint32_t value, double timeout_sec)
{
    return write_reg(M300_REG_GPIO_OUT, value, timeout_sec);
}

uint32_t m300_xdma_ctrl::read_gpio_out(double timeout_sec)
{
    return read_reg(M300_REG_GPIO_OUT, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::write_gpio_oe(uint32_t value, double timeout_sec)
{
    return write_reg(M300_REG_GPIO_OE, value, timeout_sec);
}

uint32_t m300_xdma_ctrl::read_gpio_oe(double timeout_sec)
{
    return read_reg(M300_REG_GPIO_OE, timeout_sec);
}

uint32_t m300_xdma_ctrl::read_gpio_in(double timeout_sec)
{
    return read_reg(M300_REG_GPIO_IN, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::set_rx_packet_bytes(uint32_t packet_bytes, double timeout_sec)
{
    return write_reg(M300_REG_RX_PACKET_BYTES, packet_bytes, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::start_rx(double timeout_sec)
{
    return write_reg(M300_REG_STREAM_ENABLE, 1u, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::stop_rx(double timeout_sec)
{
    return write_reg(M300_REG_STREAM_ENABLE, 0u, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::set_rx_sid(uint32_t sid, double timeout_sec)
{
    if (sid != 0u) {
        throw std::runtime_error("M300 RX SID is fixed to 0 by the current FPGA control registers");
    }
    return send_command(M300_CMD_GET_STATUS, 0x01, M300_TARGET_LOCAL, 0, 0, 0, 0, 0, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::set_timestamp(uint64_t ts, double timeout_sec)
{
    return write_reg64(M300_REG_TIMESTAMP, ts, timeout_sec);
}

uint64_t m300_xdma_ctrl::get_timestamp(double timeout_sec)
{
    return read_reg64(M300_REG_TIMESTAMP, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::multiboot(uint32_t address, double timeout_sec)
{
    if ((address & 0xffu) != 0u || address >= M300_FLASH_BYTES) {
        throw std::runtime_error("M300 MultiBoot address must be 256-byte aligned and below 32 MiB");
    }

    (void)write_reg(M300_REG_MULTIBOOT_ADDR, address, timeout_sec);
    return write_reg(M300_REG_MULTIBOOT_CTRL, M300_MULTIBOOT_UNLOCK, timeout_sec);
}

uint32_t m300_xdma_ctrl::multiboot_status(double timeout_sec)
{
    return read_reg(M300_REG_MULTIBOOT_STATUS, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::set_flash_update_mode(bool enable, double timeout_sec)
{
    return write_reg(M300_REG_FLASH_MODE,
                     enable ? M300_FLASH_MODE_ENABLE : 0u,
                     timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::configure_flash_erase(uint32_t address,
                                                          uint32_t block_count,
                                                          double timeout_sec)
{
    const uint64_t erase_bytes = static_cast<uint64_t>(block_count) << 16;
    if (block_count == 0u || (address & 0xffffu) != 0u ||
        static_cast<uint64_t>(address) + erase_bytes > M300_FLASH_ONLINE_BYTES) {
        throw std::runtime_error("M300 Flash erase range must be 64 KiB aligned and remain below 16 MiB");
    }

    (void)write_reg(M300_REG_FLASH_ERASE_ADDR, address, timeout_sec);
    return write_reg(M300_REG_FLASH_ERASE_COUNT, block_count, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::start_flash_erase(double timeout_sec)
{
    return write_reg(M300_REG_FLASH_COMMAND, M300_FLASH_CMD_ERASE, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::start_flash_program(uint32_t address, double timeout_sec)
{
    if ((address & 0xffu) != 0u ||
        address > M300_FLASH_ONLINE_BYTES - 256u) {
        throw std::runtime_error("M300 Flash program address must be 256-byte aligned and remain below 16 MiB");
    }

    (void)write_reg(M300_REG_FLASH_PROGRAM_ADDR, address, timeout_sec);
    return write_reg(M300_REG_FLASH_COMMAND, M300_FLASH_CMD_PROGRAM, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::stop_flash_program(double timeout_sec)
{
    return write_reg(M300_REG_FLASH_COMMAND, M300_FLASH_CMD_STOP, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::ack_flash_4k(double timeout_sec)
{
    return write_reg(M300_REG_FLASH_COMMAND, M300_FLASH_CMD_ACK4K, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::start_flash_read(uint32_t address,
                                                     uint32_t bytes,
                                                     double timeout_sec)
{
    if (bytes == 0u || static_cast<uint64_t>(address) + bytes >
                           M300_FLASH_ONLINE_BYTES) {
        throw std::runtime_error("M300 Flash read range must remain below 16 MiB");
    }

    (void)write_reg(M300_REG_FLASH_READ_ADDR, address, timeout_sec);
    (void)write_reg(M300_REG_FLASH_READ_BYTES, bytes, timeout_sec);
    return write_reg(M300_REG_FLASH_COMMAND, M300_FLASH_CMD_READ, timeout_sec);
}

uint32_t m300_xdma_ctrl::flash_status(double timeout_sec)
{
    return read_reg(M300_REG_FLASH_STATUS, timeout_sec);
}

uint32_t m300_xdma_ctrl::flash_fifo_level(double timeout_sec)
{
    return read_reg(M300_REG_FLASH_FIFO_LEVEL, timeout_sec);
}

m300_ctrl_response m300_xdma_ctrl::clear_counters(uint32_t mask, double timeout_sec)
{
    return send_command(M300_CMD_CLEAR_COUNTERS, 0x02, M300_TARGET_LOCAL, mask, 0, 0, 0, 0, timeout_sec);
}

uint64_t m300_xdma_ctrl::get_counter(uint32_t counter_id, double timeout_sec)
{
    const m300_ctrl_response resp =
        send_command(M300_CMD_GET_COUNTER, 0x01, M300_TARGET_LOCAL, counter_id, 0, 0, 0, 0, timeout_sec);
    return (static_cast<uint64_t>(resp.pkt.value1) << 32) | resp.pkt.value0;
}
