#ifndef SOAPY_M300_XDMA_CTRL_HPP
#define SOAPY_M300_XDMA_CTRL_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include "m300_xdma_protocol.hpp"
#include "include/sdr/core/xdma_zero_copy.hpp"

namespace sdr { namespace driver {

struct m300_ctrl_response
{
    m300_resp_packet pkt;
};

class m300_xdma_ctrl
{
public:
    typedef std::shared_ptr<m300_xdma_ctrl> sptr;

    m300_xdma_ctrl(const sdr::core::xdma_zero_copy::sptr& ctrl_xport,
                   const sdr::core::xdma_zero_copy::sptr& resp_xport);

    m300_ctrl_response send_command(uint16_t cmd_id,
                                    uint8_t flags,
                                    uint8_t target,
                                    uint32_t arg0,
                                    uint32_t arg1,
                                    uint32_t arg2,
                                    uint64_t timestamp,
                                    uint8_t sid,
                                    double timeout_sec);

    m300_ctrl_response nop(double timeout_sec = 1.0);
    m300_ctrl_response get_version(double timeout_sec = 1.0);
    m300_ctrl_response write_reg(uint32_t addr, uint32_t value, double timeout_sec = 1.0);
    m300_ctrl_response write_reg64(uint32_t addr, uint64_t value, double timeout_sec = 1.0);
    uint32_t read_reg(uint32_t addr, double timeout_sec = 1.0);
    uint64_t read_reg64(uint32_t addr, double timeout_sec = 1.0);
    m300_ctrl_response write_axi(uint32_t addr, uint32_t value, double timeout_sec = 1.0);
    uint32_t read_axi(uint32_t addr, double timeout_sec = 1.0);
    m300_ctrl_response ad9361_spi_write(uint16_t reg, uint8_t value, double timeout_sec = 1.0);
    m300_ctrl_response ad9361_spi_write_burst(uint16_t reg,
                                               const uint8_t* data,
                                               uint32_t count,
                                               double timeout_sec = 1.0);
    uint8_t ad9361_spi_read(uint16_t reg, double timeout_sec = 1.0);
    m300_ctrl_response write_gpio_out(uint32_t value, double timeout_sec = 1.0);
    uint32_t read_gpio_out(double timeout_sec = 1.0);
    m300_ctrl_response write_gpio_oe(uint32_t value, double timeout_sec = 1.0);
    uint32_t read_gpio_oe(double timeout_sec = 1.0);
    uint32_t read_gpio_in(double timeout_sec = 1.0);
    m300_ctrl_response set_rx_packet_bytes(uint32_t packet_bytes, double timeout_sec = 1.0);
    m300_ctrl_response start_rx(double timeout_sec = 1.0);
    m300_ctrl_response stop_rx(double timeout_sec = 1.0);
    m300_ctrl_response set_rx_sid(uint32_t sid, double timeout_sec = 1.0);
    m300_ctrl_response set_timestamp(uint64_t ts, double timeout_sec = 1.0);
    uint64_t get_timestamp(double timeout_sec = 1.0);
    // The current single-image layout reloads the only boot image at address 0.
    m300_ctrl_response multiboot(uint32_t address = 0u, double timeout_sec = 1.0);
    uint32_t multiboot_status(double timeout_sec = 1.0);
    m300_ctrl_response set_flash_update_mode(bool enable, double timeout_sec = 1.0);
    m300_ctrl_response configure_flash_erase(uint32_t address,
                                              uint32_t block_count,
                                              double timeout_sec = 1.0);
    m300_ctrl_response start_flash_erase(double timeout_sec = 1.0);
    m300_ctrl_response start_flash_program(uint32_t address,
                                           double timeout_sec = 1.0);
    m300_ctrl_response stop_flash_program(double timeout_sec = 1.0);
    m300_ctrl_response ack_flash_4k(double timeout_sec = 1.0);
    m300_ctrl_response start_flash_read(uint32_t address,
                                        uint32_t bytes,
                                        double timeout_sec = 1.0);
    uint32_t flash_status(double timeout_sec = 1.0);
    uint32_t flash_fifo_level(double timeout_sec = 1.0);
    m300_ctrl_response clear_counters(uint32_t mask = 0xffffffffu, double timeout_sec = 1.0);
    uint64_t get_counter(uint32_t counter_id, double timeout_sec = 1.0);

private:
    m300_ctrl_response wait_for_response(uint16_t expected_seq,
                                         uint8_t expected_sid,
                                         double timeout_sec);

private:
    sdr::core::xdma_zero_copy::sptr _ctrl_xport;
    sdr::core::xdma_zero_copy::sptr _resp_xport;
    uint16_t _seq = 0;
    std::mutex _command_mutex;
};

}} // namespace sdr::driver

#endif // SOAPY_M300_XDMA_CTRL_HPP
