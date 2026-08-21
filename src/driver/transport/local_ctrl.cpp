//
// Created by jcc on 25-4-8.
//
#include <chrono>
#include <cstring>
#include <stdexcept>
#include "./local_ctrl.hpp"

namespace {
constexpr uint32_t kCtrlPacketBytes = 32;
constexpr uint32_t kCmdWriteReg = 0x0002;
constexpr uint32_t kCmdReadReg = 0x0003;
constexpr uint32_t kFlagRead = 0x01;
constexpr uint32_t kFlagWrite = 0x02;
}

local_ctrl::local_ctrl(zero_copy_if::sptr& xport, uint32_t sid)
        :_xport(xport)
        ,_sid(sid)
        ,_tx_seq(0)
        ,_rx_seq(0)
        ,_send_buf(new uint32_t[8])
        ,_recv_buf(new uint32_t[8]){
    set_tick_rate(1.0);
    time_spec_t time = time_spec_t(0.0);
    set_time(time);

}


local_ctrl::local_ctrl(zero_copy_if::sptr& xport, uint32_t sid, uint32_t buf_len)
        :_xport(xport)
        ,_sid(sid)
        ,_rx_seq(0)
        ,_tx_seq(0)
        ,_rx_buf_len(buf_len)
        ,_tx_buf_len(buf_len)
        ,_send_buf(new uint32_t[buf_len])
        ,_recv_buf(new uint32_t[buf_len]){
    set_tick_rate(1.0);
    time_spec_t time = time_spec_t(0.0);
    set_time(time);
    set_rx_buf_size(_rx_buf_len);
    tx_buf_resize(_tx_buf_len);
}

local_ctrl::~local_ctrl() {
    if (_send_buf) {
        delete [] _send_buf;
        _send_buf=nullptr;
    }

    if(_recv_buf){
        delete [] _recv_buf;
        _recv_buf=nullptr;
    }
}

void local_ctrl::poke32(uint32_t addr, uint32_t data) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    wait_for_ack(false);
}

void local_ctrl::poke32(uint32_t addr, uint32_t data, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    wait_for_ack(false, ack_timeout);
}

uint64_t local_ctrl::poke32_ack_value(uint32_t addr, uint32_t data, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    return wait_for_ack(true, ack_timeout);
}

void local_ctrl::poke64(uint32_t addr, uint64_t data) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    wait_for_ack(false);
}

void local_ctrl::poke64(uint32_t addr, uint64_t data, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    wait_for_ack(false, ack_timeout);
}

uint64_t local_ctrl::poke64_ack_value(uint32_t addr, uint64_t data, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(addr, data);
    return wait_for_ack(true, ack_timeout);
}

uint32_t local_ctrl::peek32(uint32_t addr) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(SR_CORE_READBACK, addr);
    return static_cast<uint32_t>(wait_for_ack(true));
}

uint32_t local_ctrl::peek32(uint32_t addr, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(SR_CORE_READBACK, addr);
    return static_cast<uint32_t>(wait_for_ack(true, ack_timeout));
}

uint64_t local_ctrl::peek64(uint32_t addr) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(SR_CORE_READBACK, addr);
    return wait_for_ack(true);
}

uint64_t local_ctrl::peek64(uint32_t addr, double ack_timeout) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    send_pkt64(SR_CORE_READBACK, addr);
    return wait_for_ack(true, ack_timeout);
}

void local_ctrl::serialize_hdr(uint32_t * buf, sdr_header_t &  hdr){
    buf[0] = (((uint32_t)hdr.sid) << 24) | (hdr.packet_len);
    buf[1] = (((uint32_t)hdr.magic_type) << 16) | (hdr.seq);
    buf[2] = hdr.timestamp & 0xFFFFFFFF;
    buf[3] = (hdr.timestamp >> 32) & 0xFFFFFFFF;
}

void local_ctrl::deserialize_hdr(uint32_t * buf, sdr_header_t &  hdr){
    
    hdr.sid = (buf[0] >> 24);
    hdr.packet_len = buf[0] & 0xFFFFFF;
    hdr.magic_type = buf[1] >> 16;
    hdr.seq = buf[1] & 0xFFFF;
    hdr.timestamp = (((uint64_t)buf[3]) << 32) | buf[2];
}


void local_ctrl::send_pkt(uint32_t addr, uint32_t data) {
    send_pkt64(addr, data);
}

void local_ctrl::send_pkt64(uint32_t addr, uint64_t data) {
    sdr_header_t packet_info;
    packet_info.magic_type = PACKET_TYPE_CTRL;
    packet_info.seq = _tx_seq;
    packet_info.sid = _sid;
    packet_info.packet_len = kCtrlPacketBytes;
    packet_info.timestamp = 0x00;

    serialize_hdr(_send_buf, packet_info);

    const bool read_back = (addr == SR_CORE_READBACK);
    _send_buf[4] = (read_back ? kCmdReadReg : kCmdWriteReg) |
                   ((read_back ? kFlagRead : kFlagWrite) << 16);
    _send_buf[5] = read_back ? static_cast<uint32_t>(data) : addr;
    _send_buf[6] = read_back ? 0u : static_cast<uint32_t>(data & 0xFFFFFFFFull);
    _send_buf[7] = read_back ? 0u : static_cast<uint32_t>((data >> 32) & 0xFFFFFFFFull);

    managed_send_buffer::sptr send_buffer = _xport->get_send_buff(1.0, kCtrlPacketBytes);
    if(send_buffer){
        // std::cout << "Buffer address: " << send_buffer->cast<void*>() << std::endl;
        std::memcpy(send_buffer->cast<void*>(),_send_buf,packet_info.packet_len);
        send_buffer->commit(packet_info.packet_len);
    }
    _tx_seq++;
}

uint64_t local_ctrl::wait_for_ack(bool read_back) {
    return wait_for_ack(read_back, 1.0);
}

uint64_t local_ctrl::wait_for_ack(bool read_back, double timeout) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    const uint16_t expected_seq = static_cast<uint16_t>(_tx_seq - 1u);

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            throw std::runtime_error("control response timeout");
        }

        const double remaining_timeout =
            std::chrono::duration<double>(deadline - now).count();
        if (remaining_timeout <= 0.0) {
            throw std::runtime_error("control response timeout");
        }

        sdr_header_t packet_info;
        managed_recv_buffer::sptr recv_buffer = _xport->get_recv_buff(remaining_timeout);
        if (!recv_buffer || recv_buffer->size() == 0) {
            throw std::runtime_error("control response timeout");
        }

        std::memcpy(_recv_buf, recv_buffer->cast<void*>(), recv_buffer->size());
        deserialize_hdr(_recv_buf, packet_info);

        if (packet_info.magic_type != PACKET_TYPE_RESP) {
            continue;
        }
        if (packet_info.seq != expected_seq) {
            continue;
        }
        if (packet_info.packet_len != kCtrlPacketBytes || recv_buffer->size() < kCtrlPacketBytes) {
            continue;
        }

        _timestamp = packet_info.timestamp;
        _rx_seq = packet_info.seq;
        if (read_back) {
            uint64_t lo = _recv_buf[5];
            uint64_t hi = _recv_buf[6];
            return ((hi << 32) | lo);
        }
        return 0;
    }
}

void local_ctrl::set_time(time_spec_t &time) {
    _time = time;
    auto time_zero = time_spec_t(0.0);time_spec_t(0.0);
    _has_tsf = !(_time == time_zero);
}

time_spec_t local_ctrl::get_time() {
    return _time;
}

void local_ctrl::set_tick_rate(const double rate) {
    _tick_rate = rate;
}

void local_ctrl::clear_seq() {
    _tx_seq = 0;
    _rx_seq = 0;
}

void local_ctrl::set_rx_buf_size(uint32_t len) {
    if(_rx_buf_len != len){
        _rx_buf_len = len;
        rx_buf_resize(len);
    }
}

void local_ctrl::set_tx_buf_size(uint32_t len) {
    if(_tx_buf_len != len){
        _tx_buf_len = len;
        tx_buf_resize(len);
    }
}

void local_ctrl::rx_buf_resize(uint32_t len) {
    delete [] _recv_buf;
    _recv_buf = new uint32_t[len];
}

void local_ctrl::tx_buf_resize(uint32_t len) {
    if (_send_buf) {
        delete [] _send_buf;
        _send_buf=nullptr;
    }
    _send_buf = new uint32_t[len];
}
