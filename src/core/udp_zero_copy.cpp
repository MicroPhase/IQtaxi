//
// Created by jcc on 25-4-8.
//
#include <stdexcept>
#include <cstring>
#include "include/sdr/core/udp_zero_copy.hpp"
#include "include/sdr/core/udp_common.hpp"
#ifndef _WIN32
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

using namespace sdr;
using namespace sdr::core;

class udp_zero_copy_asio_mrb : public managed_recv_buffer
{
public:
    udp_zero_copy_asio_mrb(void* mem, sdr_socket_t sock_fd, const size_t frame_size)
            : _mem(mem), _sock_fd(sock_fd), _frame_size(frame_size), _len(0)
    { /*NOP*/
    }

    void release(void) override
    {
        _claimer.release();
    }

    inline sptr get_new(const double timeout, size_t& index)
    {
        if (! _claimer.claim_with_wait(timeout))
            return sptr();

        const int32_t timeout_ms = static_cast<int32_t>(timeout * 1000);
        _len = recv_udp_packet(_sock_fd, _mem, _frame_size, timeout_ms);

        if (_len > 0) {
            index++;
            return make(this, _mem, size_t(_len));
        }

        _claimer.release(); // undo claim
        return sptr(); // null for timeout
    }

private:
    void* _mem;
    sdr_socket_t _sock_fd;
    size_t _frame_size;
    size_t _len;
    simple_claimer _claimer;
};


class udp_zero_copy_asio_msb : public managed_send_buffer
{
public:
    udp_zero_copy_asio_msb(void* mem, sdr_socket_t sock_fd, const size_t frame_size)
            : _mem(mem), _sock_fd(sock_fd), _frame_size(frame_size)
    { /*NOP*/ }

    void release(void) override
    {
        send_udp_packet(_sock_fd, _mem, size());
        _claimer.release();
    }

    inline sptr get_new(const double timeout, size_t& index)
    {
        if (! _claimer.claim_with_wait(timeout))
            return sptr();
        index++; // advances the caller's buffer
        return make(this, _mem, _frame_size);
    }

private:
    void* _mem;
    sdr_socket_t _sock_fd;
    size_t _frame_size;
    simple_claimer _claimer;
};

class udp_zero_copy_asio_impl : public udp_zero_copy
{
public:
    typedef std::shared_ptr<udp_zero_copy_asio_impl> sptr;

    udp_zero_copy_asio_impl(const std::string& addr,
                            const std::string& port,
                            const zero_copy_xport_params& xport_params)
            : _recv_frame_size(xport_params.recv_frame_size)
            , _num_recv_frames(xport_params.num_recv_frames)
            , _send_frame_size(xport_params.send_frame_size)
            , _num_send_frames(xport_params.num_send_frames)
            , _recv_buffer_pool(buffer_pool::make(
                    xport_params.num_recv_frames, xport_params.recv_frame_size))
            , _send_buffer_pool(buffer_pool::make(
                    xport_params.num_send_frames, xport_params.send_frame_size))
            , _next_recv_buff_index(0)
            , _next_send_buff_index(0)
    {
        LOG_INFO("Create udp addr %s port %s",addr.c_str(),port.c_str());
        _socket = open_udp_socket(addr, port);
        _sock_fd = _socket->sock_fd;

        for (size_t i = 0; i < get_num_recv_frames(); i++) {
            _mrb_pool.push_back(std::make_shared<udp_zero_copy_asio_mrb>(
                    _recv_buffer_pool->at(i), _sock_fd, get_recv_frame_size()));
        }

        // allocate re-usable managed send buffers
        for (size_t i = 0; i < get_num_send_frames(); i++) {
            _msb_pool.push_back(std::make_shared<udp_zero_copy_asio_msb>(
                    _send_buffer_pool->at(i), _sock_fd, get_send_frame_size()));
        }
    }


    size_t resize_send_socket_buffer(size_t num_bytes)
    {
        return resize_udp_socket_buffer(_sock_fd, num_bytes,SOL_SOCKET, SO_SNDBUF);
    }

    size_t resize_recv_socket_buffer(size_t num_bytes)
    {
        return resize_udp_socket_buffer(_sock_fd, num_bytes,SOL_SOCKET, SO_RCVBUF);
    }

    managed_recv_buffer::sptr get_recv_buff(double timeout) override
    {
        if (_next_recv_buff_index == _num_recv_frames)
            _next_recv_buff_index = 0;
        return _mrb_pool[_next_recv_buff_index]->get_new(timeout, _next_recv_buff_index);
    }

    size_t get_num_recv_frames(void) const override
    {
        return _num_recv_frames;
    }
    
    size_t get_recv_frame_size(void) const override
    {
        return _recv_frame_size;
    }

    managed_send_buffer::sptr get_send_buff(double timeout,uint32_t len = 24) override
    {
        if (_next_send_buff_index == _num_send_frames)
            _next_send_buff_index = 0;
        return _msb_pool[_next_send_buff_index]->get_new(timeout, _next_send_buff_index);
    }

    size_t get_num_send_frames(void) const override
    {
        return _num_send_frames;
    }
    size_t get_send_frame_size(void) const override
    {
        return _send_frame_size;
    }

    uint16_t get_local_port(void) const override
    {
        return std::stoi(_socket->port);
    }

    std::string get_local_addr(void) const override
    {
        return _socket->addr;
    }


private:
    // memory management -> buffers && fifos
    const size_t _recv_frame_size, _num_recv_frames;
    const size_t _send_frame_size, _num_send_frames;
    buffer_pool::sptr _recv_buffer_pool, _send_buffer_pool;
    std::vector<std::shared_ptr<udp_zero_copy_asio_msb>> _msb_pool;
    std::vector<std::shared_ptr<udp_zero_copy_asio_mrb>> _mrb_pool;
    size_t _next_recv_buff_index, _next_send_buff_index;
    socket_sptr _socket;
    sdr_socket_t _sock_fd;
};

udp_zero_copy::sptr udp_zero_copy::make(const std::string &addr, const std::string &port,
                                        const sdr::core::zero_copy_xport_params &default_buff_args) {
    zero_copy_xport_params xport_params = default_buff_args;
    if (xport_params.num_recv_frames == 0) {
        xport_params.num_recv_frames = UDP_DEFAULT_NUM_FRAMES;
    }
    if (xport_params.num_send_frames == 0) {
        xport_params.num_send_frames = UDP_DEFAULT_NUM_FRAMES;
    }
    if (xport_params.recv_frame_size == 0) {
        xport_params.recv_frame_size = UDP_DEFAULT_FRAME_SIZE;
    }
    if (xport_params.send_frame_size == 0) {
        xport_params.send_frame_size = UDP_DEFAULT_FRAME_SIZE;
    }


    if (xport_params.recv_buff_size == 0) {
        xport_params.recv_buff_size = std::max<size_t>(
                UDP_DEFAULT_BUFF_SIZE, xport_params.num_recv_frames * MAX_ETHERNET_MTU);
    }
    if (xport_params.send_buff_size == 0) {
        xport_params.send_buff_size = std::max<size_t>(
                UDP_DEFAULT_BUFF_SIZE, xport_params.num_send_frames * MAX_ETHERNET_MTU);
    }

    udp_zero_copy_asio_impl::sptr udp_trans(
            new udp_zero_copy_asio_impl(addr, port, xport_params));

    resize_udp_socket_buffer_with_warning(
            [udp_trans](size_t size) { return udp_trans->resize_recv_socket_buffer(size); },
            xport_params.recv_buff_size,
            "recv");
    resize_udp_socket_buffer_with_warning(
            [udp_trans](size_t size) { return udp_trans->resize_send_socket_buffer(size); },
            xport_params.send_buff_size,
            "send");

    return udp_trans;
}
