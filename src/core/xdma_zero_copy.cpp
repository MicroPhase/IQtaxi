#include "include/sdr/core/xdma_zero_copy.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace sdr;
using namespace sdr::core;

namespace {
constexpr size_t kXdmaAlignment = 4096u;
constexpr size_t kXdmaCharXferBytesMax = 255u * 4096u;
constexpr size_t kDefaultPacketBytes = 16496u;
constexpr uint32_t kM300C2hRingVersion = 1u;

struct xdma_m300_c2h_ring_start
{
    uint32_t version;
    uint32_t packet_bytes;
    uint32_t depth;
    uint32_t flags;
};

struct xdma_m300_c2h_ring_get
{
    uint64_t buffer;
    uint32_t buffer_bytes;
    uint32_t timeout_ms;
    uint32_t bytes;
    uint32_t status;
    uint32_t index;
    uint32_t reserved;
    uint64_t sequence;
    uint64_t completed;
    uint64_t errors;
};

struct xdma_m300_c2h_ring_get_bulk
{
    uint64_t buffer;
    uint32_t buffer_bytes;
    uint32_t packet_stride;
    uint32_t max_packets;
    uint32_t timeout_ms;
    uint32_t packets;
    uint32_t bytes;
    uint32_t status;
    uint64_t first_sequence;
    uint64_t completed;
    uint64_t errors;
};

struct xdma_m300_c2h_ring_get_payload_bulk
{
    uint64_t buffer;
    uint32_t buffer_bytes;
    uint32_t payload_stride;
    uint32_t max_packets;
    uint32_t timeout_ms;
    uint32_t packets;
    uint32_t bytes;
    uint32_t status;
    uint64_t first_sequence;
    uint64_t first_timestamp;
    uint64_t completed;
    uint64_t errors;
};

#define IOCTL_XDMA_M300_C2H_RING_START _IOW('q', 20, struct xdma_m300_c2h_ring_start *)
#define IOCTL_XDMA_M300_C2H_RING_STOP  _IO('q', 21)
#define IOCTL_XDMA_M300_C2H_RING_GET   _IOWR('q', 22, struct xdma_m300_c2h_ring_get *)
#define IOCTL_XDMA_M300_C2H_RING_GET_BULK _IOWR('q', 23, struct xdma_m300_c2h_ring_get_bulk *)
#define IOCTL_XDMA_M300_C2H_RING_GET_PAYLOAD_BULK _IOWR('q', 24, struct xdma_m300_c2h_ring_get_payload_bulk *)

size_t align_up(size_t value, size_t align)
{
    return ((value + align - 1) / align) * align;
}

ssize_t read_retry(int fd, void* buf, size_t len)
{
    ssize_t rc;
    do {
        rc = ::read(fd, buf, len);
    } while (rc < 0 && errno == EINTR);
    return rc;
}

ssize_t write_retry(int fd, const void* buf, size_t len)
{
    size_t total = 0;
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    while (total < len) {
        ssize_t rc = ::write(fd, p + total, len - total);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return rc;
        }
        if (rc == 0)
            return static_cast<ssize_t>(total);
        total += static_cast<size_t>(rc);
    }
    return static_cast<ssize_t>(total);
}

struct xdma_header
{
    uint16_t magic_type;
    uint16_t seq;
    uint8_t sid;
    uint32_t length;
};

uint64_t load_le64(const uint8_t* p)
{
    return ((uint64_t)p[0]) |
           ((uint64_t)p[1] << 8) |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

xdma_header parse_header(const uint8_t* p)
{
    const uint64_t raw = load_le64(p);
    xdma_header h;
    h.length = raw & 0x00ffffffu;
    h.sid = (raw >> 24) & 0xffu;
    h.seq = (raw >> 32) & 0xffffu;
    h.magic_type = (raw >> 48) & 0xffffu;
    return h;
}

bool header_is_valid(const xdma_header& h, size_t max_packet_bytes)
{
    if (h.length < 16u)
        return false;
    if (h.length > max_packet_bytes)
        return false;
    switch (h.magic_type) {
    case 0x5501u:
    case 0x5502u:
    case 0x5503u:
    case 0x5504u:
    case 0x5505u:
        return true;
    default:
        return false;
    }
}

size_t find_valid_header(const uint8_t* data, size_t total, size_t max_packet_bytes)
{
    if (!data || total < 16u)
        return total;

    for (size_t offset = 0; offset + 16u <= total; offset += 16u) {
        if (header_is_valid(parse_header(data + offset), max_packet_bytes))
            return offset;
    }

    return total;
}

bool wait_fd_readable(int fd, double timeout_sec)
{
    if (timeout_sec < 0.0)
        timeout_sec = 0.0;

    const int timeout_ms = static_cast<int>(timeout_sec * 1000.0);
    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    for (;;) {
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc > 0)
            return true;
        if (rc == 0)
            return false;
        if (errno != EINTR)
            return false;
    }
}

bool trace_xdma_enabled()
{
    return std::getenv("M300_XDMA_TRACE") != nullptr;
}

class xdma_zero_copy_mrb : public managed_recv_buffer
{
public:
    xdma_zero_copy_mrb(void* mem,
                       int fd,
                       size_t packet_bytes,
                       size_t packet_stride,
                       bool driver_ring_recv)
        : _mem(mem)
        , _fd(fd)
        , _packet_bytes(packet_bytes)
        , _packet_stride(packet_stride)
        , _driver_ring_recv(driver_ring_recv)
    {
    }

    void release(void) override
    {
        _claimer.release();
    }

    bool claim(double timeout)
    {
        return _claimer.claim_with_wait(timeout);
    }

    void* raw_mem() const
    {
        return _mem;
    }

    sptr make_filled(size_t total)
    {
        if (total < 16u) {
            _claimer.release();
            return sptr();
        }

        const size_t header_offset =
            find_valid_header(static_cast<const uint8_t*>(_mem), total, _packet_bytes);
        if (header_offset >= total) {
            if (trace_xdma_enabled()) {
                const xdma_header first = parse_header(static_cast<const uint8_t*>(_mem));
                std::fprintf(stderr,
                             "xdma_recv invalid_header total=%zu first_magic=0x%04x first_len=%u\n",
                             total,
                             first.magic_type,
                             first.length);
            }
            _claimer.release();
            return sptr();
        }
        if (header_offset != 0u) {
            std::memmove(_mem,
                         static_cast<const uint8_t*>(_mem) + header_offset,
                         total - header_offset);
            total -= header_offset;
        }

        const xdma_header h = parse_header(static_cast<const uint8_t*>(_mem));
        if (!header_is_valid(h, _packet_bytes)) {
            _claimer.release();
            return sptr();
        }

        return make(this, _mem, std::min<size_t>(h.length, total));
    }

    sptr get_new(double timeout, size_t& index)
    {
        if (!_claimer.claim_with_wait(timeout))
            return sptr();

        index++;
        size_t total = 0;
        if (_driver_ring_recv) {
            xdma_m300_c2h_ring_get get {};
            get.buffer = reinterpret_cast<uint64_t>(_mem);
            get.buffer_bytes = static_cast<uint32_t>(_packet_stride);
            get.timeout_ms = static_cast<uint32_t>(std::max(0.0, timeout) * 1000.0);
            if (get.timeout_ms == 0u)
                get.timeout_ms = 1u;

            const int ioctl_rc = ::ioctl(_fd, IOCTL_XDMA_M300_C2H_RING_GET, &get);
            if (ioctl_rc != 0 || get.status != 0u || get.bytes == 0u) {
                if (trace_xdma_enabled()) {
                    std::fprintf(stderr,
                                 "xdma_ring_get rc=%d errno=%d status=%u bytes=%u completed=%llu errors=%llu\n",
                                 ioctl_rc,
                                 ioctl_rc != 0 ? errno : 0,
                                 get.status,
                                 get.bytes,
                                 static_cast<unsigned long long>(get.completed),
                                 static_cast<unsigned long long>(get.errors));
                }
                _claimer.release();
                return sptr();
            }
            total = static_cast<size_t>(get.bytes);
        } else {
            const ssize_t rc = read_retry(_fd, _mem, _packet_stride);
            if (rc < 0) {
                _claimer.release();
                return sptr();
            }
            total = static_cast<size_t>(rc);
        }
        return make_filled(total);
    }

private:
    void* _mem;
    int _fd;
    size_t _packet_bytes;
    size_t _packet_stride;
    bool _driver_ring_recv;
    simple_claimer _claimer;
};

class xdma_zero_copy_msb : public managed_send_buffer
{
public:
    xdma_zero_copy_msb(void* mem, int fd, size_t frame_size)
        : _mem(mem)
        , _fd(fd)
        , _frame_size(frame_size)
    {
    }

    void release(void) override
    {
        if (size() > 0) {
            const ssize_t rc = write_retry(_fd, _mem, size());
            (void)rc;
        }
        _claimer.release();
    }

    sptr get_new(double timeout, size_t& index)
    {
        if (!_claimer.claim_with_wait(timeout))
            return sptr();
        index++;
        return make(this, _mem, _frame_size);
    }

private:
    void* _mem;
    int _fd;
    size_t _frame_size;
    simple_claimer _claimer;
};

class xdma_zero_copy_impl : public xdma_zero_copy
{
public:
    xdma_zero_copy_impl(const xdma_zero_copy_params& xdma_params,
            const zero_copy_xport_params& xport_params)
        : _recv_device(resolve_recv_device(xdma_params))
        , _send_device(resolve_send_device(xdma_params))
        , _packet_bytes(xdma_params.packet_bytes ? xdma_params.packet_bytes : kDefaultPacketBytes)
        , _packet_stride(xdma_params.packet_stride ? xdma_params.packet_stride
                                                   : align_up(_packet_bytes, kXdmaAlignment))
        , _compact_strided_packets(xdma_params.compact_strided_packets)
        , _eop_flush(xdma_params.eop_flush)
        , _driver_ring_recv(xdma_params.driver_ring_recv)
        , _driver_ring_depth(xdma_params.driver_ring_depth ? xdma_params.driver_ring_depth : 256u)
        , _recv_frame_size(std::max(xport_params.recv_frame_size ? xport_params.recv_frame_size : _packet_stride,
                                    _packet_stride))
        , _send_frame_size(xport_params.send_frame_size ? xport_params.send_frame_size : _packet_bytes)
        , _num_recv_frames(xport_params.num_recv_frames ? xport_params.num_recv_frames : 16u)
        , _num_send_frames(xport_params.num_send_frames ? xport_params.num_send_frames : 16u)
        , _can_recv(!_recv_device.empty())
        , _can_send(!_send_device.empty())
        , _recv_buffer_pool(_can_recv ? buffer_pool::make(_num_recv_frames, _recv_frame_size, kXdmaAlignment) : nullptr)
        , _send_buffer_pool(_can_send ? buffer_pool::make(_num_send_frames, _send_frame_size, kXdmaAlignment) : nullptr)
        , _next_recv_buff_index(0)
        , _next_send_buff_index(0)
    {
        const int rd_flags = O_RDONLY;
        const int wr_flags = O_WRONLY;

        if (_can_recv) {
            _recv_fd = ::open(_recv_device.c_str(), rd_flags);
            if (_recv_fd < 0) {
                throw std::runtime_error("failed to open XDMA recv device: " + _recv_device + ": " +
                                         std::strerror(errno));
            }
        }

        if (_can_send) {
            _send_fd = ::open(_send_device.c_str(), wr_flags);
            if (_send_fd < 0) {
                if (_recv_fd >= 0)
                    ::close(_recv_fd);
                throw std::runtime_error("failed to open XDMA send device: " + _send_device + ": " +
                                         std::strerror(errno));
            }
        }

        if (_can_recv) {
            for (size_t i = 0; i < _num_recv_frames; ++i) {
                _mrb_pool.push_back(std::make_shared<xdma_zero_copy_mrb>(
                    _recv_buffer_pool->at(i), _recv_fd, _packet_bytes, _packet_stride,
                    _driver_ring_recv));
            }
        }

        if (_can_send) {
            for (size_t i = 0; i < _num_send_frames; ++i) {
                _msb_pool.push_back(std::make_shared<xdma_zero_copy_msb>(
                    _send_buffer_pool->at(i), _send_fd, _send_frame_size));
            }
        }
    }

    ~xdma_zero_copy_impl() override
    {
        stop_recv();
        if (_recv_fd >= 0) {
            ::close(_recv_fd);
            _recv_fd = -1;
        }
        if (_send_fd >= 0) {
            ::close(_send_fd);
            _send_fd = -1;
        }
    }

    managed_recv_buffer::sptr get_recv_buff(double timeout) override
    {
        if (!_can_recv)
            return managed_recv_buffer::sptr();
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

    managed_send_buffer::sptr get_send_buff(double timeout, uint32_t len) override
    {
        if (!_can_send)
            return managed_send_buffer::sptr();
        if (_next_send_buff_index == _num_send_frames)
            _next_send_buff_index = 0;
        (void)len;
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

    std::string get_device(void) const override
    {
        return _recv_device + "|" + _send_device;
    }

    size_t get_packet_bytes(void) const override
    {
        return _packet_bytes;
    }

    size_t get_packet_stride(void) const override
    {
        return _packet_stride;
    }

    bool start_recv(double timeout) override
    {
        (void)timeout;
        if (!_can_recv || !_driver_ring_recv)
            return true;

        std::lock_guard<std::mutex> lock(_driver_ring_mutex);
        if (_driver_ring_started)
            return true;

        // The kernel C2H ring is a singleton for each XDMA channel. Keep an
        // advisory lock for the complete START..STOP interval so two clients
        // using this transport cannot accidentally reset each other's ring.
        if (::flock(_recv_fd, LOCK_EX | LOCK_NB) != 0) {
            throw std::runtime_error("XDMA C2H ring on " + _recv_device +
                                     " is in use by another process: " +
                                     std::strerror(errno));
        }
        _driver_ring_lock_held = true;

        xdma_m300_c2h_ring_start cfg {};
        cfg.version = kM300C2hRingVersion;
        cfg.packet_bytes = static_cast<uint32_t>(_packet_stride);
        cfg.depth = _driver_ring_depth;

        if (::ioctl(_recv_fd, IOCTL_XDMA_M300_C2H_RING_START, &cfg) != 0) {
            int start_errno = errno;
            std::string recovery_detail;

            if (start_errno == EBUSY) {
                // No cooperating process owns the advisory lock, so EBUSY is
                // a ring left active after an abnormal exit or a failed STOP.
                // Ask the driver to tear it down, then allow its worker to
                // quiesce before retrying START a bounded number of times.
                const int stop_rc =
                    ::ioctl(_recv_fd, IOCTL_XDMA_M300_C2H_RING_STOP);
                const int stop_errno = stop_rc != 0 ? errno : 0;
                if (stop_rc == 0) {
                    const auto recovery_deadline =
                        std::chrono::steady_clock::now() +
                        std::chrono::seconds(11);
                    unsigned retry = 0u;
                    while (std::chrono::steady_clock::now() < recovery_deadline) {
                        const auto delay = retry < 5u ?
                            std::chrono::milliseconds(10u << retry) :
                            std::chrono::milliseconds(250u);
                        std::this_thread::sleep_for(delay);
                        if (::ioctl(_recv_fd,
                                    IOCTL_XDMA_M300_C2H_RING_START,
                                    &cfg) == 0) {
                            _driver_ring_started = true;
                            std::fprintf(stderr,
                                         "recovered stale XDMA C2H ring on %s\n",
                                         _recv_device.c_str());
                            return true;
                        }
                        start_errno = errno;
                        if (start_errno != EBUSY)
                            break;
                        ++retry;
                    }
                    recovery_detail =
                        " (stale-ring STOP succeeded, but START remained busy "
                        "during the 11-second recovery window)";
                } else {
                    recovery_detail = " (stale-ring STOP failed: " +
                                      std::string(std::strerror(stop_errno)) +
                                      ")";
                }
            }

            (void)::flock(_recv_fd, LOCK_UN);
            _driver_ring_lock_held = false;
            throw std::runtime_error("failed to start XDMA C2H ring on " +
                                     _recv_device + ": " +
                                     std::strerror(start_errno) +
                                     recovery_detail);
        }
        _driver_ring_started = true;
        return true;
    }

    void stop_recv(void) override
    {
        if (!_can_recv || !_driver_ring_recv)
            return;

        std::lock_guard<std::mutex> lock(_driver_ring_mutex);
        _ready_recv_buffs.clear();
        if (_driver_ring_started) {
            const int stop_rc =
                ::ioctl(_recv_fd, IOCTL_XDMA_M300_C2H_RING_STOP);
            if (stop_rc != 0 && trace_xdma_enabled()) {
                std::fprintf(stderr,
                             "xdma ring STOP failed on %s: %s\n",
                             _recv_device.c_str(),
                             std::strerror(errno));
            }
            _driver_ring_started = false;
        }
        if (_driver_ring_lock_held) {
            (void)::flock(_recv_fd, LOCK_UN);
            _driver_ring_lock_held = false;
        }
    }

    size_t recv_payload_burst(void* buffer,
                              size_t buffer_bytes,
                              size_t payload_stride,
                              size_t max_packets,
                              uint64_t& first_timestamp,
                              uint64_t& first_sequence,
                              double timeout) override
    {
        first_timestamp = 0;
        first_sequence = 0;
        if (!_can_recv || !_driver_ring_recv || !_driver_ring_started ||
            !buffer || payload_stride == 0u || max_packets == 0u) {
            return 0;
        }

        const size_t packets_by_space = buffer_bytes / payload_stride;
        max_packets = std::min<size_t>(max_packets, packets_by_space);
        max_packets = std::min<size_t>(max_packets, _driver_ring_depth);
        max_packets = std::min<size_t>(max_packets, 64u);
        if (max_packets == 0u) {
            return 0;
        }

        xdma_m300_c2h_ring_get_payload_bulk get {};
        get.buffer = reinterpret_cast<uint64_t>(buffer);
        get.buffer_bytes = static_cast<uint32_t>(buffer_bytes);
        get.payload_stride = static_cast<uint32_t>(payload_stride);
        get.max_packets = static_cast<uint32_t>(max_packets);
        get.timeout_ms = static_cast<uint32_t>(std::max(0.0, timeout) * 1000.0);
        if (get.timeout_ms == 0u)
            get.timeout_ms = 1u;

        const int ioctl_rc =
            ::ioctl(_recv_fd, IOCTL_XDMA_M300_C2H_RING_GET_PAYLOAD_BULK, &get);
        if (ioctl_rc != 0 || get.status != 0u || get.packets == 0u) {
            if (trace_xdma_enabled()) {
                std::fprintf(stderr,
                             "xdma_ring_get_payload_bulk rc=%d errno=%d status=%u packets=%u bytes=%u completed=%llu errors=%llu\n",
                             ioctl_rc,
                             ioctl_rc != 0 ? errno : 0,
                             get.status,
                             get.packets,
                             get.bytes,
                             static_cast<unsigned long long>(get.completed),
                             static_cast<unsigned long long>(get.errors));
            }
            return 0;
        }

        first_timestamp = get.first_timestamp;
        first_sequence = get.first_sequence;
        return static_cast<size_t>(get.packets);
    }

private:
    managed_recv_buffer::sptr get_driver_ring_recv_buff(double timeout)
    {
        if (!_ready_recv_buffs.empty()) {
            auto buff = _ready_recv_buffs.front();
            _ready_recv_buffs.pop_front();
            return buff;
        }

        prefetch_driver_ring_recv(timeout);
        if (_ready_recv_buffs.empty())
            return managed_recv_buffer::sptr();

        auto buff = _ready_recv_buffs.front();
        _ready_recv_buffs.pop_front();
        return buff;
    }

    void prefetch_driver_ring_recv(double timeout)
    {
        if (!_driver_ring_started) {
            if (_next_recv_buff_index == _num_recv_frames)
                _next_recv_buff_index = 0;
            auto buff = _mrb_pool[_next_recv_buff_index]->get_new(timeout, _next_recv_buff_index);
            if (buff)
                _ready_recv_buffs.push_back(buff);
            return;
        }

        if (_next_recv_buff_index == _num_recv_frames)
            _next_recv_buff_index = 0;

        const size_t first = _next_recv_buff_index;
        const size_t contiguous = _num_recv_frames - first;
        const size_t max_packets = std::min<size_t>(
            std::min<size_t>(contiguous, _driver_ring_depth), 64u);
        std::vector<std::shared_ptr<xdma_zero_copy_mrb>> claimed;
        claimed.reserve(max_packets);

        for (size_t i = 0; i < max_packets; ++i) {
            const double claim_timeout = (i == 0) ? timeout : 0.0;
            auto& mrb = _mrb_pool[first + i];
            if (!mrb->claim(claim_timeout))
                break;
            claimed.push_back(mrb);
        }
        if (claimed.empty())
            return;

        xdma_m300_c2h_ring_get_bulk get {};
        get.buffer = reinterpret_cast<uint64_t>(claimed.front()->raw_mem());
        get.buffer_bytes = static_cast<uint32_t>(_recv_frame_size * claimed.size());
        get.packet_stride = static_cast<uint32_t>(_recv_frame_size);
        get.max_packets = static_cast<uint32_t>(claimed.size());
        get.timeout_ms = static_cast<uint32_t>(std::max(0.0, timeout) * 1000.0);
        if (get.timeout_ms == 0u)
            get.timeout_ms = 1u;

        const int ioctl_rc = ::ioctl(_recv_fd, IOCTL_XDMA_M300_C2H_RING_GET_BULK, &get);
        if (ioctl_rc != 0 || get.status != 0u || get.packets == 0u) {
            if (trace_xdma_enabled()) {
                std::fprintf(stderr,
                             "xdma_ring_get_bulk rc=%d errno=%d status=%u packets=%u bytes=%u completed=%llu errors=%llu\n",
                             ioctl_rc,
                             ioctl_rc != 0 ? errno : 0,
                             get.status,
                             get.packets,
                             get.bytes,
                             static_cast<unsigned long long>(get.completed),
                             static_cast<unsigned long long>(get.errors));
            }
            for (auto& mrb : claimed)
                mrb->release();
            return;
        }

        const size_t packets = std::min<size_t>(get.packets, claimed.size());
        for (size_t i = 0; i < packets; ++i) {
            auto buff = claimed[i]->make_filled(_packet_stride);
            if (buff)
                _ready_recv_buffs.push_back(buff);
        }
        for (size_t i = packets; i < claimed.size(); ++i)
            claimed[i]->release();

        _next_recv_buff_index = first + packets;
        if (_next_recv_buff_index == _num_recv_frames)
            _next_recv_buff_index = 0;
    }

    static std::string resolve_recv_device(const xdma_zero_copy_params& params)
    {
        if (!params.recv_device.empty())
            return params.recv_device;
        if (params.recv_device.empty() && params.send_device.empty())
            return params.device;
        return std::string();
    }

    static std::string resolve_send_device(const xdma_zero_copy_params& params)
    {
        if (!params.send_device.empty())
            return params.send_device;
        if (params.recv_device.empty() && params.send_device.empty())
            return params.device;
        return std::string();
    }

    std::string _recv_device;
    std::string _send_device;
    size_t _packet_bytes;
    size_t _packet_stride;
    bool _compact_strided_packets;
    bool _eop_flush;
    bool _driver_ring_recv;
    uint32_t _driver_ring_depth;
    const size_t _recv_frame_size;
    const size_t _send_frame_size;
    const size_t _num_recv_frames;
    const size_t _num_send_frames;
    bool _can_recv;
    bool _can_send;
    int _recv_fd = -1;
    int _send_fd = -1;
    bool _driver_ring_started = false;
    bool _driver_ring_lock_held = false;
    std::mutex _driver_ring_mutex;
    buffer_pool::sptr _recv_buffer_pool;
    buffer_pool::sptr _send_buffer_pool;
    std::vector<std::shared_ptr<xdma_zero_copy_msb>> _msb_pool;
    std::vector<std::shared_ptr<xdma_zero_copy_mrb>> _mrb_pool;
    std::deque<managed_recv_buffer::sptr> _ready_recv_buffs;
    size_t _next_recv_buff_index;
    size_t _next_send_buff_index;
};

} // namespace

xdma_zero_copy::sptr xdma_zero_copy::make(const std::string& device,
        const zero_copy_xport_params& default_buff_args)
{
    xdma_zero_copy_params params;
    params.device = device;
    return make(params, default_buff_args);
}

xdma_zero_copy::sptr xdma_zero_copy::make(const xdma_zero_copy_params& xdma_params,
        const zero_copy_xport_params& default_buff_args)
{
    zero_copy_xport_params xport_params = default_buff_args;
    if (xport_params.num_recv_frames == 0)
        xport_params.num_recv_frames = 16u;
    if (xport_params.num_send_frames == 0)
        xport_params.num_send_frames = 16u;
    if (xport_params.recv_frame_size == 0)
        xport_params.recv_frame_size = xdma_params.packet_stride ? xdma_params.packet_stride : align_up(kDefaultPacketBytes, kXdmaAlignment);
    if (xport_params.send_frame_size == 0)
        xport_params.send_frame_size = xdma_params.packet_bytes ? xdma_params.packet_bytes : kDefaultPacketBytes;

    return std::make_shared<xdma_zero_copy_impl>(xdma_params, xport_params);
}
