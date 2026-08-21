#ifndef SOAPY_SUPER_RECV_PACKET_HANDLER_HPP
#define SOAPY_SUPER_RECV_PACKET_HANDLER_HPP

#include "../../../include/sdr/api/DataStream.hpp"
#include "../../../include/sdr/config.hpp"
#include "../../../include/sdr/core/zero_copy.hpp"
#include "./local_ctrl.hpp"
#include "FIFO.hpp"
#include "local_regs.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#endif

using namespace sdr::api;
using namespace sdr::core;

struct rx_stream_continuity_snapshot {
    bool have_last = false;
    uint64_t packet_count = 0;
    uint64_t sample_count = 0;
    uint64_t seq_errors = 0;
    uint64_t timestamp_errors = 0;
    uint64_t timestamp_backwards = 0;
    uint64_t last_timestamp = 0;
    uint64_t expected_timestamp = 0;
    int64_t last_timestamp_delta = 0;
    uint16_t last_seq = 0;
    uint16_t expected_seq = 0;
    uint32_t last_packet_samples = 0;
    uint64_t host_queue_drops = 0;
    uint64_t host_queue_depth = 0;
    uint64_t host_queue_depth_peak = 0;
};

struct queued_rx_packet {
    managed_recv_buffer::sptr buff;
    sdr_header_t hdr{};
    size_t sample_count = 0;
};

// recv_from_fifo() status bits. A discontinuity is reported on the first
// returned sample after the gap. The call never joins samples from the two
// sides of that gap, so callers can treat each non-empty result as one
// timestamp-contiguous segment.
enum recv_fifo_status_flags : int {
    IQTAXI_RX_FIFO_FLAG_NONE = 0,
    IQTAXI_RX_FIFO_FLAG_DISCONTINUITY_BEFORE = 1 << 0,
    IQTAXI_RX_FIFO_FLAG_TIMESTAMP_BACKWARDS = 1 << 1,
};

class recv_packet_streamer : public sdr::api::rx_streamer
{
public:
    typedef std::shared_ptr<recv_packet_streamer> sptr;

    recv_packet_streamer(local_ctrl::sptr& local_sptr, local_ctrl::sptr& stream_port)
        : _local_port(local_sptr)
        , _stream_port(stream_port)
        , rx_thread()
        , _rx_thread_stop(false)
        // Drop oldest when full so HDSDR backpressure does not pin stale IQ.
        , _rx_fifo(kRxQueuedPackets, true)
        , buff_size_samples(8192)
    {
        set_max_sample_per_packet(8192);
        set_rx_enable_chan(1);
    }

    ~recv_packet_streamer() override
    {
        stop_rx_worker();
    }

    size_t get_request_num_samps(void) const override
    {
        return _request_num_samples;
    }

    size_t get_num_channels(void) const override
    {
        return _channel;
    }

    void set_rx_enable_chan(uint8_t chans) const override
    {
        _local_port->poke32(SET_CHANNEL_ENABLE_ADDR, chans);
        _channel = count_enabled_channels(chans);
    }

    void set_max_sample_per_packet(uint32_t sample_nums)
    {
        static const size_t hdr_size = 16;
        const size_t bpp = _stream_port->get_xport()->get_recv_frame_size() - hdr_size;
        const size_t bpi = 4;
        _max_sample_per_packet = size_t(bpp / bpi);
        (void)sample_nums;
        _local_port->poke32(SET_RX_MAX_PACKET_BYTES, static_cast<uint32_t>(_max_sample_per_packet * 4));
    }

    void set_rx_sample_nums_per_packet(uint32_t sample_nums)
    {
        _local_port->poke32(SET_RX_SAMPLE_NUMS_ADDR, sample_nums);
    }

    void set_rx_mode(uint8_t mode)
    {
        _local_port->poke32(SET_RX_MODE, mode);
    }

    void set_rx_mode_exit()
    {
        _local_port->poke32(SET_RX_MODE_EXIT, 1);
    }

    void set_stream_rx_start()
    {
        _local_port->poke32(SET_START_RX, 1);
    }

    void set_stream_rx_stop()
    {
        _local_port->poke32(SET_STOP_RX, 1);
    }

    void set_max_sample_nums_per_packet(uint32_t sample_nums)
    {
        uint32_t sample_bytes = sample_nums * 4;
        _local_port->poke32(SET_RX_MAX_PACKET_BYTES, sample_bytes);
    }

    void enable_xfft(const size_t fft_point) override
    {
        (void)fft_point;
    }

    size_t set_recv_param(uint8_t rx_mode,
                          const size_t nsamps_per_buff,
                          uint64_t& time_stamp,
                          uint8_t stream_start,
                          uint8_t stream_stop) override
    {
        _request_num_samples = nsamps_per_buff + (8 - nsamps_per_buff % 8) % 8;
        if ((rx_mode == STREAM_MODE) & stream_start & (!stream_stop)) {
            reset_rx_continuity_stats();
            _local_port->poke32(SET_RX_SAMPLE_NUMS_ADDR, static_cast<uint32_t>(_request_num_samples));
            _local_port->poke32(SET_START_RX, 1);
            _start();
        } else if ((rx_mode == STREAM_MODE) & (!stream_start) & (stream_stop)) {
            _local_port->poke32(SET_STOP_RX, 1);
            _stop();
        } else if (rx_mode == PACKET_MODE) {
            _local_port->poke32(SET_RX_SAMPLE_NUMS_ADDR, static_cast<uint32_t>(_request_num_samples));
            _local_port->poke32(SET_CAPTURE_START_ADDR, 1);
        } else if (rx_mode == SYNC_MODE) {
            _local_port->poke32(SET_RX_SAMPLE_NUMS_ADDR, static_cast<uint32_t>(_request_num_samples));
            _local_port->poke32(SET_SYNC_RX_TIMESTAMPS_HI, uint32_t(time_stamp >> 32));
            _local_port->poke32(SET_SYNC_RX_TIMESTAMPS_LO, uint32_t(time_stamp >> 0));
        } else {
            printf("!!!!!! mode not support\n");
        }
        return 0;
    }

    void reset_rx_continuity_stats()
    {
        drain_rx_fifo();
        const size_t drained_transport_packets = drain_rx_transport();
        if (std::getenv("IQTAXI_RX_TRACE") != nullptr && drained_transport_packets != 0) {
            std::cerr << "[IQTAXI_RX_TRACE] native=" << this
                      << " event=transport_drain packets="
                      << drained_transport_packets << std::endl;
        }
        stop_pending_packet();
        std::lock_guard<std::mutex> lock(_continuity_mutex);
        _continuity = rx_stream_continuity_snapshot{};
        _pending_rx_buff.reset();
        _pending_sample_offset = 0;
        _pending_sample_count = 0;
    }

    rx_stream_continuity_snapshot get_rx_continuity_stats() const
    {
        std::lock_guard<std::mutex> lock(_continuity_mutex);
        return _continuity;
    }

    size_t recv(const buffs_type& buffs,
                const size_t nsamps_per_buff,
                uint64_t& time_stamp,
                uint32_t sample_format) override
    {
        return recv_with_timeout(
            buffs, nsamps_per_buff, time_stamp, sample_format, kRxConsumerTimeoutUs, nullptr, false);
    }

    void set_sample_format(uint32_t format)
    {
        sample_format = format;
    }

    size_t recv_fifo(const buffs_type& buffs, const size_t nsamps_per_buff)
    {
        uint64_t timestamp = 0;
        return recv(buffs, nsamps_per_buff, timestamp, sample_format);
    }

    size_t recv_from_fifo(void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs){
            uint64_t timestamp = 0;
            const size_t items = recv_from_fifo_ticks(
                buffs, numElems, flags, timestamp, timeoutUs);
            if (items > 0) {
                timeNs = static_cast<int64_t>(
                    (static_cast<long double>(timestamp) * 1e9L) / std::max<size_t>(sample_rate, 1u));
            }
            return items;
    }

    // Native E200 path for consumers that schedule directly in sample ticks.
    // This avoids precision loss and int64 overflow in the legacy nanosecond
    // compatibility output above.
    size_t recv_from_fifo_ticks(void * const *buffs,
        const size_t numElems,
        int &flags,
        uint64_t &timeTicks,
        const long timeoutUs) override {
            flags = IQTAXI_RX_FIFO_FLAG_NONE;
            const unsigned int timeout_us = (timeoutUs > 0)
                                                ? static_cast<unsigned int>(timeoutUs)
                                                : kRxConsumerTimeoutUs;
            return recv_with_timeout(ref_vector<void*>(buffs, 1),
                                     numElems,
                                     timeTicks,
                                     sample_format,
                                     timeout_us,
                                     &flags,
                                     true);
    }
    void rx_thread_func(uint32_t buff_size_samples)
    {
        (void)buff_size_samples;
        while (!_rx_thread_stop.load()) {
            managed_recv_buffer::sptr buff;
            try {
                buff = _stream_port->get_xport()->get_recv_buff(kRxWorkerRecvTimeoutSec);
            } catch (const std::exception&) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!buff) {
                continue;
            }

            const size_t buff_size = buff->size() / sizeof(int16_t) / 2;
            if (buff_size <= 4) {
                continue;
            }

            queued_rx_packet packet;
            packet.buff = std::move(buff);
            packet.sample_count = buff_size - 4;
            uint32_t* vrt_hdr = packet.buff->cast<uint32_t*>();
            _stream_port->deserialize_hdr(vrt_hdr, packet.hdr);
            update_rx_continuity_stats(packet.hdr, static_cast<uint32_t>(packet.sample_count));

            if (!_rx_fifo.push(packet, false, 0)) {
                note_host_queue_drop();
                continue;
            }
            update_host_queue_depth(_rx_fifo.size());
        }
    }

    void _start(void)
    {
        if (!rx_thread.joinable()) {
            if (std::getenv("IQTAXI_RX_TRACE") != nullptr) {
                std::cerr << "[IQTAXI_RX_TRACE] native=" << this
                          << " event=worker_start" << std::endl;
            }
            _rx_thread_stop = false;
            rx_thread = std::thread(&recv_packet_streamer::rx_thread_func, this, buff_size_samples);
#ifndef _WIN32
            int max_prio = sched_get_priority_max(SCHED_RR);
            if (max_prio > 0) {
                sched_param sch;
                sch.sched_priority = max_prio;
                (void)pthread_setschedparam(rx_thread.native_handle(), SCHED_RR, &sch);
            }
#endif
        }
    }

    void _stop()
    {
        if (rx_thread.joinable()) {
            if (std::getenv("IQTAXI_RX_TRACE") != nullptr) {
                std::cerr << "[IQTAXI_RX_TRACE] native=" << this
                          << " event=worker_stop_begin" << std::endl;
            }
            _rx_thread_stop = true;
            rx_thread.join();
            if (std::getenv("IQTAXI_RX_TRACE") != nullptr) {
                std::cerr << "[IQTAXI_RX_TRACE] native=" << this
                          << " event=worker_stop_done" << std::endl;
            }
        }
        drain_rx_fifo();
        stop_pending_packet();
    }

    void set_sampleRate(size_t sampleRate)
    {
        sample_rate = sampleRate;
    }

private:
    static constexpr size_t kRxQueuedPackets = 4096u;
    static constexpr double kRxWorkerRecvTimeoutSec = 0.01;
    static constexpr unsigned int kRxConsumerTimeoutUs = 20000u;

    bool pop_next_packet(queued_rx_packet& packet, unsigned int timeout_us)
    {
        _start();
        if (!_rx_fifo.pop(packet, true, timeout_us)) {
            return false;
        }
        update_host_queue_depth(_rx_fifo.size());
        return packet.buff != nullptr;
    }

    size_t recv_with_timeout(const buffs_type& buffs,
                             const size_t nsamps_per_buff,
                             uint64_t& time_stamp,
                             uint32_t format,
                             unsigned int timeout_us,
                             int* status_flags,
                             bool stop_at_discontinuity)
    {
        size_t samples_copied = 0;
        bool first_packet = true;

        for (size_t i = 0; i < buffs.size(); ++i) {
            size_t samples_left_to_copy = nsamps_per_buff;
            void* dest_ptr = buffs[i];

            while (samples_left_to_copy > 0) {
                if (!_pending_rx_buff) {
                    queued_rx_packet packet;
                    if (!pop_next_packet(packet, timeout_us)) {
                        break;
                    }

                    _pending_rx_buff = std::move(packet.buff);
                    _pending_hdr = packet.hdr;
                    _pending_sample_count = packet.sample_count;
                    _pending_sample_offset = 0;

                    const bool timestamp_gap =
                        _consumer_have_expected_timestamp &&
                        _pending_hdr.timestamp != _consumer_expected_timestamp;
                    if (timestamp_gap && stop_at_discontinuity) {
                        _pending_discontinuity_before = true;
                        _pending_timestamp_backwards =
                            _pending_hdr.timestamp < _consumer_expected_timestamp;
                        // Finish the continuous prefix now. The packet remains
                        // pending and becomes the first packet of the next call.
                        if (samples_copied > 0) {
                            return samples_copied;
                        }
                    }
                }

                if (first_packet) {
                    first_packet = false;
                    time_stamp = _pending_hdr.timestamp +
                                 (_pending_sample_offset / get_active_channel_count());
                    if (status_flags != nullptr && _pending_discontinuity_before) {
                        *status_flags |= IQTAXI_RX_FIFO_FLAG_DISCONTINUITY_BEFORE;
                        if (_pending_timestamp_backwards) {
                            *status_flags |= IQTAXI_RX_FIFO_FLAG_TIMESTAMP_BACKWARDS;
                        }
                    }
                    _pending_discontinuity_before = false;
                    _pending_timestamp_backwards = false;
                }

                uint32_t* vrt_hdr = _pending_rx_buff->cast<uint32_t*>();
                _copy_buff = reinterpret_cast<const void*>(
                    reinterpret_cast<const int16_t*>(vrt_hdr + 4) + _pending_sample_offset * 2);
                const size_t to_copy_samples = std::min<size_t>(
                    _pending_sample_count - _pending_sample_offset, samples_left_to_copy);

                if (format == MICRORF_FORMAT_FLOAT32) {
                    const int16_t* src = static_cast<const int16_t*>(_copy_buff);
                    float* dst = static_cast<float*>(dest_ptr);

                    for (size_t s = 0; s < 2 * to_copy_samples; ++s) {
                        dst[s] = float(src[s]) / 32768.0f;
                    }
                    dest_ptr = static_cast<void*>(dst + to_copy_samples * 2);
                } else if (format == MICRORF_FORMAT_INT16) {
                    std::memcpy(dest_ptr, _copy_buff, to_copy_samples * sizeof(int16_t) * 2);
                    dest_ptr = static_cast<void*>(
                        static_cast<int16_t*>(dest_ptr) + to_copy_samples * 2);
                } else {
                    break;
                }

                samples_copied += to_copy_samples;
                samples_left_to_copy -= to_copy_samples;
                _pending_sample_offset += to_copy_samples;
                _consumer_have_expected_timestamp = true;
                _consumer_expected_timestamp =
                    _pending_hdr.timestamp +
                    (_pending_sample_offset / get_active_channel_count());

                if (_pending_sample_offset >= _pending_sample_count) {
                    _pending_rx_buff.reset();
                    _pending_sample_offset = 0;
                    _pending_sample_count = 0;
                }
            }
        }

        return samples_copied;
    }

    void drain_rx_fifo()
    {
        queued_rx_packet packet;
        while (_rx_fifo.pop(packet, false, 0)) {
        }
        update_host_queue_depth(0);
    }

    size_t drain_rx_transport()
    {
        // A prior process can leave the FPGA streaming into this socket for
        // several seconds before UHD issues START.  Those datagrams live in
        // the kernel socket queue, not in _rx_fifo, and otherwise become the
        // first packets of the new epoch.  STOP is issued before this method,
        // so draining until the socket is momentarily empty is bounded.
        static constexpr size_t kMaxDrainPackets = 65536u;
        size_t drained = 0;
        while (drained != kMaxDrainPackets) {
            managed_recv_buffer::sptr packet =
                _stream_port->get_xport()->get_recv_buff(0.0);
            if (!packet) {
                break;
            }
            ++drained;
        }
        return drained;
    }

    void stop_pending_packet()
    {
        _pending_rx_buff.reset();
        _pending_sample_offset = 0;
        _pending_sample_count = 0;
        _pending_discontinuity_before = false;
        _pending_timestamp_backwards = false;
        _consumer_have_expected_timestamp = false;
        _consumer_expected_timestamp = 0;
    }

    void stop_rx_worker()
    {
        _stop();
    }

    void update_rx_continuity_stats(const sdr_header_t& hdr, uint32_t packet_samples)
    {
        std::lock_guard<std::mutex> lock(_continuity_mutex);

        if (!_continuity.have_last) {
            _continuity.have_last = true;
            _continuity.packet_count = 1;
            _continuity.sample_count = packet_samples;
            _continuity.last_seq = hdr.seq;
            _continuity.expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
            _continuity.last_timestamp = hdr.timestamp;
            _continuity.expected_timestamp = hdr.timestamp + packet_timestamp_step(packet_samples);
            _continuity.last_timestamp_delta = 0;
            _continuity.last_packet_samples = packet_samples;
            return;
        }

        if (hdr.seq != _continuity.expected_seq) {
            _continuity.seq_errors++;
        }

        if (hdr.timestamp != _continuity.expected_timestamp) {
            if (_continuity.timestamp_errors < 8) {
                std::cerr << "RX TS mismatch:"
                          << " seq=" << hdr.seq
                          << " packet_samples=" << packet_samples
                          << " chans=" << get_active_channel_count()
                          << " prev_samples=" << _continuity.last_packet_samples
                          << " expected=" << _continuity.expected_timestamp
                          << " actual=" << hdr.timestamp
                          << " step=" << packet_timestamp_step(packet_samples)
                          << '\n';
            }
            _continuity.timestamp_errors++;
            if (hdr.timestamp < _continuity.expected_timestamp) {
                _continuity.timestamp_backwards++;
                _continuity.last_timestamp_delta =
                    -static_cast<int64_t>(_continuity.expected_timestamp - hdr.timestamp);
            } else {
                _continuity.last_timestamp_delta =
                    static_cast<int64_t>(hdr.timestamp - _continuity.expected_timestamp);
            }
        } else {
            _continuity.last_timestamp_delta = 0;
        }

        _continuity.packet_count++;
        _continuity.sample_count += packet_samples;
        _continuity.last_seq = hdr.seq;
        _continuity.expected_seq = static_cast<uint16_t>(hdr.seq + 1u);
        _continuity.last_timestamp = hdr.timestamp;
        _continuity.expected_timestamp = hdr.timestamp + packet_timestamp_step(packet_samples);
        _continuity.last_packet_samples = packet_samples;
    }

    size_t get_active_channel_count() const
    {
        return std::max<size_t>(_channel, 1u);
    }

    static size_t count_enabled_channels(uint8_t channel_mask)
    {
        size_t count = 0;
        for (uint8_t mask = channel_mask; mask != 0; mask >>= 1) {
            count += mask & 0x1u;
        }
        return std::max<size_t>(count, 1u);
    }

    uint32_t packet_timestamp_step(uint32_t packet_cs16_items) const
    {
        const size_t channels = get_active_channel_count();
        return static_cast<uint32_t>((packet_cs16_items + channels - 1u) / channels);
    }

    void note_host_queue_drop()
    {
        std::lock_guard<std::mutex> lock(_continuity_mutex);
        _continuity.host_queue_drops++;
    }

    void update_host_queue_depth(size_t depth)
    {
        std::lock_guard<std::mutex> lock(_continuity_mutex);
        _continuity.host_queue_depth = depth;
        _continuity.host_queue_depth_peak =
            std::max<uint64_t>(_continuity.host_queue_depth_peak, static_cast<uint64_t>(depth));
    }

private:
    sdr_header_t _hdr;
    sdr_header_t _pending_hdr;
    const void* _copy_buff = nullptr;
    uint32_t _max_sample_per_packet = 0;
    size_t _request_num_samples = 0;
    mutable size_t _channel = 1;
    size_t seq = 0;
    local_ctrl::sptr _local_port, _stream_port;

    std::thread rx_thread;
    std::atomic<bool> _rx_thread_stop;
    TSQueue<queued_rx_packet> _rx_fifo;
    managed_recv_buffer::sptr _pending_rx_buff;
    size_t _pending_sample_offset = 0;
    size_t _pending_sample_count = 0;
    bool _pending_discontinuity_before = false;
    bool _pending_timestamp_backwards = false;
    bool _consumer_have_expected_timestamp = false;
    uint64_t _consumer_expected_timestamp = 0;
    uint32_t buff_size_samples;
    size_t sample_size_bytes = 2;

    uint32_t sample_format = MICRORF_FORMAT_INT16;
    size_t sample_rate = 1;

    mutable std::mutex _continuity_mutex;
    rx_stream_continuity_snapshot _continuity;
};

#endif // SOAPY_SUPER_RECV_PACKET_HANDLER_HPP
