#ifndef SOAPY_SUPER_SEND_PACKET_HANDLER_HPP
#define SOAPY_SUPER_SEND_PACKET_HANDLER_HPP

#include "../../../include/sdr/api/DataStream.hpp"
#include "../../../include/sdr/config.hpp"
#include "../../../include/sdr/core/zero_copy.hpp"
#include "./local_ctrl.hpp"
#include "local_regs.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace sdr::api;
using namespace sdr::core;

namespace {
constexpr uint32_t kTxFlowControlPacketBytesLegacy = 24u;
constexpr uint32_t kTxFlowControlPacketBytes = 32u;
}

struct tx_flow_control_snapshot {
    bool monitoring_active = false;
    bool ready_to_send = true;
    uint32_t last_fc_word = 0u;
    uint64_t monitor_elapsed_us = 0;
    uint64_t fc_pause_count = 0;
    uint64_t fc_pause_total_us = 0;
    uint64_t fc_wait_count = 0;
    uint64_t fc_wait_timeout_count = 0;
    uint64_t send_buff_timeout_count = 0;
    uint64_t packets_sent = 0;
    uint64_t samples_sent = 0;
    uint64_t last_timestamp = 0;
    uint16_t last_seq = 0;
};

inline void copy_standard_iq_to_e100_qi_cs16(
    const int16_t* src_iq,
    std::size_t nsamps,
    int16_t* dst_qi)
{
    for (std::size_t n = 0; n < nsamps; ++n) {
        const std::size_t base = n * 2;
        dst_qi[base + 0] = src_iq[base + 1];
        dst_qi[base + 1] = src_iq[base + 0];
    }
}

inline void copy_standard_iq_to_e100_qi_cs16(
    const float* src_iq,
    std::size_t nsamps,
    int16_t* dst_qi)
{
    for (std::size_t n = 0; n < nsamps; ++n) {
        const std::size_t base = n * 2;
        const float i_val = std::clamp(src_iq[base + 0], -1.0f, 1.0f);
        const float q_val = std::clamp(src_iq[base + 1], -1.0f, 1.0f);
        dst_qi[base + 0] = static_cast<int16_t>(std::lround(q_val * 32767.0f));
        dst_qi[base + 1] = static_cast<int16_t>(std::lround(i_val * 32767.0f));
    }
}

class send_packet_streamer : public sdr::api::tx_streamer {
public:
    typedef std::function<managed_recv_buffer::sptr(double)> get_buff_type;
    static constexpr double kFlowControlRecvTimeoutSec = 0.01;
    static constexpr auto kSendReadyWait = std::chrono::milliseconds(20);

    send_packet_streamer(local_ctrl::sptr& local_sptr, local_ctrl::sptr& stream_port)
    {
        _local_port = local_sptr;
        _stream_port = stream_port;
        _last_seq_out = 0;
        _last_seq_ack = 0;
        _fc_pkt_window = 800;
        start_flow_control_thread();
    }

    ~send_packet_streamer() override
    {
        stop_flow_control_thread();
    }

    void begin_tx_flow_control_monitoring()
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        _stop_requested = false;
        _flow_stats = tx_flow_control_snapshot{};
        _flow_stats.monitoring_active = true;
        _flow_stats.ready_to_send = _ready_to_send;
        _monitor_start = std::chrono::steady_clock::now();
        if (!_ready_to_send) {
            _pause_start = _monitor_start;
        } else {
            _pause_start = std::chrono::steady_clock::time_point{};
        }
    }

    void end_tx_flow_control_monitoring()
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        _stop_requested = true;
        _pause_start = std::chrono::steady_clock::time_point{};
        _monitor_start = std::chrono::steady_clock::time_point{};
        _flow_stats = tx_flow_control_snapshot{};
        _flow_stats.ready_to_send = _ready_to_send;
    }

    tx_flow_control_snapshot get_tx_flow_control_stats() const
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        tx_flow_control_snapshot snapshot = _flow_stats;
        snapshot.ready_to_send = _ready_to_send;
        if (snapshot.monitoring_active) {
            const auto now = std::chrono::steady_clock::now();
            if (_monitor_start.time_since_epoch().count() != 0) {
                snapshot.monitor_elapsed_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - _monitor_start).count());
            }
            if (!_ready_to_send && _pause_start.time_since_epoch().count() != 0) {
                snapshot.fc_pause_total_us += static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - _pause_start).count());
            }
        }
        return snapshot;
    }

    void request_send_abort()
    {
        {
            std::lock_guard<std::mutex> lock(_flow_control_mutex);
            _stop_requested = true;
        }
        _flow_control_cv.notify_all();
    }

    size_t get_max_num_samps(void) const override
    {
        return _request_num_samples;
    }

    size_t get_num_channels(void) const override
    {
        return _channel;
    }

    void start_flow_control_thread()
    {
        _flow_control_running = true;
        _flow_control_thread = std::thread(&send_packet_streamer::flow_control_receiver, this);
    }

    void stop_flow_control_thread()
    {
        _flow_control_running = false;
        _flow_control_cv.notify_all();
        _async_event_cv.notify_all();

        if (_flow_control_thread.joinable()) {
            _flow_control_thread.join();
        }
    }

    void flow_control_receiver()
    {
        managed_recv_buffer::sptr buff;
        while (_flow_control_running) {
            buff = _stream_port->get_xport()->get_recv_buff(kFlowControlRecvTimeoutSec);
            if (!buff) {
                continue;
            }

            const uint32_t* packet_buff = buff->cast<const uint32_t*>();
            const uint32_t packet_len = packet_buff[0] & 0xFFFFFFu;
            if (packet_len != kTxFlowControlPacketBytesLegacy &&
                packet_len != kTxFlowControlPacketBytes) {
                continue;
            }
            uint32_t recv_info = packet_buff[4];

            if (recv_info == 0x5555) {
                update_flow_control_state(false, recv_info);
            } else if (recv_info == 0xAAAA) {
                update_flow_control_state(true, recv_info);
            } else if (recv_info == static_cast<uint32_t>(tx_async_event_code::burst_ack) ||
                       recv_info == static_cast<uint32_t>(tx_async_event_code::underflow) ||
                       recv_info == static_cast<uint32_t>(tx_async_event_code::sequence_error) ||
                       recv_info == static_cast<uint32_t>(tx_async_event_code::time_error)) {
                tx_async_event event;
                event.code = static_cast<tx_async_event_code>(recv_info);
                event.has_time = true;
                event.timestamp = (static_cast<uint64_t>(packet_buff[3]) << 32) |
                                  static_cast<uint64_t>(packet_buff[2]);
                {
                    std::lock_guard<std::mutex> lock(_async_event_mutex);
                    _async_events.push_back(event);
                }
                // OAI does not poll UHD TX async metadata.  Emit only error
                // events here when explicitly requested; burst ACKs remain
                // queued silently to avoid a 1 kHz log stream in per-TTI mode.
                if (recv_info != static_cast<uint32_t>(tx_async_event_code::burst_ack) &&
                    std::getenv("IQTAXI_TX_ASYNC_TRACE") != nullptr) {
                    std::cerr << "[IQTAXI_TX_ASYNC] code=0x" << std::hex
                              << recv_info << std::dec
                              << " tick=" << event.timestamp << std::endl;
                }
                _async_event_cv.notify_one();
            }
        }
    }

    bool recv_async_event(tx_async_event& event, double timeout) override
    {
        std::unique_lock<std::mutex> lock(_async_event_mutex);
        const auto wait_time = std::chrono::duration<double>(std::max(0.0, timeout));
        if (!_async_event_cv.wait_for(lock, wait_time, [this]() {
                return !_async_events.empty() || !_flow_control_running.load();
            })) {
            return false;
        }
        if (_async_events.empty()) {
            return false;
        }
        event = _async_events.front();
        _async_events.pop_front();
        return true;
    }

    void set_tx_source(uint32_t sel) override
    {
        _local_port->poke32(SET_TX_SOURCE_SEL, sel);
    }

    void set_stream_tx_start() override
    {
    }

    void set_stream_tx_stop() override
    {
    }

    void dds_ctrl(uint32_t phase_ctrl) override
    {
        (void)phase_ctrl;
    }

    size_t send(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        uint64_t& time_stamp,
        uint32_t sample_format) override {
        const tx_packet_metadata metadata{};
        return send_impl(buffs, nsamps_per_buff, time_stamp, sample_format, true, metadata);
    }

    size_t send_with_metadata(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        uint64_t& time_stamp,
        uint32_t sample_format,
        const tx_packet_metadata& metadata) override {
        return send_impl(buffs, nsamps_per_buff, time_stamp, sample_format, true, metadata);
    }

    size_t send_nonblocking(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        uint64_t& time_stamp,
        uint32_t sample_format) override {
        const tx_packet_metadata metadata{};
        return send_impl(buffs, nsamps_per_buff, time_stamp, sample_format, false, metadata);
    }

    size_t send_impl(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        uint64_t& time_stamp,
        uint32_t sample_format,
        bool wait_for_ready,
        const tx_packet_metadata& metadata) {
        size_t total_sent_samples = 0;
        uint64_t current_timestamp = time_stamp;

        for (size_t i = 0; i < buffs.size(); ++i) {
            const void* dest_ptr = buffs[i];
            size_t samples_left_to_send = nsamps_per_buff;

            while (samples_left_to_send > 0) {
                if (_stop_requested.load()) {
                    time_stamp += total_sent_samples;
                    return total_sent_samples;
                }
                if (!wait_until_send_ready()) {
                    if (wait_for_ready) {
                        continue;
                    }
                    time_stamp += total_sent_samples;
                    return total_sent_samples;
                }

                managed_send_buffer::sptr buff =
                    _stream_port->get_xport()->get_send_buff(0.1, nsamps_per_buff * 4 + 16);
                if (!buff) {
                    note_send_buff_timeout();
                    continue;
                }

                size_t max_samples_per_packet = (buff->size() - 16) / (2 * sizeof(int16_t));
                size_t to_send_samples = std::min<size_t>(max_samples_per_packet, samples_left_to_send);

                uint32_t* vrt_hdr = buff->cast<uint32_t*>();
                _hdr.timestamp = current_timestamp;
                const bool first_packet = total_sent_samples == 0;
                const bool final_packet =
                    (to_send_samples == samples_left_to_send) && (i + 1 == buffs.size());
                _hdr.seq = encode_tx_seq_flags(
                    static_cast<uint16_t>(_last_seq_out),
                    metadata.flags_valid,
                    metadata.has_time,
                    metadata.start_of_burst && first_packet,
                    metadata.end_of_burst && final_packet);
                _hdr.sid = 0x50;
                _hdr.packet_len = static_cast<uint32_t>(16 + to_send_samples * 2 * sizeof(int16_t));
                _hdr.magic_type = PACKET_TYPE_TX_IQ;
                _stream_port->serialize_hdr(vrt_hdr, _hdr);

                void* payload_ptr = vrt_hdr + 4;
                if (sample_format == MICRORF_FORMAT_FLOAT32) {
                    int16_t* dst_qi = static_cast<int16_t*>(payload_ptr);
                    const float* src_iq = static_cast<const float*>(dest_ptr);
                    copy_standard_iq_to_e100_qi_cs16(src_iq, to_send_samples, dst_qi);
                    dest_ptr = static_cast<const float*>(dest_ptr) + to_send_samples * 2;
                } else if (sample_format == MICRORF_FORMAT_INT16) {
                    int16_t* dst_qi = static_cast<int16_t*>(payload_ptr);
                    const int16_t* src_iq = static_cast<const int16_t*>(dest_ptr);
                    copy_standard_iq_to_e100_qi_cs16(src_iq, to_send_samples, dst_qi);
                    dest_ptr = static_cast<const int16_t*>(dest_ptr) + to_send_samples * 2;
                }

                buff->commit(16 + to_send_samples * 2 * sizeof(int16_t));

                samples_left_to_send -= to_send_samples;
                total_sent_samples += to_send_samples;
                current_timestamp += to_send_samples;

                note_packet_sent(_hdr.seq, _hdr.timestamp, to_send_samples);
                _last_seq_out++;
            }
        }
        time_stamp += total_sent_samples;
        return total_sent_samples;
    }

private:
    void update_flow_control_state(bool ready, uint32_t fc_word)
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        if (_flow_stats.monitoring_active) {
            _flow_stats.last_fc_word = fc_word;
        }
        if (_ready_to_send == ready) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (_flow_stats.monitoring_active && !ready) {
            _flow_stats.fc_pause_count++;
            _pause_start = now;
        } else if (_flow_stats.monitoring_active && _pause_start.time_since_epoch().count() != 0) {
            _flow_stats.fc_pause_total_us += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - _pause_start).count());
            _pause_start = std::chrono::steady_clock::time_point{};
        }

        _ready_to_send = ready;
        _flow_stats.ready_to_send = ready;
        _flow_control_cv.notify_all();
    }

    bool wait_until_send_ready()
    {
        std::unique_lock<std::mutex> lock(_flow_control_mutex);
        if (_ready_to_send) {
            return true;
        }

        if (_flow_stats.monitoring_active) {
            _flow_stats.fc_wait_count++;
        }
        if (!_flow_control_cv.wait_for(lock, kSendReadyWait, [this]() {
                return _ready_to_send || !_flow_control_running.load() || _stop_requested.load();
            })) {
            if (_flow_stats.monitoring_active) {
                _flow_stats.fc_wait_timeout_count++;
            }
            return false;
        }
        return _ready_to_send && !_stop_requested.load();
    }

    void note_send_buff_timeout()
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        if (_flow_stats.monitoring_active) {
            _flow_stats.send_buff_timeout_count++;
        }
    }

    void note_packet_sent(uint16_t seq, uint64_t timestamp, size_t samples)
    {
        std::lock_guard<std::mutex> lock(_flow_control_mutex);
        if (!_flow_stats.monitoring_active) {
            return;
        }
        _flow_stats.packets_sent++;
        _flow_stats.samples_sent += samples;
        _flow_stats.last_seq = seq;
        _flow_stats.last_timestamp = timestamp;
    }

private:
    get_buff_type _get_buff;
    sdr_header_t _hdr;
    const void* _copy_buff = nullptr;
    size_t _request_num_samples = 0;
    size_t _channel = 1;
    size_t _last_seq_ack = 0;
    size_t _fc_pkt_window = 0;
    local_ctrl::sptr _local_port, _stream_port;

    mutable std::mutex _flow_control_mutex;
    std::condition_variable _flow_control_cv;
    std::mutex _async_event_mutex;
    std::condition_variable _async_event_cv;
    std::deque<tx_async_event> _async_events;
    uint32_t _last_seq_out = 0;
    bool _ready_to_send{true};
    std::thread _flow_control_thread;
    std::atomic<bool> _flow_control_running{false};
    std::atomic<bool> _stop_requested{false};
    std::chrono::steady_clock::time_point _monitor_start{};
    std::chrono::steady_clock::time_point _pause_start{};
    tx_flow_control_snapshot _flow_stats{};
};

#endif // SOAPY_SUPER_SEND_PACKET_HANDLER_HPP
