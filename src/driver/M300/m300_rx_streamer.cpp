#include "m300_rx_streamer.hpp"

#include "include/sdr/api/Device.hpp"
#include "../transport/local_regs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <thread>

#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
#include <immintrin.h>
#define M300_HAS_AVX2_DEMUX 1
#else
#define M300_HAS_AVX2_DEMUX 0
#endif

using sdr::core::managed_recv_buffer;
using sdr::driver::M300_HDR_BYTES;
using sdr::driver::M300_MAGIC_RX;
using sdr::driver::M300_REG_RX_SOURCE_SEL;

namespace {
constexpr double kCtrlTimeoutSec = 1.0;
constexpr double kRecvTimeoutSec = 0.2;

uint32_t load_le32_local(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

#if M300_HAS_AVX2_DEMUX
__attribute__((target("avx2")))
size_t demux_dual_avx2(const uint8_t* source,
                       void* dest0,
                       void* dest1,
                       size_t samples,
                       uint32_t sample_format)
{
    const size_t vector_samples = samples & ~size_t(3u);
    const __m256 scale = _mm256_set1_ps(1.0f / 32768.0f);
    for (size_t sample = 0u; sample < vector_samples; sample += 4u) {
        const __m128i input01 = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(source + sample * 2u * sizeof(uint32_t)));
        const __m128i input23 = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(source + (sample * 2u + 4u) * sizeof(uint32_t)));
        const __m128i channel0_01 = _mm_shuffle_epi32(
            input01, _MM_SHUFFLE(2, 0, 2, 0));
        const __m128i channel0_23 = _mm_shuffle_epi32(
            input23, _MM_SHUFFLE(2, 0, 2, 0));
        const __m128i channel1_01 = _mm_shuffle_epi32(
            input01, _MM_SHUFFLE(3, 1, 3, 1));
        const __m128i channel1_23 = _mm_shuffle_epi32(
            input23, _MM_SHUFFLE(3, 1, 3, 1));
        const __m128i channel0 = _mm_unpacklo_epi64(channel0_01, channel0_23);
        const __m128i channel1 = _mm_unpacklo_epi64(channel1_01, channel1_23);

        if (sample_format == MICRORF_FORMAT_INT16) {
            _mm_storeu_si128(reinterpret_cast<__m128i*>(
                                 static_cast<int16_t*>(dest0) + sample * 2u),
                             channel0);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(
                                 static_cast<int16_t*>(dest1) + sample * 2u),
                             channel1);
        } else {
            const __m256 float0 = _mm256_mul_ps(
                _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(channel0)), scale);
            const __m256 float1 = _mm256_mul_ps(
                _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(channel1)), scale);
            _mm256_storeu_ps(static_cast<float*>(dest0) + sample * 2u, float0);
            _mm256_storeu_ps(static_cast<float*>(dest1) + sample * 2u, float1);
        }
    }
    return vector_samples;
}
#endif
}

m300_rx_streamer::m300_rx_streamer(const sdr::driver::m300_xdma_ctrl::sptr& ctrl,
                                   const sdr::core::xdma_zero_copy::sptr& rx_xport,
                                   uint32_t packet_bytes)
    : _ctrl(ctrl)
    , _rx_xport(rx_xport)
    , _packet_bytes(packet_bytes)
    , _payload_bytes(packet_bytes - M300_HDR_BYTES)
    , _request_num_samples((packet_bytes - M300_HDR_BYTES) / sizeof(uint32_t))
{
    if (!_ctrl || !_rx_xport) {
        throw std::runtime_error("m300_rx_streamer requires valid ctrl and RX transport");
    }
    if (packet_bytes <= M300_HDR_BYTES || (packet_bytes & 0xfu) != 0u) {
        throw std::runtime_error("invalid M300 RX packet size");
    }
}

size_t m300_rx_streamer::get_num_channels(void) const
{
    return _channels;
}

size_t m300_rx_streamer::get_request_num_samps(void) const
{
    return _request_num_samples;
}

void m300_rx_streamer::set_rx_enable_chan(uint8_t chans) const
{
    if (chans != 0x01u && chans != 0x03u) {
        throw std::runtime_error("M300 RX channel mask must be 0x01 or 0x03");
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _channel_enable = chans;
    _channels = count_enabled_channels(chans);
    _ctrl->write_axi(kCenterCtrlBase + kRegChannelEnable, chans, kCtrlTimeoutSec);
}

void m300_rx_streamer::set_rx_mode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _rx_mode = mode;
    _ctrl->write_axi(kCenterCtrlBase + kRegRxMode, mode, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegRxModeStrobe, 1u, kCtrlTimeoutSec);
}

void m300_rx_streamer::set_rx_mode_exit()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _ctrl->write_axi(kCenterCtrlBase + kRegModeExit, 1u, kCtrlTimeoutSec);
}

void m300_rx_streamer::set_stream_rx_start()
{
    std::lock_guard<std::mutex> lock(_mutex);
    start_stream_locked();
}

void m300_rx_streamer::set_stream_rx_stop()
{
    std::lock_guard<std::mutex> lock(_mutex);
    stop_stream_locked();
}

void m300_rx_streamer::set_max_sample_nums_per_packet(uint32_t sample_nums)
{
    (void)sample_nums;
    std::lock_guard<std::mutex> lock(_mutex);
    _ctrl->write_axi(kCenterCtrlBase + kRegMaxSampleBytesPerPacket,
                     _payload_bytes, kCtrlTimeoutSec);
}

void m300_rx_streamer::set_rx_sample_nums_per_packet(uint32_t sample_nums)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const uint32_t bytes = sample_nums ? sample_nums * sizeof(uint32_t) : _payload_bytes;
    const uint32_t aligned = std::min<uint32_t>(bytes & ~0xfu, _payload_bytes);
    _ctrl->write_axi(kCenterCtrlBase + kRegRxSampleBytes,
                     aligned ? aligned : _payload_bytes, kCtrlTimeoutSec);
}

size_t m300_rx_streamer::set_recv_param(uint8_t rx_mode,
                                        const size_t nsamps_per_buff,
                                        uint64_t& time_stamp,
                                        uint8_t stream_start,
                                        uint8_t stream_stop)
{
    (void)time_stamp;
    std::lock_guard<std::mutex> lock(_mutex);
    _request_num_samples = nsamps_per_buff;
    _rx_mode = rx_mode;

    if (rx_mode == STREAM_MODE && stream_start && !stream_stop) {
        start_stream_locked();
    } else if (rx_mode == STREAM_MODE && !stream_start && stream_stop) {
        stop_stream_locked();
    } else if (rx_mode == PACKET_MODE || rx_mode == SYNC_MODE) {
        _ctrl->write_axi(kCenterCtrlBase + kRegRxMode, rx_mode, kCtrlTimeoutSec);
        _ctrl->write_axi(kCenterCtrlBase + kRegRxModeStrobe, 1u, kCtrlTimeoutSec);
    }

    return 0;
}

size_t m300_rx_streamer::recv(const buffs_type& buffs,
                              const size_t nsamps_per_buff,
                              uint64_t& time_stamp,
                              uint32_t sample_format)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (buffs.size() == 0u || nsamps_per_buff == 0u) {
        return 0u;
    }
    if (buffs.size() > 1u) {
        if (_channel_enable != 0x03u || buffs.size() != 2u ||
            buffs[0] == nullptr || buffs[1] == nullptr) {
            throw std::runtime_error(
                "M300 RX separate buffers require channel mask 0x03 and two buffers");
        }

        size_t received = 0u;
        bool set_timestamp = false;
        while (received < nsamps_per_buff) {
            const size_t payload_words = _payload_bytes / sizeof(uint32_t);
            const size_t payload_frames = payload_words / 2u;
            const size_t remaining_frames = nsamps_per_buff - received;

            // Fetch several payloads with one ring ioctl before deinterleaving
            // them.  The former dual-channel path acquired a managed buffer
            // for every 16 KiB packet, which became visible at 2R2T/61.44 MS/s.
            if (!_pending_buff && payload_frames != 0u &&
                remaining_frames >= payload_frames) {
                const size_t max_packets = std::min<size_t>(
                    remaining_frames / payload_frames, 64u);
                _burst_payload.resize(max_packets * _payload_bytes);
                uint64_t first_timestamp = 0u;
                uint64_t first_sequence = 0u;
                const size_t packets = _rx_xport->recv_payload_burst(
                    _burst_payload.data(),
                    _burst_payload.size(),
                    _payload_bytes,
                    max_packets,
                    first_timestamp,
                    first_sequence,
                    kRecvTimeoutSec);
                if (packets != 0u) {
                    const size_t burst_frames = packets * payload_frames;
                    note_packet_sequence(
                        static_cast<uint16_t>(first_sequence), packets);
                    if (!set_timestamp) {
                        time_stamp = first_timestamp;
                        set_timestamp = true;
                    }

                    void* burst_out0 = sample_format == MICRORF_FORMAT_FLOAT32
                        ? static_cast<void*>(
                              static_cast<float*>(buffs[0]) + received * 2u)
                        : static_cast<void*>(
                              static_cast<int16_t*>(buffs[0]) + received * 2u);
                    void* burst_out1 = sample_format == MICRORF_FORMAT_FLOAT32
                        ? static_cast<void*>(
                              static_cast<float*>(buffs[1]) + received * 2u)
                        : static_cast<void*>(
                              static_cast<int16_t*>(buffs[1]) + received * 2u);
                    const size_t copied = demux_dual_channel_samples(
                        _burst_payload.data(),
                        burst_out0,
                        burst_out1,
                        burst_frames,
                        sample_format);
                    if (copied == 0u) {
                        break;
                    }
                    received += copied;
                    continue;
                }
            }

            if (!_pending_buff && !load_next_packet(kRecvTimeoutSec)) {
                break;
            }

            // Dual-channel cpack data is frame-interleaved as CH0, CH1.  A
            // packet is AXI-word aligned, so every packet starts on CH0.
            const size_t available_words = _pending_words - _pending_word_offset;
            const size_t available_frames = available_words / 2u;
            if (available_frames == 0u) {
                _pending_buff.reset();
                _pending_payload = nullptr;
                _pending_words = 0u;
                _pending_word_offset = 0u;
                continue;
            }

            if (!set_timestamp) {
                time_stamp = _pending_timestamp + (_pending_word_offset / 2u);
                set_timestamp = true;
            }

            const size_t frames = std::min(
                nsamps_per_buff - received, available_frames);
            void* out0 = sample_format == MICRORF_FORMAT_FLOAT32
                ? static_cast<void*>(static_cast<float*>(buffs[0]) + received * 2u)
                : static_cast<void*>(static_cast<int16_t*>(buffs[0]) + received * 2u);
            void* out1 = sample_format == MICRORF_FORMAT_FLOAT32
                ? static_cast<void*>(static_cast<float*>(buffs[1]) + received * 2u)
                : static_cast<void*>(static_cast<int16_t*>(buffs[1]) + received * 2u);

            const size_t copied = copy_dual_channel_samples(
                out0, out1, frames, sample_format);
            if (copied == 0u) {
                break;
            }

            received += copied;
            _pending_word_offset += copied * 2u;
            if (_pending_word_offset >= _pending_words) {
                _pending_buff.reset();
                _pending_payload = nullptr;
                _pending_words = 0u;
                _pending_word_offset = 0u;
            }
        }
        // GNU Radio expects the number of samples produced on each port.
        return received;
    }

    size_t total_samples = 0;
    bool set_timestamp = false;

    for (size_t i = 0; i < buffs.size(); ++i) {
        void* dest = buffs[i];
        size_t left = nsamps_per_buff;
        while (left > 0) {
            const size_t payload_words = _payload_bytes / sizeof(uint32_t);
            if (!_pending_buff && sample_format == MICRORF_FORMAT_INT16 &&
                payload_words != 0u && left >= payload_words) {
                uint64_t first_timestamp = 0;
                uint64_t first_sequence = 0;
                const size_t max_packets = left / payload_words;
                const size_t packets = _rx_xport->recv_payload_burst(
                    dest, left * sizeof(uint32_t), _payload_bytes,
                    max_packets, first_timestamp, first_sequence, kRecvTimeoutSec);
                if (packets > 0u) {
                    const size_t copied = packets * payload_words;
                    note_packet_sequence(static_cast<uint16_t>(first_sequence), packets);
                    if (!set_timestamp) {
                        time_stamp = first_timestamp;
                        set_timestamp = true;
                    }
                    dest = static_cast<void*>(
                        static_cast<int16_t*>(dest) + copied * 2u);
                    total_samples += copied;
                    left -= copied;
                    continue;
                }
            }

            if (!_pending_buff && !load_next_packet(kRecvTimeoutSec))
                break;

            if (!set_timestamp) {
                time_stamp = _pending_timestamp +
                             (_pending_word_offset / std::max<size_t>(_channels, 1u));
                set_timestamp = true;
            }

            const size_t available = _pending_words - _pending_word_offset;
            const size_t to_copy = std::min(left, available);
            const size_t copied = copy_samples(dest, to_copy, sample_format);
            if (copied == 0)
                break;

            if (sample_format == MICRORF_FORMAT_FLOAT32) {
                dest = static_cast<void*>(static_cast<float*>(dest) + copied * 2u);
            } else {
                dest = static_cast<void*>(static_cast<int16_t*>(dest) + copied * 2u);
            }

            total_samples += copied;
            left -= copied;
            _pending_word_offset += copied;
            if (_pending_word_offset >= _pending_words) {
                _pending_buff.reset();
                _pending_payload = nullptr;
                _pending_words = 0;
                _pending_word_offset = 0;
            }
        }
    }

    return total_samples;
}

size_t m300_rx_streamer::recv_from_fifo(void* const* buffs,
                                        const size_t numElems,
                                        int& flags,
                                        long long& timeNs,
                                        const long timeoutUs)
{
    (void)flags;
    (void)timeoutUs;
    uint64_t timestamp = 0;
    const size_t got = recv(sdr::api::ref_vector<void*>(buffs, 1),
                            numElems, timestamp, _sample_format);
    timeNs = static_cast<long long>(
        (static_cast<long double>(timestamp) * 1e9L) / std::max<size_t>(_sample_rate, 1u));
    return got;
}

size_t m300_rx_streamer::recv_from_fifo_ticks(void* const* buffs,
                                              const size_t numElems,
                                              int& flags,
                                              uint64_t& timeTicks,
                                              const long timeoutUs)
{
    (void)timeoutUs;
    flags = 0;
    return recv(sdr::api::ref_vector<void*>(buffs, 1),
                numElems, timeTicks, _sample_format);
}

size_t m300_rx_streamer::recv_fifo(const buffs_type& buffs,
                                   const size_t nsamps_per_buff)
{
    uint64_t timestamp = 0;
    return recv(buffs, nsamps_per_buff, timestamp, _sample_format);
}

void m300_rx_streamer::rx_thread_func(uint32_t buff_size_samples)
{
    (void)buff_size_samples;
}

void m300_rx_streamer::_start(void)
{
    set_stream_rx_start();
}

void m300_rx_streamer::_stop(void)
{
    set_stream_rx_stop();
}

void m300_rx_streamer::enable_xfft(const size_t fft_point)
{
    (void)fft_point;
}

void m300_rx_streamer::set_sample_format(uint32_t format)
{
    _sample_format = format;
}

void m300_rx_streamer::set_sampleRate(size_t sampleRate)
{
    _sample_rate = sampleRate;
}

void m300_rx_streamer::prepare_framer_locked()
{
    _payload_bytes = _packet_bytes - M300_HDR_BYTES;
    _ctrl->write_axi(kCenterCtrlBase + kRegModeExit, 1u, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegRxSampleBytes, _payload_bytes, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegMaxSampleBytesPerPacket, _payload_bytes, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegChannelEnable, _channel_enable, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegDmaPktPerBurst, _dma_pkt_per_burst, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegRxMode, _rx_mode, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegRxModeStrobe, 1u, kCtrlTimeoutSec);
}

void m300_rx_streamer::start_stream_locked()
{
    // Keep both FPGA producers stopped while the host-side C2H ring is being
    // prepared. Starting the producer first can race the XDMA engine setup and
    // makes a subsequent ring STOP much harder to quiesce.
    _ctrl->stop_rx(kCtrlTimeoutSec);
    _ctrl->write_reg(M300_REG_RX_SOURCE_SEL, 0u, kCtrlTimeoutSec);
    _ctrl->set_rx_packet_bytes(_packet_bytes, kCtrlTimeoutSec);
    _ctrl->clear_counters();
    // STREAM_ENABLE, RX_SOURCE_SEL and RX_PACKET_BYTES each assert the FPGA
    // rx_restart signal, which clears the IQ framework. Complete all of those
    // writes first and wait for the restart pulse to end before programming
    // the framer mode. Otherwise the later clear returns the mode FSM to IDLE
    // and the stream-start pulse produces no C2H packets.
    _ctrl->start_rx(kCtrlTimeoutSec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    prepare_framer_locked();
    clear_pending_locked();
    reset_continuity_stats();

    try {
        _rx_xport->start_recv(kCtrlTimeoutSec);
        _ctrl->write_axi(kCenterCtrlBase + kRegStreamStart, 1u, kCtrlTimeoutSec);
        trace_status_locked("rx_stream_started");
    } catch (...) {
        try {
            _ctrl->write_axi(kCenterCtrlBase + kRegModeExit, 1u, kCtrlTimeoutSec);
        } catch (...) {
        }
        try {
            _ctrl->stop_rx(kCtrlTimeoutSec);
        } catch (...) {
        }
        _rx_xport->stop_recv();
        throw;
    }
}

void m300_rx_streamer::stop_stream_locked()
{
    // Stop FPGA packet production first, let the final AXI beat drain, then
    // tear down the kernel ring. The previous reverse order was the main path
    // that could leave C2H_1 busy across GNU Radio runs.
    std::exception_ptr control_error;
    try {
        _ctrl->write_axi(kCenterCtrlBase + kRegModeExit, 1u, kCtrlTimeoutSec);
    } catch (...) {
        control_error = std::current_exception();
    }
    try {
        _ctrl->stop_rx(kCtrlTimeoutSec);
    } catch (...) {
        if (!control_error)
            control_error = std::current_exception();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    _rx_xport->stop_recv();
    clear_pending_locked();

    if (control_error)
        std::rethrow_exception(control_error);
}

void m300_rx_streamer::clear_pending_locked()
{
    _pending_buff.reset();
    _pending_payload = nullptr;
    _pending_words = 0;
    _pending_word_offset = 0;
    _have_pending_timestamp = false;
}

void m300_rx_streamer::trace_status_locked(const char* where)
{
    if (std::getenv("M300_XDMA_TRACE") == nullptr) {
        return;
    }

    try {
        const uint32_t stream_enable = _ctrl->read_reg(sdr::driver::M300_REG_STREAM_ENABLE,
                                                       kCtrlTimeoutSec);
        const uint32_t rx_packet_bytes = _ctrl->read_reg(sdr::driver::M300_REG_RX_PACKET_BYTES,
                                                         kCtrlTimeoutSec);
        const uint32_t rx_source_sel = _ctrl->read_reg(M300_REG_RX_SOURCE_SEL, kCtrlTimeoutSec);
        const uint32_t xdma_status = _ctrl->read_reg(kRegXdmaStatus, kCtrlTimeoutSec);
        const uint32_t rx_sample_bytes = _ctrl->read_axi(kCenterCtrlBase + kRegRxSampleBytes, kCtrlTimeoutSec);
        const uint32_t max_sample_bytes = _ctrl->read_axi(kCenterCtrlBase + kRegMaxSampleBytesPerPacket, kCtrlTimeoutSec);
        const uint32_t rx_mode = _ctrl->read_axi(kCenterCtrlBase + kRegRxMode, kCtrlTimeoutSec);
        const uint32_t channel_enable = _ctrl->read_axi(kCenterCtrlBase + kRegChannelEnable, kCtrlTimeoutSec);
        const uint32_t dma_pkt_per_burst = _ctrl->read_axi(kCenterCtrlBase + kRegDmaPktPerBurst, kCtrlTimeoutSec);

        std::fprintf(stderr,
                     "%s local.stream_enable=0x%x local.rx_packet_bytes=0x%x local.rx_source_sel=0x%x local.xdma_status=0x%x center.rx_sample_bytes=0x%x center.max_sample_bytes=0x%x center.rx_mode=0x%x center.channel_enable=0x%x center.dma_pkt_per_burst=0x%x\n",
                     where,
                     stream_enable,
                     rx_packet_bytes,
                     rx_source_sel,
                     xdma_status,
                     rx_sample_bytes,
                     max_sample_bytes,
                     rx_mode,
                     channel_enable,
                     dma_pkt_per_burst);
    } catch (...) {
        std::fprintf(stderr, "%s status trace failed\n", where);
    }
}

void m300_rx_streamer::note_packet_sequence(uint16_t first_seq, size_t packets)
{
    if (packets == 0u) {
        return;
    }

    if (!_have_seq) {
        _have_seq = true;
    } else if (first_seq != _expected_seq) {
        const uint16_t lost = static_cast<uint16_t>(first_seq - _expected_seq);
        _continuity.seq_jumps++;
        _continuity.lost_packets += lost;
        if (std::getenv("M300_XDMA_TRACE") != nullptr && _continuity.seq_jumps <= 8u) {
            std::fprintf(stderr,
                         "m300_rx_streamer seq_jump expected=%u got=%u lost=%u\n",
                         static_cast<unsigned>(_expected_seq),
                         static_cast<unsigned>(first_seq),
                         static_cast<unsigned>(lost));
        }
    }

    _continuity.packets += packets;
    _continuity.last_seq = static_cast<uint16_t>(first_seq + packets - 1u);
    _expected_seq = static_cast<uint16_t>(first_seq + packets);
}

bool m300_rx_streamer::load_next_packet(double timeout_sec)
{
    for (;;) {
        managed_recv_buffer::sptr buff = _rx_xport->get_recv_buff(timeout_sec);
        if (!buff)
            return false;
        if (buff->size() < M300_HDR_BYTES)
            continue;

        const uint8_t* p = static_cast<const uint8_t*>(buff->cast<const void*>());
        const auto hdr = sdr::driver::parse_header(p);
        if (hdr.magic_type != M300_MAGIC_RX)
            continue;
        if (hdr.length < M300_HDR_BYTES || hdr.length > _packet_bytes)
            continue;

        _pending_timestamp = sdr::driver::load_le64(p + 8);
        _pending_hdr = hdr;
        note_packet_sequence(hdr.seq, 1u);
        _pending_buff = std::move(buff);
        _pending_payload = p + M300_HDR_BYTES;
        const size_t actual_payload = std::min<size_t>(
            _pending_hdr.length, _pending_buff->size()) - M300_HDR_BYTES;
        _pending_words = actual_payload / sizeof(uint32_t);
        _pending_word_offset = 0;
        _have_pending_timestamp = true;
        return _pending_words != 0u;
    }
}

m300_rx_streamer::continuity_stats m300_rx_streamer::get_continuity_stats() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _continuity;
}

void m300_rx_streamer::reset_continuity_stats()
{
    _continuity = continuity_stats{};
    _have_seq = false;
    _expected_seq = 0;
}

size_t m300_rx_streamer::copy_samples(void* dest,
                                      size_t samples,
                                      uint32_t sample_format)
{
    if (!dest || !_pending_payload || samples == 0)
        return 0;

    const uint8_t* src = _pending_payload + _pending_word_offset * sizeof(uint32_t);
    if (sample_format == MICRORF_FORMAT_FLOAT32) {
        float* out = static_cast<float*>(dest);
        for (size_t i = 0; i < samples; ++i) {
            const uint32_t raw = load_le32_local(src + i * sizeof(uint32_t));
            const int16_t i_sample = static_cast<int16_t>(raw & 0xffffu);
            const int16_t q_sample = static_cast<int16_t>((raw >> 16) & 0xffffu);
            out[2u * i + 0u] = static_cast<float>(i_sample) / 32768.0f;
            out[2u * i + 1u] = static_cast<float>(q_sample) / 32768.0f;
        }
        return samples;
    }

    if (sample_format == MICRORF_FORMAT_INT16) {
        std::memcpy(dest, src, samples * sizeof(uint32_t));
        return samples;
    }

    return 0;
}

size_t m300_rx_streamer::copy_channel_samples(void* dest,
                                              size_t samples,
                                              size_t channel,
                                              uint32_t sample_format)
{
    if (!dest || !_pending_payload || samples == 0u || channel > 1u) {
        return 0u;
    }

    const size_t first_word = _pending_word_offset + channel;
    if (sample_format == MICRORF_FORMAT_FLOAT32) {
        float* out = static_cast<float*>(dest);
        for (size_t i = 0; i < samples; ++i) {
            const uint32_t raw = load_le32_local(
                _pending_payload + (first_word + i * 2u) * sizeof(uint32_t));
            const int16_t i_sample = static_cast<int16_t>(raw & 0xffffu);
            const int16_t q_sample = static_cast<int16_t>((raw >> 16u) & 0xffffu);
            out[2u * i] = static_cast<float>(i_sample) / 32768.0f;
            out[2u * i + 1u] = static_cast<float>(q_sample) / 32768.0f;
        }
        return samples;
    }

    if (sample_format == MICRORF_FORMAT_INT16) {
        int16_t* out = static_cast<int16_t*>(dest);
        for (size_t i = 0; i < samples; ++i) {
            const uint32_t raw = load_le32_local(
                _pending_payload + (first_word + i * 2u) * sizeof(uint32_t));
            out[2u * i] = static_cast<int16_t>(raw & 0xffffu);
            out[2u * i + 1u] = static_cast<int16_t>((raw >> 16u) & 0xffffu);
        }
        return samples;
    }

    return 0u;
}

size_t m300_rx_streamer::copy_dual_channel_samples(void* dest0,
                                                   void* dest1,
                                                   size_t samples,
                                                   uint32_t sample_format)
{
    if (!_pending_payload) {
        return 0u;
    }
    const uint8_t* source =
        _pending_payload + _pending_word_offset * sizeof(uint32_t);
    return demux_dual_channel_samples(
        source, dest0, dest1, samples, sample_format);
}

size_t m300_rx_streamer::demux_dual_channel_samples(const uint8_t* source,
                                                    void* dest0,
                                                    void* dest1,
                                                    size_t samples,
                                                    uint32_t sample_format)
{
    if (!source || !dest0 || !dest1 || samples == 0u ||
        (sample_format != MICRORF_FORMAT_FLOAT32 &&
         sample_format != MICRORF_FORMAT_INT16)) {
        return 0u;
    }

    size_t first_scalar_sample = 0u;
#if M300_HAS_AVX2_DEMUX
    if (__builtin_cpu_supports("avx2")) {
        first_scalar_sample = demux_dual_avx2(
            source, dest0, dest1, samples, sample_format);
    }
#endif

    for (size_t sample = first_scalar_sample; sample < samples; ++sample) {
        for (size_t channel = 0u; channel < 2u; ++channel) {
            const uint32_t raw = load_le32_local(
                source + (sample * 2u + channel) * sizeof(uint32_t));
            if (sample_format == MICRORF_FORMAT_FLOAT32) {
                float* output = channel == 0u
                    ? static_cast<float*>(dest0)
                    : static_cast<float*>(dest1);
                output[sample * 2u] =
                    static_cast<float>(static_cast<int16_t>(raw & 0xffffu)) / 32768.0f;
                output[sample * 2u + 1u] =
                    static_cast<float>(static_cast<int16_t>((raw >> 16u) & 0xffffu)) / 32768.0f;
            } else {
                int16_t* output = channel == 0u
                    ? static_cast<int16_t*>(dest0)
                    : static_cast<int16_t*>(dest1);
                output[sample * 2u] = static_cast<int16_t>(raw & 0xffffu);
                output[sample * 2u + 1u] =
                    static_cast<int16_t>((raw >> 16u) & 0xffffu);
            }
        }
    }
    return samples;
}

size_t m300_rx_streamer::count_enabled_channels(uint8_t channel_mask)
{
    size_t count = 0;
    for (uint8_t mask = channel_mask; mask != 0; mask >>= 1)
        count += mask & 0x1u;
    return std::max<size_t>(count, 1u);
}
