#include "m300_tx_streamer.hpp"

#include "include/sdr/api/Device.hpp"
#include "m300_xdma_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#if (defined(__x86_64__) || defined(__i386__)) && defined(__GNUC__)
#include <immintrin.h>
#define M300_HAS_AVX2_PACKER 1
#else
#define M300_HAS_AVX2_PACKER 0
#endif

using sdr::core::managed_send_buffer;
using sdr::driver::M300_HDR_BYTES;
using sdr::driver::M300_MAGIC_TX;
using sdr::driver::m300_header;
using sdr::driver::store_le64;
using sdr::driver::write_header;

namespace {
constexpr double kCtrlTimeoutSec = 1.0;
constexpr size_t kAxisBytes = 16u;
constexpr size_t kSamplesPerAxisWord = 4u;
constexpr size_t kMaxAggregatePackets = 16u;

size_t align_up(size_t value, size_t alignment)
{
    return ((value + alignment - 1u) / alignment) * alignment;
}

void store_le32(uint8_t* p, uint32_t value)
{
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8u);
    p[2] = static_cast<uint8_t>(value >> 16u);
    p[3] = static_cast<uint8_t>(value >> 24u);
}

uint32_t pack_iq(int16_t i_sample, int16_t q_sample)
{
    return (static_cast<uint32_t>(static_cast<uint16_t>(i_sample)) << 16u) |
           static_cast<uint16_t>(q_sample);
}

int16_t float_to_i16(float value)
{
    // std::lround() is prohibitively expensive in the 2T hot path (four
    // calls per sample time).  Saturation plus a native float-to-int cast is
    // vectorizable and differs by at most one LSB from the former rounding.
    if (value >= 1.0f) {
        return 32767;
    }
    if (value <= -1.0f) {
        return -32767;
    }
    return static_cast<int16_t>(value * 32767.0f);
}

#if M300_HAS_AVX2_PACKER
__attribute__((target("avx2")))
__m128i pack_four_cf32(const float* input)
{
    const __m256 scale = _mm256_set1_ps(32767.0f);
    const __m256 minimum = _mm256_set1_ps(-1.0f);
    const __m256 maximum = _mm256_set1_ps(1.0f);
    __m256 value = _mm256_loadu_ps(input);
    value = _mm256_min_ps(_mm256_max_ps(value, minimum), maximum);
    const __m256i converted = _mm256_cvttps_epi32(_mm256_mul_ps(value, scale));
    const __m128i low = _mm256_castsi256_si128(converted);
    const __m128i high = _mm256_extracti128_si256(converted, 1);
    __m128i packed = _mm_packs_epi32(low, high);

    // Input is I,Q while custom_tx_upack consumes {I,Q}, which is stored as
    // low-word Q/high-word I on the little-endian H2C byte stream.
    constexpr int swap_iq = _MM_SHUFFLE(2, 3, 0, 1);
    packed = _mm_shufflelo_epi16(packed, swap_iq);
    return _mm_shufflehi_epi16(packed, swap_iq);
}

__attribute__((target("avx2")))
size_t pack_cf32_avx2(uint8_t* payload,
                      const float* channel0,
                      const float* channel1,
                      size_t samples,
                      size_t channels)
{
    const size_t vector_samples = samples & ~size_t(3u);
    for (size_t sample = 0u; sample < vector_samples; sample += 4u) {
        const __m128i packed0 = pack_four_cf32(channel0 + sample * 2u);
        if (channels == 2u) {
            const __m128i packed1 = pack_four_cf32(channel1 + sample * 2u);
            const __m128i interleaved01 = _mm_unpacklo_epi32(packed0, packed1);
            const __m128i interleaved23 = _mm_unpackhi_epi32(packed0, packed1);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(
                                 payload + sample * 2u * sizeof(uint32_t)),
                             interleaved01);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(
                                 payload + (sample * 2u + 4u) * sizeof(uint32_t)),
                             interleaved23);
        } else {
            _mm_storeu_si128(reinterpret_cast<__m128i*>(
                                 payload + sample * sizeof(uint32_t)),
                             packed0);
        }
    }
    return vector_samples;
}
#endif
}

m300_tx_streamer::m300_tx_streamer(const sdr::driver::m300_xdma_ctrl::sptr& ctrl,
                                   const sdr::core::xdma_zero_copy::sptr& tx_xport,
                                   size_t max_packet_bytes,
                                   std::function<void()> tx_active_callback)
    : _ctrl(ctrl)
    , _tx_xport(tx_xport)
    , _max_packet_bytes(max_packet_bytes)
    , _tx_active_callback(std::move(tx_active_callback))
{
    if (!_ctrl || !_tx_xport) {
        throw std::runtime_error("m300_tx_streamer requires valid ctrl and TX transport");
    }

    if (_max_packet_bytes <= M300_HDR_BYTES + kAxisBytes ||
        (_max_packet_bytes % kAxisBytes) != 0u) {
        throw std::runtime_error("M300 TX protocol packet size is invalid");
    }

    const size_t frame_bytes = _tx_xport->get_send_frame_size();
    if (frame_bytes < _max_packet_bytes) {
        throw std::runtime_error("M300 TX transport frame is too small");
    }
    _max_packets_per_write = std::min(
        kMaxAggregatePackets, frame_bytes / _max_packet_bytes);
    _max_wire_samples_per_packet =
        ((_max_packet_bytes - M300_HDR_BYTES) / kAxisBytes) * kSamplesPerAxisWord;
    _max_samples_per_packet = _max_wire_samples_per_packet;
    _max_samples_per_send = _max_samples_per_packet * _max_packets_per_write;
    _samples_per_packet = _max_samples_per_packet;
}

m300_tx_streamer::~m300_tx_streamer()
{
    try {
        set_stream_tx_stop();
    } catch (...) {
    }
}

size_t m300_tx_streamer::get_num_channels() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _channels;
}

size_t m300_tx_streamer::get_max_num_samps() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _max_samples_per_send;
}

void m300_tx_streamer::configure(size_t samples_per_packet,
                                 bool timed,
                                 uint8_t channel_enable)
{
    if (channel_enable != 0x01u && channel_enable != 0x03u) {
        throw std::runtime_error("M300 TX channel mask must be 0x01 or 0x03");
    }

    std::lock_guard<std::mutex> lock(_mutex);
    _channel_enable = channel_enable;
    _channels = channel_enable == 0x03u ? 2u : 1u;
    _max_samples_per_packet = _max_wire_samples_per_packet / _channels;
    _max_samples_per_send = _max_samples_per_packet * _max_packets_per_write;
    _samples_per_packet = std::max<size_t>(
        1u, std::min(samples_per_packet, _max_samples_per_packet));
    _ignore_timestamps = !timed;
    if (_active) {
        configure_hardware_locked();
    }
}

void m300_tx_streamer::set_tx_source(uint32_t source)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _tx_source = source;
    if (_active) {
        _ctrl->write_axi(kCenterCtrlBase + kRegTxSourceSel, source, kCtrlTimeoutSec);
    }
}

void m300_tx_streamer::set_stream_tx_start()
{
    std::lock_guard<std::mutex> lock(_mutex);
    start_send_worker_locked();
    _sequence = 0u;
    _active = true;
    _tx_active_callback_pending = static_cast<bool>(_tx_active_callback);
    configure_hardware_locked();
}

void m300_tx_streamer::set_stream_tx_stop()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _active = false;
    _tx_active_callback_pending = false;
    // Drain all queued H2C buffers before disconnecting the FPGA IQ source.
    stop_send_worker_locked();
    _ctrl->write_axi(kCenterCtrlBase + kRegTxSourceSel, kTxSourceOff, kCtrlTimeoutSec);
}

void m300_tx_streamer::dds_ctrl(uint32_t phase_ctrl)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _ctrl->write_axi(kCenterCtrlBase + kRegTxDdsFreqCtrl, phase_ctrl, kCtrlTimeoutSec);
}

size_t m300_tx_streamer::send(const buffs_type& buffs,
                              size_t nsamps_per_buff,
                              uint64_t& time_stamp,
                              uint32_t sample_format)
{
    return send_impl(buffs, nsamps_per_buff, time_stamp, sample_format, 1.0);
}

size_t m300_tx_streamer::send_nonblocking(const buffs_type& buffs,
                                          size_t nsamps_per_buff,
                                          uint64_t& time_stamp,
                                          uint32_t sample_format)
{
    return send_impl(buffs, nsamps_per_buff, time_stamp, sample_format, 0.0);
}

size_t m300_tx_streamer::send_impl(const buffs_type& buffs,
                                   size_t nsamps_per_buff,
                                   uint64_t& time_stamp,
                                   uint32_t sample_format,
                                   double timeout_sec)
{
    if (sample_format != MICRORF_FORMAT_FLOAT32 &&
        sample_format != MICRORF_FORMAT_INT16) {
        throw std::runtime_error("M300 TX supports CF32 and CS16 samples");
    }

    std::unique_lock<std::mutex> lock(_mutex);
    if (!_active || nsamps_per_buff == 0u) {
        return 0u;
    }
    if (buffs.size() != _channels) {
        throw std::runtime_error("M300 TX buffer count does not match configured channels");
    }
    for (size_t channel = 0u; channel < _channels; ++channel) {
        if (buffs[channel] == nullptr) {
            return 0u;
        }
    }

    size_t total_sent = 0u;

    while (total_sent < nsamps_per_buff) {
        managed_send_buffer::sptr buffer =
            _tx_xport->get_send_buff(timeout_sec, static_cast<uint32_t>(_max_packet_bytes));
        if (!buffer) {
            break;
        }

        uint8_t* aggregate = buffer->cast<uint8_t*>();
        const size_t aggregate_capacity = buffer->size();
        size_t aggregate_bytes = 0u;
        size_t aggregate_samples = 0u;
        size_t aggregate_packets = 0u;

        while (total_sent + aggregate_samples < nsamps_per_buff &&
               aggregate_packets < _max_packets_per_write) {
            const size_t packet_samples = std::min(
                _samples_per_packet,
                nsamps_per_buff - total_sent - aggregate_samples);
            const size_t samples_per_axis_word = kSamplesPerAxisWord / _channels;
            const size_t padded_samples = align_up(packet_samples, samples_per_axis_word);
            const size_t packet_bytes =
                M300_HDR_BYTES + padded_samples * _channels * sizeof(uint32_t);
            if (aggregate_bytes + packet_bytes > aggregate_capacity) {
                break;
            }

            fill_packet(aggregate + aggregate_bytes,
                        buffs,
                        total_sent + aggregate_samples,
                        packet_samples,
                        padded_samples,
                        time_stamp + total_sent + aggregate_samples,
                        sample_format);
            aggregate_bytes += packet_bytes;
            aggregate_samples += packet_samples;
            ++aggregate_packets;
        }

        if (aggregate_packets == 0u) {
            buffer->commit(0u);
            buffer.reset();
            break;
        }

        buffer->commit(aggregate_bytes);
        enqueue_send_buffer(std::move(buffer));
        total_sent += aggregate_samples;
    }

    time_stamp += total_sent;

    // Configuration writes happen before real TX clock/data activity.  A GUI
    // attenuation change works because it updates the AD9361 only after H2C
    // has populated the FPGA TX path.  Reproduce that ordering once for every
    // stream start, immediately after the first successful data submission.
    const bool notify_tx_active = total_sent > 0u &&
                                  _tx_active_callback_pending &&
                                  static_cast<bool>(_tx_active_callback);
    auto tx_active_callback = _tx_active_callback;
    if (notify_tx_active) {
        // Match the complete path taken by GNU Radio's runtime attenuation
        // setter.  Besides touching the RFIC it rewrites all FPGA TX controls
        // after data is already flowing.
        configure_hardware_locked();
        _tx_active_callback_pending = false;
    }
    lock.unlock();

    if (notify_tx_active) {
        try {
            tx_active_callback();
            std::cout << "M300 TX path and attenuation relatched after first IQ data"
                      << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "M300 TX post-data attenuation apply failed: "
                      << ex.what() << std::endl;
            std::lock_guard<std::mutex> retry_lock(_mutex);
            if (_active) {
                _tx_active_callback_pending = true;
            }
        }
    }
    return total_sent;
}

void m300_tx_streamer::start_send_worker_locked()
{
    if (_send_worker.joinable()) {
        return;
    }
    {
        std::lock_guard<std::mutex> queue_lock(_send_queue_mutex);
        _send_worker_stop = false;
        _send_queue.clear();
    }
    _send_worker = std::thread(&m300_tx_streamer::send_worker_func, this);
}

void m300_tx_streamer::stop_send_worker_locked()
{
    {
        std::lock_guard<std::mutex> queue_lock(_send_queue_mutex);
        _send_worker_stop = true;
    }
    _send_queue_condition.notify_all();
    if (_send_worker.joinable()) {
        _send_worker.join();
    }
}

void m300_tx_streamer::enqueue_send_buffer(managed_send_buffer::sptr buffer)
{
    {
        std::lock_guard<std::mutex> queue_lock(_send_queue_mutex);
        _send_queue.push_back(std::move(buffer));
    }
    _send_queue_condition.notify_one();
}

void m300_tx_streamer::send_worker_func()
{
    for (;;) {
        managed_send_buffer::sptr buffer;
        {
            std::unique_lock<std::mutex> queue_lock(_send_queue_mutex);
            _send_queue_condition.wait(queue_lock, [this]() {
                return _send_worker_stop || !_send_queue.empty();
            });
            if (_send_queue.empty()) {
                if (_send_worker_stop) {
                    return;
                }
                continue;
            }
            buffer = std::move(_send_queue.front());
            _send_queue.pop_front();
        }

        // Releasing the managed buffer performs the blocking XDMA write.
        // It runs here while the caller fills another buffer from the pool.
        buffer.reset();
    }
}

void m300_tx_streamer::configure_hardware_locked()
{
    _ctrl->write_axi(kCenterCtrlBase + kRegChannelEnable,
                     _channel_enable, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegTxSamplesPerPacket,
                     static_cast<uint32_t>(_samples_per_packet * _channels),
                     kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegIgnoreTxTimestamps,
                     _ignore_timestamps ? 1u : 0u, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegFcWindow, 0u, kCtrlTimeoutSec);
    _ctrl->write_axi(kCenterCtrlBase + kRegTxSourceSel,
                     _tx_source, kCtrlTimeoutSec);
}

void m300_tx_streamer::fill_packet(uint8_t* packet,
                                   const buffs_type& buffs,
                                   size_t sample_offset,
                                   size_t sample_count,
                                   size_t padded_samples,
                                   uint64_t timestamp,
                                   uint32_t sample_format)
{
    const size_t packet_bytes =
        M300_HDR_BYTES + padded_samples * _channels * sizeof(uint32_t);

    m300_header header;
    header.magic_type = M300_MAGIC_TX;
    header.seq = _sequence++;
    header.sid = 0u;
    header.length = static_cast<uint32_t>(packet_bytes);
    write_header(packet, header);
    store_le64(packet + 8u, timestamp);

    uint8_t* payload = packet + M300_HDR_BYTES;
    size_t first_scalar_sample = 0u;
#if M300_HAS_AVX2_PACKER
    if (sample_format == MICRORF_FORMAT_FLOAT32 &&
        __builtin_cpu_supports("avx2")) {
        const float* channel0 = static_cast<const float*>(buffs[0]) + sample_offset * 2u;
        const float* channel1 = _channels == 2u
            ? static_cast<const float*>(buffs[1]) + sample_offset * 2u
            : nullptr;
        first_scalar_sample = pack_cf32_avx2(
            payload, channel0, channel1, sample_count, _channels);
    }
#endif
    for (size_t sample = first_scalar_sample; sample < sample_count; ++sample) {
        for (size_t channel = 0; channel < _channels; ++channel) {
            int16_t i_sample = 0;
            int16_t q_sample = 0;
            const size_t input_sample = sample_offset + sample;
            if (sample_format == MICRORF_FORMAT_FLOAT32) {
                const float* input = static_cast<const float*>(buffs[channel]);
                i_sample = float_to_i16(input[input_sample * 2u]);
                q_sample = float_to_i16(input[input_sample * 2u + 1u]);
            } else {
                const int16_t* input = static_cast<const int16_t*>(buffs[channel]);
                i_sample = input[input_sample * 2u];
                q_sample = input[input_sample * 2u + 1u];
            }
            const size_t wire_sample = sample * _channels + channel;
            store_le32(payload + wire_sample * sizeof(uint32_t),
                       pack_iq(i_sample, q_sample));
        }
    }

    // The old implementation cleared the complete 16 KiB packet before
    // immediately overwriting it.  At 2T this added another ~470 MiB/s of
    // memory traffic.  Only the final AXI padding slots need clearing.
    const size_t payload_samples = sample_count * _channels;
    const size_t padded_payload_samples = padded_samples * _channels;
    if (payload_samples < padded_payload_samples) {
        std::memset(payload + payload_samples * sizeof(uint32_t),
                    0,
                    (padded_payload_samples - payload_samples) * sizeof(uint32_t));
    }
}
