#ifndef SOAPY_M300_RX_STREAMER_HPP
#define SOAPY_M300_RX_STREAMER_HPP

#include "m300_xdma_ctrl.hpp"
#include "m300_xdma_protocol.hpp"
#include "include/sdr/api/DataStream.hpp"
#include "include/sdr/core/xdma_zero_copy.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

class m300_rx_streamer : public sdr::api::rx_streamer
{
public:
    struct continuity_stats {
        uint64_t packets = 0;
        uint64_t seq_jumps = 0;
        uint64_t lost_packets = 0;
        uint16_t last_seq = 0;
    };

    m300_rx_streamer(const sdr::driver::m300_xdma_ctrl::sptr& ctrl,
                     const sdr::core::xdma_zero_copy::sptr& rx_xport,
                     uint32_t packet_bytes);
    ~m300_rx_streamer() override = default;

    size_t get_num_channels(void) const override;
    size_t get_request_num_samps(void) const override;
    void set_rx_enable_chan(uint8_t chans) const override;

    void set_rx_mode(uint8_t mode) override;
    void set_rx_mode_exit() override;
    void set_stream_rx_start() override;
    void set_stream_rx_stop() override;

    void set_max_sample_nums_per_packet(uint32_t sample_nums) override;
    void set_rx_sample_nums_per_packet(uint32_t sample_nums) override;

    size_t set_recv_param(uint8_t rx_mode,
                          const size_t nsamps_per_buff,
                          uint64_t& time_stamp,
                          uint8_t stream_start,
                          uint8_t stream_stop) override;

    size_t recv(const buffs_type& buffs,
                const size_t nsamps_per_buff,
                uint64_t& time_stamp,
                uint32_t sample_format) override;

    size_t recv_from_fifo(void* const* buffs,
                          const size_t numElems,
                          int& flags,
                          long long& timeNs,
                          const long timeoutUs) override;

    size_t recv_from_fifo_ticks(void* const* buffs,
                                const size_t numElems,
                                int& flags,
                                uint64_t& timeTicks,
                                const long timeoutUs) override;

    size_t recv_fifo(const buffs_type& buffs,
                     const size_t nsamps_per_buff) override;

    void rx_thread_func(uint32_t buff_size_samples) override;
    void _start(void) override;
    void _stop(void) override;
    void enable_xfft(const size_t fft_point) override;
    void set_sample_format(uint32_t format) override;
    void set_sampleRate(size_t sampleRate) override;
    continuity_stats get_continuity_stats() const;
    void reset_continuity_stats();

private:
    static constexpr uint32_t kCenterCtrlBase = 0x44a10000u;
    static constexpr uint32_t kRegRxSampleBytes = 10u * 4u;
    static constexpr uint32_t kRegRxMode = 14u * 4u;
    static constexpr uint32_t kRegRxModeStrobe = 15u * 4u;
    static constexpr uint32_t kRegModeExit = 16u * 4u;
    static constexpr uint32_t kRegStreamStart = 17u * 4u;
    static constexpr uint32_t kRegChannelEnable = 18u * 4u;
    static constexpr uint32_t kRegDmaPktPerBurst = 19u * 4u;
    static constexpr uint32_t kRegMaxSampleBytesPerPacket = 27u * 4u;
    static constexpr uint32_t kRegXdmaStatus = 0x00030000u;

    void prepare_framer_locked();
    void start_stream_locked();
    void stop_stream_locked();
    void clear_pending_locked();
    void trace_status_locked(const char* where);
    void note_packet_sequence(uint16_t first_seq, size_t packets);
    bool load_next_packet(double timeout_sec);
    size_t copy_samples(void* dest,
                        size_t samples,
                        uint32_t sample_format);
    size_t copy_channel_samples(void* dest,
                                size_t samples,
                                size_t channel,
                                uint32_t sample_format);
    size_t copy_dual_channel_samples(void* dest0,
                                     void* dest1,
                                     size_t samples,
                                     uint32_t sample_format);
    static size_t demux_dual_channel_samples(const uint8_t* source,
                                             void* dest0,
                                             void* dest1,
                                             size_t samples,
                                             uint32_t sample_format);
    static size_t count_enabled_channels(uint8_t channel_mask);

private:
    sdr::driver::m300_xdma_ctrl::sptr _ctrl;
    sdr::core::xdma_zero_copy::sptr _rx_xport;
    uint32_t _packet_bytes = 0;
    uint32_t _payload_bytes = 0;
    uint32_t _dma_pkt_per_burst = 1;
    mutable uint8_t _channel_enable = 0x01;
    mutable size_t _channels = 1;
    uint8_t _rx_mode = 1;
    size_t _request_num_samples = 0;
    size_t _sample_rate = 1;
    uint32_t _sample_format = 1;

    sdr::core::managed_recv_buffer::sptr _pending_buff;
    const uint8_t* _pending_payload = nullptr;
    size_t _pending_words = 0;
    size_t _pending_word_offset = 0;
    sdr::driver::m300_header _pending_hdr;
    uint64_t _pending_timestamp = 0;
    bool _have_pending_timestamp = false;
    std::vector<uint8_t> _burst_payload;
    continuity_stats _continuity;
    bool _have_seq = false;
    uint16_t _expected_seq = 0;
    mutable std::mutex _mutex;
};

#endif // SOAPY_M300_RX_STREAMER_HPP
