#ifndef SOAPY_M300_TX_STREAMER_HPP
#define SOAPY_M300_TX_STREAMER_HPP

#include "include/sdr/api/DataStream.hpp"
#include "include/sdr/core/xdma_zero_copy.hpp"
#include "m300_xdma_ctrl.hpp"

#include <cstdint>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

class m300_tx_streamer : public sdr::api::tx_streamer
{
public:
    m300_tx_streamer(const sdr::driver::m300_xdma_ctrl::sptr& ctrl,
                     const sdr::core::xdma_zero_copy::sptr& tx_xport,
                     size_t max_packet_bytes,
                     std::function<void()> tx_active_callback = {});
    ~m300_tx_streamer() override;

    size_t get_num_channels() const override;
    size_t get_max_num_samps() const override;
    void set_tx_source(uint32_t source) override;
    size_t send(const buffs_type& buffs,
                size_t nsamps_per_buff,
                uint64_t& time_stamp,
                uint32_t sample_format) override;
    size_t send_nonblocking(const buffs_type& buffs,
                            size_t nsamps_per_buff,
                            uint64_t& time_stamp,
                            uint32_t sample_format) override;
    void dds_ctrl(uint32_t phase_ctrl) override;
    void set_stream_tx_start() override;
    void set_stream_tx_stop() override;

    void configure(size_t samples_per_packet, bool timed, uint8_t channel_enable = 0x01u);

private:
    static constexpr uint32_t kCenterCtrlBase = 0x44a10000u;
    static constexpr uint32_t kRegChannelEnable = 18u * 4u;
    static constexpr uint32_t kRegTxSamplesPerPacket = 20u * 4u;
    static constexpr uint32_t kRegTxSourceSel = 21u * 4u;
    static constexpr uint32_t kRegIgnoreTxTimestamps = 22u * 4u;
    static constexpr uint32_t kRegTxDdsFreqCtrl = 26u * 4u;
    static constexpr uint32_t kRegFcWindow = 28u * 4u;
    static constexpr uint32_t kTxSourceOff = 0u;
    static constexpr uint32_t kTxSourceIq = 1u;

    size_t send_impl(const buffs_type& buffs,
                     size_t nsamps_per_buff,
                     uint64_t& time_stamp,
                     uint32_t sample_format,
                     double timeout_sec);
    void configure_hardware_locked();
    void start_send_worker_locked();
    void stop_send_worker_locked();
    void enqueue_send_buffer(sdr::core::managed_send_buffer::sptr buffer);
    void send_worker_func();
    void fill_packet(uint8_t* packet,
                     const buffs_type& buffs,
                     size_t sample_offset,
                     size_t sample_count,
                     size_t padded_samples,
                     uint64_t timestamp,
                     uint32_t sample_format);

    sdr::driver::m300_xdma_ctrl::sptr _ctrl;
    sdr::core::xdma_zero_copy::sptr _tx_xport;
    size_t _max_packet_bytes = 0;
    size_t _max_packets_per_write = 0;
    size_t _max_wire_samples_per_packet = 0;
    size_t _max_samples_per_packet = 0;
    size_t _max_samples_per_send = 0;
    size_t _samples_per_packet = 0;
    uint8_t _channel_enable = 0x01u;
    size_t _channels = 1u;
    uint32_t _tx_source = kTxSourceIq;
    bool _ignore_timestamps = true;
    bool _active = false;
    bool _tx_active_callback_pending = false;
    uint16_t _sequence = 0;
    std::function<void()> _tx_active_callback;
    mutable std::mutex _mutex;
    std::mutex _send_queue_mutex;
    std::condition_variable _send_queue_condition;
    std::deque<sdr::core::managed_send_buffer::sptr> _send_queue;
    std::thread _send_worker;
    bool _send_worker_stop = false;
};

#endif // SOAPY_M300_TX_STREAMER_HPP
