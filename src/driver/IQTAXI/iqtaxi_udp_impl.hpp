#ifndef SOAPY_IQTAXI_UDP_IMPL_HPP
#define SOAPY_IQTAXI_UDP_IMPL_HPP

#include "../transport/local_ctrl.hpp"
#include "../../../include/sdr/api/Device.hpp"
#include "cstdint"
#include "../transport/super_recv_packet_handler.hpp"
#include "../transport/super_send_packet_handler.hpp"

#define MICROPHASE_IQTAXI_UDP_FIND_PORT 49100
#define MICROPHASE_IQTAXI_UDP_CTRL_PORT 49208
#define MICROPHASE_IQTAXI_UDP_DATA_TX_PORT 49202
#define MICROPHASE_IQTAXI_UDP_DATA_TX1_PORT 49203
#define MICROPHASE_IQTAXI_UDP_DATA_RX_PORT 49200
#define MICROPHASE_IQTAXI_UDP_HW_CTRL_PORT 49208

#define SDR_STRINGIZE_DETAIL(x) #x
#define SDR_STRINGIZE(x) SDR_STRINGIZE_DETAIL(x)

using namespace sdr::api;

class IqtaxiUdpImpl: public Device{
public:
    IqtaxiUdpImpl(const std::string port, const DeviceProfile& profile);
    ~IqtaxiUdpImpl() override;

    std::string get_device_name() override;
    const DeviceProfile& get_profile() const override;

    rx_streamer::sptr get_rx_stream() override;
    tx_streamer::sptr get_tx_stream() override;

    void setTimestamp(uint64_t time_stamp,uint32_t mode) override;
    uint64_t getTimeTicks() override;

    void set_channel_enable(uint32_t channel_enable) override;
    uint32_t getSampleRate() override;
    void setSampleRate(double rate) override;

    void set_rx_freq(uint64_t rx_lo,size_t channel) override;
    void set_tx_freq(uint64_t tx_lo,size_t channel) override;

    uint64_t get_rx_freq(size_t channel) override;
    uint64_t get_tx_freq(size_t channel) override;

    uint32_t get_rx_gain(size_t channel) override;
    uint32_t get_tx_atten(size_t channel) override;

    void set_rx_gain(uint32_t rx_gain, size_t channel) override;
    void set_tx_atten(uint32_t tx_atten, size_t channel) override;
    bool isInitialSuccess();

    void set_dma_mode(uint32_t mode) override;

    std::shared_ptr<local_ctrl> get_local_bus();
    std::shared_ptr<local_ctrl> get_rx_stream_bus();
    std::shared_ptr<local_ctrl> get_tx_stream_bus();

private:
    bool iqtaxi_udp_init(const std::string port);
    void send_rx_hello();
    void send_tx_hello();

private:
    zero_copy_if::sptr _udp_ctrl,_udp_rx_stream,_udp_tx_stream;
    std::shared_ptr<local_ctrl> _local_bus,_rx_stream_bus,_tx_stream_bus;

    // A device has one RX and one TX data path. Keep the native stream
    // objects attached to the device so a second get_*_stream() call cannot
    // reset the FPGA state of an active stream.
    rx_streamer::sptr _rx_stream;
    tx_streamer::sptr _tx_stream;

    std::mutex _transport_setup_mutex;
    std::mutex _settings_mutex;
    bool _sample_rate_valid = false;
    uint32_t _sample_rate_hz = 0;
    bool _channel_enable_valid = false;
    uint32_t _channel_enable = 0;
    bool _dma_mode_valid = false;
    uint32_t _dma_mode = 0;
    bool _rx_freq_valid = false;
    uint64_t _rx_freq = 0;
    bool _tx_freq_valid = false;
    uint64_t _tx_freq = 0;
    bool _rx_gain_valid = false;
    uint32_t _rx_gain = 0;
    bool _tx_atten_valid = false;
    uint32_t _tx_atten = 0;

    bool initial_success = true;
    const DeviceProfile& _profile;

};
#endif
