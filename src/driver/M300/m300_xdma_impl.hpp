#ifndef SOAPY_M300_XDMA_IMPL_HPP
#define SOAPY_M300_XDMA_IMPL_HPP

#include <mutex>
#include <utility>
#include <string>
#include "include/sdr/api/Device.hpp"
#include "include/sdr/core/xdma_zero_copy.hpp"
#include "m300_rf_ctrl.hpp"
#include "m300_rx_streamer.hpp"
#include "m300_tx_streamer.hpp"
#include "m300_xdma_ctrl.hpp"

using namespace sdr::api;

class M300XdmaImpl : public Device {
public:
    explicit M300XdmaImpl(const std::string& device_path);
    M300XdmaImpl(const std::string& device_path, bool open_data_channels);
    ~M300XdmaImpl() override = default;

    std::string get_device_name() override;
    const DeviceProfile& get_profile() const override;
    rx_streamer::sptr get_rx_stream() override;
    tx_streamer::sptr get_tx_stream() override;

    uint64_t getTimeTicks() override;
    void setTimestamp(uint64_t time_stamp,uint32_t mode) override;
    void set_channel_enable(uint32_t channel_enable) override;
    uint32_t getSampleRate() override;
    void setSampleRate(double rate) override;
    void set_bandwidth(uint32_t bandwidth_hz);
    uint32_t get_bandwidth() const;
    void set_rx_freq(uint64_t rx_lo,size_t channel) override;
    void set_tx_freq(uint64_t tx_lo,size_t channel) override;
    uint64_t get_rx_freq(size_t channel) override;
    uint64_t get_tx_freq(size_t channel) override;
    uint32_t get_rx_gain(size_t channel) override;
    uint32_t get_tx_atten(size_t channel) override;
    void set_rx_gain(uint32_t rx_gain, size_t channel) override;
    void set_tx_atten(uint32_t tx_atten, size_t channel) override;
    void set_dma_mode(uint32_t mode) override;

    bool isInitialSuccess() const;
    std::string last_error() const;

    void configure_rx_packet_bytes(uint32_t packet_bytes);
    std::shared_ptr<sdr::driver::m300_xdma_ctrl> get_ctrl();
    sdr::core::xdma_zero_copy::sptr get_ctrl_xport();
    sdr::core::xdma_zero_copy::sptr get_resp_xport();
    sdr::core::xdma_zero_copy::sptr get_rx_xport();
    sdr::core::xdma_zero_copy::sptr get_tx_xport();

private:
    void parse_device_path(const std::string& device_path);

private:
    std::string _device_path;
    std::string _ctrl_h2c_path;
    std::string _resp_c2h_path;
    std::string _rx_c2h_path;
    std::string _tx_h2c_path;
    sdr::core::xdma_zero_copy::sptr _ctrl_xport;
    sdr::core::xdma_zero_copy::sptr _resp_xport;
    sdr::core::xdma_zero_copy::sptr _rx_xport;
    sdr::core::xdma_zero_copy::sptr _tx_xport;
    std::shared_ptr<sdr::driver::m300_xdma_ctrl> _ctrl;
    std::shared_ptr<sdr::driver::m300_rf_ctrl> _rf_ctrl;
    rx_streamer::sptr _rx_stream;
    tx_streamer::sptr _tx_stream;
    bool _initial_success = false;
    std::string _last_error;
    uint32_t _dma_mode = 0;
    mutable std::mutex _mutex;
};

#endif // SOAPY_M300_XDMA_IMPL_HPP
