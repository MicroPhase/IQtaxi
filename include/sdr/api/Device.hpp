//
// Created by jcc on 25-4-8.
//

#ifndef SOAPY_DEVICE_HPP
#define SOAPY_DEVICE_HPP

#include "cstdint"
#include "memory"
#include "vector"
#include "../config.hpp"
#include "./DataStream.hpp"
#include "./DeviceProfile.hpp"

enum MicroRF_Format{
    MICRORF_FORMAT_FLOAT32 = 0,
    MICRORF_FORMAT_INT16 = 1,
    MICRORF_FORMAT_INT8 = 2,
    MICRORF_FORMAT_FLOAT64 = 3,
};

enum MicroRF_mode_t{
    MICRORF_TRANSCEIVER_MODE_OFF = 0,
	MICRORF_TRANSCEIVER_MODE_RX = 1,
	MICRORF_TRANSCEIVER_MODE_TX = 2,
};

namespace sdr::api{
    class API_EXPORT Device {
    public:
        typedef std::shared_ptr<Device> sptr;

        virtual ~Device() = default;

        virtual std::string get_device_name() = 0;
        virtual const DeviceProfile& get_profile() const = 0;

        // 基础配置
        virtual rx_streamer::sptr get_rx_stream() = 0;
        virtual tx_streamer::sptr get_tx_stream() = 0;

        // 时间同步
        virtual uint64_t getTimeTicks() = 0;
        virtual void setTimestamp(uint64_t time_stamp,uint32_t mode) = 0;

        virtual void set_channel_enable(uint32_t channel_enable) = 0;
        virtual uint32_t getSampleRate() = 0;
        virtual void setSampleRate(double rate) = 0;

        virtual void set_rx_freq(uint64_t rx_lo,size_t channel) = 0;
        virtual void set_tx_freq(uint64_t tx_lo,size_t channel) = 0;

        virtual uint64_t get_rx_freq(size_t channel) = 0;
        virtual uint64_t get_tx_freq(size_t channel) = 0;

        virtual uint32_t get_rx_gain(size_t channel) = 0;
        virtual uint32_t get_tx_atten(size_t channel) = 0;

        virtual void set_rx_gain(uint32_t rx_gain, size_t channel) = 0;
        virtual void set_tx_atten(uint32_t tx_atten, size_t channel) = 0;

        static sptr makeDevice(const std::string interface_type,const std::string addr);

        virtual void set_dma_mode(uint32_t mode) = 0;

        // virtual std::shared_ptr<local_ctrl> get_local_bus() = 0;
        // virtual std::shared_ptr<local_ctrl> get_rx_stream_bus() = 0;
        // virtual std::shared_ptr<local_ctrl> get_tx_stream_bus() = 0;

    protected:
        void acquire_exclusive_access(const std::string& backend,
                                      const std::string& address);

    private:
        // Kept type-erased so the public header remains platform-neutral.
        // A backend constructor installs an OS-level lease here before it
        // creates transports or sends commands to the hardware.
        std::shared_ptr<void> _exclusive_access_lease;
    };
}

#endif
