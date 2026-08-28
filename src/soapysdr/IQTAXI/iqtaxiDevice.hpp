#ifndef IQTAXI_DEVICE_HPP
#define IQTAXI_DEVICE_HPP

#include <queue>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include "include/sdr/api/Device.hpp"
#include "src/driver/transport/super_recv_packet_handler.hpp"
#include "include/sdr/log.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>

using namespace sdr::api;
#define MAX_BUF_FRAMES 8192

class IQTaxiDevice: public SoapySDR::Device{
public:
    IQTaxiDevice(const SoapySDR::Kwargs &args);
    ~IQTaxiDevice();
    
    int activateStream(SoapySDR::Stream *stream,
                               const int flags,
                               const long long timeNs,
                               const size_t numElems);

    int deactivateStream(SoapySDR::Stream *handle,
                            const int flags,
                            const long long timeNs);

    int readStream(SoapySDR::Stream*,void* const *buffs,size_t numElems,
                    int &flags,long long &timeNs,long timeoutUs) ;

    int writeStream(SoapySDR::Stream*,const void* const *buffs,size_t numElems,
        int &flags,long long timeNs,long timeoutUs) ;

    void closeStream( SoapySDR::Stream *handle);
    
    size_t getStreamMTU( SoapySDR::Stream *handle) const;
    std::vector<std::string> getStreamFormats(int direction, size_t channel) const;
    std::string getNativeStreamFormat(int direction, size_t channel, double &fullScale) const;

    std::string getHardwareKey() const override;
    SoapySDR::Kwargs getHardwareInfo() const override;

    SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;
    std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;
    void setFrequency(int direction, size_t channel, const std::string &name, double frequency, const SoapySDR::Kwargs &args) ;
    double getFrequency(int direction, size_t channel, const std::string &name) const ;

    size_t getNumChannels(int direction) const;

    std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;
    void setAntenna( const int direction, const size_t channel, const std::string &name );
    std::string getAntenna( const int direction, const size_t channel ) const;

    std::vector<std::string> listGains( const int direction, const size_t channel ) const;
    void setGain(int direction, size_t channel, const std::string &name, double value) ;
    double getGain(int direction, size_t channel, const std::string &name) const ;
    double getSampleRate(int direction, size_t channel);
    void setSampleRate(int direction, size_t channel, const double rate);
    std::vector<double> listSampleRates( const int direction, const size_t channel ) const;
    void setBandwidth(int direction, size_t channel, double bandwidth) override;
    double getBandwidth(int direction, size_t channel) const override;
    SoapySDR::RangeList getBandwidthRange(int direction, size_t channel) const override;
    SoapySDR::Range getGainRange(int direction, size_t channel, const std::string &name) const ;
    SoapySDR::Stream* setupStream(const int direction,const std::string &format,const std::vector<size_t> &channels,const SoapySDR::Kwargs & ) ;

    bool IsValidRxStreamHandle(SoapySDR::Stream* handle) const;
    bool IsValidTxStreamHandle(SoapySDR::Stream* handle) const;
private:
    // SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x1;
	// SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x2;

    sdr::api::Device::sptr device;
    rx_streamer::sptr rx_stream;
    tx_streamer::sptr tx_stream;

    void startRxStreamLocked();
    void stopRxStreamLocked();
    void startTxStreamLocked();
    void stopTxStreamLocked();
    const sdr::api::DeviceProfile& profile() const;

    mutable std::mutex _ctrl_mutex,_stream_rx_mutex,_stream_tx_mutex;
    std::mutex queue_mutex;

    uint32_t sample_format;
    MicroRF_mode_t _current_mode = MICRORF_TRANSCEIVER_MODE_OFF;
    bool _rx_stream_active = false;
    bool _tx_stream_active = false;
    bool _tx_supported = true;
    bool _is_m300 = false;
    std::atomic<double> _sample_rate_hz{1.0};
    uint64_t timestamp = 0;
};

#endif
