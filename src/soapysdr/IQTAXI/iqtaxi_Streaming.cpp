#include "iqtaxiDevice.hpp"

IQTaxiDevice::IQTaxiDevice(const SoapySDR::Kwargs &args){
    const std::string transport = args.count("transport") ? args.at("transport") : "";
    const std::string iface = args.count("iface") ? args.at("iface") :
        (!transport.empty() ? transport : "udp");
    const std::string driver = args.count("driver") ? args.at("driver") : "";
    const std::string requested_device = args.count("device") ? args.at("device") :
        (args.count("product") ? args.at("product") : "");

    // Prefer explicit UDP product names over a stamped M300_XDMA factory name.
    // SoapySDR overwrites kwargs["driver"] with the factory that found the
    // device; a shared find path historically caused E206 to be opened as M300.
    const bool explicit_udp_device =
        requested_device == "E100" ||
        requested_device == "E200" ||
        requested_device == "E206" ||
        driver == "E100_UDP" ||
        driver == "E200_UDP" ||
        driver == "E206_UDP" ||
        driver == "IQTAXI" ||
        transport == "udp";
    const bool use_m300 =
        !explicit_udp_device &&
        (driver == "M300_XDMA" || driver == "FNIC_XDMA" ||
         requested_device == "M300" || requested_device == "M300_XDMA" ||
         requested_device == "FNIC_XDMA" ||
         iface == "pcie" || iface == "xdma");

    if (use_m300) {
        const std::string addr = args.count("addr") ? args.at("addr") : "/dev/xdma0";
        device = sdr::api::Device::makeDevice("M300_XDMA", addr);
        _tx_supported = true;
        _is_m300 = true;
        std::cout << "Making M300_XDMA device at " << addr << std::endl;
        if (device != nullptr) {
            device->set_dma_mode(0);
            timestamp = device->getTimeTicks();
        }
    }

    if (!use_m300 && iface == "udp"){
        std::string device_name = !requested_device.empty() ? requested_device : "E100";
        if (driver == "E200_UDP") {
            device_name = "E200";
        } else if (driver == "E206_UDP") {
            device_name = "E206";
        } else if (driver == "E100_UDP") {
            device_name = "E100";
        }
        if (!args.count("addr")) {
            std::cout << "makeDevice fail: missing addr for UDP device" << std::endl;
            return;
        }
        device = sdr::api::Device::makeDevice(device_name, args.at("addr"));
        std::cout << "Making " << device_name << " device" << std::endl;
        //uoe
        if (device != nullptr) {
            device->set_dma_mode(0);
            timestamp = device->getTimeTicks();
        }
    }
    if(device == nullptr)
        std::cout << "makeDevice fail" << std::endl;
}

IQTaxiDevice::~IQTaxiDevice(){
    if(rx_stream != nullptr){
       
    }
}

void IQTaxiDevice::startRxStreamLocked()
{
    if (!rx_stream) {
        return;
    }

    uint64_t timestampe = 0;
    rx_stream->set_rx_mode(STREAM_MODE);
    if (!_is_m300) {
        constexpr size_t kUdpPayloadBytes = 1472u - 16u;
        rx_stream->set_max_sample_nums_per_packet(kUdpPayloadBytes / 4u);
    }
    rx_stream->set_recv_param(STREAM_MODE, 4096, timestampe, 1, 0);
    _rx_stream_active = true;
    _current_mode = MICRORF_TRANSCEIVER_MODE_RX;
}

void IQTaxiDevice::stopRxStreamLocked()
{
    if (!rx_stream) {
        _rx_stream_active = false;
        return;
    }

    uint64_t time_stampe = 0;
    rx_stream->set_recv_param(STREAM_MODE, 8192, time_stampe, 0, 1);
    rx_stream->set_rx_mode_exit();
    _rx_stream_active = false;
    if (!_tx_stream_active) {
        _current_mode = MICRORF_TRANSCEIVER_MODE_OFF;
    }
}

void IQTaxiDevice::startTxStreamLocked()
{
    if (!tx_stream) {
        return;
    }

    if (!_is_m300) {
        device->set_channel_enable(1);
    }
    tx_stream->set_tx_source(1);
    tx_stream->set_stream_tx_start();
    _tx_stream_active = true;
    _current_mode = MICRORF_TRANSCEIVER_MODE_TX;
}

void IQTaxiDevice::stopTxStreamLocked()
{
    if (!tx_stream) {
        _tx_stream_active = false;
        return;
    }

    tx_stream->set_stream_tx_stop();
    _tx_stream_active = false;
    if (!_rx_stream_active) {
        _current_mode = MICRORF_TRANSCEIVER_MODE_OFF;
    }
}

SoapySDR::Stream* IQTaxiDevice::setupStream(const int direction,const std::string &format,const std::vector<size_t> &channels,const SoapySDR::Kwargs &args)
{
    if ( channels.size() > 1 ||( channels.size() > 0 && channels.at( 0 ) != 0 ) )
	{
		throw std::runtime_error( "setupStream invalid channel selection" );
	}
    if (!device) {
        throw std::runtime_error("setupStream called before IQTAXI device was opened");
    }

    if (_is_m300 && format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
        throw std::runtime_error("M300 RX supports CS16 and CF32 stream formats");
    }

    if ( format == SOAPY_SDR_CS8 )
    {
        SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS8." );
        sample_format = MICRORF_FORMAT_INT8;
    }else if ( format == SOAPY_SDR_CS16 )
    {
        std::cout << "Using format CS16." <<std::endl;
        SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS16." );
        sample_format = MICRORF_FORMAT_INT16;
    }else if ( format == SOAPY_SDR_CF32 )
    {
        std::cout << "Using format CF32." <<std::endl;
        SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF32." );
        sample_format= MICRORF_FORMAT_FLOAT32;
    }else if(format==SOAPY_SDR_CF64){
        SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF64." );
        sample_format= MICRORF_FORMAT_FLOAT64;
    }else 
        throw std::runtime_error( "setupStream invalid format " + format );

    if(direction == SOAPY_SDR_RX){
        std::lock_guard<std::mutex> lock(_stream_rx_mutex);
        rx_stream = device->get_rx_stream();
        if (!rx_stream) {
            throw std::runtime_error("IQTAXI RX stream is not supported by this device");
        }
        if (!_is_m300) {
            device->set_channel_enable(1);
        }
        // rx_stream->set_stream_rx_stop();
        // rx_stream->set_rx_mode_exit();
        rx_stream->set_sample_format(sample_format);
        return reinterpret_cast<SoapySDR::Stream*>(this->rx_stream.get());
    }else{
        if (!_tx_supported) {
            throw std::runtime_error("IQTAXI TX stream is not supported by this device yet");
        }
        std::lock_guard<std::mutex> lock(_stream_tx_mutex);
        tx_stream = device->get_tx_stream();
        if (!tx_stream) {
            throw std::runtime_error("IQTAXI TX stream is not supported by this device yet");
        }
        return reinterpret_cast<SoapySDR::Stream*>(this->tx_stream.get());
    }
    _current_mode = MICRORF_TRANSCEIVER_MODE_OFF;

    return nullptr;
}

void IQTaxiDevice::closeStream( SoapySDR::Stream *stream)
{
    if(IsValidRxStreamHandle(stream)){
        std::lock_guard<std::mutex> lock(_stream_rx_mutex);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Close Rx Stream");
        stopRxStreamLocked();
    }

    if(IsValidTxStreamHandle(stream)){
        std::lock_guard<std::mutex> lock(_stream_tx_mutex);
        stopTxStreamLocked();
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Close Tx Stream");
    }
    std::cout << "Close Stream" << std::endl;
}

size_t IQTaxiDevice::getStreamMTU( SoapySDR::Stream *stream) const
{
    if(IsValidRxStreamHandle(stream)){
        SoapySDR_log(SOAPY_SDR_INFO, "getRXstreamMTU");
        return _is_m300 ? (16384u - 16u) / sizeof(uint32_t) : 8192u;
    }
    
    if(IsValidTxStreamHandle(stream)){
        SoapySDR_log(SOAPY_SDR_INFO, "getTXstreamMTU");
        return tx_stream ? tx_stream->get_max_num_samps() : 0u;
    }
    return 0;
}

std::vector<std::string> IQTaxiDevice::getStreamFormats(int direction, size_t channel) const
{
    (void)channel;
    if (direction == SOAPY_SDR_TX && !_tx_supported) {
        return {};
    }
    if (_is_m300) {
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
    }
    return {SOAPY_SDR_CS8, SOAPY_SDR_CS16, SOAPY_SDR_CF32, SOAPY_SDR_CF64};
}

std::string IQTaxiDevice::getNativeStreamFormat(int direction,
                                                 size_t channel,
                                                 double &fullScale) const
{
    (void)direction;
    (void)channel;
    fullScale = 32768.0;
    return SOAPY_SDR_CS16;
}

int IQTaxiDevice::activateStream(SoapySDR::Stream *stream,
                               const int flags,
                               const long long timeNs,
                               const size_t numElems){
    SoapySDR_logf(SOAPY_SDR_WARNING,"activateStream") ;
    if(IsValidRxStreamHandle(stream)){
        // if(_current_mode == MICRORF_TRANSCEIVER_MODE_RX)
        //     return 0;
        std::lock_guard<std::mutex> lock(_stream_rx_mutex);
        SoapySDR_logf(SOAPY_SDR_INFO, "Start RX");
        startRxStreamLocked();
        // rx_stream->_start();
        SoapySDR_logf(SOAPY_SDR_WARNING,"_start") ;
    }
    
    if(IsValidTxStreamHandle(stream)){
        // if(_current_mode == MICRORF_TRANSCEIVER_MODE_TX)
        //     return 0;
        std::lock_guard<std::mutex> lock(_stream_tx_mutex);
        startTxStreamLocked();
        // timestamp = device->getTimeTicks();
        
        std::cout << "Start TX" << std::endl;
    }
    return 0;
}

int IQTaxiDevice::deactivateStream(SoapySDR::Stream *handle,
                                const int flags,
                                const long long timeNs){
    {
        if(IsValidRxStreamHandle(handle)){
            std::lock_guard<std::mutex> lock(_stream_rx_mutex);
            stopRxStreamLocked();
            std::cout << "deactivateStream Rx Stream" << std::endl;
        }
    
        if(IsValidTxStreamHandle(handle)){
            std::lock_guard<std::mutex> lock(_stream_tx_mutex);
            stopTxStreamLocked();
            std::cout << "deactivateStream TX Stream" << std::endl;
        }
    }
    return 0;
}

int IQTaxiDevice::readStream(SoapySDR::Stream* stream,void* const *buffs,size_t numElems,
                int &flags,long long &timeNs,long timeoutUs) {
    if(!IsValidRxStreamHandle(stream)){
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    // printf("start read\n");
    // auto recv_once_t1 = std::chrono::high_resolution_clock::now(); 

    std::lock_guard<std::mutex> lock(_stream_rx_mutex);
    uint64_t timestampe = 0;
    size_t received = rx_stream->recv(ref_vector<void*>(buffs,1),numElems,timestampe,sample_format); 
    if (received > 0) {
        const double rate = std::max(1.0, _sample_rate_hz.load());
        timeNs = static_cast<long long>(
            (static_cast<long double>(timestampe) * 1.0e9L) / rate);
        flags |= SOAPY_SDR_HAS_TIME;
    }
    // std::cout << received << " " << numElems << std::endl;
    // size_t received = rx_stream->recv_from_fifo(buffs,numElems,flags,timeNs,timeoutUs);  
    // float *dst =(float* )buffs[0];
    // std::cout << *dst << std::endl;

    // auto recv_once_t2 = std::chrono::high_resolution_clock::now(); 
    // SoapySDR_logf(SOAPY_SDR_WARNING, "Receive one frame,use time %d,received:%zu",std::chrono::duration_cast<std::chrono::microseconds>(recv_once_t2-recv_once_t1).count(),received);
    // printf("end read\n");
    return (received>0)? received:SOAPY_SDR_TIMEOUT;
}

int IQTaxiDevice::writeStream(SoapySDR::Stream* stream,const void * const* buffs,size_t numElems,
    int &flags,long long timeNs,long timeoutUs) {
    if(!IsValidTxStreamHandle(stream)){
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    // std::cout << "writeStream:" << timestamp << std::endl;
    // float *src = (float*)buffs[0];
    // std::cout << "first value " << src[0] << " "<< src[1] << " "<< src[2] << " "<< src[3] << " "<< src[4] << " "<< src[5] << " "<< src[6] << std::endl;
    std::lock_guard<std::mutex> lock(_stream_tx_mutex);
    uint32_t sent = tx_stream->send(ref_vector<const void*>(static_cast<const void* const*>(buffs),1),numElems,timestamp,sample_format);
    // std::cout << "over" << std::endl;
    return (sent>0)? sent:SOAPY_SDR_TIMEOUT;
}

bool IQTaxiDevice::IsValidRxStreamHandle(SoapySDR::Stream* handle) const
{
    if (handle == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_stream_rx_mutex);
    //handle is an opaque pointer hiding either rx_stream || tx_streamer:
    //check that the handle matches one of them, consistently with direction:
    if (rx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<rx_streamer*>(handle) == rx_stream.get()) {
            return true;
        }
    }

    return false;
}

bool IQTaxiDevice::IsValidTxStreamHandle(SoapySDR::Stream* handle) const
{
    if (handle == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(_stream_tx_mutex);
    //handle is an opaque pointer hiding either rx_stream || tx_streamer:
    //check that the handle matches one of them, consistently with direction:
    if (tx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<tx_streamer*>(handle) == tx_stream.get()) {
            return true;
        }
    }

    return false;
}
