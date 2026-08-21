#include "iqtaxiDevice.hpp"
#include "src/driver/M300/m300_xdma_impl.hpp"

#include <exception>
#include <mutex>

const sdr::api::DeviceProfile& IQTaxiDevice::profile() const
{
    return device ? device->get_profile() : sdr::api::e100_udp_profile();
}

size_t IQTaxiDevice::getNumChannels(int direction) const
{
    if (direction == SOAPY_SDR_TX && !_tx_supported) {
        return 0;
    }
    return 1;
}

std::vector<double> IQTaxiDevice::listSampleRates( const int direction, const size_t channel ) const
{
    SoapySDR_logf(SOAPY_SDR_DEBUG, "listSampleRates");
	std::vector<double> options;

    if (_is_m300) {
        return {61.44e6, 30.72e6, 15.36e6, 7.68e6, 3.84e6, 2.083333e6};
    }

    options.push_back(122.88e6);
    options.push_back(61.44e6);
    options.push_back(30.72e6);
    options.push_back(15.36e6);
    options.push_back(7.68e6);
    options.push_back(3.84e6);
    options.push_back(1.92e6);
    options.push_back(46.08e6);
    options.push_back(23.04e6);
    options.push_back(11.52e6);
    options.push_back(5.76e6);
    options.push_back(80.00e6);
    options.push_back(40.00e6);
    options.push_back(20.00e6);
    options.push_back(10.00e6);
    options.push_back(5.00e6);
    options.push_back(64.00e6);
    options.push_back(32.00e6);
    options.push_back(16.00e6);
    options.push_back(8.00e6);
    options.push_back(4.00e6);
    options.push_back(2.00e6);

	return(options);
}

void IQTaxiDevice::setSampleRate(int direction, size_t channel, const double rate ){
    SoapySDR_logf(SOAPY_SDR_DEBUG, "setSampleRate");
    std::cout << "setSampleRate" << std::endl;
    long long samplerate =(long long) rate;
    std::lock_guard<std::mutex> ctrl_lock(_ctrl_mutex);
    std::unique_lock<std::mutex> rx_lock(_stream_rx_mutex, std::defer_lock);
    std::unique_lock<std::mutex> tx_lock(_stream_tx_mutex, std::defer_lock);
    std::lock(rx_lock, tx_lock);

    const bool restart_rx = _rx_stream_active;
    const bool restart_tx = _tx_stream_active;

    if (restart_tx) {
        stopTxStreamLocked();
    }
    if (restart_rx) {
        stopRxStreamLocked();
    }

    auto restart_active_streams = [&]() {
        if (restart_rx) {
            startRxStreamLocked();
        }
        if (restart_tx) {
            startTxStreamLocked();
        }
    };

    try {
        device->setSampleRate(samplerate);
        _sample_rate_hz.store(static_cast<double>(samplerate));
    } catch (...) {
        const std::exception_ptr sample_rate_error = std::current_exception();
        try {
            restart_active_streams();
        } catch (const std::exception& ex) {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "failed to restart streams after sample-rate error: %s",
                          ex.what());
        } catch (...) {
            SoapySDR_logf(SOAPY_SDR_ERROR,
                          "failed to restart streams after sample-rate error");
        }
        std::rethrow_exception(sample_rate_error);
    }

    restart_active_streams();
    std::cout << samplerate << std::endl;
}


double IQTaxiDevice::getSampleRate(int direction, size_t channel){
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    uint32_t sampleRate = device->getSampleRate();
    SoapySDR_logf(SOAPY_SDR_WARNING, "getSampleRate:%d",sampleRate);
    std::cout << "getSampleRate:" << sampleRate << std::endl;
    return sampleRate;
}

void IQTaxiDevice::setBandwidth(int direction, size_t channel, double bandwidth)
{
    (void)direction;
    (void)channel;
    if (!_is_m300) {
        return;
    }

    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    auto m300 = std::dynamic_pointer_cast<M300XdmaImpl>(device);
    if (!m300) {
        throw std::runtime_error("M300 bandwidth control is unavailable");
    }
    const uint32_t bandwidth_hz = bandwidth <= 0.0 ? 0u :
        static_cast<uint32_t>(bandwidth);
    m300->set_bandwidth(bandwidth_hz);
}

double IQTaxiDevice::getBandwidth(int direction, size_t channel) const
{
    (void)direction;
    (void)channel;
    if (!_is_m300) {
        return 0.0;
    }

    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    auto m300 = std::dynamic_pointer_cast<M300XdmaImpl>(device);
    return m300 ? static_cast<double>(m300->get_bandwidth()) : 0.0;
}

SoapySDR::RangeList IQTaxiDevice::getBandwidthRange(int direction, size_t channel) const
{
    (void)direction;
    (void)channel;
    if (!_is_m300) {
        return {};
    }
    return {SoapySDR::Range(200000.0, 56000000.0)};
}

std::vector<std::string> IQTaxiDevice::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	if(direction == SOAPY_SDR_RX) options.push_back( "RX1" );
	if(direction == SOAPY_SDR_TX) options.push_back( "TX1" );
	return(options);
}

void IQTaxiDevice::setAntenna( const int direction, const size_t channel, const std::string &name )
{
    std::cout << "setAntenna" <<std::endl;
    if (direction == SOAPY_SDR_RX) {
       
	}

	else if (direction == SOAPY_SDR_TX) {
        

	}
}

std::string IQTaxiDevice::getAntenna( const int direction, const size_t channel ) const
{
	std::string options;

	if (direction == SOAPY_SDR_RX) {
		options = "RX1";
	}
	else if (direction == SOAPY_SDR_TX) {

		options = "TX1";
	}
	return options;
}

std::vector<std::string> IQTaxiDevice::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}

SoapySDR::RangeList IQTaxiDevice::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
    const auto& device_profile = profile();
    if (direction == SOAPY_SDR_TX) {
        return SoapySDR::RangeList(1, SoapySDR::Range(
            device_profile.tx_frequency_hz.minimum,
            device_profile.tx_frequency_hz.maximum,
            device_profile.tx_frequency_hz.step));
    }

    return SoapySDR::RangeList(1, SoapySDR::Range(
        device_profile.rx_frequency_hz.minimum,
        device_profile.rx_frequency_hz.maximum,
        device_profile.rx_frequency_hz.step));
}

void IQTaxiDevice::setFrequency(int direction, size_t channel, const std::string &name, double frequency, const SoapySDR::Kwargs &args)
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    uint64_t freq;
    freq = (uint64_t)frequency; 
    if (name == "RF"){
        std::cout << "setFrequency" << frequency << std::endl;
        if (direction == SOAPY_SDR_RX) {
            device->set_rx_freq(freq,channel+1);
        } else {
            device->set_tx_freq(freq,channel+1);
        }
    }
    
}

double IQTaxiDevice::getFrequency(int direction, size_t channel, const std::string &name) const
{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    double lo_freq = 0.0;
    if (name == "RF"){
        if (direction == SOAPY_SDR_RX) {
            lo_freq = device->get_rx_freq(channel+1);
        } else {
            lo_freq = device->get_tx_freq(channel+1);
        }
        std::cout << "getFrequency:" << lo_freq <<std::endl;
        return lo_freq;
    } else {
        return 0.0;
    }
}

std::vector<std::string> IQTaxiDevice::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back("PGA");
	return(options);
}

void IQTaxiDevice::setGain(int direction, size_t channel, const std::string &name, double value_) {
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    uint32_t value = (uint32_t)value_;
    if (direction== SOAPY_SDR_TX){
		device->set_tx_atten(value,channel + 1u);
        std::cout << "set Tx attenuation: " << value << std::endl;
	} else
	{
		device->set_rx_gain(value,channel + 1u);
        std::cout << "set Rx gain: " << value << std::endl;
	}
}

double IQTaxiDevice::getGain(int direction, size_t channel, const std::string &name) const{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    uint32_t value = 0;
    if (direction== SOAPY_SDR_TX){
		value = device->get_tx_atten(channel+1);
        std::cout << "Tx attenuation: " << value <<std::endl;
	} else
	{
		value = device->get_rx_gain(channel+1);
        std::cout << "Rx gain: " << value <<std::endl;
	}
    return (double) value;
}

SoapySDR::Range IQTaxiDevice::getGainRange(int direction, size_t channel, const std::string &name) const{
    std::lock_guard<std::mutex> lock(_ctrl_mutex);
    const auto& device_profile = profile();
    if (direction == SOAPY_SDR_TX) {
        return SoapySDR::Range(
            device_profile.tx_attenuation_db.minimum,
            device_profile.tx_attenuation_db.maximum,
            device_profile.tx_attenuation_db.step);
    }

    return SoapySDR::Range(
        device_profile.rx_gain_db.minimum,
        device_profile.rx_gain_db.maximum,
        device_profile.rx_gain_db.step);
}
