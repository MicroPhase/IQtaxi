#include "iqtaxi_worker.h"

#include <chrono>
#include <exception>
#include <thread>

#include "src/driver/transport/local_regs.hpp"

namespace
{
constexpr size_t kRecvChunkSamples = 4096u;
constexpr size_t kMaxPacketSamples = (1472u - 16u) / 4u;
} // namespace

IqtaxiWorker::IqtaxiWorker() = default;

IqtaxiWorker::~IqtaxiWorker()
{
    stop();
}

bool IqtaxiWorker::start(const IqtaxiSettings &settings,
                             SamplesCallback on_samples,
                             ErrorCallback on_error)
{
    if (_running.load() || !settings.is_valid())
    {
        if (!settings.is_valid() && on_error)
        {
            on_error("invalid settings: check IP/sample rate/frequency/channel");
        }
        return false;
    }

    _settings = settings;
    _on_samples = std::move(on_samples);
    _on_error = std::move(on_error);

    if (!open_device(_settings))
    {
        return false;
    }
    return start_stream();
}

void IqtaxiWorker::stop()
{
    stop_stream();
    close_device();
}

bool IqtaxiWorker::apply_settings(const IqtaxiSettings &settings)
{
    if (!settings.is_valid())
    {
        if (_on_error)
        {
            _on_error("invalid settings: check IP/sample rate/frequency/channel");
        }
        return false;
    }

    if (!_running.load())
    {
        _settings = settings;
        return true;
    }

    const bool ipChanged = settings.device_addr != _settings.device_addr;
    const bool modelChanged = settings.device_model != _settings.device_model;
    const bool rateChanged = settings.sample_rate_hz != _settings.sample_rate_hz;
    const bool freqChanged = settings.center_freq_hz != _settings.center_freq_hz;
    const bool gainChanged = settings.rx_gain != _settings.rx_gain;
    const bool channelChanged = settings.channel != _settings.channel;

    if (!ipChanged && !modelChanged && !rateChanged && !freqChanged && !gainChanged && !channelChanged)
    {
        return true;
    }

    // IP / model change: rebuild device + stream.
    if (ipChanged || modelChanged)
    {
        stop_stream();
        close_device();
        _settings = settings;
        if (!open_device(_settings))
        {
            return false;
        }
        return start_stream();
    }

    // Sample-rate change: keep device, stop/restart RX stream only.
    if (rateChanged)
    {
        stop_stream();
        _settings = settings;
        if (!configure_device(_settings))
        {
            return false;
        }
        return start_stream();
    }

    // Frequency / gain / channel: hot poke while stream keeps running.
    try
    {
        std::lock_guard<std::mutex> lock(_device_mutex);
        if (!_device)
        {
            return false;
        }
        if (freqChanged || channelChanged)
        {
            _device->set_rx_freq(settings.center_freq_hz, settings.channel);
        }
        if (gainChanged || channelChanged)
        {
            _device->set_rx_gain(settings.rx_gain, settings.channel);
        }
        _settings = settings;
        return true;
    }
    catch (const std::exception &ex)
    {
        if (_on_error)
        {
            _on_error(std::string("hot apply exception: ") + ex.what());
        }
        return false;
    }
}

bool IqtaxiWorker::open_device(const IqtaxiSettings &settings)
{
    std::lock_guard<std::mutex> lock(_device_mutex);
    try
    {
        _device = sdr::api::Device::makeDevice(settings.device_model, settings.device_addr);
        if (!_device)
        {
            if (_on_error)
            {
                _on_error("failed to open " + settings.device_model +
                          " via IQTAXI (check device IP and network)");
            }
            return false;
        }

        _device->set_channel_enable(1u);
        _device->set_dma_mode(0u);
        _device->setSampleRate(static_cast<double>(settings.sample_rate_hz));
        _device->set_rx_freq(settings.center_freq_hz, settings.channel);
        _device->set_rx_gain(settings.rx_gain, settings.channel);
        return true;
    }
    catch (const std::exception &ex)
    {
        if (_on_error)
        {
            _on_error(std::string("setup exception: ") + ex.what());
        }
        _rx_stream.reset();
        _device.reset();
        return false;
    }
}

void IqtaxiWorker::close_device()
{
    std::lock_guard<std::mutex> lock(_device_mutex);
    _rx_stream.reset();
    _device.reset();
}

bool IqtaxiWorker::configure_device(const IqtaxiSettings &settings)
{
    std::lock_guard<std::mutex> lock(_device_mutex);
    if (!_device)
    {
        return false;
    }

    try
    {
        _device->setSampleRate(static_cast<double>(settings.sample_rate_hz));
        _device->set_rx_freq(settings.center_freq_hz, settings.channel);
        _device->set_rx_gain(settings.rx_gain, settings.channel);
        return true;
    }
    catch (const std::exception &ex)
    {
        if (_on_error)
        {
            _on_error(std::string("configure exception: ") + ex.what());
        }
        return false;
    }
}

bool IqtaxiWorker::start_stream()
{
    if (_running.load())
    {
        return true;
    }

    try
    {
        std::lock_guard<std::mutex> lock(_device_mutex);
        if (!_device)
        {
            return false;
        }

        _rx_stream = _device->get_rx_stream();
        if (!_rx_stream)
        {
            if (_on_error)
            {
                _on_error("failed to create IQTAXI RX stream");
            }
            return false;
        }

        _rx_stream->set_rx_mode(STREAM_MODE);
        _rx_stream->set_max_sample_nums_per_packet(kMaxPacketSamples);
        uint64_t start_ts = 0;
        (void)_rx_stream->set_recv_param(STREAM_MODE, kRecvChunkSamples, start_ts, 1u, 0u);
    }
    catch (const std::exception &ex)
    {
        if (_on_error)
        {
            _on_error(std::string("start stream exception: ") + ex.what());
        }
        std::lock_guard<std::mutex> lock(_device_mutex);
        _rx_stream.reset();
        return false;
    }

    _running.store(true);
    _thread = std::thread(&IqtaxiWorker::worker_loop, this);
    return true;
}

void IqtaxiWorker::stop_stream()
{
    if (!_running.exchange(false))
    {
        // Still drop a leftover stream handle if start failed mid-way.
        std::lock_guard<std::mutex> lock(_device_mutex);
        if (_rx_stream)
        {
            try
            {
                uint64_t stop_ts = 0;
                (void)_rx_stream->set_recv_param(STREAM_MODE, kRecvChunkSamples, stop_ts, 0u, 1u);
                _rx_stream->set_rx_mode_exit();
            }
            catch (...)
            {
            }
            _rx_stream.reset();
        }
        return;
    }

    if (_thread.joinable())
    {
        _thread.join();
    }
}

void IqtaxiWorker::worker_loop()
{
    std::vector<int16_t> rawSamples(kRecvChunkSamples * 2u);
    std::vector<void *> buffs{rawSamples.data()};

    while (_running.load())
    {
        sdr::api::rx_streamer::sptr rx;
        {
            std::lock_guard<std::mutex> lock(_device_mutex);
            rx = _rx_stream;
        }
        if (!rx)
        {
            break;
        }

        uint64_t ts = 0;
        size_t received = 0;

        try
        {
            received = rx->recv(buffs, kRecvChunkSamples, ts, MICRORF_FORMAT_INT16);
        }
        catch (const std::exception &ex)
        {
            if (_on_error)
            {
                _on_error(std::string("recv exception: ") + ex.what());
            }
            break;
        }

        if (received == 0u)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (_on_samples)
        {
            _on_samples(rawSamples.data(), received, ts);
        }
    }

    std::lock_guard<std::mutex> lock(_device_mutex);
    try
    {
        if (_rx_stream)
        {
            uint64_t stop_ts = 0;
            (void)_rx_stream->set_recv_param(STREAM_MODE, kRecvChunkSamples, stop_ts, 0u, 1u);
            _rx_stream->set_rx_mode_exit();
        }
    }
    catch (...)
    {
    }
    _rx_stream.reset();
    // Keep _device alive for sample-rate hot restart / freq/gain hot apply.
}
