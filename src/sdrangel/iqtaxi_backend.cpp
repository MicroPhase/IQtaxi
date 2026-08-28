#include "iqtaxi_backend.h"

IqtaxiBackend::IqtaxiBackend() = default;
IqtaxiBackend::~IqtaxiBackend()
{
    stop();
}

bool IqtaxiBackend::start(const IqtaxiSettings &settings,
                            SamplesCallback on_samples,
                            ErrorCallback on_error)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!settings.is_valid())
    {
        return false;
    }
    _settings = settings;
    _onSamples = std::move(on_samples);
    _onError = std::move(on_error);
    return _worker.start(_settings, _onSamples, _onError);
}

void IqtaxiBackend::stop()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _worker.stop();
}

bool IqtaxiBackend::apply_settings(const IqtaxiSettings &settings)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!settings.is_valid())
    {
        return false;
    }

    const bool wasRunning = _worker.running();
    if (!wasRunning)
    {
        _settings = settings;
        return true;
    }

    // While running: IP rebuilds device, sample-rate restarts stream only,
    // frequency/gain are hot-written without touching the RX stream.
    const bool ok = _worker.apply_settings(settings);
    if (ok)
    {
        _settings = settings;
    }
    return ok;
}

IqtaxiSettings IqtaxiBackend::settings() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _settings;
}

bool IqtaxiBackend::running() const
{
    return _worker.running();
}

std::string IqtaxiBackend::rf_band() const
{
    return _worker.rf_band();
}

std::string IqtaxiBackend::board_label() const
{
    return _worker.board_label();
}
