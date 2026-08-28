#pragma once

#include "iqtaxi_settings.h"
#include "iqtaxi_worker.h"

#include <functional>
#include <mutex>
#include <string>

class IqtaxiBackend
{
public:
    using SamplesCallback = IqtaxiWorker::SamplesCallback;
    using ErrorCallback = IqtaxiWorker::ErrorCallback;

    IqtaxiBackend();
    ~IqtaxiBackend();

    bool start(const IqtaxiSettings &settings,
               SamplesCallback on_samples,
               ErrorCallback on_error);
    void stop();

    bool apply_settings(const IqtaxiSettings &settings);
    IqtaxiSettings settings() const;
    bool running() const;
    std::string rf_band() const;
    std::string board_label() const;

private:
    mutable std::mutex _mutex;
    IqtaxiSettings _settings{};
    SamplesCallback _onSamples;
    ErrorCallback _onError;
    IqtaxiWorker _worker;
};
