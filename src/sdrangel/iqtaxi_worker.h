#pragma once

#include "iqtaxi_settings.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "include/sdr/api/Device.hpp"

class IqtaxiWorker
{
public:
    using SamplesCallback = std::function<void(const int16_t *, size_t, uint64_t)>;
    using ErrorCallback = std::function<void(const std::string &)>;

    IqtaxiWorker();
    ~IqtaxiWorker();

    bool start(const IqtaxiSettings &settings,
               SamplesCallback on_samples,
               ErrorCallback on_error);
    void stop();
    bool apply_settings(const IqtaxiSettings &settings);
    bool running() const { return _running.load(); }

private:
    bool open_device(const IqtaxiSettings &settings);
    void close_device();
    bool configure_device(const IqtaxiSettings &settings);
    bool start_stream();
    void stop_stream();
    void worker_loop();

private:
    std::mutex _device_mutex;
    std::atomic<bool> _running{false};
    std::thread _thread;

    IqtaxiSettings _settings{};
    SamplesCallback _on_samples;
    ErrorCallback _on_error;

    sdr::api::Device::sptr _device;
    sdr::api::rx_streamer::sptr _rx_stream;
};
