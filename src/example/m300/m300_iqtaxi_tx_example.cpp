#include "include/sdr/api/Device.hpp"
#include "src/driver/M300/m300_tx_streamer.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using sdr::api::Device;
using sdr::api::tx_streamer;

namespace {
constexpr uint32_t kTxSourceIq = 1u;
constexpr uint32_t kTxSourceH2cSink = 7u;

struct Config {
    std::string addr = "/dev/xdma0";
    double sample_rate = 61.44e6;
    uint64_t center_freq = 2'400'000'000ull;
    uint32_t attenuation_db = 30u;
    double tone_hz = 1.0e6;
    float amplitude = 0.1f;
    size_t samples_per_send = 4096u;
    size_t sends = 64u;
    size_t channels = 1u;
    bool cs16 = false;
    bool iq_output = false;
};

void parse_args(Config& cfg, int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&](const char* option) {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + option);
            }
            return argv[++i];
        };
        if (arg == "--addr") cfg.addr = value("--addr");
        else if (arg == "--sample-rate") cfg.sample_rate = std::stod(value("--sample-rate"));
        else if (arg == "--center-freq") cfg.center_freq = std::stoull(value("--center-freq"));
        else if (arg == "--attenuation") cfg.attenuation_db = std::stoul(value("--attenuation"));
        else if (arg == "--tone-hz") cfg.tone_hz = std::stod(value("--tone-hz"));
        else if (arg == "--amplitude") cfg.amplitude = std::stof(value("--amplitude"));
        else if (arg == "--samples") cfg.samples_per_send = std::stoull(value("--samples"));
        else if (arg == "--sends") cfg.sends = std::stoull(value("--sends"));
        else if (arg == "--channels") cfg.channels = std::stoull(value("--channels"));
        else if (arg == "--cs16") cfg.cs16 = true;
        else if (arg == "--iq-output") cfg.iq_output = true;
        else if (arg == "--help") {
            std::cout << "Usage: m300_iqtaxi_tx_example [--addr /dev/xdma0] [--samples 4096]"
                         " [--sends 64] [--channels 1|2] [--cs16] [--iq-output]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (cfg.samples_per_send == 0u || cfg.sends == 0u) {
        throw std::runtime_error("samples and sends must be non-zero");
    }
    if (cfg.channels != 1u && cfg.channels != 2u) {
        throw std::runtime_error("channels must be 1 or 2");
    }
}

std::vector<float> make_tone(const Config& cfg, size_t channel)
{
    constexpr double pi = 3.14159265358979323846;
    std::vector<float> samples(cfg.samples_per_send * 2u);
    for (size_t i = 0; i < cfg.samples_per_send; ++i) {
        const double phase = 2.0 * pi * cfg.tone_hz * static_cast<double>(i) /
                             cfg.sample_rate + channel * pi / 2.0;
        samples[i * 2u] = cfg.amplitude * static_cast<float>(std::cos(phase));
        samples[i * 2u + 1u] = cfg.amplitude * static_cast<float>(std::sin(phase));
    }
    return samples;
}

std::vector<int16_t> make_tone_cs16(const Config& cfg, size_t channel)
{
    const std::vector<float> source = make_tone(cfg, channel);
    std::vector<int16_t> samples(source.size());
    for (size_t i = 0u; i < source.size(); ++i) {
        samples[i] = static_cast<int16_t>(source[i] * 32767.0f);
    }
    return samples;
}
}

int main(int argc, char** argv)
{
    try {
        Config cfg;
        parse_args(cfg, argc, argv);

        Device::sptr device = Device::makeDevice("M300_XDMA", cfg.addr);
        if (!device) {
            throw std::runtime_error("failed to open M300_XDMA device");
        }
        tx_streamer::sptr tx = device->get_tx_stream();
        if (!tx) {
            throw std::runtime_error("M300 device did not provide a TX streamer");
        }

        if (cfg.iq_output) {
            device->setSampleRate(cfg.sample_rate);
            device->set_tx_freq(cfg.center_freq, 1u);
            device->set_tx_atten(cfg.attenuation_db, 1u);
            if (cfg.channels == 2u) {
                device->set_tx_atten(cfg.attenuation_db, 2u);
            }
        }

        if (auto m300_tx = std::dynamic_pointer_cast<m300_tx_streamer>(tx)) {
            m300_tx->configure(cfg.samples_per_send, false,
                               cfg.channels == 2u ? 0x03u : 0x01u);
        }

        std::vector<std::vector<float>> samples;
        std::vector<std::vector<int16_t>> samples_cs16;
        std::vector<const void*> sample_ptrs;
        samples.reserve(cfg.channels);
        samples_cs16.reserve(cfg.channels);
        sample_ptrs.reserve(cfg.channels);
        for (size_t channel = 0u; channel < cfg.channels; ++channel) {
            if (cfg.cs16) {
                samples_cs16.push_back(make_tone_cs16(cfg, channel));
                sample_ptrs.push_back(samples_cs16.back().data());
            } else {
                samples.push_back(make_tone(cfg, channel));
                sample_ptrs.push_back(samples.back().data());
            }
        }
        const tx_streamer::buffs_type buffs(sample_ptrs);
        uint64_t timestamp = device->getTimeTicks();
        size_t total_samples = 0u;

        tx->set_tx_source(cfg.iq_output ? kTxSourceIq : kTxSourceH2cSink);
        tx->set_stream_tx_start();
        const auto send_begin = std::chrono::steady_clock::now();
        for (size_t i = 0; i < cfg.sends; ++i) {
            const size_t sent = tx->send(
                buffs, cfg.samples_per_send, timestamp,
                cfg.cs16 ? MICRORF_FORMAT_INT16 : MICRORF_FORMAT_FLOAT32);
            if (sent != cfg.samples_per_send) {
                throw std::runtime_error("short M300 TX send");
            }
            total_samples += sent;
        }
        tx->set_stream_tx_stop();
        const double send_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - send_begin).count();

        const double per_channel_rate_msps =
            static_cast<double>(total_samples) / send_seconds / 1.0e6;
        const double aggregate_rate_msps = per_channel_rate_msps * cfg.channels;
        const double payload_mib_s =
            static_cast<double>(total_samples * cfg.channels * sizeof(uint32_t)) /
            send_seconds / (1024.0 * 1024.0);

        std::cout << "m300_iqtaxi_tx_example=PASS"
                  << " mode=" << (cfg.iq_output ? "iq" : "h2c_sink")
                  << " channels=" << cfg.channels
                  << " format=" << (cfg.cs16 ? "CS16" : "CF32")
                  << " samples=" << total_samples
                  << " elapsed=" << send_seconds
                  << " per_channel_msps=" << per_channel_rate_msps
                  << " aggregate_msps=" << aggregate_rate_msps
                  << " payload_mib_s=" << payload_mib_s
                  << " final_timestamp=" << timestamp << '\n';
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "m300_iqtaxi_tx_example_error: " << ex.what() << '\n';
        return 1;
    }
}
