#pragma once

#include "include/sdr/api/Device.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace sdr::driver {

using sdr::api::ClockReferenceSource;

// Distinguish an accepted command that failed confirmation from a rejected
// command, so callers can keep their selected-source state consistent.
class clock_reference_not_ready : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

inline void validate_clock_reference(ClockReferenceSource source, double timeout_seconds)
{
    if (source != ClockReferenceSource::pps &&
        source != ClockReferenceSource::external_10mhz &&
        source != ClockReferenceSource::manual_dac) {
        throw std::invalid_argument("invalid VCXO reference source");
    }
    if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
        throw std::invalid_argument("clock_lock_timeout must be finite and positive");
    }
}

// VCXO status: [0] locked, [1] reference valid, [2] 10 MHz detected,
// [3] PPS detected, [5:4] selected source, [31:16] DAC value.
inline bool clock_reference_ready(uint32_t status, ClockReferenceSource source)
{
    const uint32_t selected_source = (status >> 4) & 0x3u;
    if (selected_source != static_cast<uint32_t>(source)) return false;
    switch (source) {
    case ClockReferenceSource::pps:
        return (status & 0xbu) == 0xbu;
    case ClockReferenceSource::external_10mhz:
        return (status & 0x7u) == 0x7u;
    case ClockReferenceSource::manual_dac:
        // Manual DAC is free-running: no external loop lock is expected.
        return true;
    }
    return false;
}

// The reader takes the remaining per-read timeout in seconds. Keeping this
// independent of the transport also lets tests simulate acquisition/failure.
template <typename ReadStatus>
uint32_t wait_for_clock_reference(ReadStatus read_status, ClockReferenceSource source,
                                 double timeout_seconds)
{
    validate_clock_reference(source, timeout_seconds);
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::duration<double>(timeout_seconds);
    uint32_t status = 0u;
    while (clock::now() < deadline) {
        const double remaining = std::chrono::duration<double>(deadline - clock::now()).count();
        if (remaining <= 0.0) break;
        status = read_status(std::min(0.5, remaining));
        if (clock::now() <= deadline && clock_reference_ready(status, source)) {
            return status;
        }
        std::this_thread::sleep_until(std::min(
            deadline, clock::now() + std::chrono::duration<double>(0.5)));
    }

    std::ostringstream message;
    message << "Timed out after " << timeout_seconds << " s waiting for "
            << (source == ClockReferenceSource::pps ? "PPS reference lock" :
                source == ClockReferenceSource::external_10mhz ?
                    "external 10 MHz reference lock" : "internal manual-DAC mode")
            << "; status=0x" << std::hex << status << std::dec
            << ", locked=" << ((status & 1u) != 0u)
            << ", reference_valid=" << ((status & 2u) != 0u)
            << ", reference_is_10mhz=" << ((status & 4u) != 0u)
            << ", reference_is_pps=" << ((status & 8u) != 0u)
            << ", selected_source=" << ((status >> 4) & 3u);
    throw std::runtime_error(message.str());
}

// All E-series drivers use this transaction, with their own register map.
template <typename Control>
void set_vcxo_reference_source(Control& control, uint32_t command_address,
                               uint32_t status_address, ClockReferenceSource source,
                               double timeout_seconds)
{
    validate_clock_reference(source, timeout_seconds);
    const uint64_t ack = control.poke32_ack_value(
        command_address, static_cast<uint32_t>(source), 1.0);
    if (ack != 0u) {
        throw std::runtime_error("VCXO reference-source command rejected: ack="
                                 + std::to_string(ack));
    }
    try {
        wait_for_clock_reference(
            [&](double timeout) { return control.peek32(status_address, timeout); },
            source, timeout_seconds);
    } catch (const std::exception& error) {
        throw clock_reference_not_ready(
            std::string("VCXO reference-source command accepted, but readiness was not confirmed: ")
            + error.what());
    }
}

} // namespace sdr::driver
