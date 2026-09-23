#include "src/driver/IQTAXI/clock_reference.hpp"
#include "src/driver/E100/local_e100_regs.hpp"

#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using Source = sdr::api::ClockReferenceSource;

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

template <typename Action>
void expect_failure(Action action, const std::string& expected)
{
    try {
        action();
    } catch (const std::exception& error) {
        require(std::string(error.what()).find(expected) != std::string::npos,
                "failure did not contain the expected diagnostic");
        return;
    }
    throw std::runtime_error("operation incorrectly reported success");
}

void test_external_acquisition()
{
    // A stale lock from PPS, a missing input, and an unlocked loop must all
    // be rejected before the final valid external lock is accepted.
    const std::vector<uint32_t> samples{0x0fu, 0x11u, 0x16u, 0x80000017u};
    size_t reads = 0;
    const auto status = sdr::driver::wait_for_clock_reference(
        [&](double timeout) {
            require(timeout > 0.0 && timeout <= 0.5, "invalid read timeout");
            require(reads < samples.size(), "kept polling after lock");
            return samples.at(reads++);
        }, Source::external_10mhz, 2.0);
    require(reads == samples.size() && status == samples.back(),
            "accepted an unusable external reference");
}

void test_internal_mode()
{
    size_t reads = 0;
    const auto status = sdr::driver::wait_for_clock_reference(
        [&](double) { return ++reads == 1 ? 0x17u : 0x12340020u; }, Source::manual_dac, 1.0);
    require(reads == 2 && status == 0x12340020u,
            "internal switch must wait for manual DAC, without external lock");
    require(!sdr::driver::clock_reference_ready(status, Source::external_10mhz),
            "manual DAC must not count as external lock");
}

void test_timeouts()
{
    // Missing reference-valid, missing 10 MHz, wrong source, and no lock.
    for (uint32_t status : {0x15u, 0x13u, 0x27u, 0x16u}) {
        expect_failure([&] {
            sdr::driver::wait_for_clock_reference(
                [=](double timeout) {
                    require(timeout > 0.0 && timeout <= 0.02,
                            "read exceeded the remaining acquisition budget");
                    return status;
                }, Source::external_10mhz, 0.02);
        }, "external 10 MHz reference lock; status=0x");
    }
    expect_failure([] {
        sdr::driver::wait_for_clock_reference(
            [](double) { return 0x17u; }, Source::manual_dac, 0.02);
    }, "internal manual-DAC mode");
}

void test_errors()
{
    expect_failure([] {
        sdr::driver::wait_for_clock_reference([](double) -> uint32_t {
            throw std::runtime_error("simulated transport failure");
        }, Source::external_10mhz, 1.0);
    }, "simulated transport failure");

    for (double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        expect_failure([&] {
            sdr::driver::wait_for_clock_reference([](double) -> uint32_t {
                throw std::runtime_error("invalid timeout reached the transport");
            }, Source::external_10mhz, timeout);
        }, "clock_lock_timeout must be finite and positive");
    }
}

struct FakeControl {
    uint32_t command_address;
    uint32_t status_address;
    Source source;
    std::vector<uint32_t> statuses;
    uint64_t ack = 0;
    bool fail_read = false;
    bool fail_write = false;
    size_t writes = 0;
    size_t reads = 0;

    uint64_t poke32_ack_value(uint32_t address, uint32_t value, double timeout)
    {
        require(address == command_address && value == static_cast<uint32_t>(source),
                "incorrect command register or source");
        require(timeout > 0.0, "invalid command timeout");
        ++writes;
        if (fail_write) throw std::runtime_error("command transport failure");
        return ack;
    }

    uint32_t peek32(uint32_t address, double timeout)
    {
        require(writes == 1 && ack == 0, "status read before successful command ACK");
        require(address == status_address && timeout > 0.0, "incorrect status read");
        ++reads;
        if (fail_read) throw std::runtime_error("status transport failure");
        return statuses.at(std::min(reads - 1, statuses.size() - 1));
    }

    void switch_source(double timeout = 1.0)
    {
        sdr::driver::set_vcxo_reference_source(
            *this, command_address, status_address, source, timeout);
    }
};

void test_driver_transaction()
{
    // Cover both register maps and all three sources through the entire
    // command -> ACK -> delayed confirmation transaction used by the drivers.
    for (bool e100 : {true, false}) {
        const auto command = e100 ? e100::CUSTOM_SET_E100_VCXO_REF_SOURCE
                                  : e100::CUSTOM_SET_VCXO_REF_SOURCE;
        const auto status = e100 ? e100::CUSTOM_RB_GET_E100_VCXO_STATUS
                                 : e100::CUSTOM_RB_GET_VCXO_STATUS;
        for (Source source : {Source::pps, Source::external_10mhz, Source::manual_dac}) {
            const uint32_t ready = source == Source::pps ? 0xbu :
                source == Source::external_10mhz ? 0x17u : 0x20u;
            FakeControl control{command, status, source, {0u, ready}};
            control.switch_source();
            require(control.writes == 1 && control.reads == 2,
                    "switch did not wait for confirmation, or resent the command");
        }
    }

    FakeControl rejected{0x3fu, 0x21u, Source::external_10mhz, {0x17u}};
    rejected.ack = 4;
    expect_failure([&] { rejected.switch_source(); }, "command rejected: ack=4");
    require(rejected.reads == 0, "polled status after command rejection");

    FakeControl write_failure{0x3fu, 0x21u, Source::external_10mhz, {0x17u}};
    write_failure.fail_write = true;
    expect_failure([&] { write_failure.switch_source(); }, "command transport failure");
    require(write_failure.reads == 0, "polled after command transport failure");

    for (bool fail_read : {false, true}) {
        FakeControl not_ready{0x3fu, 0x21u, Source::external_10mhz, {0x16u}};
        not_ready.fail_read = fail_read;
        bool classified = false;
        try {
            not_ready.switch_source(0.02);
        } catch (const sdr::driver::clock_reference_not_ready& error) {
            classified = true;
            require(std::string(error.what()).find(fail_read ? "status transport failure" :
                    "selected_source=1") != std::string::npos, "missing failure details");
        }
        require(classified && not_ready.writes == 1 && not_ready.reads >= 1,
                "failed confirmation was not distinguished from command rejection");
    }

    FakeControl invalid{0x3fu, 0x21u, static_cast<Source>(3u), {0x17u}};
    expect_failure([&] { invalid.switch_source(); }, "invalid VCXO reference source");
    require(invalid.writes == 0, "invalid source was sent to hardware");
    invalid.source = Source::pps;
    for (double timeout : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        expect_failure([&] { invalid.switch_source(timeout); }, "finite and positive");
    }
    require(invalid.writes == 0, "invalid timeout was checked after hardware mutation");
}

void test_pps_acquisition()
{
    // Do not confuse a 10 MHz lock, missing PPS, invalid input, or an
    // unlocked PPS loop with a valid PPS lock.
    for (uint32_t status : {0x17u, 0x3u, 0x9u, 0xau}) {
        expect_failure([&] {
            sdr::driver::wait_for_clock_reference(
                [=](double) { return status; }, Source::pps, 0.02);
        }, "PPS reference lock");
    }
}

} // namespace

int main()
{
    try {
        test_external_acquisition();
        test_internal_mode();
        test_timeouts();
        test_errors();
        test_driver_transaction();
        test_pps_acquisition();
        std::cout << "Clock-reference acquisition tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
