#include "src/driver/M300/m300_xdma_discovery.hpp"
#include "src/driver/M300/m300_xdma_protocol.hpp"

#include <array>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

using namespace sdr::driver;

constexpr uint16_t kSeq = 0x6241u;
constexpr uint64_t kNonce = 0x0123456789abcdefull;

std::array<uint8_t, M300_RESP_BYTES> make_response()
{
    std::array<uint8_t, M300_RESP_BYTES> bytes {};
    m300_resp_packet response;
    response.hdr.magic_type = M300_MAGIC_RESP;
    response.hdr.seq = kSeq;
    response.hdr.sid = M300_DISCOVERY_SID;
    response.hdr.length = M300_RESP_BYTES;
    response.timestamp = kNonce ^ M300_DISCOVERY_NONCE_XOR;
    response.cmd_id = M300_CMD_DISCOVER;
    response.status = M300_STATUS_OK;
    response.value0 = M300_DEVICE_MAGIC;
    response.value1 = M300_PRODUCT_ID;
    response.value2 = M300_DISCOVERY_INFO;
    write_resp_packet(bytes.data(), response);
    return bytes;
}

bool accepted(const std::array<uint8_t, M300_RESP_BYTES>& bytes)
{
    m300_discovery_info info;
    std::string error;
    return validate_m300_discovery_response(bytes.data(), bytes.size(), kSeq,
                                            kNonce, &info, &error);
}

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

} // namespace

int main()
{
    using namespace sdr::driver;

    require(normalize_m300_xdma_base("/dev/xdma12_c2h_0") == "/dev/xdma12",
            "XDMA channel path normalization failed");

    auto response = make_response();
    require(accepted(response), "valid response was rejected");

    m300_resp_packet packet = parse_resp_packet(response.data());
    packet.hdr.magic_type ^= 1u;
    write_resp_packet(response.data(), packet);
    require(!accepted(response), "bad response magic was accepted");

    response = make_response();
    packet = parse_resp_packet(response.data());
    packet.hdr.seq ^= 1u;
    write_resp_packet(response.data(), packet);
    require(!accepted(response), "wrong sequence was accepted");

    response = make_response();
    packet = parse_resp_packet(response.data());
    packet.cmd_id = M300_CMD_GET_VERSION;
    write_resp_packet(response.data(), packet);
    require(!accepted(response), "wrong command ID was accepted");

    response = make_response();
    packet = parse_resp_packet(response.data());
    packet.timestamp ^= 1u;
    write_resp_packet(response.data(), packet);
    require(!accepted(response), "wrong nonce response was accepted");

    response = make_response();
    packet = parse_resp_packet(response.data());
    packet.value1 ^= 1u;
    write_resp_packet(response.data(), packet);
    require(!accepted(response), "wrong product ID was accepted");

    std::cout << "PASS: M300 host discovery response validation\n";
    return 0;
}
