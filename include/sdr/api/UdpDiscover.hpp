#ifndef SDR_API_UDP_DISCOVER_HPP
#define SDR_API_UDP_DISCOVER_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace sdr::api {

constexpr int kIqtaxiUdpDiscoverPort = 49100;
constexpr std::size_t kIqtaxiDiscoverSerialLen = 8;

#pragma pack(push, 1)
struct IqtaxiUdpDiscoverPacket {
    char check[16];
    char name[16];
    char serial_number[kIqtaxiDiscoverSerialLen];
    char board_version[8];
};
#pragma pack(pop)

static_assert(sizeof(IqtaxiUdpDiscoverPacket) == 48,
              "IQTAXI UDP discovery packet must match firmware layout");

struct IqtaxiUdpDiscoverInfo {
    std::string addr;
    std::string name;
    std::string serial;
    std::string board_version;
};

std::vector<IqtaxiUdpDiscoverInfo> iqtaxi_udp_discover(int timeout_ms = 250);

} // namespace sdr::api

#endif
