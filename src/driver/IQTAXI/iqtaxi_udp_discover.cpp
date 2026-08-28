#include "include/sdr/api/UdpDiscover.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sdr::api {
namespace {

constexpr char kCheck[] = "MicroPhase";
constexpr char kProbeName[] = "e100";

bool is_supported_name(const char* name)
{
    return std::strcmp(name, "E100") == 0 ||
           std::strcmp(name, "E200") == 0 ||
           std::strcmp(name, "E206") == 0;
}

std::string field_to_string(const char* field, std::size_t field_len)
{
    std::size_t length = 0;
    while (length < field_len && field[length] != '\0') {
        ++length;
    }
    return std::string(field, length);
}

#ifndef _WIN32
struct InterfaceAddress {
    std::string address;
    std::string broadcast;
};

std::vector<InterfaceAddress> get_interface_addresses()
{
    std::vector<InterfaceAddress> result;
    ifaddrs* interfaces = nullptr;
    if (getifaddrs(&interfaces) == -1) {
        return result;
    }

    for (ifaddrs* interface = interfaces; interface != nullptr;
         interface = interface->ifa_next) {
        if (!interface->ifa_addr ||
            interface->ifa_addr->sa_family != AF_INET ||
            !(interface->ifa_flags & IFF_UP) ||
            !interface->ifa_netmask) {
            continue;
        }

        const auto* address =
            reinterpret_cast<const sockaddr_in*>(interface->ifa_addr);
        char address_text[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &address->sin_addr, address_text, sizeof(address_text));
        if (std::strcmp(address_text, "127.0.0.1") == 0) {
            continue;
        }

        const auto* netmask =
            reinterpret_cast<const sockaddr_in*>(interface->ifa_netmask);
        const uint32_t address_value = ntohl(address->sin_addr.s_addr);
        const uint32_t mask_value = ntohl(netmask->sin_addr.s_addr);
        in_addr broadcast_address{};
        broadcast_address.s_addr = htonl(address_value | ~mask_value);

        char broadcast_text[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET,
                  &broadcast_address,
                  broadcast_text,
                  sizeof(broadcast_text));
        result.push_back({address_text, broadcast_text});
    }

    freeifaddrs(interfaces);
    return result;
}
#endif

} // namespace

std::vector<IqtaxiUdpDiscoverInfo> iqtaxi_udp_discover(int timeout_ms)
{
    std::vector<IqtaxiUdpDiscoverInfo> results;
#ifdef _WIN32
    (void)timeout_ms;
    return results;
#else
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        return results;
    }

    fcntl(socket_fd, F_SETFL, O_NONBLOCK);
    int enable = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(kIqtaxiUdpDiscoverPort));
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(socket_fd, reinterpret_cast<sockaddr*>(&local), sizeof(local));

    IqtaxiUdpDiscoverPacket request{};
    std::strncpy(request.check, kCheck, sizeof(request.check));
    std::strncpy(request.name, kProbeName, sizeof(request.name));

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<uint16_t>(kIqtaxiUdpDiscoverPort));

    std::set<std::string> local_addresses;
    for (const InterfaceAddress& interface : get_interface_addresses()) {
        local_addresses.insert(interface.address);
        inet_pton(AF_INET, interface.broadcast.c_str(), &destination.sin_addr);
        sendto(socket_fd,
               &request,
               sizeof(request),
               0,
               reinterpret_cast<sockaddr*>(&destination),
               sizeof(destination));
    }

    inet_pton(AF_INET, "255.255.255.255", &destination.sin_addr);
    sendto(socket_fd,
           &request,
           sizeof(request),
           0,
           reinterpret_cast<sockaddr*>(&destination),
           sizeof(destination));
    inet_pton(AF_INET, "192.168.1.10", &destination.sin_addr);
    sendto(socket_fd,
           &request,
           sizeof(request),
           0,
           reinterpret_cast<sockaddr*>(&destination),
           sizeof(destination));

    std::set<std::string> seen;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms * 10);

    while (std::chrono::steady_clock::now() < deadline) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd, &read_fds);
        timeval timeout{};
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        if (select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) <= 0) {
            break;
        }

        IqtaxiUdpDiscoverPacket response{};
        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t received =
            recvfrom(socket_fd,
                     &response,
                     sizeof(response),
                     0,
                     reinterpret_cast<sockaddr*>(&source),
                     &source_length);
        if (received != static_cast<ssize_t>(sizeof(response))) {
            continue;
        }

        char address[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
        if (local_addresses.count(address) != 0 ||
            std::strcmp(response.check, kCheck) != 0 ||
            !is_supported_name(response.name)) {
            continue;
        }

        source.sin_port = htons(static_cast<uint16_t>(kIqtaxiUdpDiscoverPort));
        sendto(socket_fd,
               &request,
               sizeof(request),
               0,
               reinterpret_cast<sockaddr*>(&source),
               sizeof(source));

        FD_ZERO(&read_fds);
        FD_SET(socket_fd, &read_fds);
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        if (select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) <= 0) {
            continue;
        }

        source_length = sizeof(source);
        const ssize_t confirmed =
            recvfrom(socket_fd,
                     &response,
                     sizeof(response),
                     0,
                     reinterpret_cast<sockaddr*>(&source),
                     &source_length);
        if (confirmed != static_cast<ssize_t>(sizeof(response)) ||
            std::strcmp(response.check, kCheck) != 0 ||
            !is_supported_name(response.name)) {
            continue;
        }

        inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
        const std::string key = std::string(address) + ":" + response.name;
        if (!seen.insert(key).second) {
            continue;
        }

        IqtaxiUdpDiscoverInfo info;
        info.addr = address;
        info.name = field_to_string(response.name, sizeof(response.name));
        info.serial = field_to_string(response.serial_number,
                                      sizeof(response.serial_number));
        info.board_version = field_to_string(response.board_version,
                                             sizeof(response.board_version));
        results.push_back(std::move(info));
    }

    close(socket_fd);
    return results;
#endif
}

} // namespace sdr::api
