#include <SoapySDR/Registry.hpp>
#include "iqtaxiDevice.hpp"
#include "include/sdr/api/UdpDiscover.hpp"
#include "src/driver/M300/m300_xdma_discovery.hpp"
// #include <boost/filesystem.hpp>
// #include <boost/format.hpp>
// #include <boost/functional/hash.hpp>
// #include <boost/lexical_cast.hpp>
#include <cstddef>
#include <cstring>
#include <set>
#include <string>
#include <fcntl.h>          // fcntl(), O_NONBLOCK

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <ifaddrs.h>
    #include <net/if.h>
#endif

#include <errno.h>
// 查找函数（返回可发现的设备列表）
#define LISTEN_PORT 49100
#define MICROPHASE_CHECK "MicroPhase"

#define    MICROPHASE_DRIVER_IQTAXI "IQTAXI"
#define    MICROPHASE_NAME_BR0 "e100"
#define    MICROPHASE_NAME_E100 "E100"
#define    MICROPHASE_NAME_E200 "E200"
#define    MICROPHASE_NAME_E206 "E206"
#define    MICROPHASE_NAME_M300 "M300"
#define    MICROPHASE_DRIVER_M300_XDMA "M300_XDMA"
#define    MICROPHASE_DRIVER_FNIC_XDMA "FNIC_XDMA"

using unit_t = sdr::api::IqtaxiUdpDiscoverPacket;

static bool is_supported_device_name(const char *name)
{
    return strcmp(name, MICROPHASE_NAME_E100) == 0 ||
           strcmp(name, MICROPHASE_NAME_E200) == 0 ||
           strcmp(name, MICROPHASE_NAME_E206) == 0;
}

static bool kwargs_match_device(const SoapySDR::Kwargs &args, const char *device_name)
{
    const std::string name(device_name);

    if (args.count("device") != 0 && args.at("device") != name) {
        return false;
    }
    if (args.count("product") != 0 && args.at("product") != name) {
        return false;
    }

    if (args.count("driver") != 0) {
        const std::string expected_driver = name + "_UDP";
        if (args.at("driver") != MICROPHASE_DRIVER_IQTAXI &&
            args.at("driver") != expected_driver) {
            return false;
        }
    }

    return true;
}

static bool is_m300_request(const SoapySDR::Kwargs &args)
{
    if (args.count("driver") != 0) {
        const std::string driver = args.at("driver");
        if (driver == MICROPHASE_DRIVER_M300_XDMA ||
            driver == MICROPHASE_DRIVER_FNIC_XDMA) {
            return true;
        }
    }
    if (args.count("device") != 0) {
        const std::string device = args.at("device");
        if (device == MICROPHASE_NAME_M300 ||
            device == MICROPHASE_DRIVER_M300_XDMA ||
            device == MICROPHASE_DRIVER_FNIC_XDMA) {
            return true;
        }
    }
    if (args.count("product") != 0 && args.at("product") == MICROPHASE_NAME_M300) {
        return true;
    }
    if (args.count("transport") != 0) {
        const std::string transport = args.at("transport");
        if (transport == "pcie" || transport == "xdma") {
            return true;
        }
    }
    if (args.count("iface") != 0) {
        const std::string iface = args.at("iface");
        if (iface == "pcie" || iface == "xdma") {
            return true;
        }
    }
    return false;
}

static std::string field_to_string(const char *field, std::size_t field_len)
{
    std::size_t length = 0;
    while (length < field_len && field[length] != '\0') {
        ++length;
    }
    return std::string(field, length);
}

static SoapySDR::Kwargs make_device_info(
    const char *device_name,
    const std::string &addr,
    const std::string &board_version = {})
{
    // Keep discovery kwargs minimal (aligned with ref/E100). Extra keys such as
    // serial/version/rf_band make later Soapy open/match fragile. Put the
    // E100 6G/10G band in label only; Gqrx and SoapySDRUtil --find read that.
    SoapySDR::Kwargs devInfo;
    const std::string name(device_name);
    const std::string model = sdr::api::iqtaxi_model_label(name, board_version);

    devInfo["type"] = "soapy";
    devInfo["driver"] = MICROPHASE_DRIVER_IQTAXI;
    devInfo["device"] = name;
    devInfo["product"] = name;
    devInfo["transport"] = "udp";
    devInfo["addr"] = addr;
    devInfo["label"] = "Microphase " + model;

    return devInfo;
}

static SoapySDR::Kwargs make_m300_device_info(
    const sdr::driver::m300_discovery_info &info)
{
    SoapySDR::Kwargs devInfo;
    devInfo["type"] = "soapy";
    devInfo["driver"] = MICROPHASE_DRIVER_M300_XDMA;
    devInfo["device"] = MICROPHASE_DRIVER_M300_XDMA;
    devInfo["product"] = MICROPHASE_NAME_M300;
    devInfo["transport"] = "pcie";
    devInfo["iface"] = "xdma";
    devInfo["addr"] = info.addr;
    devInfo["serial"] = info.serial;
    devInfo["protocol"] = std::to_string(info.protocol_version);
    devInfo["capabilities"] = std::to_string(info.capabilities);
    if (!info.pci_bdf.empty()) {
        devInfo["pci_bdf"] = info.pci_bdf;
    }
    devInfo["label"] = "Microphase M300 PCIe [" + info.serial + "]";
    return devInfo;
}

struct if_addrs_t {
    std::string inet;   // 本机 IP
    std::string bcast;  // 广播地址
};

std::vector<if_addrs_t> get_if_addrs()
{
    std::vector<if_addrs_t> result;

    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return result;
    }

    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {

        if (!ifa->ifa_addr)
            continue;

        // 只要 IPv4
        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;

        // 接口必须 UP
        if (!(ifa->ifa_flags & IFF_UP))
            continue;

        sockaddr_in* addr = (sockaddr_in*)ifa->ifa_addr;
        sockaddr_in* netmask = (sockaddr_in*)ifa->ifa_netmask;

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr->sin_addr, ip, sizeof(ip));

        // 跳过 loopback
        if (strcmp(ip, "127.0.0.1") == 0)
            continue;

        // 计算广播地址
        uint32_t ip_u   = ntohl(addr->sin_addr.s_addr);
        uint32_t mask_u = ntohl(netmask->sin_addr.s_addr);
        uint32_t bcast_u = ip_u | ~mask_u;

        in_addr bcast_addr{};
        bcast_addr.s_addr = htonl(bcast_u);

        char bcast[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &bcast_addr, bcast, sizeof(bcast));

        result.push_back({
            ip,
            bcast
        });
    }

    freeifaddrs(ifaddr);
    return result;
}

SoapySDR::KwargsList discover_device(int timeout_ms, unit_t &out_dev, const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList results;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return results;

    // 非阻塞
    fcntl(sock, F_SETFL, O_NONBLOCK);

    int enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port   = htons(LISTEN_PORT); 
    local.sin_addr.s_addr = htonl(INADDR_ANY); 
    bind(sock, (struct sockaddr*)&local, sizeof(local));

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(LISTEN_PORT);
    
    unit_t send_data;
    strncpy(send_data.check,MICROPHASE_CHECK,sizeof(send_data.check));
    strncpy(send_data.name,MICROPHASE_NAME_BR0,sizeof(send_data.name));
    uint8_t send_buf[sizeof(unit_t)] = {0};
    memcpy(send_buf,(uint8_t *)&send_data,sizeof(unit_t));
    std::set<std::string> local_ips;
    std::set<std::string> seen_devices;
    for(const if_addrs_t& if_addrs : get_if_addrs()){
        local_ips.insert(if_addrs.inet);
        inet_pton(AF_INET, if_addrs.bcast.c_str(), &dest.sin_addr);
        sendto(sock, send_buf, sizeof(send_buf), 0,
        (sockaddr *)&dest, sizeof(dest));
    }
    inet_pton(AF_INET, "255.255.255.255", &dest.sin_addr);
    sendto(sock, send_buf, sizeof(send_buf), 0,
    (sockaddr *)&dest, sizeof(dest));
    inet_pton(AF_INET, "192.168.1.10", &dest.sin_addr);
    sendto(sock, send_buf, sizeof(send_buf), 0,
    (sockaddr *)&dest, sizeof(dest));

    // 等待响应
    fd_set rfds;
    auto start = std::chrono::steady_clock::now();

    while(true){
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        timeval tv{};
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(sock + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            close(sock);
            return results;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
            > (timeout_ms*10)) {
            break;
        }

        sockaddr_in from{};
        socklen_t len = sizeof(from);
        ssize_t n = recvfrom(sock, &out_dev, sizeof(out_dev), 0,
                            (sockaddr *)&from, &len);
        
        if(n <= 0)
            continue;

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));

        if (local_ips.count(ip))
            continue;

        if (n == sizeof(unit_t) 
        and is_supported_device_name(out_dev.name)
        and strcmp(out_dev.check,MICROPHASE_CHECK) == 0){
            if (!kwargs_match_device(args, out_dev.name)) {
                continue;
            }

            from.sin_port = htons(LISTEN_PORT);
            sendto(sock, send_buf, sizeof(send_buf), 0,
                   (sockaddr *)&from, sizeof(from));

            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;

            ret = select(sock + 1, &rfds, nullptr, nullptr, &tv);
            if (ret <= 0) {
                continue;
            }
            len = sizeof(from);
            n = recvfrom(sock, &out_dev, sizeof(out_dev), 0,
                         (sockaddr *)&from, &len);
            // perror("recv failed");
            // printf("errno=%d\n", errno);
            if (n == sizeof(unit_t) 
            and is_supported_device_name(out_dev.name)
            and strcmp(out_dev.check,MICROPHASE_CHECK) == 0){
                inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
                const std::string device_key = std::string(ip) + ":" + out_dev.name;
                if (seen_devices.count(device_key))
                    continue;
                seen_devices.insert(device_key);
                results.push_back(make_device_info(
                    out_dev.name,
                    ip,
                    field_to_string(out_dev.board_version,
                                    sizeof(out_dev.board_version))));
            }
        }
    }

    close(sock);

    return results;
}

static SoapySDR::KwargsList findUdpSDR(const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList results;

    // SoapySDR stamps kwargs["driver"] with the factory name that found the
    // device. Keep UDP discovery out of the M300 factory, otherwise a real
    // E206/E200/E100 board can be advertised as driver=M300_XDMA.
    if (is_m300_request(args)) {
        return results;
    }

    const bool direct_addr = args.count("addr") != 0;
    if (direct_addr) {
        if (kwargs_match_device(args, MICROPHASE_NAME_E100)) {
            results.push_back(make_device_info(MICROPHASE_NAME_E100, args.at("addr")));
        }
        if (kwargs_match_device(args, MICROPHASE_NAME_E200)) {
            results.push_back(make_device_info(MICROPHASE_NAME_E200, args.at("addr")));
        }
        if (kwargs_match_device(args, MICROPHASE_NAME_E206)) {
            results.push_back(make_device_info(MICROPHASE_NAME_E206, args.at("addr")));
        }
        return results;
    }

    unit_t dev;
    return discover_device(250, dev, args);
}

static SoapySDR::KwargsList findE206SDR(const SoapySDR::Kwargs &args)
{
    // Avoid duplicate enumeration with IQTAXI on empty --find. The E206_UDP
    // factory still answers explicit driver/device/product/addr requests.
    if (args.empty()) {
        return {};
    }
    return findUdpSDR(args);
}

static SoapySDR::KwargsList findM300SDR(const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList results;

    // Empty-arg enumeration still probes every factory. Only report M300 when
    // the request is explicitly PCIe/XDMA, or when no UDP-style selectors are
    // present and an XDMA node exists.
    const bool has_udp_selector =
        (args.count("device") != 0 &&
         (args.at("device") == MICROPHASE_NAME_E100 ||
          args.at("device") == MICROPHASE_NAME_E200 ||
          args.at("device") == MICROPHASE_NAME_E206)) ||
        (args.count("product") != 0 &&
         (args.at("product") == MICROPHASE_NAME_E100 ||
          args.at("product") == MICROPHASE_NAME_E200 ||
          args.at("product") == MICROPHASE_NAME_E206)) ||
        (args.count("transport") != 0 && args.at("transport") == "udp") ||
        (args.count("driver") != 0 &&
         (args.at("driver") == MICROPHASE_DRIVER_IQTAXI ||
          args.at("driver") == "E100_UDP" ||
          args.at("driver") == "E200_UDP" ||
          args.at("driver") == "E206_UDP"));

    if (has_udp_selector) {
        return results;
    }

    if (!args.empty() && !is_m300_request(args)) {
        return results;
    }

    const bool direct_addr = args.count("addr") != 0;
    std::vector<std::string> candidates;
    if (direct_addr) {
        candidates.push_back(
            sdr::driver::normalize_m300_xdma_base(args.at("addr")));
    } else {
        candidates = sdr::driver::enumerate_m300_xdma_candidates();
    }

    for (const std::string& addr : candidates) {
        sdr::driver::m300_discovery_info info;
        std::string error;
        if (!sdr::driver::probe_m300_xdma(addr, &info, &error)) {
            SoapySDR::logf(SOAPY_SDR_DEBUG,
                           "Ignoring XDMA candidate %s: %s",
                           addr.c_str(), error.c_str());
            continue;
        }
        if (args.count("serial") != 0 && args.at("serial") != info.serial) {
            continue;
        }
        results.push_back(make_m300_device_info(info));
    }
    return results;
}

// 创建设备实例
static SoapySDR::Device *makeMySDR(const SoapySDR::Kwargs &args)
{
    return new IQTaxiDevice(args);
}

// Register the generic IQTAXI Soapy driver. Device generation is selected
// through the discovery payload/kwargs, so the shared plugin should not also
// claim the legacy E100_UDP/E200_UDP driver names. Leaving those aliases to the
// old modules avoids duplicate Soapy registry entries when both plugins are
// installed during migration.
static SoapySDR::Registry registerIQTaxiSDR(
    MICROPHASE_DRIVER_IQTAXI,
    &findUdpSDR,
    &makeMySDR,
    SOAPY_SDR_ABI_VERSION
);

static SoapySDR::Registry registerE206SDR(
    "E206_UDP",
    &findE206SDR,
    &makeMySDR,
    SOAPY_SDR_ABI_VERSION
);

static SoapySDR::Registry registerM300SDR(
    MICROPHASE_DRIVER_M300_XDMA,
    &findM300SDR,
    &makeMySDR,
    SOAPY_SDR_ABI_VERSION
);
