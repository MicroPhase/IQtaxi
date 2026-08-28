#define GLFW_INCLUDE_NONE
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "include/sdr/api/UdpDiscover.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kDefaultPort = 49312;
constexpr int kMicrophaseDiscoverPort = 49100;
constexpr std::size_t kMaxResponseBytes = 128u * 1024u;
constexpr std::size_t kLogLimitBytes = 512u * 1024u;

struct GuiState {
    char host[128] = "192.168.1.10";
    char qspi_package_path[1024] = "/home/wcc/vm_box/xilinx_image_builder/build/e206/firmware/e206-qspi.frm";
    char sd_package_path[1024] = "/home/wcc/vm_box/xilinx_image_builder/build/e206/firmware/e206-sd.frm";
    char uoe_ip[64] = "192.168.1.10";
    char uoe_mac[64] = "E0:78:A3:00:00:11";
    int port = kDefaultPort;
    int device_index = 0;
    bool auto_scroll = true;

    std::atomic<bool> busy{false};
    std::atomic<bool> cancel_requested{false};
    std::atomic<int> active_socket{-1};
    std::thread worker;
    std::mutex mutex;
    std::string status = "idle";
    std::string log;
    std::string last_progress_log_line;
    std::string pending_uoe_ip;
    std::string pending_uoe_mac;
    std::string pending_host;
    bool pending_uoe_fields = false;
    bool pending_host_field = false;
    int progress = 0;
    bool error = false;
};

using MicrophaseDiscoverPacket = sdr::api::IqtaxiUdpDiscoverPacket;

class TcpClient {
public:
    TcpClient(const std::string& host, int port)
    {
        connect_to(host, port);
    }

    ~TcpClient()
    {
        if (_fd >= 0) {
            close(_fd);
        }
    }

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    int fd() const
    {
        return _fd;
    }

    void send_all(const void* data, std::size_t size)
    {
        const char* ptr = static_cast<const char*>(data);
        std::size_t sent = 0;
        while (sent < size) {
            const ssize_t n = send(_fd, ptr + sent, size - sent, 0);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("send failed: ") + std::strerror(errno));
            }
            if (n == 0) {
                throw std::runtime_error("connection closed while sending");
            }
            sent += static_cast<std::size_t>(n);
        }
    }

    std::string recv_line(std::size_t max_len = 1024)
    {
        std::string line;
        line.reserve(128);
        while (line.size() < max_len) {
            char ch = '\0';
            const ssize_t n = recv(_fd, &ch, 1, 0);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
            }
            if (n == 0) {
                throw std::runtime_error("connection closed while receiving line");
            }
            if (ch == '\n') {
                return line;
            }
            if (ch != '\r') {
                line.push_back(ch);
            }
        }
        throw std::runtime_error("response line is too long");
    }

    std::string recv_exact(std::size_t size)
    {
        std::string data(size, '\0');
        std::size_t received = 0;
        while (received < size) {
            const ssize_t n = recv(_fd, &data[received], size - received, 0);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
            }
            if (n == 0) {
                throw std::runtime_error("connection closed while receiving payload");
            }
            received += static_cast<std::size_t>(n);
        }
        return data;
    }

private:
    void connect_to(const std::string& host, int port)
    {
        if (port <= 0 || port > 65535) {
            throw std::runtime_error("invalid TCP port");
        }

        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        const std::string service = std::to_string(port);
        addrinfo* result = nullptr;
        const int rc = getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
        if (rc != 0) {
            throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(rc));
        }

        std::string last_error;
        for (addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
            const int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (fd < 0) {
                last_error = std::strerror(errno);
                continue;
            }

            timeval timeout{};
            timeout.tv_sec = 30;
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

            if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
                _fd = fd;
                freeaddrinfo(result);
                return;
            }

            last_error = std::strerror(errno);
            close(fd);
        }

        freeaddrinfo(result);
        throw std::runtime_error("connect failed: " + last_error);
    }

    int _fd = -1;
};

class ActiveSocket {
public:
    ActiveSocket(GuiState& state, int fd) : _state(state), _fd(fd)
    {
        _state.active_socket.store(fd);
    }

    ~ActiveSocket()
    {
        int expected = _fd;
        _state.active_socket.compare_exchange_strong(expected, -1);
    }

    ActiveSocket(const ActiveSocket&) = delete;
    ActiveSocket& operator=(const ActiveSocket&) = delete;

private:
    GuiState& _state;
    int _fd = -1;
};

static void request_cancel(GuiState& state)
{
    state.cancel_requested.store(true);
    const int fd = state.active_socket.load();
    if (fd >= 0) {
        shutdown(fd, SHUT_RDWR);
    }
}

static void throw_if_cancelled(const GuiState& state)
{
    if (state.cancel_requested.load()) {
        throw std::runtime_error("operation canceled");
    }
}

static void glfw_error_callback(int error, const char* description)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

static bool path_is_regular_file(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}

static bool path_contains_token(const std::string& path, const std::string& token)
{
    std::string lower = path;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return lower.find(token) != std::string::npos;
}

static bool is_e200_device(const std::string& device)
{
    return device == "E200";
}

static std::uint64_t file_size(const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("cannot open firmware package");
    }
    return static_cast<std::uint64_t>(file.tellg());
}

static std::string shell_quote(const std::string& value)
{
    std::string out = "'";
    for (char ch : value) {
        if (ch == '\'') {
            out += "'\\''";
        } else {
            out += ch;
        }
    }
    out += "'";
    return out;
}

static std::string trim_copy(std::string text)
{
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' ||
                             text.back() == ' ' || text.back() == '\t')) {
        text.pop_back();
    }
    std::size_t start = 0;
    while (start < text.size() &&
           (text[start] == ' ' || text[start] == '\t' ||
            text[start] == '\n' || text[start] == '\r')) {
        start++;
    }
    if (start != 0) {
        text.erase(0, start);
    }
    return text;
}

static std::string run_file_dialog()
{
    const char* home = std::getenv("HOME");
    const std::string start_dir = home ? home : ".";
    const std::vector<std::string> commands = {
        "zenity --file-selection --title='Select IQTAXI firmware package' "
        "--file-filter='FRM packages (*.frm) | *.frm' --file-filter='All files | *'",
        "kdialog --getopenfilename " + shell_quote(start_dir) +
            " '*.frm|FRM packages (*.frm)'"
    };

    for (const std::string& command : commands) {
        FILE* pipe = popen((command + " 2>/dev/null").c_str(), "r");
        if (!pipe) {
            continue;
        }

        char buffer[4096];
        std::string output;
        while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            output += buffer;
        }
        const int rc = pclose(pipe);
        output = trim_copy(output);
        if (rc == 0 && !output.empty()) {
            return output;
        }
    }

    return {};
}

static void append_log(GuiState& state, const std::string& text)
{
    std::string normalized;
    normalized.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r') {
            normalized.push_back('\n');
            if (i + 1 < text.size() && text[i + 1] == '\n') {
                ++i;
            }
        } else {
            normalized.push_back(text[i]);
        }
    }

    std::lock_guard<std::mutex> lock(state.mutex);
    state.log += normalized;
    if (state.log.size() > kLogLimitBytes) {
        state.log.erase(0, state.log.size() - kLogLimitBytes);
    }
}

static void set_status(GuiState& state, std::string status, bool error, int progress = -1)
{
    std::lock_guard<std::mutex> lock(state.mutex);
    state.status = std::move(status);
    state.error = error;
    if (progress >= 0) {
        state.progress = std::clamp(progress, 0, 100);
    }
}

static std::string read_sized_payload(TcpClient& client, const std::string& header,
                                      const std::string& context)
{
    const std::size_t space = header.find(' ');
    if (space == std::string::npos) {
        throw std::runtime_error("invalid " + context + " header: " + header);
    }
    const std::string size_text = header.substr(space + 1);
    const std::size_t size = static_cast<std::size_t>(std::stoull(size_text));
    if (size > kMaxResponseBytes) {
        throw std::runtime_error(context + " payload too large");
    }
    return client.recv_exact(size);
}

static std::string payload_value(const std::string& payload, const std::string& key)
{
    std::istringstream stream(payload);
    std::string line;
    const std::string prefix = key + "=";

    while (std::getline(stream, line)) {
        line = trim_copy(line);
        if (line.rfind(prefix, 0) == 0) {
            return line.substr(prefix.size());
        }
    }
    return {};
}

static void add_unique_host(std::vector<std::string>& hosts, const std::string& host)
{
    if (!host.empty() && std::find(hosts.begin(), hosts.end(), host) == hosts.end()) {
        hosts.push_back(host);
    }
}

static bool parse_ipv4_host(const std::string& host, std::uint32_t& ip_host_order)
{
    in_addr addr{};
    if (inet_pton(AF_INET, host.c_str(), &addr) != 1) {
        return false;
    }
    ip_host_order = ntohl(addr.s_addr);
    return true;
}

static std::string ipv4_to_string(std::uint32_t ip_host_order)
{
    in_addr addr{};
    addr.s_addr = htonl(ip_host_order);
    char text[INET_ADDRSTRLEN] = {};
    if (!inet_ntop(AF_INET, &addr, text, sizeof(text))) {
        return {};
    }
    return text;
}

static bool tcp_port_open_ipv4(const std::string& host, int port, int timeout_ms)
{
    std::uint32_t ip_host_order = 0;
    if (!parse_ipv4_host(host, ip_host_order) || port <= 0 || port > 65535) {
        return false;
    }

    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    addr.sin_addr.s_addr = htonl(ip_host_order);

    const int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc == 0) {
        close(fd);
        return true;
    }
    if (errno != EINPROGRESS) {
        close(fd);
        return false;
    }

    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int sel = select(fd + 1, nullptr, &wfds, nullptr, &tv);
    if (sel <= 0 || !FD_ISSET(fd, &wfds)) {
        close(fd);
        return false;
    }

    int so_error = 0;
    socklen_t so_error_len = sizeof(so_error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) != 0) {
        close(fd);
        return false;
    }
    close(fd);
    return so_error == 0;
}

static std::vector<std::string> scan_ipv4_subnet_for_port(std::uint32_t seed_ip,
                                                          int port,
                                                          int timeout_ms,
                                                          const GuiState& state)
{
    struct Probe {
        int fd = -1;
        std::string host;
    };

    std::vector<std::string> found;
    std::vector<Probe> probes;
    const std::uint32_t base = seed_ip & 0xFFFFFF00u;

    for (std::uint32_t i = 1; i < 255; ++i) {
        if (state.cancel_requested.load()) {
            break;
        }
        const int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            continue;
        }
        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<std::uint16_t>(port));
        addr.sin_addr.s_addr = htonl(base | i);

        const int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (rc == 0) {
            add_unique_host(found, ipv4_to_string(base | i));
            close(fd);
        } else if (errno == EINPROGRESS) {
            probes.push_back({fd, ipv4_to_string(base | i)});
        } else {
            close(fd);
        }
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!probes.empty() && std::chrono::steady_clock::now() < deadline) {
        if (state.cancel_requested.load()) {
            break;
        }

        fd_set wfds;
        FD_ZERO(&wfds);
        int max_fd = -1;
        for (const Probe& probe : probes) {
            if (probe.fd >= 0 && probe.fd < FD_SETSIZE) {
                FD_SET(probe.fd, &wfds);
                max_fd = std::max(max_fd, probe.fd);
            }
        }
        if (max_fd < 0) {
            break;
        }

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50000;
        const int sel = select(max_fd + 1, nullptr, &wfds, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (sel == 0) {
            continue;
        }

        probes.erase(std::remove_if(probes.begin(), probes.end(), [&](const Probe& probe) {
            if (probe.fd < 0 || probe.fd >= FD_SETSIZE || !FD_ISSET(probe.fd, &wfds)) {
                return false;
            }

            int so_error = 0;
            socklen_t so_error_len = sizeof(so_error);
            if (getsockopt(probe.fd, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) == 0 &&
                so_error == 0) {
                add_unique_host(found, probe.host);
            }
            close(probe.fd);
            return true;
        }), probes.end());
    }

    for (const Probe& probe : probes) {
        if (probe.fd >= 0) {
            close(probe.fd);
        }
    }
    return found;
}

static void send_discover_request(GuiState& state, int port)
{
    try {
        std::string seed_host;
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            seed_host = state.host;
        }

        set_status(state, "discovering devices", false, 0);
        append_log(state, "$ FIND_DEVICES udp:" + std::to_string(kMicrophaseDiscoverPort) +
                          " tcp:" + std::to_string(port) + "\n");

        std::vector<std::string> found_hosts;
        if (tcp_port_open_ipv4(seed_host, port, 300)) {
            add_unique_host(found_hosts, seed_host);
            append_log(state, "found firmware service at " + seed_host + ":" +
                              std::to_string(port) + "\n");
        }

        const int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) {
            throw std::runtime_error(std::string("discover socket failed: ") + std::strerror(errno));
        }
        ActiveSocket active_socket(state, fd);

        const int yes = 1;
        setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 250000;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(kMicrophaseDiscoverPort);
        dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        std::vector<sockaddr_in> discover_targets;
        discover_targets.push_back(dst);
        std::uint32_t seed_ip = 0;
        if (parse_ipv4_host(seed_host, seed_ip)) {
            sockaddr_in unicast = dst;
            unicast.sin_addr.s_addr = htonl(seed_ip);
            discover_targets.push_back(unicast);

            sockaddr_in directed = dst;
            directed.sin_addr.s_addr = htonl((seed_ip & 0xFFFFFF00u) | 0xFFu);
            discover_targets.push_back(directed);
        }

        MicrophaseDiscoverPacket request{};
        std::snprintf(request.check, sizeof(request.check), "%s", "MicroPhase");
        std::snprintf(request.name, sizeof(request.name), "%s", "e100");

        for (int attempt = 0; attempt < 3; ++attempt) {
            throw_if_cancelled(state);
            for (sockaddr_in& target : discover_targets) {
                const ssize_t sent = sendto(fd, &request, sizeof(request), 0,
                                            reinterpret_cast<sockaddr*>(&target), sizeof(target));
                if (sent != static_cast<ssize_t>(sizeof(request))) {
                    append_log(state, std::string("discover send failed: ") +
                                      std::strerror(errno) + "\n");
                }
            }
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
        while (std::chrono::steady_clock::now() < deadline) {
            throw_if_cancelled(state);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 200000;
            const int rc = select(fd + 1, &rfds, nullptr, nullptr, &tv);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error(std::string("discover select failed: ") +
                                         std::strerror(errno));
            }
            if (rc == 0 || !FD_ISSET(fd, &rfds)) {
                continue;
            }

            MicrophaseDiscoverPacket response{};
            sockaddr_in src{};
            socklen_t src_len = sizeof(src);
            const ssize_t n = recvfrom(fd, &response, sizeof(response), 0,
                                       reinterpret_cast<sockaddr*>(&src), &src_len);
            if (n != static_cast<ssize_t>(sizeof(response))) {
                continue;
            }
            const bool is_known_device =
                std::strncmp(response.name, "E100", 4) == 0 ||
                std::strncmp(response.name, "E206", 4) == 0 ||
                std::strncmp(response.name, "E200", 4) == 0;
            if (std::strncmp(response.check, "MicroPhase", sizeof(response.check)) != 0 ||
                !is_known_device) {
                continue;
            }

            char ip[INET_ADDRSTRLEN] = {};
            if (!inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip))) {
                continue;
            }
            const std::string host = ip;
            if (std::find(found_hosts.begin(), found_hosts.end(), host) == found_hosts.end()) {
                found_hosts.push_back(host);
                append_log(state, "found " + std::string(response.name) +
                                  " discovery response at " + host + ":" +
                                  std::to_string(port) + "\n");
            }
        }

        close(fd);
        state.active_socket.store(-1);

        if (found_hosts.empty() && seed_ip != 0u) {
            append_log(state, "no UDP response; scanning " +
                              ipv4_to_string(seed_ip & 0xFFFFFF00u) + "/24 on tcp:" +
                              std::to_string(port) + "\n");
            for (const std::string& host : scan_ipv4_subnet_for_port(seed_ip, port, 650, state)) {
                add_unique_host(found_hosts, host);
                append_log(state, "found firmware service at " + host + ":" +
                                  std::to_string(port) + "\n");
            }
        }

        if (found_hosts.empty()) {
            append_log(state, "no firmware service found\n");
            set_status(state, "no device found", true);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.pending_host = found_hosts.front();
            state.pending_host_field = true;
        }
        set_status(state, "device found: " + found_hosts.front(), false, 100);
    } catch (const std::exception& ex) {
        if (state.cancel_requested.load()) {
            append_log(state, "operation canceled\n");
            set_status(state, "operation canceled", false);
            return;
        }
        append_log(state, std::string("error: ") + ex.what() + "\n");
        set_status(state, ex.what(), true);
    }
}

static void send_status_request(GuiState& state, std::string host, int port)
{
    try {
        set_status(state, "connecting", false, 0);
        append_log(state, "$ STATUS " + host + ":" + std::to_string(port) + "\n");
        TcpClient client(host, port);
        ActiveSocket active_socket(state, client.fd());
        throw_if_cancelled(state);
        const std::string command = "STATUS\n";
        client.send_all(command.data(), command.size());

        const std::string header = client.recv_line();
        if (header.rfind("ERR ", 0) == 0) {
            throw std::runtime_error(header.substr(4));
        }
        if (header.rfind("OK ", 0) != 0) {
            throw std::runtime_error("unexpected status response: " + header);
        }
        const std::string payload = read_sized_payload(client, header, "status");
        append_log(state, payload);
        if (payload.empty() || payload.back() != '\n') {
            append_log(state, "\n");
        }
        set_status(state, "service ready", false, 100);
    } catch (const std::exception& ex) {
        if (state.cancel_requested.load()) {
            append_log(state, "operation canceled\n");
            set_status(state, "operation canceled", false);
            return;
        }
        append_log(state, std::string("error: ") + ex.what() + "\n");
        set_status(state, ex.what(), true);
    }
}

static void send_uoe_netcfg_request(GuiState& state, std::string host, int port,
                                    std::string command, std::string label)
{
    try {
        set_status(state, "connecting", false, 0);
        append_log(state, "$ " + command + " " + host + ":" + std::to_string(port) + "\n");
        TcpClient client(host, port);
        ActiveSocket active_socket(state, client.fd());
        throw_if_cancelled(state);

        command += "\n";
        client.send_all(command.data(), command.size());

        const std::string header = client.recv_line();
        if (header.rfind("ERR ", 0) == 0) {
            throw std::runtime_error(header.substr(4));
        }
        if (header.rfind("OK ", 0) != 0) {
            throw std::runtime_error("unexpected UOE netcfg response: " + header);
        }
        const std::string payload = read_sized_payload(client, header, "UOE netcfg");
        append_log(state, payload);
        if (payload.empty() || payload.back() != '\n') {
            append_log(state, "\n");
        }
        if (command.rfind("SET_IP_ADDR ", 0) == 0 ||
            command.rfind("SET_MAC_ADDR ", 0) == 0 ||
            command.rfind("SET_UOE_IP ", 0) == 0 ||
            command.rfind("SET_UOE_MAC ", 0) == 0) {
            append_log(state, "persistent UOE netcfg saved; restart the device to apply it\n");
        }
        const std::string ip = payload_value(payload, "ip");
        const std::string mac = payload_value(payload, "mac");
        if (!ip.empty() || !mac.empty()) {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.pending_uoe_ip = ip;
            state.pending_uoe_mac = mac;
            state.pending_uoe_fields = true;
        }
        set_status(state, std::move(label), false, 100);
    } catch (const std::exception& ex) {
        if (state.cancel_requested.load()) {
            append_log(state, "operation canceled\n");
            set_status(state, "operation canceled", false);
            return;
        }
        append_log(state, std::string("error: ") + ex.what() + "\n");
        if (std::string(ex.what()).find("write uoe-netcfg failed") != std::string::npos) {
            append_log(state,
                       "hint: the running board firmware must expose the uoe-netcfg MTD partition; "
                       "update firmware and restart before saving persistent UOE IP/MAC\n");
        }
        set_status(state, ex.what(), true);
    }
}

static void send_update_request(GuiState& state, std::string host, int port,
                                std::string device,
                                std::string package_path, std::string mode)
{
    try {
        if (mode != "qspi" && mode != "sd") {
            throw std::runtime_error("unsupported firmware update mode");
        }
        if (is_e200_device(device) && mode == "qspi") {
            throw std::runtime_error(
                "E200 does not support QSPI firmware updates; use SD update. "
                "QSPI is intentionally left for the PlutoSDR-compatible firmware.");
        }
        if (!path_contains_token(package_path, mode)) {
            throw std::runtime_error("selected FRM path does not look like a " + mode + " package");
        }
        if (!path_is_regular_file(package_path)) {
            throw std::runtime_error("firmware package not found");
        }
        const std::uint64_t size = file_size(package_path);
        if (size == 0) {
            throw std::runtime_error("firmware package is empty");
        }

        {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.log.clear();
            state.last_progress_log_line.clear();
            state.progress = 0;
            state.error = false;
            state.status = "connecting";
        }
        append_log(state, "$ UPDATE " + device + " " + mode + " " + host + ":" + std::to_string(port) +
                          " " + package_path + "\n");

        TcpClient client(host, port);
        ActiveSocket active_socket(state, client.fd());
        throw_if_cancelled(state);
        std::ostringstream header;
        header << "UPDATE " << mode << " 0 " << size << "\n";
        const std::string header_text = header.str();
        client.send_all(header_text.data(), header_text.size());

        std::ifstream file(package_path, std::ios::binary);
        if (!file) {
            throw std::runtime_error("cannot read firmware package");
        }

        std::vector<char> buffer(64u * 1024u);
        std::uint64_t sent = 0;
        while (file) {
            throw_if_cancelled(state);
            file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize n = file.gcount();
            if (n <= 0) {
                break;
            }
            client.send_all(buffer.data(), static_cast<std::size_t>(n));
            sent += static_cast<std::uint64_t>(n);
            const int pct = size ? static_cast<int>((sent * 50u) / size) : 50;
            set_status(state, "uploading firmware package", false, pct);
        }

        bool script_reported_complete = false;
        while (true) {
            throw_if_cancelled(state);
            std::string line;
            try {
                line = client.recv_line();
            } catch (const std::exception&) {
                if (script_reported_complete) {
                    set_status(state, "firmware update complete; restart device manually", false, 100);
                    break;
                }
                throw;
            }
            if (line.rfind("ERR ", 0) == 0) {
                throw std::runtime_error(line.substr(4));
            }
            if (line.rfind("PROGRESS ", 0) == 0) {
                std::istringstream iss(line);
                std::string tag;
                int pct = 0;
                iss >> tag >> pct;
                std::string message;
                std::getline(iss, message);
                message = trim_copy(message);
                set_status(state, message.empty() ? "updating firmware" : message, false, pct);
                if (script_reported_complete && pct >= 100) {
                    append_log(state, "firmware update complete; restart device manually\n");
                    set_status(state, "firmware update complete; restart device manually", false, 100);
                    break;
                }
                {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    if (state.last_progress_log_line != line) {
                        state.log += line + "\n";
                        state.last_progress_log_line = line;
                        if (state.log.size() > kLogLimitBytes) {
                            state.log.erase(0, state.log.size() - kLogLimitBytes);
                        }
                    }
                }
                continue;
            }
            if (line.rfind("LOG ", 0) == 0) {
                const std::string log = read_sized_payload(client, line, "log");
                if (log.find("E206 firmware update complete") != std::string::npos ||
                    log.find("E200 firmware update complete") != std::string::npos) {
                    script_reported_complete = true;
                }
                append_log(state, log);
                continue;
            }
            if (line.rfind("FAIL ", 0) == 0) {
                const std::string failure = trim_copy(read_sized_payload(client, line, "failure"));
                throw std::runtime_error(failure.empty() ? "firmware update failed" : failure);
            }
            if (line.rfind("DONE ", 0) == 0) {
                const std::string done = read_sized_payload(client, line, "done");
                if (!done.empty()) {
                    append_log(state, done);
                    if (done.back() != '\n') {
                        append_log(state, "\n");
                    }
                }
                set_status(state, "firmware update complete; restart device manually", false, 100);
                break;
            }
            throw std::runtime_error("unexpected update response: " + line);
        }
    } catch (const std::exception& ex) {
        if (state.cancel_requested.load()) {
            append_log(state, "operation canceled\n");
            set_status(state, "operation canceled", false);
            return;
        }
        append_log(state, std::string("error: ") + ex.what() + "\n");
        set_status(state, ex.what(), true);
    }
}

template <typename Fn>
static void start_worker(GuiState& state, Fn&& fn)
{
    if (state.worker.joinable()) {
        state.worker.join();
    }
    state.cancel_requested.store(false);
    state.active_socket.store(-1);
    state.busy.store(true);
    state.worker = std::thread([&state, task = std::forward<Fn>(fn)]() mutable {
        task();
        state.busy.store(false);
    });
}

static void render_gui(GuiState& state)
{
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        if (state.pending_uoe_fields) {
            if (!state.pending_uoe_ip.empty()) {
                std::snprintf(state.uoe_ip, sizeof(state.uoe_ip), "%s", state.pending_uoe_ip.c_str());
            }
            if (!state.pending_uoe_mac.empty()) {
                std::snprintf(state.uoe_mac, sizeof(state.uoe_mac), "%s", state.pending_uoe_mac.c_str());
            }
            state.pending_uoe_ip.clear();
            state.pending_uoe_mac.clear();
            state.pending_uoe_fields = false;
        }
        if (state.pending_host_field) {
            if (!state.pending_host.empty()) {
                std::snprintf(state.host, sizeof(state.host), "%s", state.pending_host.c_str());
            }
            state.pending_host.clear();
            state.pending_host_field = false;
        }
    }

    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
    constexpr ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("IQTAXI Firmware Update", nullptr, window_flags);

    const char* devices[] = {"E206", "E200"};
    ImGui::TextUnformatted("Device");
    ImGui::SameLine(92.0f);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::Combo("##Device", &state.device_index, devices, IM_ARRAYSIZE(devices));
    const std::string selected_device = devices[state.device_index];
    const bool e200_selected = is_e200_device(selected_device);

    ImGui::TextUnformatted("Address");
    ImGui::SameLine(92.0f);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::InputText("##Host", state.host, sizeof(state.host));
    ImGui::SameLine();
    ImGui::TextUnformatted("Port");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::InputInt("##Port", &state.port, 0, 0);
    ImGui::SameLine();
    const bool discovery_busy = state.busy.load();
    if (discovery_busy) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Find Devices")) {
        const int port = state.port;
        start_worker(state, [&state, port]() {
            send_discover_request(state, port);
        });
    }
    if (discovery_busy) {
        ImGui::EndDisabled();
    }

    ImGui::TextUnformatted("QSPI FRM");
    ImGui::SameLine(92.0f);
    ImGui::SetNextItemWidth(-92.0f);
    if (e200_selected) {
        ImGui::BeginDisabled();
    }
    ImGui::InputText("##QspiPackage", state.qspi_package_path, sizeof(state.qspi_package_path));
    ImGui::SameLine();
    if (ImGui::Button("Browse##Qspi")) {
        const std::string selected = run_file_dialog();
        if (!selected.empty()) {
            std::snprintf(state.qspi_package_path, sizeof(state.qspi_package_path), "%s", selected.c_str());
        } else {
            append_log(state, "file dialog unavailable or canceled\n");
        }
    }
    if (e200_selected) {
        ImGui::EndDisabled();
        ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.18f, 1.0f),
                           "E200 QSPI update is disabled; use SD update.");
    }

    ImGui::TextUnformatted("SD FRM");
    ImGui::SameLine(92.0f);
    ImGui::SetNextItemWidth(-92.0f);
    ImGui::InputText("##SdPackage", state.sd_package_path, sizeof(state.sd_package_path));
    ImGui::SameLine();
    if (ImGui::Button("Browse##Sd")) {
        const std::string selected = run_file_dialog();
        if (!selected.empty()) {
            std::snprintf(state.sd_package_path, sizeof(state.sd_package_path), "%s", selected.c_str());
        } else {
            append_log(state, "file dialog unavailable or canceled\n");
        }
    }

    ImGui::Checkbox("Auto scroll log", &state.auto_scroll);

    ImGui::TextUnformatted("UOE IP");
    ImGui::SameLine(92.0f);
    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputText("##UoeIp", state.uoe_ip, sizeof(state.uoe_ip));
    ImGui::SameLine();
    ImGui::TextUnformatted("UOE MAC");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputText("##UoeMac", state.uoe_mac, sizeof(state.uoe_mac));

    const bool busy = state.busy.load();
    if (busy) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Check Service")) {
        const std::string host = state.host;
        const int port = state.port;
        start_worker(state, [&state, host, port]() {
            send_status_request(state, host, port);
        });
    }
    ImGui::SameLine();
    if (e200_selected) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button("Update QSPI")) {
        const std::string host = state.host;
        const int port = state.port;
        const std::string device = selected_device;
        const std::string package_path = state.qspi_package_path;
        start_worker(state, [&state, host, port, device, package_path]() {
            send_update_request(state, host, port, device, package_path, "qspi");
        });
    }
    if (e200_selected) {
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("E200 only supports SD firmware update.");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Update SD")) {
        const std::string host = state.host;
        const int port = state.port;
        const std::string device = selected_device;
        const std::string package_path = state.sd_package_path;
        start_worker(state, [&state, host, port, device, package_path]() {
            send_update_request(state, host, port, device, package_path, "sd");
        });
    }
    ImGui::SameLine();
    if (ImGui::Button("Read UOE NetCfg")) {
        const std::string host = state.host;
        const int port = state.port;
        start_worker(state, [&state, host, port]() {
            send_uoe_netcfg_request(state, host, port, "GET_UOE_NETCFG", "UOE netcfg read complete");
        });
    }
    ImGui::SameLine();
    if (ImGui::Button("Save UOE IP")) {
        const std::string host = state.host;
        const int port = state.port;
        const std::string ip = state.uoe_ip;
        start_worker(state, [&state, host, port, ip]() {
            send_uoe_netcfg_request(state, host, port, "SET_IP_ADDR " + ip,
                                    "UOE IP saved; restart device to apply");
        });
    }
    ImGui::SameLine();
    if (ImGui::Button("Save UOE MAC")) {
        const std::string host = state.host;
        const int port = state.port;
        const std::string mac = state.uoe_mac;
        start_worker(state, [&state, host, port, mac]() {
            send_uoe_netcfg_request(state, host, port, "SET_MAC_ADDR " + mac,
                                    "UOE MAC saved; restart device to apply");
        });
    }
    if (busy) {
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Log")) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.log.clear();
        state.last_progress_log_line.clear();
        state.progress = 0;
        state.status = "idle";
        state.error = false;
    }

    std::string status;
    std::string log;
    int progress = 0;
    bool error = false;
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        status = state.status;
        log = state.log;
        progress = state.progress;
        error = state.error;
    }

    ImGui::ProgressBar(static_cast<float>(progress) / 100.0f, ImVec2(-1.0f, 0.0f));
    if (error) {
        ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.25f, 1.0f), "%s", status.c_str());
    } else {
        ImGui::TextWrapped("%s", status.c_str());
    }

    ImGui::BeginChild("UpdateLog", ImVec2(0.0f, 0.0f), true,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted(log.empty() ? "idle" : log.c_str());
    if (state.auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();

    ImGui::End();
}

} // namespace

int main()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        return 1;
    }

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(900, 620, "IQTAXI Firmware Update", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (glewInit() != GLEW_OK) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    GuiState state;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        render_gui(state);

        ImGui::Render();
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    if (state.worker.joinable()) {
        request_cancel(state);
        state.worker.join();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
