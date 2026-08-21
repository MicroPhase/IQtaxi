#include "m300_xdma_discovery.hpp"

#include "m300_xdma_protocol.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <glob.h>
#include <set>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace sdr { namespace driver {
namespace {

#define IOCTL_XDMA_M300_SYNC_TIMEOUT_SET _IOW('q', 25, uint32_t*)

class fd_guard
{
public:
    explicit fd_guard(int fd = -1) : _fd(fd) {}
    ~fd_guard()
    {
        if (_fd >= 0) {
            ::close(_fd);
        }
    }

    fd_guard(const fd_guard&) = delete;
    fd_guard& operator=(const fd_guard&) = delete;

    int get() const { return _fd; }

private:
    int _fd;
};

bool ends_with(const std::string& value, const std::string& suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void set_error(std::string* error, const std::string& message)
{
    if (error) {
        *error = message;
    }
}

uint64_t make_nonce()
{
    static std::atomic<uint64_t> counter{1u};
    const uint64_t ticks = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return ticks ^ (counter.fetch_add(1u, std::memory_order_relaxed) << 32) ^
           static_cast<uint64_t>(::getpid());
}

std::string device_node_name(const std::string& base)
{
    const std::size_t slash = base.find_last_of('/');
    return slash == std::string::npos ? base : base.substr(slash + 1u);
}

std::string pci_bdf_for_base(const std::string& base)
{
    std::error_code ec;
    const std::filesystem::path device_path =
        std::filesystem::canonical(
            std::filesystem::path("/sys/class/xdma") /
                (device_node_name(base) + "_control") / "device",
            ec);
    if (ec || device_path.empty()) {
        return std::string();
    }
    return device_path.filename().string();
}

bool required_nodes_available(const std::string& base)
{
    static const std::array<std::pair<const char*, int>, 4> channels = {{
        {"_h2c_0", W_OK}, {"_c2h_0", R_OK},
        {"_h2c_1", W_OK}, {"_c2h_1", R_OK}
    }};
    for (const auto& channel : channels) {
        if (::access((base + channel.first).c_str(), channel.second) != 0) {
            return false;
        }
    }
    return true;
}

} // namespace

std::string normalize_m300_xdma_base(const std::string& path)
{
    const std::string value = path.empty() ? "/dev/xdma0" : path;
    static const std::array<const char*, 5> suffixes = {
        "_h2c_0", "_c2h_0", "_h2c_1", "_c2h_1", "_control"
    };
    for (const char* suffix : suffixes) {
        if (ends_with(value, suffix)) {
            return value.substr(0, value.size() - std::strlen(suffix));
        }
    }
    return value;
}

std::vector<std::string> enumerate_m300_xdma_candidates()
{
    glob_t matches {};
    std::set<std::string> unique;
    if (::glob("/dev/xdma*_h2c_0", 0, nullptr, &matches) == 0) {
        for (std::size_t i = 0; i < matches.gl_pathc; ++i) {
            const std::string base = normalize_m300_xdma_base(matches.gl_pathv[i]);
            if (required_nodes_available(base)) {
                unique.insert(base);
            }
        }
    }
    ::globfree(&matches);
    return std::vector<std::string>(unique.begin(), unique.end());
}

bool validate_m300_discovery_response(const uint8_t* data,
                                      std::size_t size,
                                      uint16_t expected_seq,
                                      uint64_t nonce,
                                      m300_discovery_info* info,
                                      std::string* error)
{
    if (!data || size < M300_RESP_BYTES) {
        set_error(error, "short M300 discovery response");
        return false;
    }

    const m300_resp_packet response = parse_resp_packet(data);
    if (response.hdr.magic_type != M300_MAGIC_RESP ||
        response.hdr.length != M300_RESP_BYTES) {
        set_error(error, "invalid M300 discovery response header");
        return false;
    }
    if (response.hdr.seq != expected_seq || response.hdr.sid != M300_DISCOVERY_SID ||
        response.cmd_id != M300_CMD_DISCOVER) {
        set_error(error, "M300 discovery response does not match request");
        return false;
    }
    if (response.status != M300_STATUS_OK) {
        set_error(error, "M300 discovery command rejected, status=" +
                         std::to_string(response.status));
        return false;
    }
    if (response.timestamp != (nonce ^ M300_DISCOVERY_NONCE_XOR)) {
        set_error(error, "M300 discovery nonce validation failed");
        return false;
    }
    if (response.value0 != M300_DEVICE_MAGIC || response.value1 != M300_PRODUCT_ID) {
        set_error(error, "XDMA endpoint is not an IQTAXI M300");
        return false;
    }

    const uint16_t protocol = static_cast<uint16_t>(response.value2 >> 16);
    const uint16_t capabilities = static_cast<uint16_t>(response.value2 & 0xffffu);
    if (protocol != M300_DISCOVERY_PROTOCOL ||
        (capabilities & M300_CAP_CONTROL) == 0u) {
        set_error(error, "unsupported M300 discovery protocol or capabilities");
        return false;
    }

    if (info) {
        info->protocol_version = protocol;
        info->capabilities = capabilities;
    }
    return true;
}

bool probe_m300_xdma(const std::string& path,
                     m300_discovery_info* info,
                     std::string* error,
                     double timeout_sec)
{
    const std::string base = normalize_m300_xdma_base(path);
    if (!required_nodes_available(base)) {
        set_error(error, "M300 XDMA channel set is incomplete or inaccessible: " + base);
        return false;
    }

    const std::string c2h_path = base + "_c2h_0";
    const std::string h2c_path = base + "_h2c_0";
    fd_guard c2h_fd(::open(c2h_path.c_str(), O_RDONLY));
    if (c2h_fd.get() < 0) {
        set_error(error, "failed to open discovery response channel " + c2h_path +
                         ": " + std::strerror(errno));
        return false;
    }

    uint32_t timeout_ms = static_cast<uint32_t>(
        std::max(0.001, timeout_sec) * 1000.0);
    if (::ioctl(c2h_fd.get(), IOCTL_XDMA_M300_SYNC_TIMEOUT_SET, &timeout_ms) != 0) {
        set_error(error, "XDMA driver does not provide bounded M300 discovery reads on " +
                         c2h_path + ": " + std::strerror(errno));
        return false;
    }

    fd_guard h2c_fd(::open(h2c_path.c_str(), O_WRONLY));
    if (h2c_fd.get() < 0) {
        set_error(error, "failed to open discovery request channel " + h2c_path +
                         ": " + std::strerror(errno));
        return false;
    }

    const uint64_t nonce = make_nonce();
    const uint16_t seq = static_cast<uint16_t>(nonce ^ (nonce >> 32));
    alignas(4096) std::array<uint8_t, 4096> request {};
    m300_ctrl_packet packet;
    packet.hdr.magic_type = M300_MAGIC_CTRL;
    packet.hdr.seq = seq;
    packet.hdr.sid = M300_DISCOVERY_SID;
    packet.hdr.length = M300_CTRL_BYTES;
    packet.timestamp = nonce;
    packet.cmd_id = M300_CMD_DISCOVER;
    packet.flags = M300_DISCOVERY_FLAGS;
    packet.target = M300_TARGET_LOCAL;
    packet.arg0 = static_cast<uint32_t>(nonce);
    packet.arg1 = static_cast<uint32_t>(nonce >> 32);
    packet.arg2 = M300_DISCOVERY_COOKIE;
    write_ctrl_packet(request.data(), packet);

    const ssize_t written = ::write(h2c_fd.get(), request.data(), M300_CTRL_BYTES);
    if (written != static_cast<ssize_t>(M300_CTRL_BYTES)) {
        set_error(error, "failed to send M300 discovery request on " + h2c_path +
                         ": " + (written < 0 ? std::string(std::strerror(errno)) :
                                             std::string("short write")));
        return false;
    }

    alignas(4096) std::array<uint8_t, 4096> response {};
    const ssize_t received = ::read(c2h_fd.get(), response.data(), response.size());
    if (received < 0) {
        set_error(error, "M300 discovery response timeout on " + c2h_path +
                         ": " + std::strerror(errno));
        return false;
    }

    m300_discovery_info discovered;
    if (!validate_m300_discovery_response(response.data(),
                                          static_cast<std::size_t>(received), seq, nonce,
                                          &discovered, error)) {
        return false;
    }

    discovered.addr = base;
    discovered.pci_bdf = pci_bdf_for_base(base);
    discovered.serial = "M300-" +
        (discovered.pci_bdf.empty() ? device_node_name(base) : discovered.pci_bdf);
    if (info) {
        *info = discovered;
    }
    if (error) {
        error->clear();
    }
    return true;
}

}} // namespace sdr::driver
