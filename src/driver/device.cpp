#include "include/sdr/api/Device.hpp"
#include "./E100/e100_impl.hpp"
#include "./E200/e200_impl.hpp"
#include "./E206/e206_impl.hpp"
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#ifndef _WIN32
#include "./M300/m300_xdma_impl.hpp"
#include "./M300/m300_xdma_discovery.hpp"
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

using namespace sdr::api;
using namespace sdr::core;

namespace {
#ifndef _WIN32
class device_access_lease
{
public:
    explicit device_access_lease(int fd) : _fd(fd) {}

    ~device_access_lease()
    {
        if (_fd >= 0) {
            (void)::flock(_fd, LOCK_UN);
            (void)::close(_fd);
        }
    }

    device_access_lease(const device_access_lease&) = delete;
    device_access_lease& operator=(const device_access_lease&) = delete;

private:
    int _fd;
};

uint64_t stable_device_hash(const std::string& value)
{
    // FNV-1a is deterministic across executables and library versions,
    // unlike std::hash whose persistence is not part of the C++ contract.
    uint64_t hash = UINT64_C(14695981039346656037);
    for (const unsigned char byte : value) {
        hash ^= byte;
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

std::string lock_path_for(const std::string& addr)
{
    std::ostringstream name;
    name << "/run/lock/iqtaxi-device-" << std::hex << std::setw(16)
         << std::setfill('0') << stable_device_hash(addr) << ".lock";
    return name.str();
}

std::shared_ptr<void> acquire_device_access(const std::string& device,
                                            const std::string& addr)
{
    const std::string identity = addr.empty() ? "default" : addr;
    const std::string path = lock_path_for(identity);
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0666);
    if (fd < 0) {
        throw std::runtime_error(
            "IQTAXI cannot create device lock " + path + ": " + std::strerror(errno));
    }

    // Each backend invokes this at the start of its constructor, before it
    // creates transports or sends HELLO/control commands. Therefore a
    // rejected process cannot alter device state.
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        const int lock_error = errno;
        char owner[256] = {};
        (void)::lseek(fd, 0, SEEK_SET);
        const ssize_t owner_len = ::read(fd, owner, sizeof(owner) - 1u);
        (void)::close(fd);

        std::string detail;
        if (owner_len > 0) {
            detail.assign(owner, static_cast<size_t>(owner_len));
            while (!detail.empty() && (detail.back() == '\n' || detail.back() == '\r')) {
                detail.pop_back();
            }
        }
        if (lock_error == EWOULDBLOCK || lock_error == EAGAIN) {
            throw std::runtime_error(
                "IQTAXI device " + identity + " is busy" +
                (detail.empty() ? std::string() : " (owner: " + detail + ")"));
        }
        throw std::runtime_error(
            "IQTAXI cannot lock device " + identity + ": " + std::strerror(lock_error));
    }

    // Make a lock created under a restrictive umask usable by another local
    // user after this process exits. flock itself prevents concurrent access.
    (void)::fchmod(fd, 0666);
    const std::string owner = "pid=" + std::to_string(static_cast<long long>(::getpid())) +
                              " backend=" + device + " addr=" + identity + "\n";
    const int truncate_result = ::ftruncate(fd, 0);
    (void)::lseek(fd, 0, SEEK_SET);
    const ssize_t write_result = ::write(fd, owner.data(), owner.size());
    (void)truncate_result;
    (void)write_result;
    return std::make_shared<device_access_lease>(fd);
}
#else
std::shared_ptr<void> acquire_device_access(const std::string&, const std::string&)
{
    return {};
}
#endif
} // namespace

namespace sdr::api {
    void Device::acquire_exclusive_access(const std::string& backend,
                                          const std::string& address) {
        _exclusive_access_lease = acquire_device_access(backend, address);
    }

    Device::sptr Device::makeDevice(const std::string device,const std::string addr) {
        if(device == "E100" || device == "e100" ||
            device == "E100-6G" || device == "E100-10G" ||
            device == "E100_6G" || device == "E100_10G" ||
            device == "ANTSDR_E100" ||
            device == "ANTSDR-E100" ||
            device == "ANTSDR-E100-6G" ||
            device == "ANTSDR-E100-10G"){
            auto dev = std::make_shared<E100Impl>(addr);
            if(dev->isInitialSuccess())
                return dev;
        }
        else if(device == "E206" || device == "e206" ||
                device == "ANTSDR_E206" ||
                device == "ANTSDR-E206"){
            auto dev = std::make_shared<E206Impl>(addr);
            if(dev->isInitialSuccess())
                return dev;
        }
        else if(device == "E200" || device == "e200" ||
            device == "ANTSDR_E200" ||
            device == "ANTSDr_E200"){
            auto dev = std::make_shared<E200Impl>(addr);
            if(dev->isInitialSuccess())
                return dev;
        }
#ifndef _WIN32
        else if (device == "M300" || device == "M300_XDMA" ||
                 device == "M300_PCIE" || device == "FNIC_XDMA") {
            const std::string path = addr.empty() ? "/dev/xdma0" : addr;
            sdr::driver::m300_discovery_info info;
            std::string error;
            if (!sdr::driver::probe_m300_xdma(path, &info, &error)) {
                return nullptr;
            }
            auto dev = std::make_shared<M300XdmaImpl>(info.addr);
            if (dev->isInitialSuccess()) {
                return dev;
            }
        }
#endif
        return nullptr;
    }

}
