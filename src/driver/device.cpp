#include "include/sdr/api/Device.hpp"
#include "./E100/e100_impl.hpp"
#include "./E200/e200_impl.hpp"
#include "./E206/e206_impl.hpp"
#ifndef _WIN32
#include "./M300/m300_xdma_impl.hpp"
#include "./M300/m300_xdma_discovery.hpp"
#endif

using namespace sdr::api;
using namespace sdr::core;

namespace sdr::api {
    Device::sptr Device::makeDevice(const std::string device,const std::string addr) {
        if(device == "E100" || device == "e100" ||
            device == "ANTSDR_E100" ||
            device == "ANTSDR-E100"){
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
