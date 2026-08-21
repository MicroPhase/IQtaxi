#include "src/driver/M300/m300_xdma_discovery.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    std::vector<std::string> candidates;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--addr" && i + 1 < argc) {
            candidates.push_back(
                sdr::driver::normalize_m300_xdma_base(argv[++i]));
        } else if (arg == "--help") {
            std::cout << "Usage: m300_discovery_probe [--addr /dev/xdmaN]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    if (candidates.empty()) {
        candidates = sdr::driver::enumerate_m300_xdma_candidates();
    }
    if (candidates.empty()) {
        std::cerr << "No complete XDMA channel sets found\n";
        return 2;
    }

    bool found = false;
    for (const std::string& candidate : candidates) {
        sdr::driver::m300_discovery_info info;
        std::string error;
        if (!sdr::driver::probe_m300_xdma(candidate, &info, &error)) {
            std::cout << candidate << " rejected: " << error << "\n";
            continue;
        }

        found = true;
        std::cout << info.addr
                  << " M300 serial=" << info.serial
                  << " pci=" << (info.pci_bdf.empty() ? "unknown" : info.pci_bdf)
                  << " protocol=" << info.protocol_version
                  << " capabilities=0x" << std::hex << std::setw(4)
                  << std::setfill('0') << info.capabilities << std::dec << "\n";
    }

    return found ? 0 : 3;
}
