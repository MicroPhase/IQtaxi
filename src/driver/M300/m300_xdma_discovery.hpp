#ifndef IQTAXI_M300_XDMA_DISCOVERY_HPP
#define IQTAXI_M300_XDMA_DISCOVERY_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sdr { namespace driver {

struct m300_discovery_info
{
    std::string addr;
    std::string pci_bdf;
    std::string serial;
    uint16_t protocol_version = 0;
    uint16_t capabilities = 0;
};

std::string normalize_m300_xdma_base(const std::string& path);
std::vector<std::string> enumerate_m300_xdma_candidates();

bool validate_m300_discovery_response(const uint8_t* data,
                                      std::size_t size,
                                      uint16_t expected_seq,
                                      uint64_t nonce,
                                      m300_discovery_info* info,
                                      std::string* error);

bool probe_m300_xdma(const std::string& path,
                     m300_discovery_info* info = nullptr,
                     std::string* error = nullptr,
                     double timeout_sec = 0.25);

}} // namespace sdr::driver

#endif // IQTAXI_M300_XDMA_DISCOVERY_HPP
