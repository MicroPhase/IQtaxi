#ifndef SOAPY_XDMA_ZERO_COPY_HPP
#define SOAPY_XDMA_ZERO_COPY_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include "zero_copy.hpp"

namespace sdr { namespace core {

        struct xdma_zero_copy_params
        {
            std::string device;
            std::string recv_device;
            std::string send_device;
            size_t packet_bytes = 0;
            size_t packet_stride = 0;
            bool compact_strided_packets = false;
            bool eop_flush = false;
            bool driver_ring_recv = false;
            uint32_t driver_ring_depth = 256;
        };

        class API_EXPORT xdma_zero_copy : public zero_copy_if {
        public:
            typedef std::shared_ptr<xdma_zero_copy> sptr;

            static sptr make(const std::string& device,
                    const zero_copy_xport_params& default_buff_args);

            static sptr make(const xdma_zero_copy_params& xdma_params,
                    const zero_copy_xport_params& default_buff_args);

            virtual std::string get_device(void) const = 0;
            virtual size_t get_packet_bytes(void) const = 0;
            virtual size_t get_packet_stride(void) const = 0;
            virtual bool start_recv(double timeout = 1.0) { (void)timeout; return true; }
            virtual void stop_recv(void) {}
            virtual size_t recv_payload_burst(void* buffer,
                                              size_t buffer_bytes,
                                              size_t payload_stride,
                                              size_t max_packets,
                                              uint64_t& first_timestamp,
                                              uint64_t& first_sequence,
                                              double timeout = 1.0)
            {
                (void)buffer;
                (void)buffer_bytes;
                (void)payload_stride;
                (void)max_packets;
                (void)first_timestamp;
                (void)first_sequence;
                (void)timeout;
                return 0;
            }
        };

}} // namespace sdr::core

#endif // SOAPY_XDMA_ZERO_COPY_HPP
