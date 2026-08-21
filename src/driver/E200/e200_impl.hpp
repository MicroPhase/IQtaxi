#ifndef SOAPY_E200_IMPL_HPP
#define SOAPY_E200_IMPL_HPP

#include "../IQTAXI/iqtaxi_udp_impl.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#define MICROPHASE_E200_UDP_RECORD_PORT 49209

class E200Impl : public IqtaxiUdpImpl {
public:
    enum class VcxoReferenceSource : uint32_t {
        pps = 0u,
        external_10mhz = 1u,
        manual_dac = 2u,
    };

    struct VcxoStatus {
        bool locked = false;
        bool reference_valid = false;
        bool reference_is_10mhz = false;
        bool reference_is_pps = false;
        VcxoReferenceSource selected_source = VcxoReferenceSource::pps;
        uint16_t dac_value = 0u;
        uint32_t raw = 0u;
    };

    static constexpr uint32_t RECORD_STATUS_DONE = 0x1u;
    static constexpr uint32_t RECORD_STATUS_ACTIVE = 0x2u;
    static constexpr uint32_t RECORD_STATUS_ERROR = 0x4u;

    explicit E200Impl(const std::string port);
    ~E200Impl() override = default;

    void set_vcxo_reference_source(VcxoReferenceSource source);
    void set_vcxo_manual_dac(uint16_t value);
    VcxoStatus get_vcxo_status();

    void configure_iq_record(uint32_t length_bytes, uint32_t dma_block_size);
    void start_iq_record(uint32_t length_bytes = 0);
    uint32_t get_iq_record_status();
    bool is_iq_record_done();
    uint32_t get_iq_record_transfered_len();
    uint32_t get_iq_record_dma_offset();
    uint32_t get_iq_record_last_chunk_bytes();
    size_t read_iq_record_chunk(void* dst, size_t max_bytes, double timeout_sec = 1.0);
    std::vector<uint8_t> read_iq_record_all(double timeout_sec = 1.0);

    void configure_iq_replay(uint32_t length_bytes);
    void start_iq_replay(uint32_t length_bytes = 0);
    void stop_iq_replay();
    uint32_t get_iq_replay_length_bytes();
    uint32_t get_iq_replay_dma_offset();
    uint32_t get_iq_replay_last_chunk_bytes();
    void set_iq_replay_packet_gap_us(uint32_t gap_us);
    size_t write_iq_replay_chunk(const void* src, size_t bytes, double timeout_sec = 1.0);
    size_t write_iq_replay_all(const void* src, size_t bytes, uint32_t chunk_bytes = 4u * 1024u * 1024u, double timeout_sec = 1.0);

private:
    void send_replay_iq_payload(const void* src, uint32_t bytes, uint64_t seqno, double timeout_sec);

    zero_copy_if::sptr _record_udp;
    std::shared_ptr<local_ctrl> _record_data_bus;
    uint32_t _replay_packet_gap_us = 25u;
};

#endif
