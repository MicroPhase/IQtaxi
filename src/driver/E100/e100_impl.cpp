#include "./e100_impl.hpp"
#include "./local_e100_regs.hpp"
#include "include/sdr/api/SampleRates.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace e100;

namespace {
constexpr uint32_t kIqPacketHeaderBytes = 16u;
constexpr uint32_t kUdpPayloadBytes = 1500u - 20u - 8u;
constexpr uint32_t kIqPayloadBytes = kUdpPayloadBytes - kIqPacketHeaderBytes;
constexpr double kRecordDataHelloTimeoutSec = 0.1;
constexpr uint32_t kMiB = 1024u * 1024u;
constexpr uint32_t kE100RecordLengthGranularityBytes = 4u * kMiB;
constexpr uint32_t kE100RecordLengthMinBytes = kE100RecordLengthGranularityBytes;
constexpr uint32_t kE100RecordLengthMaxBytes = 240u * kMiB;
constexpr size_t kE100RecordSocketBufferBytes = 64u * 1024u * 1024u;
constexpr size_t kE100RecordRecvFrames = 2048u;
constexpr size_t kE100RecordSendFrames = 2048u;

double remaining_timeout_sec(const std::chrono::steady_clock::time_point& deadline)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
        return 0.0;
    }
    return std::chrono::duration<double>(deadline - now).count();
}

void throw_if_command_failed(uint64_t ack_value, const char* command)
{
    if (ack_value == 0u) {
        return;
    }
    throw std::runtime_error(std::string("E100 IQ record/replay command failed: ") + command);
}

void throw_if_vcxo_command_failed(uint64_t ack_value, const char* operation)
{
    if (ack_value != 0u) {
        throw std::runtime_error(std::string("E100 VCXO command failed: ") + operation);
    }
}

void validate_record_length(uint32_t length_bytes)
{
    if (length_bytes < kE100RecordLengthMinBytes ||
        length_bytes > kE100RecordLengthMaxBytes ||
        (length_bytes % kE100RecordLengthGranularityBytes) != 0u) {
        throw std::invalid_argument(
            "E100 IQ record length must be 4..240 MiB and a multiple of 4 MiB");
    }
}

void validate_replay_length(uint32_t length_bytes)
{
    if (length_bytes == 0u || length_bytes > kE100RecordLengthMaxBytes ||
        (length_bytes & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ replay length must be 4 bytes..240 MiB and 4-byte aligned");
    }
}

uint32_t nearest_e100_sample_rate(double rate)
{
    const uint32_t requested_rate = rate <= 0.0 ? 0u : static_cast<uint32_t>(rate + 0.5);

    return *std::min_element(
        sdr::api::kGc080xLegacySampleRatesHz.begin(),
        sdr::api::kGc080xLegacySampleRatesHz.end(),
        [requested_rate](uint32_t lhs, uint32_t rhs) {
            const uint32_t lhs_delta =
                lhs > requested_rate ? lhs - requested_rate : requested_rate - lhs;
            const uint32_t rhs_delta =
                rhs > requested_rate ? rhs - requested_rate : requested_rate - rhs;
            return lhs_delta < rhs_delta;
        });
}

void send_record_data_hello(const std::shared_ptr<local_ctrl>& data_bus)
{
    managed_send_buffer::sptr send_buffer =
        data_bus->get_xport()->get_send_buff(kRecordDataHelloTimeoutSec, kIqPacketHeaderBytes);
    if (!send_buffer || send_buffer->size() < kIqPacketHeaderBytes) {
        throw std::runtime_error("E100 failed to allocate record data hello buffer");
    }

    sdr_header_t hdr{};
    hdr.magic_type = PACKET_TYPE_TX_IQ;
    hdr.seq = 0u;
    hdr.sid = 0x73;
    hdr.packet_len = kIqPacketHeaderBytes;
    hdr.timestamp = 0u;

    data_bus->serialize_hdr(send_buffer->cast<uint32_t*>(), hdr);
    send_buffer->commit(kIqPacketHeaderBytes);
}
} // namespace

E100Impl::E100Impl(const std::string port)
    : E100Impl(port, e100_udp_profile())
{
}

E100Impl::E100Impl(const std::string port, const sdr::api::DeviceProfile& profile)
    : IqtaxiUdpImpl(port, profile)
{
    zero_copy_xport_params record_buff_args;
    record_buff_args.send_frame_size = kUdpPayloadBytes;
    record_buff_args.recv_frame_size = kUdpPayloadBytes;
    record_buff_args.num_send_frames = kE100RecordSendFrames;
    record_buff_args.num_recv_frames = kE100RecordRecvFrames;
    record_buff_args.send_buff_size = kE100RecordSocketBufferBytes;
    record_buff_args.recv_buff_size = kE100RecordSocketBufferBytes;

    _record_udp = udp_zero_copy::make(
        port, SDR_STRINGIZE(MICROPHASE_E100_UDP_RECORD_PORT), record_buff_args);
    _record_data_bus = std::make_shared<local_ctrl>(_record_udp, 0x73, 8192);
    // E100 firmware receives record/replay commands on the record data port.
    // E206 uses the newer split control/data-port protocol and needs this
    // packet to establish the return path for record data.
    if (profile.product != "E100") {
        send_record_data_hello(_record_data_bus);
    }
}

void E100Impl::setSampleRate(double rate)
{
    IqtaxiUdpImpl::setSampleRate(static_cast<double>(nearest_e100_sample_rate(rate)));
}

void E100Impl::set_tx_freq(uint64_t tx_lo, size_t channel)
{
    IqtaxiUdpImpl::set_tx_freq(tx_lo, channel);
}

void E100Impl::set_vcxo_reference_source(VcxoReferenceSource source)
{
    throw_if_vcxo_command_failed(
        get_local_bus()->poke32_ack_value(
            CUSTOM_SET_E100_VCXO_REF_SOURCE, static_cast<uint32_t>(source), 1.0),
        "set reference source");
}

void E100Impl::set_vcxo_manual_dac(uint16_t value)
{
    throw_if_vcxo_command_failed(
        get_local_bus()->poke32_ack_value(CUSTOM_SET_E100_VCXO_DAC_VALUE, value, 1.0),
        "set manual DAC");
}

E100Impl::VcxoStatus E100Impl::get_vcxo_status()
{
    VcxoStatus status;
    status.raw = get_local_bus()->peek32(CUSTOM_RB_GET_E100_VCXO_STATUS, 1.0);
    status.locked = (status.raw & (1u << 0)) != 0u;
    status.reference_valid = (status.raw & (1u << 1)) != 0u;
    status.reference_is_10mhz = (status.raw & (1u << 2)) != 0u;
    status.reference_is_pps = (status.raw & (1u << 3)) != 0u;
    const uint32_t source = (status.raw >> 4) & 0x3u;
    status.selected_source = source <= static_cast<uint32_t>(VcxoReferenceSource::manual_dac)
                                 ? static_cast<VcxoReferenceSource>(source)
                                 : VcxoReferenceSource::pps;
    status.dac_value = static_cast<uint16_t>(status.raw >> 16);
    return status;
}

void E100Impl::configure_iq_record(uint32_t length_bytes, uint32_t dma_block_size)
{
    if (length_bytes == 0u || dma_block_size == 0u) {
        throw std::invalid_argument("E100 IQ record length and DMA block size must be non-zero");
    }
    if ((length_bytes & 0x3u) != 0u || (dma_block_size & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ record length and DMA block size must be 4-byte aligned");
    }
    validate_record_length(length_bytes);

    auto local_bus = get_local_bus();
    throw_if_command_failed(
        local_bus->poke32_ack_value(CUSTOM_SET_RECORD_DMA_BLOCK_SIZE, dma_block_size, 1.0),
        "set record DMA block size");
    throw_if_command_failed(
        local_bus->poke32_ack_value(CUSTOM_SET_RECORD_LENGTH_BYTES, length_bytes, 1.0),
        "set record length");
}

void E100Impl::start_iq_record(uint32_t length_bytes)
{
    if ((length_bytes & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ record length must be 4-byte aligned");
    }
    if (length_bytes != 0u) {
        validate_record_length(length_bytes);
    }

    throw_if_command_failed(
        get_local_bus()->poke32_ack_value(CUSTOM_SET_RECORD_START, length_bytes, 1.0),
        "start record");
}

uint32_t E100Impl::get_iq_record_status()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_RECORD_STATUS);
}

bool E100Impl::is_iq_record_done()
{
    return (get_iq_record_status() & RECORD_STATUS_DONE) != 0u;
}

uint32_t E100Impl::get_iq_record_transfered_len()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_RECORD_TRANSFERED_LEN);
}

uint32_t E100Impl::get_iq_record_dma_offset()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_RECORD_DMA_OFFSET);
}

uint32_t E100Impl::get_iq_record_last_chunk_bytes()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_RECORD_DMA_LAST_BYTES);
}

size_t E100Impl::read_iq_record_chunk(void* dst, size_t max_bytes, double timeout_sec)
{
    if (!dst) {
        throw std::invalid_argument("E100 IQ record destination buffer is null");
    }
    if (max_bytes == 0u) {
        return 0u;
    }

    auto data_bus = _record_data_bus;
    uint32_t chunk_bytes = 0u;
    if (get_profile().product == "E100") {
        chunk_bytes = static_cast<uint32_t>(
            data_bus->poke32_ack_value(CUSTOM_SET_RECORD_DMA_READ_NEXT, 0u, timeout_sec));
    } else {
        send_record_data_hello(data_bus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        chunk_bytes = static_cast<uint32_t>(
            get_local_bus()->poke32_ack_value(
                CUSTOM_SET_RECORD_DMA_READ_NEXT, 0u, timeout_sec));
    }
    if (chunk_bytes == 0u) {
        return 0u;
    }
    if (chunk_bytes == std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("E100 IQ record chunk request failed");
    }
    if (chunk_bytes > max_bytes) {
        throw std::runtime_error("E100 IQ record destination buffer is smaller than the next DMA chunk");
    }

    uint8_t* out = static_cast<uint8_t*>(dst);
    size_t copied = 0u;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));

    while (copied < chunk_bytes) {
        const double remaining = remaining_timeout_sec(deadline);
        if (remaining <= 0.0) {
            throw std::runtime_error("E100 IQ record data receive timeout");
        }

        managed_recv_buffer::sptr recv_buffer = data_bus->get_xport()->get_recv_buff(remaining);
        if (!recv_buffer || recv_buffer->size() < kIqPacketHeaderBytes) {
            throw std::runtime_error("E100 IQ record data receive timeout");
        }

        uint32_t hdr_words[4] = {};
        std::memcpy(hdr_words, recv_buffer->cast<const void*>(), sizeof(hdr_words));

        sdr_header_t hdr{};
        data_bus->deserialize_hdr(hdr_words, hdr);
        if (hdr.magic_type != PACKET_TYPE_RX_IQ ||
            hdr.packet_len < kIqPacketHeaderBytes ||
            recv_buffer->size() < kIqPacketHeaderBytes) {
            continue;
        }

        const size_t packet_bytes = std::min<size_t>(hdr.packet_len, recv_buffer->size());
        const size_t payload_bytes = packet_bytes - kIqPacketHeaderBytes;
        const size_t bytes_to_copy = std::min<size_t>(payload_bytes, chunk_bytes - copied);
        if (bytes_to_copy == 0u) {
            continue;
        }

        const uint8_t* payload =
            recv_buffer->cast<const uint8_t*>() + kIqPacketHeaderBytes;
        std::memcpy(out + copied, payload, bytes_to_copy);
        copied += bytes_to_copy;
    }

    return copied;
}

std::vector<uint8_t> E100Impl::read_iq_record_all(double timeout_sec)
{
    const uint32_t length_bytes = get_local_bus()->peek32(CUSTOM_RB_GET_RECORD_LENGTH_BYTES);
    std::vector<uint8_t> data(length_bytes);
    size_t offset = 0u;

    if (length_bytes == 0u) {
        return data;
    }
    if (!is_iq_record_done()) {
        throw std::runtime_error("E100 IQ record is not done yet");
    }

    while (offset < data.size()) {
        const size_t got = read_iq_record_chunk(data.data() + offset,
                                                data.size() - offset,
                                                timeout_sec);
        if (got == 0u) {
            throw std::runtime_error("E100 IQ record ended before the configured length");
        }
        offset += got;
    }

    return data;
}

void E100Impl::configure_iq_replay(uint32_t length_bytes)
{
    validate_replay_length(length_bytes);
    auto command_bus =
        get_profile().product == "E100" ? _record_data_bus : get_local_bus();
    throw_if_command_failed(
        command_bus->poke32_ack_value(CUSTOM_SET_REPLAY_LENGTH_BYTES, length_bytes, 1.0),
        "set replay length");
}

void E100Impl::start_iq_replay(uint32_t length_bytes)
{
    if (length_bytes != 0u) {
        validate_replay_length(length_bytes);
    }
    auto command_bus =
        get_profile().product == "E100" ? _record_data_bus : get_local_bus();
    throw_if_command_failed(
        command_bus->poke32_ack_value(CUSTOM_SET_REPLAY_START, length_bytes, 1.0),
        "start replay");
}

void E100Impl::stop_iq_replay()
{
    auto command_bus =
        get_profile().product == "E100" ? _record_data_bus : get_local_bus();
    throw_if_command_failed(
        command_bus->poke32_ack_value(CUSTOM_SET_REPLAY_STOP, 1u, 1.0),
        "stop replay");
}

void E100Impl::poke_amp_enable(bool enable)
{
    get_local_bus()->poke32(CUSTOM_SET_AMP_ENABLE, enable ? 1u : 0u);
}

void E100Impl::set_amp_enable(bool enable)
{
    if (get_profile().product != "E100") {
        throw std::logic_error("AMP control is only available on E100");
    }
    poke_amp_enable(enable);
    _amp_enable = enable;
    if (get_amp_enable() != enable) {
        throw std::runtime_error(
            enable ? "E100 AMP enable failed (firmware readback still off)"
                   : "E100 AMP disable failed (firmware readback still on)");
    }
}

bool E100Impl::get_amp_enable()
{
    if (get_profile().product != "E100") {
        throw std::logic_error("AMP status is only available on E100");
    }
    // Firmware exposes AMP state as a readback selector on 0xFFFF, not a SET.
    return get_local_bus()->peek32(CUSTOM_GET_AMP_ENABLE) != 0u;
}

void E100Impl::set_rf_front_dsa_att(uint32_t attenuation)
{
    if (get_profile().product != "E100") {
        throw std::logic_error("RF front-end DSA control is only available on E100");
    }
    get_local_bus()->poke32(CUSTOM_SET_RF_FRONT_DSA_ATT, attenuation);
}

uint32_t E100Impl::get_iq_replay_length_bytes()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_REPLAY_LENGTH_BYTES);
}

uint32_t E100Impl::get_iq_replay_dma_offset()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_REPLAY_DMA_OFFSET);
}

uint32_t E100Impl::get_iq_replay_last_chunk_bytes()
{
    return get_local_bus()->peek32(CUSTOM_RB_GET_REPLAY_DMA_LAST_BYTES);
}

void E100Impl::set_iq_replay_packet_gap_us(uint32_t gap_us)
{
    _replay_packet_gap_us = gap_us;
}

void E100Impl::send_replay_iq_payload(const void* src, uint32_t bytes, uint64_t seqno, double timeout_sec)
{
    const uint8_t* in = static_cast<const uint8_t*>(src);
    uint32_t sent = 0u;
    uint16_t seq = 0u;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));

    while (sent < bytes) {
        const uint32_t payload_bytes = std::min<uint32_t>(kIqPayloadBytes, bytes - sent);
        const uint32_t packet_bytes = kIqPacketHeaderBytes + payload_bytes;
        const double remaining = remaining_timeout_sec(deadline);
        if (remaining <= 0.0) {
            throw std::runtime_error("E100 IQ replay data send timeout");
        }

        managed_send_buffer::sptr send_buffer =
            _record_data_bus->get_xport()->get_send_buff(remaining, packet_bytes);
        if (!send_buffer || send_buffer->size() < packet_bytes) {
            throw std::runtime_error("E100 IQ replay failed to allocate UDP send buffer");
        }

        sdr_header_t hdr{};
        hdr.magic_type = PACKET_TYPE_TX_IQ;
        hdr.seq = seq++;
        hdr.sid = 0x73;
        hdr.packet_len = packet_bytes;
        hdr.timestamp = seqno + sent;

        uint32_t* hdr_words = send_buffer->cast<uint32_t*>();
        _record_data_bus->serialize_hdr(hdr_words, hdr);
        std::memcpy(send_buffer->cast<uint8_t*>() + kIqPacketHeaderBytes,
                    in + sent,
                    payload_bytes);
        send_buffer->commit(packet_bytes);
        if (get_profile().product == "E100") {
            // E100 needs each packet submitted before the pacing delay.
            send_buffer.reset();
        }
        sent += payload_bytes;
        if (sent < bytes) {
            if (_replay_packet_gap_us != 0u) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(_replay_packet_gap_us));
            }
        }
    }
}

size_t E100Impl::write_iq_replay_chunk(const void* src, size_t bytes, double timeout_sec)
{
    if (!src) {
        throw std::invalid_argument("E100 IQ replay source buffer is null");
    }
    if (bytes == 0u) {
        return 0u;
    }
    if (bytes > std::numeric_limits<uint32_t>::max() || (bytes & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ replay chunk must be 4-byte aligned and fit in uint32_t");
    }

    const uint32_t chunk_bytes = static_cast<uint32_t>(bytes);
    const uint64_t chunk_offset = get_iq_replay_dma_offset();
    const bool e100_record_protocol = get_profile().product == "E100";
    auto command_bus = e100_record_protocol ? _record_data_bus : get_local_bus();
    command_bus->send_pkt(CUSTOM_SET_REPLAY_DMA_WRITE_NEXT, chunk_bytes);
    if (!e100_record_protocol) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    send_replay_iq_payload(src, chunk_bytes, chunk_offset, timeout_sec);

    const uint64_t ack = command_bus->wait_for_ack(true, timeout_sec);
    if (ack != chunk_bytes) {
        std::ostringstream oss;
        oss << "E100 IQ replay DMA write failed: ack=0x"
            << std::hex << ack << " expected=0x" << chunk_bytes;
        throw std::runtime_error(oss.str());
    }
    return chunk_bytes;
}

size_t E100Impl::write_iq_replay_all(const void* src, size_t bytes, uint32_t chunk_bytes, double timeout_sec)
{
    if (!src) {
        throw std::invalid_argument("E100 IQ replay source buffer is null");
    }
    if (chunk_bytes == 0u || (chunk_bytes & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ replay chunk size must be non-zero and 4-byte aligned");
    }
    if ((bytes & 0x3u) != 0u) {
        throw std::invalid_argument("E100 IQ replay total byte count must be 4-byte aligned");
    }

    const uint8_t* in = static_cast<const uint8_t*>(src);
    size_t offset = 0u;
    while (offset < bytes) {
        const size_t todo = std::min<size_t>(chunk_bytes, bytes - offset);
        offset += write_iq_replay_chunk(in + offset, todo, timeout_sec);
    }
    return offset;
}

std::shared_ptr<local_ctrl> E100Impl::get_local_bus()
{
    return IqtaxiUdpImpl::get_local_bus();
}

std::shared_ptr<local_ctrl> E100Impl::get_rx_stream_bus()
{
    return IqtaxiUdpImpl::get_rx_stream_bus();
}

std::shared_ptr<local_ctrl> E100Impl::get_tx_stream_bus()
{
    return IqtaxiUdpImpl::get_tx_stream_bus();
}
