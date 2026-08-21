#ifndef SOAPY_M300_XDMA_PROTOCOL_HPP
#define SOAPY_M300_XDMA_PROTOCOL_HPP

#include <cstdint>
#include <cstring>

namespace sdr { namespace driver {

static constexpr uint16_t M300_MAGIC_CTRL = 0x5501u;
static constexpr uint16_t M300_MAGIC_RESP = 0x5502u;
static constexpr uint16_t M300_MAGIC_RX   = 0x5503u;
static constexpr uint16_t M300_MAGIC_TX   = 0x5504u;
static constexpr uint16_t M300_MAGIC_FC   = 0x5505u;

static constexpr uint32_t M300_CTRL_BYTES = 32u;
static constexpr uint32_t M300_RESP_BYTES = 32u;
static constexpr uint32_t M300_HDR_BYTES   = 16u;

static constexpr uint16_t M300_STATUS_OK         = 0x0000u;
static constexpr uint16_t M300_STATUS_BAD_MAGIC  = 0x0001u;
static constexpr uint16_t M300_STATUS_BAD_LENGTH = 0x0002u;
static constexpr uint16_t M300_STATUS_BAD_CMD     = 0x0003u;
static constexpr uint16_t M300_STATUS_BAD_SID     = 0x0004u;
static constexpr uint16_t M300_STATUS_BUSY        = 0x0005u;
static constexpr uint16_t M300_STATUS_DENIED      = 0x0006u;
static constexpr uint16_t M300_STATUS_TIMEOUT     = 0x0007u;

static constexpr uint16_t M300_CMD_NOP                 = 0x0000u;
static constexpr uint16_t M300_CMD_GET_VERSION         = 0x0001u;
static constexpr uint16_t M300_CMD_WRITE_REG           = 0x0002u;
static constexpr uint16_t M300_CMD_READ_REG            = 0x0003u;
static constexpr uint16_t M300_CMD_DISCOVER            = 0x0004u;
static constexpr uint16_t M300_CMD_GET_STATUS          = 0x0010u;
static constexpr uint16_t M300_CMD_SET_RX_PACKET_BYTES = 0x0020u;
static constexpr uint16_t M300_CMD_START_RX            = 0x0021u;
static constexpr uint16_t M300_CMD_STOP_RX             = 0x0022u;
static constexpr uint16_t M300_CMD_SET_RX_SID          = 0x0023u;
static constexpr uint16_t M300_CMD_SET_TIMESTAMP       = 0x0030u;
static constexpr uint16_t M300_CMD_GET_TIMESTAMP       = 0x0031u;
static constexpr uint16_t M300_CMD_CLEAR_COUNTERS      = 0x0040u;
static constexpr uint16_t M300_CMD_GET_COUNTER         = 0x0041u;

static constexpr uint8_t M300_TARGET_LOCAL      = 0x00u;
static constexpr uint8_t M300_TARGET_AXI_LITE   = 0x20u;
static constexpr uint8_t M300_TARGET_AD9361_SPI = 0x21u;

static constexpr uint8_t M300_DISCOVERY_FLAGS = 0xa5u;
static constexpr uint8_t M300_DISCOVERY_SID = 0xd3u;
static constexpr uint32_t M300_DISCOVERY_COOKIE = 0x44534356u; // "DSCV"
static constexpr uint64_t M300_DISCOVERY_NONCE_XOR = 0x495154584d333030ull; // "IQTXM300"
static constexpr uint32_t M300_DEVICE_MAGIC = 0x49515458u; // "IQTX"
static constexpr uint32_t M300_PRODUCT_ID = 0x4d333030u; // "M300"
static constexpr uint16_t M300_DISCOVERY_PROTOCOL = 0x0001u;

static constexpr uint16_t M300_CAP_CONTROL = 1u << 0;
static constexpr uint16_t M300_CAP_RX = 1u << 1;
static constexpr uint16_t M300_CAP_TX = 1u << 2;
static constexpr uint16_t M300_CAP_AD9361 = 1u << 3;
static constexpr uint16_t M300_CAP_TIMESTAMP = 1u << 4;
static constexpr uint16_t M300_CAPABILITIES =
    M300_CAP_CONTROL | M300_CAP_RX | M300_CAP_TX |
    M300_CAP_AD9361 | M300_CAP_TIMESTAMP;
static constexpr uint32_t M300_DISCOVERY_INFO =
    (static_cast<uint32_t>(M300_DISCOVERY_PROTOCOL) << 16) | M300_CAPABILITIES;

static constexpr uint32_t M300_REG_TIMESTAMP = 0x00010010u;
static constexpr uint32_t M300_REG_RX_PACKET_BYTES = 0x00010018u;
static constexpr uint32_t M300_REG_STREAM_ENABLE = 0x00010028u;
static constexpr uint32_t M300_REG_GPIO_OUT = 0x00010040u;
static constexpr uint32_t M300_REG_GPIO_OE  = 0x00010048u;
static constexpr uint32_t M300_REG_GPIO_IN  = 0x00010050u;
static constexpr uint32_t M300_REG_RX_SOURCE_SEL = 0x00010060u;
static constexpr uint32_t M300_REG_DEVICE_MAGIC = 0x00010068u;
static constexpr uint32_t M300_REG_PRODUCT_ID = 0x00010070u;
static constexpr uint32_t M300_REG_DISCOVERY_PROTOCOL = 0x00010078u;
static constexpr uint32_t M300_REG_CAPABILITIES = 0x00010080u;
static constexpr uint32_t M300_REG_MULTIBOOT_ADDR   = 0x00010088u;
static constexpr uint32_t M300_REG_MULTIBOOT_CTRL   = 0x00010090u;
static constexpr uint32_t M300_REG_MULTIBOOT_STATUS = 0x00010098u;
static constexpr uint32_t M300_MULTIBOOT_UNLOCK     = 0x4d424f54u; // "MBOT"
static constexpr uint32_t M300_REG_FLASH_MODE         = 0x000100a0u;
static constexpr uint32_t M300_REG_FLASH_ERASE_ADDR   = 0x000100a8u;
static constexpr uint32_t M300_REG_FLASH_ERASE_COUNT  = 0x000100b0u;
static constexpr uint32_t M300_REG_FLASH_PROGRAM_ADDR = 0x000100b8u;
static constexpr uint32_t M300_REG_FLASH_COMMAND      = 0x000100c0u;
static constexpr uint32_t M300_REG_FLASH_STATUS       = 0x000100c8u;
static constexpr uint32_t M300_REG_FLASH_FIFO_LEVEL   = 0x000100d0u;
static constexpr uint32_t M300_REG_FLASH_READ_ADDR    = 0x000100d8u;
static constexpr uint32_t M300_REG_FLASH_READ_BYTES   = 0x000100e0u;
static constexpr uint32_t M300_FLASH_MODE_ENABLE = 0x46555044u; // "FUPD"
static constexpr uint32_t M300_FLASH_CMD_ERASE   = 0x45524153u; // "ERAS"
static constexpr uint32_t M300_FLASH_CMD_PROGRAM = 0x50524f47u; // "PROG"
static constexpr uint32_t M300_FLASH_CMD_STOP    = 0x53544f50u; // "STOP"
static constexpr uint32_t M300_FLASH_CMD_ACK4K   = 0x41434b34u; // "ACK4"
static constexpr uint32_t M300_FLASH_CMD_READ    = 0x52454144u; // "READ"
static constexpr uint32_t M300_FLASH_BYTES       = 0x02000000u;
static constexpr uint32_t M300_FLASH_ONLINE_BYTES = 0x01000000u;
static constexpr uint32_t M300_FLASH_IMAGE_ADDR = 0x00000000u;
static constexpr uint32_t M300_FLASH_STATUS_MODE       = 1u << 0;
static constexpr uint32_t M300_FLASH_STATUS_BUSY       = 1u << 1;
static constexpr uint32_t M300_FLASH_STATUS_ERASE_DONE = 1u << 2;
static constexpr uint32_t M300_FLASH_STATUS_4K_DONE    = 1u << 3;
static constexpr uint32_t M300_FLASH_STATUS_READ_DONE  = 1u << 4;
static constexpr uint32_t M300_FLASH_STATUS_READ_OVERFLOW = 1u << 5;

static constexpr uint32_t M300_GPIO_AD9361_RESETB = 0u;
static constexpr uint32_t M300_GPIO_AD9361_SYNC   = 1u;
static constexpr uint32_t M300_GPIO_AD9361_EN_AGC = 2u;
static constexpr uint32_t M300_GPIO_AD9361_CTL0   = 3u;
static constexpr uint32_t M300_GPIO_AD9361_CTL1   = 4u;
static constexpr uint32_t M300_GPIO_AD9361_CTL2   = 5u;
static constexpr uint32_t M300_GPIO_AD9361_CTL3   = 6u;
static constexpr uint32_t M300_GPIO_AD9361_ENABLE = 7u;
static constexpr uint32_t M300_GPIO_AD9361_TXNRX  = 8u;
static constexpr uint32_t M300_GPIO_IN_AD9361_RESETB_MASK = 1u << 0;
static constexpr uint32_t M300_GPIO_IN_LP8758_DONE_MASK   = 1u << 24;

struct m300_header
{
    uint16_t magic_type = 0;
    uint16_t seq = 0;
    uint8_t sid = 0;
    uint32_t length = 0;
};

struct m300_ctrl_packet
{
    m300_header hdr;
    uint64_t timestamp = 0;
    uint16_t cmd_id = 0;
    uint8_t flags = 0;
    uint8_t target = 0;
    uint32_t arg0 = 0;
    uint32_t arg1 = 0;
    uint32_t arg2 = 0;
};

struct m300_resp_packet
{
    m300_header hdr;
    uint64_t timestamp = 0;
    uint16_t cmd_id = 0;
    uint16_t status = 0;
    uint32_t value0 = 0;
    uint32_t value1 = 0;
    uint32_t value2 = 0;
};

inline uint64_t load_le64(const uint8_t* p)
{
    return ((uint64_t)p[0]) |
           ((uint64_t)p[1] << 8) |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

inline void store_le64(uint8_t* p, uint64_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
    p[4] = static_cast<uint8_t>(v >> 32);
    p[5] = static_cast<uint8_t>(v >> 40);
    p[6] = static_cast<uint8_t>(v >> 48);
    p[7] = static_cast<uint8_t>(v >> 56);
}

inline m300_header parse_header(const uint8_t* p)
{
    const uint64_t raw = load_le64(p);
    m300_header h;
    h.length = raw & 0x00ffffffu;
    h.sid = static_cast<uint8_t>((raw >> 24) & 0xffu);
    h.seq = static_cast<uint16_t>((raw >> 32) & 0xffffu);
    h.magic_type = static_cast<uint16_t>((raw >> 48) & 0xffffu);
    return h;
}

inline void write_header(uint8_t* p, const m300_header& h)
{
    const uint64_t raw = (static_cast<uint64_t>(h.magic_type) << 48) |
                         (static_cast<uint64_t>(h.seq) << 32) |
                         (static_cast<uint64_t>(h.sid) << 24) |
                         (static_cast<uint64_t>(h.length) & 0x00ffffffu);
    store_le64(p, raw);
}

inline void write_ctrl_packet(uint8_t* p, const m300_ctrl_packet& pkt)
{
    write_header(p, pkt.hdr);
    store_le64(p + 8, pkt.timestamp);
    p[16] = static_cast<uint8_t>(pkt.cmd_id & 0xffu);
    p[17] = static_cast<uint8_t>((pkt.cmd_id >> 8) & 0xffu);
    p[18] = pkt.flags;
    p[19] = pkt.target;
    p[20] = static_cast<uint8_t>(pkt.arg0);
    p[21] = static_cast<uint8_t>(pkt.arg0 >> 8);
    p[22] = static_cast<uint8_t>(pkt.arg0 >> 16);
    p[23] = static_cast<uint8_t>(pkt.arg0 >> 24);
    p[24] = static_cast<uint8_t>(pkt.arg1);
    p[25] = static_cast<uint8_t>(pkt.arg1 >> 8);
    p[26] = static_cast<uint8_t>(pkt.arg1 >> 16);
    p[27] = static_cast<uint8_t>(pkt.arg1 >> 24);
    p[28] = static_cast<uint8_t>(pkt.arg2);
    p[29] = static_cast<uint8_t>(pkt.arg2 >> 8);
    p[30] = static_cast<uint8_t>(pkt.arg2 >> 16);
    p[31] = static_cast<uint8_t>(pkt.arg2 >> 24);
}

inline m300_ctrl_packet parse_ctrl_packet(const uint8_t* p)
{
    m300_ctrl_packet pkt;
    pkt.hdr = parse_header(p);
    pkt.timestamp = load_le64(p + 8);
    pkt.cmd_id = static_cast<uint16_t>(p[16] | (static_cast<uint16_t>(p[17]) << 8));
    pkt.flags = p[18];
    pkt.target = p[19];
    pkt.arg0 = static_cast<uint32_t>(p[20]) |
               (static_cast<uint32_t>(p[21]) << 8) |
               (static_cast<uint32_t>(p[22]) << 16) |
               (static_cast<uint32_t>(p[23]) << 24);
    pkt.arg1 = static_cast<uint32_t>(p[24]) |
               (static_cast<uint32_t>(p[25]) << 8) |
               (static_cast<uint32_t>(p[26]) << 16) |
               (static_cast<uint32_t>(p[27]) << 24);
    pkt.arg2 = static_cast<uint32_t>(p[28]) |
               (static_cast<uint32_t>(p[29]) << 8) |
               (static_cast<uint32_t>(p[30]) << 16) |
               (static_cast<uint32_t>(p[31]) << 24);
    return pkt;
}

inline void write_resp_packet(uint8_t* p, const m300_resp_packet& pkt)
{
    write_header(p, pkt.hdr);
    store_le64(p + 8, pkt.timestamp);
    p[16] = static_cast<uint8_t>(pkt.cmd_id & 0xffu);
    p[17] = static_cast<uint8_t>((pkt.cmd_id >> 8) & 0xffu);
    p[18] = static_cast<uint8_t>(pkt.status & 0xffu);
    p[19] = static_cast<uint8_t>((pkt.status >> 8) & 0xffu);
    p[20] = static_cast<uint8_t>(pkt.value0);
    p[21] = static_cast<uint8_t>(pkt.value0 >> 8);
    p[22] = static_cast<uint8_t>(pkt.value0 >> 16);
    p[23] = static_cast<uint8_t>(pkt.value0 >> 24);
    p[24] = static_cast<uint8_t>(pkt.value1);
    p[25] = static_cast<uint8_t>(pkt.value1 >> 8);
    p[26] = static_cast<uint8_t>(pkt.value1 >> 16);
    p[27] = static_cast<uint8_t>(pkt.value1 >> 24);
    p[28] = static_cast<uint8_t>(pkt.value2);
    p[29] = static_cast<uint8_t>(pkt.value2 >> 8);
    p[30] = static_cast<uint8_t>(pkt.value2 >> 16);
    p[31] = static_cast<uint8_t>(pkt.value2 >> 24);
}

inline m300_resp_packet parse_resp_packet(const uint8_t* p)
{
    m300_resp_packet pkt;
    pkt.hdr = parse_header(p);
    pkt.timestamp = load_le64(p + 8);
    pkt.cmd_id = static_cast<uint16_t>(p[16] | (static_cast<uint16_t>(p[17]) << 8));
    pkt.status = static_cast<uint16_t>(p[18] | (static_cast<uint16_t>(p[19]) << 8));
    pkt.value0 = static_cast<uint32_t>(p[20]) |
                 (static_cast<uint32_t>(p[21]) << 8) |
                 (static_cast<uint32_t>(p[22]) << 16) |
                 (static_cast<uint32_t>(p[23]) << 24);
    pkt.value1 = static_cast<uint32_t>(p[24]) |
                 (static_cast<uint32_t>(p[25]) << 8) |
                 (static_cast<uint32_t>(p[26]) << 16) |
                 (static_cast<uint32_t>(p[27]) << 24);
    pkt.value2 = static_cast<uint32_t>(p[28]) |
                 (static_cast<uint32_t>(p[29]) << 8) |
                 (static_cast<uint32_t>(p[30]) << 16) |
                 (static_cast<uint32_t>(p[31]) << 24);
    return pkt;
}

}} // namespace sdr::driver

#endif // SOAPY_M300_XDMA_PROTOCOL_HPP
