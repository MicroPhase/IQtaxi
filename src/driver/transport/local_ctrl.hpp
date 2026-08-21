//
// Created by jcc on 25-4-8.
//

#ifndef SOAPY_LOCAL_CTRL_HPP
#define SOAPY_LOCAL_CTRL_HPP

#include "../../../include/sdr/core/udp_zero_copy.hpp"
#include "../../../include/sdr/core/chdr_header.hpp"
#include "../../../include/sdr/core/time_spec.hpp"
#include <mutex>
#include "memory"


#define SR_CORE_READBACK 0xFFFF

using namespace sdr::core;

enum packet_type_t {
    // VRT language:
    PACKET_TYPE_CTRL    = 0x5501,
    PACKET_TYPE_RESP    = 0x5502,
    PACKET_TYPE_RX_IQ   = 0x5503,
    PACKET_TYPE_TX_IQ   = 0x5504,
    PACKET_TYPE_TX_FC   = 0x5505,
};


typedef struct  
{
    uint16_t magic_type;
    uint16_t seq;
    uint8_t  sid;
    uint32_t packet_len;
    uint64_t timestamp;
    
} sdr_header_t;

// TX IQ packet flags share the sequence field without changing the legacy
// 16-byte header.  This mirrors CHDR's 12-bit packet count and leaves the
// packet length, magic and SID fields untouched for the existing UOE route.
constexpr uint16_t SDR_TX_SEQ_MASK          = 0x0FFFu;
constexpr uint16_t SDR_TX_FLAG_HAS_TIME     = 0x1000u;
constexpr uint16_t SDR_TX_FLAG_EOB          = 0x2000u;
constexpr uint16_t SDR_TX_FLAG_SOB          = 0x4000u;
constexpr uint16_t SDR_TX_FLAG_PROTOCOL_V1  = 0x8000u;

inline uint16_t encode_tx_seq_flags(uint16_t sequence,
                                    bool flags_valid,
                                    bool has_time,
                                    bool start_of_burst,
                                    bool end_of_burst)
{
    uint16_t value = sequence & SDR_TX_SEQ_MASK;
    if (!flags_valid) {
        return sequence;
    }
    value |= SDR_TX_FLAG_PROTOCOL_V1;
    if (has_time) value |= SDR_TX_FLAG_HAS_TIME;
    if (start_of_burst) value |= SDR_TX_FLAG_SOB;
    if (end_of_burst) value |= SDR_TX_FLAG_EOB;
    return value;
}




class local_ctrl{
public:
    typedef std::shared_ptr<local_ctrl> sptr;

    local_ctrl(zero_copy_if::sptr& xport,uint32_t sid);
    local_ctrl(zero_copy_if::sptr& xport,uint32_t sid, uint32_t buf_len);
    ~local_ctrl();

    void poke32(uint32_t addr,uint32_t data);
    void poke32(uint32_t addr,uint32_t data,double ack_timeout);
    uint64_t poke32_ack_value(uint32_t addr,uint32_t data,double ack_timeout);
    void poke64(uint32_t addr,uint64_t data);
    void poke64(uint32_t addr,uint64_t data,double ack_timeout);
    uint64_t poke64_ack_value(uint32_t addr,uint64_t data,double ack_timeout);
    uint32_t peek32(uint32_t addr);
    uint32_t peek32(uint32_t addr,double ack_timeout);
    uint64_t peek64(uint32_t addr);
    uint64_t peek64(uint32_t addr,double ack_timeout);

    void serialize_hdr(uint32_t * buf, sdr_header_t &  hdr);
    void deserialize_hdr(uint32_t * buf, sdr_header_t &  hdr);
    void send_pkt(uint32_t addr, uint32_t data);
    void send_pkt64(uint32_t addr, uint64_t data);
    uint64_t wait_for_ack(bool read_back);
    uint64_t wait_for_ack(bool read_back,double timeout);


    void set_time(time_spec_t &time);
    time_spec_t get_time();
    void set_tick_rate(double rate);

    void clear_seq();

    void set_rx_buf_size(uint32_t len);
    void set_tx_buf_size(uint32_t len);

    void rx_buf_resize(uint32_t len);
    void tx_buf_resize(uint32_t len);

    zero_copy_if::sptr get_xport()
    {
        return _xport;
    }

private:
    bool _has_sid;
    uint32_t _sid;
    bool _has_tsf;
    uint64_t _timestamp;
    time_spec_t _time;
    double _tick_rate;
    uint16_t _tx_seq;
    uint16_t _rx_seq;
    uint32_t * _send_buf;
    uint32_t * _recv_buf;
    uint32_t _rx_buf_len;
    uint32_t _tx_buf_len;

    zero_copy_if::sptr& _xport;
    std::mutex _ctrl_mutex;
};



#endif //SOAPY_LOCAL_CTRL_HPP
