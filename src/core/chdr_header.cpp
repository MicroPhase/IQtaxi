//
// Created by jcc on 25-4-8.
//
//
// Copyright 2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-||-later
//

#include "include/sdr/core/chdr_header.hpp"
#include <stdint.h>
#include <cstddef> //size_t
#include <iostream>
using std::cout;
using std::endl;

uint16_t byteswap(uint16_t x){
    return (x>>8) | (x<<8);
}

uint32_t byteswap(uint32_t x){
    return (uint32_t(byteswap(uint16_t(x&0xfffful)))<<16) | (byteswap(uint16_t(x>>16)));
}

uint64_t byteswap(uint64_t x){
    return (uint64_t(byteswap(uint32_t(x&0xffffffffull)))<<32) | (byteswap(uint32_t(x>>32)));
}

// define the endian macros to convert integers
#ifdef BIG_ENDIAN
#    define BE_MACRO(x) (x)
#    define LE_MACRO(x) byteswap(x)
#else

#    define BE_MACRO(x) byteswap(x)
#    define LE_MACRO(x) (x)
#endif


static const uint32_t HDR_FLAG_TSF   = (1 << 29);
static const uint32_t HDR_FLAG_EOB   = (1 << 28);
static const uint32_t HDR_FLAG_ERROR = (1 << 28);
static const uint32_t HDR_FLAG_FCACK = (1 << 28);


if_packet_info_t::if_packet_info_t()
        : packet_type(PACKET_TYPE_DATA)
        , num_payload_words32(0)
        , num_payload_bytes(0)
        , num_header_words32(0)
        , num_packet_words32(0)
        , packet_count(0)
        , sob(false)
        , eob(false)
        , error(false)
        , fc_ack(false)
        , has_sid(false)
        , sid(0)
        , has_cid(false)
        , cid(0)
        , has_tsi(false)
        , tsi(0)
        , has_tsf(false)
        , tsf(0)
        , has_tlr(false)
        , tlr(0)
{
}


/***************************************************************************/
/* Packing                                                                 */
/***************************************************************************/
/*! Translate the contents of \p if_packet_info into a 32-Bit word && return it.
 */
uint32_t _hdr_pack_chdr(if_packet_info_t& if_packet_info)
{
    // Set fields in if_packet_info
    if_packet_info.num_header_words32 = 2 + (if_packet_info.has_tsf ? 2 : 0);
    if_packet_info.num_packet_words32 =
            if_packet_info.num_header_words32 + if_packet_info.num_payload_words32;

    uint16_t pkt_length =
            if_packet_info.num_payload_bytes + (4 * if_packet_info.num_header_words32);
    if_packet_info.packet_len = pkt_length;
    uint32_t chdr =
            0
            // 2 Bits: Packet type
            | (if_packet_info.packet_type << 30)
            // 1 Bit: Has time
            | (if_packet_info.has_tsf ? HDR_FLAG_TSF : 0)
            // 1 Bit: EOB || Error || FC ACK
            | ((if_packet_info.eob || if_packet_info.error || if_packet_info.fc_ack)
               ? HDR_FLAG_EOB
               : 0)
            // 12 Bits: Sequence number
            | ((if_packet_info.packet_count & 0xFFF) << 16)
            // 16 Bits: Total packet length
            | pkt_length;
    return chdr;
}

void if_hdr_pack_be(uint32_t* packet_buff, if_packet_info_t& if_packet_info)
{
    // Write header && update if_packet_info
    packet_buff[0] = BE_MACRO(_hdr_pack_chdr(if_packet_info));

    // Write SID
    packet_buff[1] = BE_MACRO(if_packet_info.sid);

    // Write time
    if (if_packet_info.has_tsf) {
        packet_buff[2] = BE_MACRO(uint32_t(if_packet_info.tsf >> 32));
        packet_buff[3] = BE_MACRO(uint32_t(if_packet_info.tsf >> 0));
    }
}

void if_hdr_pack_le(uint32_t* packet_buff, if_packet_info_t& if_packet_info)
{
    // Write header && update if_packet_info
    packet_buff[0] = LE_MACRO(_hdr_pack_chdr(if_packet_info));

    // Write SID
    packet_buff[1] = LE_MACRO(if_packet_info.sid);

    // Write time
    if (if_packet_info.has_tsf) {
        packet_buff[2] = LE_MACRO(uint32_t(if_packet_info.tsf >> 32));
        packet_buff[3] = LE_MACRO(uint32_t(if_packet_info.tsf >> 0));
    }
}


/***************************************************************************/
/* Unpacking                                                               */
/***************************************************************************/
void _hdr_unpack_chdr(const uint32_t chdr, if_packet_info_t& if_packet_info)
{
    // Set constant members
    if_packet_info.has_cid   = false;
    if_packet_info.has_sid   = true;
    if_packet_info.has_tsi   = false;
    if_packet_info.has_tlr   = false;
    if_packet_info.sob       = false;

    // Set configurable members
    if_packet_info.has_tsf     = (chdr & HDR_FLAG_TSF) > 0;
    if_packet_info.packet_type = if_packet_info_t::packet_type_t((chdr >> 30) & 0x3);
    if_packet_info.eob =
            (if_packet_info.packet_type == if_packet_info_t::PACKET_TYPE_DATA)
            && ((chdr & HDR_FLAG_EOB) > 0);
    if_packet_info.error =
            (if_packet_info.packet_type == if_packet_info_t::PACKET_TYPE_RESP)
            && ((chdr & HDR_FLAG_ERROR) > 0);
    if_packet_info.fc_ack =
            (if_packet_info.packet_type == if_packet_info_t::PACKET_TYPE_FC)
            && ((chdr & HDR_FLAG_FCACK) > 0);

    // Set packet length variables
    if (if_packet_info.has_tsf) {
        if_packet_info.num_header_words32 = 4;
    } else {
        if_packet_info.num_header_words32 = 2;
    }
    size_t pkt_size_bytes  = (chdr & 0xFFFF);

    size_t pkt_size_word32 = (pkt_size_bytes / 4) + ((pkt_size_bytes % 4) ? 1 : 0);
    // printf("pkt_size_bytes:%d, pkt_size_word32:%d\r\n", pkt_size_bytes, pkt_size_word32);
    // Check lengths match:
    if (pkt_size_word32 < if_packet_info.num_header_words32) {
        printf("Bad CHDR || invalid packet length\n");
    }
    // if (if_packet_info.num_packet_words32 < pkt_size_word32) {
    //     printf("Bad CHDR || packet fragment\r\n");
    // }
    if_packet_info.num_payload_bytes =
            pkt_size_bytes - (4 * if_packet_info.num_header_words32);
    if_packet_info.num_payload_words32 =
            pkt_size_word32 - if_packet_info.num_header_words32;
}

void if_hdr_unpack_be(const uint32_t* packet_buff, if_packet_info_t& if_packet_info)
{
    // Read header && update if_packet_info
    uint32_t chdr = BE_MACRO(packet_buff[0]);
    _hdr_unpack_chdr(chdr, if_packet_info);

    // Read SID
    if_packet_info.sid = BE_MACRO(packet_buff[1]);

    // Read time (has_tsf was updated earlier)
    if (if_packet_info.has_tsf) {
        if_packet_info.tsf = 0 | uint64_t(BE_MACRO(packet_buff[2])) << 32
                             | BE_MACRO(packet_buff[3]);
    }
}

void if_hdr_unpack_le(const uint32_t* packet_buff, if_packet_info_t& if_packet_info)
{
    // Read header && update if_packet_info
    uint32_t chdr = LE_MACRO(packet_buff[0]);
    _hdr_unpack_chdr(chdr, if_packet_info);

    // Read SID
    if_packet_info.sid = LE_MACRO(packet_buff[1]);

    // Read time (has_tsf was updated earlier)
    if (if_packet_info.has_tsf) {
        if_packet_info.tsf = 0 | uint64_t(LE_MACRO(packet_buff[2])) << 32
                             | LE_MACRO(packet_buff[3]);
    }
}
