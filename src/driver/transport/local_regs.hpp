#ifndef __LOCAL_REGS_H_
#define __LOCAL_REGS_H_

#include "cstdint"

constexpr uint8_t STREAM_MODE = 0x1;
constexpr uint8_t PACKET_MODE = 0x2;
constexpr uint8_t SYNC_MODE = 0x3;

constexpr uint32_t SET_CMD_PORT  =  0x0404;

constexpr uint32_t SET_CHANNEL_ENABLE_ADDR         =  0x0012;
constexpr uint32_t SET_RX_SAMPLE_NUMS_ADDR         =  0x0013;
constexpr uint32_t SET_CAPTURE_START_ADDR          =  0x0014;
constexpr uint32_t SET_RX_MODE                     =  0x0015;
constexpr uint32_t SET_RX_MODE_EXIT                =  0x0016;
constexpr uint32_t SET_RX_MAX_PACKET_BYTES         =  0x0018;
constexpr uint32_t SET_SYNC_RX_TIMESTAMPS_HI       =  0x0019;
constexpr uint32_t SET_SYNC_RX_TIMESTAMPS_LO       =  0x001a;
constexpr uint32_t SET_START_RX                    =  0x001b;
constexpr uint32_t SET_STOP_RX                     =  0x001c;
constexpr uint32_t SET_TX_SAMPLES_PER_PACKET       =  0x001e;
constexpr uint32_t SET_TX_SOURCE_SEL               =  0x001f;
constexpr uint32_t SET_RECORD_DMA_BLOCK_SIZE       =  0x0035;
constexpr uint32_t SET_RECORD_LENGTH_BYTES         =  0x0036;
constexpr uint32_t SET_RECORD_START                =  0x0037;
constexpr uint32_t SET_RECORD_DMA_READ_NEXT        =  0x0038;

constexpr uint32_t RB_GET_RECORD_STATUS            =  0x0015;
constexpr uint32_t RB_GET_RECORD_LENGTH_BYTES      =  0x0016;
constexpr uint32_t RB_GET_RECORD_DMA_BLOCK_SIZE    =  0x0017;
constexpr uint32_t RB_GET_RECORD_TRANSFERED_LEN    =  0x0018;
constexpr uint32_t RB_GET_RECORD_DMA_OFFSET        =  0x0019;
constexpr uint32_t RB_GET_RECORD_DMA_LAST_BYTES    =  0x001a;


#endif
