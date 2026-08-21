#ifndef __LOCAL_E100_REGS_H__
#define __LOCAL_E100_REGS_H__

#include "cstdint"

namespace e100{
    constexpr uint32_t CUSTOM_SET_RX_CH1_GAIN_ADDR            =  0x0000;
    constexpr uint32_t CUSTOM_SET_RX_CH2_GAIN_ADDR            =  0x0001;
    constexpr uint32_t CUSTOM_SET_TX_CH1_ATTEN_ADDR            =  0x0002;
    constexpr uint32_t CUSTOM_SET_TX_CH2_ATTEN_ADDR            =  0x0003;
    constexpr uint32_t CUSTOM_SET_RX_CH1_AGC_MODE_ADDR        =  0x0004;
    constexpr uint32_t CUSTOM_SET_RX_CH2_AGC_MODE_ADDR        =  0x0005;
 
    constexpr uint32_t CUSTOM_SET_SAMPLE_CLOCK_RATE_ADDR      =  0x0006;
 
    constexpr uint32_t CUSTOM_SET_ACTIVE_CHANNEL_ADDR         =  0x0007;
 
 
    constexpr uint32_t CUSTOM_SET_RX_CH1_LO_FREQ_LOW_ADDR     =  0x0008;
    constexpr uint32_t CUSTOM_SET_RX_CH1_LO_FREQ_HIGH_ADDR    =  0x0009;
    constexpr uint32_t CUSTOM_SET_TX_CH1_LO_FREQ_LOW_ADDR     =  0x000a;
    constexpr uint32_t CUSTOM_SET_TX_CH1_LO_FREQ_HIGH_ADDR    =  0x000b;
 
    constexpr uint32_t CUSTOM_SET_TIMING_MODE_ADDR            =  0x000c;
    constexpr uint32_t CUSTOM_SET_TIME_MODE_ADDR              =  0x000d;
    constexpr uint32_t CUSTOM_SET_VITA_TIMESTAMP_LOW_ADDR     =  0x000e;
    constexpr uint32_t CUSTOM_SET_VITA_TIMESTAMP_HIGH_ADDR    =  0x000f;
    constexpr uint32_t CUSTOM_SET_TX_TIMESTAMP_LOW_ADDR       =  0x0010;
    constexpr uint32_t CUSTOM_SET_TX_TIMESTAMP_HIGH_ADDR      =  0x0011;
 
    constexpr uint32_t CUSTOM_SET_CHANNEL_ENABLE_ADDR         =  0x0012;
 
    constexpr uint32_t CUSTOM_SET_RX_SAMPLE_NUMS_ADDR         =  0x0013;
    constexpr uint32_t CUSTOM_SET_CAPTURE_START_ADDR          =  0x0014;
    constexpr uint32_t CUSTOM_SET_RX_MODE                     =  0x0015;
    constexpr uint32_t CUSTOM_SET_RX_MODE_EXIT                =  0x0016;
    constexpr uint32_t CUSTOM_SET_RX_STREAM_START             =  0x0017;
    constexpr uint32_t CUSTOM_SET_RX_MAX_PACKET_BYTES         =  0x0018;
    constexpr uint32_t CUSTOM_SET_SYNC_RX_TIMESTAMPS_HI       =  0x0019;
    constexpr uint32_t CUSTOM_SET_SYNC_RX_TIMESTAMPS_LO       =  0x001a;
    constexpr uint32_t CUSTOM_SET_START_RX                    =  0x001b;
    constexpr uint32_t CUSTOM_SET_STOP_RX                     =  0x001c;
    constexpr uint32_t CUSTOM_SET_DMA_S2MM_PKT_PER_BURST      =  0x001d;
 
    constexpr uint32_t CUSTOM_SET_TX_SAMPLES_PER_PACKET       =  0x001e;
    constexpr uint32_t CUSTOM_SET_TX_SOURCE_SEL               =  0x001f;
    constexpr uint32_t CUSTOM_SET_TX_IGNORE_TIMESTAMPS        =  0x0020;
    constexpr uint32_t CUSTOM_SET_TX_NOISE_CFG_START_IDX      =  0x0021;
    constexpr uint32_t CUSTOM_SET_TX_NOISE_CFG_STOP_IDX       =  0x0022;
    constexpr uint32_t CUSTOM_SET_TX_DDS_CTRL_WORD            =  0x0023;
 
    constexpr uint32_t CUSTOM_SET_SWAP_IQ                     =  0x0024;
    constexpr uint32_t CUSTOM_SET_START_TX                    =  0x0025;
    constexpr uint32_t CUSTOM_SET_STOP_TX                     =  0x0026;
 
    constexpr uint32_t CUSTOM_SET_XFFT_ENABLE_ADDR            =  0x0027;
    constexpr uint32_t CUSTOM_SET_XFFT_OVERLAP_ADDR           =  0x0028;
    constexpr uint32_t CUSTOM_SET_XFFT_FFT_LEN                =  0x0029;
    constexpr uint32_t CUSTOM_SET_TIME_SYNC                   =  0x002a;
    constexpr uint32_t CUSTOM_SET_SAMPLE_RATE_DY              =  0x002c;
    constexpr uint32_t CUSTOM_SET_DMA_MODE                    =  0x002d;
    constexpr uint32_t CUSTOM_SET_LVDS_IF_RST                 =  0x002e;
    constexpr uint32_t CUSTOM_SET_RX_CH1_LO_FREQ_ADDR         =  0x0031;
    constexpr uint32_t CUSTOM_SET_TX_CH1_LO_FREQ_ADDR         =  0x0032;
    constexpr uint32_t CUSTOM_SET_VITA_TIMESTAMP_ADDR         =  0x0033;
    constexpr uint32_t CUSTOM_SET_TX_TIMESTAMP_ADDR           =  0x0034;
    constexpr uint32_t CUSTOM_SET_RECORD_DMA_BLOCK_SIZE       =  0x0035;
    constexpr uint32_t CUSTOM_SET_RECORD_LENGTH_BYTES         =  0x0036;
    constexpr uint32_t CUSTOM_SET_RECORD_START                =  0x0037;
    constexpr uint32_t CUSTOM_SET_RECORD_DMA_READ_NEXT        =  0x0038;
    constexpr uint32_t CUSTOM_SET_REPLAY_LENGTH_BYTES         =  0x0039;
    constexpr uint32_t CUSTOM_SET_REPLAY_START                =  0x003a;
    constexpr uint32_t CUSTOM_SET_REPLAY_STOP                 =  0x003b;
    constexpr uint32_t CUSTOM_SET_REPLAY_DMA_WRITE_NEXT       =  0x003c;
    constexpr uint32_t CUSTOM_SET_VCXO_REF_SOURCE             =  0x003d;
    constexpr uint32_t CUSTOM_SET_VCXO_DAC_VALUE              =  0x003e;
 
    constexpr uint32_t CUSTOM_SET_RB_ADDR                     =  0x0030;
 
    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_GAIN_ADDR         =  0x0001;
    constexpr uint32_t CUSTOM_RB_GET_RX_CH2_GAIN_ADDR         =  0x0002;
    constexpr uint32_t CUSTOM_RB_GET_TX_CH1_ATTEN_ADDR         =  0x0003;
    constexpr uint32_t CUSTOM_RB_GET_TX_CH2_ATTEN_ADDR         =  0x0004;
 
    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_AGC_MODE_ADDR     =  0x0005;
    constexpr uint32_t CUSTOM_RB_GET_RX_CH2_AGC_MODE_ADDR     =  0x0006;
 
    constexpr uint32_t CUSTOM_RB_GET_SAMPLE_CLOCK_RATE_ADDR   =  0x0007;
    constexpr uint32_t CUSTOM_RB_GET_ACTIVE_CHANNEL_ADDR      =  0x0008;

    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_LO_FREQ_LOW_ADDR  =  0x0009;
    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_LO_FREQ_HIGH_ADDR =  0x000a;
    constexpr uint32_t CUSTOM_RB_GET_TX_CH1_LO_FREQ_LOW_ADDR  =  0x000b;
    constexpr uint32_t CUSTOM_RB_GET_TX_CH1_LO_FREQ_HIGH_ADDR =  0x000c;

    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_RSSI_ADDR         =  0x000d;
    constexpr uint32_t CUSTOM_RB_GET_RX_CH2_RSSI_ADDR         =  0x000e;
 
    constexpr uint32_t CUSTOM_RB_GET_TIMING_MODE_ADDR         =  0x000f;
    constexpr uint32_t CUSTOM_RB_GET_VITA_TIME_ADDR           =  0x0010;
    constexpr uint32_t CUSTOM_RB_GET_VITA_TIME_LAST_PPS_ADDR  =  0x0011;
 
    constexpr uint32_t CUSTOM_RB_GET_RX_PACKET                =  0x0012;
    constexpr uint32_t CUSTOM_RB_XFFT_ONE_BLOCK_ADDR          =  0x0013;
    constexpr uint32_t CUSTOM_RB_TX_STREAM_BUFFER_STATUS      =  0x0014;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_STATUS            =  0x0015;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_LENGTH_BYTES      =  0x0016;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_DMA_BLOCK_SIZE    =  0x0017;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_TRANSFERED_LEN    =  0x0018;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_DMA_OFFSET        =  0x0019;
    constexpr uint32_t CUSTOM_RB_GET_RECORD_DMA_LAST_BYTES    =  0x001a;
    constexpr uint32_t CUSTOM_RB_GET_REPLAY_LENGTH_BYTES      =  0x001b;
    constexpr uint32_t CUSTOM_RB_GET_REPLAY_DMA_OFFSET        =  0x001c;
    constexpr uint32_t CUSTOM_RB_GET_REPLAY_DMA_LAST_BYTES    =  0x001d;
    constexpr uint32_t CUSTOM_RB_GET_RX_CH1_LO_FREQ_ADDR      =  0x001e;
    constexpr uint32_t CUSTOM_RB_GET_TX_CH1_LO_FREQ_ADDR      =  0x001f;
    constexpr uint32_t CUSTOM_RB_GET_VCXO_STATUS              =  0x0020;

    constexpr uint32_t CUSTOM_SR_CORE_SPI      = 0x1100+8;
    constexpr uint32_t CUSTOM_SR_CORE_MISC     = 0x1100+16;
    constexpr uint32_t CUSTOM_SR_CORE_COMPAT   = 0x1100+24;
    constexpr uint32_t CUSTOM_SR_CORE_GPSDO_ST = 0x1100+40;
    constexpr uint32_t CUSTOM_SR_CORE_SYNC     = 0x1100+48;
    constexpr uint32_t CUSTOM_RB32_CORE_SPI    = 0x1100+8;
    constexpr uint32_t CUSTOM_RB32_CORE_MISC   = 0x1100+16;
    constexpr uint32_t CUSTOM_RB32_CORE_STATUS = 0x1100+20;
    constexpr uint32_t CUSTOM_RB32_CORE_PLL    = 0x1100+24;
    constexpr uint32_t CUSTOM_SR_SPI          = 0x1100+8;
    constexpr uint32_t CUSTOM_SR_ATR          = 0x1100+12;
    constexpr uint32_t CUSTOM_SR_TEST         = 0x1100+21;
    constexpr uint32_t CUSTOM_SR_CODEC_IDLE   = 0x1100+22;
    constexpr uint32_t CUSTOM_SR_READBACK     = 0x1100+32;
    constexpr uint32_t CUSTOM_SR_TX_CTRL      = 0x1100+64;
    constexpr uint32_t CUSTOM_SR_RX_CTRL      = 0x1100+96;
    constexpr uint32_t CUSTOM_SR_RX_DSP       = 0x1100+144;
    constexpr uint32_t CUSTOM_SR_TX_DSP       = 0x1100+184;
    constexpr uint32_t CUSTOM_SR_TIME         = 0x1100+128;
    constexpr uint32_t CUSTOM_SR_RX_FMT       = 0x1100+136;
    constexpr uint32_t CUSTOM_SR_TX_FMT       = 0x1100+138;
    constexpr uint32_t CUSTOM_SR_FP_GPIO      = 0x1100+200;
    constexpr uint32_t CUSTOM_SR_USER_SR_BASE = 0x1100+253;
    constexpr uint32_t CUSTOM_SR_USER_RB_ADDR = 0x1100+255;
    constexpr uint32_t CUSTOM_RB32_TEST           = 0x1100+0;
    constexpr uint32_t RB64_TIME_NOW       = 0x1100+8;
    constexpr uint32_t RB64_TIME_PPS       = 0x1100+16;
    constexpr uint32_t RB64_CODEC_READBACK = 0x1100+24;
    constexpr uint32_t CUSTOM_RB32_FP_GPIO        = 0x1100+32;
}

#endif
