/***************************************************************************//**
 *   @file   ad9361/src/parameters.h
 *   @brief  Parameters Definitions.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__




#define AD9361_RX_0_BASEADDR		0x43C00000
#define AD9361_TX_0_BASEADDR		0x43C00000 + 0x4000
#define RX_CORE_BASEADDR	        AD9361_RX_0_BASEADDR
#define TX_CORE_BASEADDR	        AD9361_TX_0_BASEADDR

#define AD9361_RX_1_BASEADDR		0x80010000
#define AD9361_TX_1_BASEADDR		0x80010000 + 0x4000

#define AD9361_RX_2_BASEADDR		0x80020000
#define AD9361_TX_2_BASEADDR		0x80020000 + 0x4000

#define AD9361_RX_3_BASEADDR		0x80030000
#define AD9361_TX_3_BASEADDR		0x80030000 + 0x4000

//#define AD9361_RX_2_BASEADDR		XPAR_AXI_AD9361_2_BASEADDR
//#define AD9361_TX_2_BASEADDR		XPAR_AXI_AD9361_2_BASEADDR + 0x4000
//
//#define AD9361_RX_3_BASEADDR		XPAR_AXI_AD9361_3_BASEADDR
//#define AD9361_TX_3_BASEADDR		XPAR_AXI_AD9361_3_BASEADDR + 0x4000

#define DAC_BUFFER_SAMPLES 1024
#define ADC_BUFFER_SAMPLES 16384
#define ADC_CHANNELS 4

#define SPI_DEVICE_ID		0
#define SPI_CS			    0
#define SPI_CS_B			1
#define SPI_CS_C			2
#define SPI_CS_D			3
#define SPI_CS_ADF4002		4


/*
 * GPIO numbers are gpiochip-local offsets used with /dev/gpiochipN.
 * On Zynq-7000, EMIO GPIO starts after 54 MIO lines. e200_iqtaxi.v connects
 * EMIO[14:0] to {resetb, sync, en_agc, ctl[3:0], status[7:0]}.
 */
#define ZYNQ_GPIO_EMIO_BASE		54
#define GPIO_SYNC_PIN			(ZYNQ_GPIO_EMIO_BASE + 13)
#define GPIO_RESET_PIN			(ZYNQ_GPIO_EMIO_BASE + 14)


#endif // __PARAMETERS_H__
