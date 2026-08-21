/***************************************************************************//**
 *   @file   no_os_spi.c
 *   @brief  Implementation of the SPI Interface
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#include <inttypes.h>
#include "no_os_spi.h"
#include <stdlib.h>
#include "no_os_error.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
// #include "no_os_alloc.h"

#define AD_READ		(0 << 15)
#define AD_WRITE		(1 << 15)
#define AD_CNT(x)	((((x) - 1) & 0x7) << 12)
#define AD_ADDR(x)	((x) & 0xFFF)

#define SPI_DEFAULT_SPEED_HZ	20000000
#define SPI_DEFAULT_BITS	8

static uint8_t no_os_spi_to_linux_mode(enum no_os_spi_mode no_os_mode)
{
	return no_os_mode & (SPI_CPHA | SPI_CPOL);
}



/**
 * @brief Initialize the SPI communication peripheral.
 * @param desc - The SPI descriptor.
 * @param param - The structure that contains the SPI parameters.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t no_os_spi_init(struct no_os_spi_desc **desc,
		       const struct no_os_spi_init_param *param)
{
	uint8_t mode;
	uint8_t read_mode;
	uint8_t lsb;
	uint8_t bits = SPI_DEFAULT_BITS;
	uint32_t speed;
	char path[64];
	int fd_spi;
	int ret;

	struct no_os_spi_desc *descriptor;

	if (!desc || !param)
		return -1;

	descriptor = (struct no_os_spi_desc *)malloc(sizeof(*descriptor));
	if (!descriptor)
		return -1;

	snprintf(path, sizeof(path), "/dev/spidev%d.%d",
		 param->device_id, param->chip_select);
    fd_spi = open(path,O_RDWR);
    if(fd_spi < 0)
    {
        fprintf(stderr, "%s: cannot open %s: %s\n", __func__, path, strerror(errno));
        free(descriptor);
        return -1;
    }

    mode = no_os_spi_to_linux_mode(param->mode);
    ret = ioctl(fd_spi,SPI_IOC_WR_MODE,&mode);
    if(ret == -1) {
        fprintf(stderr,"%s: cannot set spi mode on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_MODE, &read_mode);
    if (ret == -1) {
        fprintf(stderr,"%s: cannot get spi mode on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }

    if (read_mode != mode){
        printf("WARNING %s does not support requested mode 0x%x, got 0x%x\n",
               path, mode, read_mode);
        goto error;
    }

    lsb = (param->bit_order == NO_OS_SPI_BIT_ORDER_LSB_FIRST) ? 1 : 0;
    ret = ioctl(fd_spi,SPI_IOC_WR_LSB_FIRST, &lsb);
    if (ret == -1) {
        fprintf(stderr,"%s: cannot set bit order on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }


    ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1){
        fprintf(stderr,"%s: cannot set bits per word on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }

    speed = param->max_speed_hz ? param->max_speed_hz : SPI_DEFAULT_SPEED_HZ;

    ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1){
        fprintf(stderr,"%s: cannot set max speed hz on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1){
        fprintf(stderr,"%s: cannot get max speed hz on %s: %s\n", __func__, path, strerror(errno));
        goto error;
    }

	descriptor->fd_spi = fd_spi;
	descriptor->device_id = param->device_id;
	descriptor->max_speed_hz = speed;
	descriptor->chip_select = param->chip_select;
	descriptor->mode = param->mode;
	descriptor->bit_order = param->bit_order;
	
	*desc = descriptor;
	return 0;

error:
	close(fd_spi);
	free(descriptor);
	return -1;
}


int32_t no_os_spi_remove(struct no_os_spi_desc *desc)
{
	if (!desc)
		return 0;
	close(desc->fd_spi);
	free(desc);
	return 0;
}

/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc,
				 uint8_t *data,
				 uint16_t bytes_number)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)data,
		.rx_buf = (unsigned long)data,
		.len = bytes_number,
		.speed_hz = desc->max_speed_hz,
		.bits_per_word = SPI_DEFAULT_BITS,
	};

	int ret;

	if (!desc || desc->fd_spi < 0 || !data)
		return -1;

	ret = ioctl(desc->fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 0) {
		printf("%s: Can't send spi message: %s\n\r", __func__, strerror(errno));
		return -1;
	}

	if (ret != bytes_number) {
		printf("%s: Short SPI transfer, expected %u bytes, got %d\n\r",
		       __func__, bytes_number, ret);
		return -1;
	}

	return 0;
}
