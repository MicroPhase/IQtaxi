#ifndef _NO_OS_SPI_H_
#define _NO_OS_SPI_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define	NO_OS_SPI_CPHA	0x01
#define	NO_OS_SPI_CPOL	0x02


#include <linux/ioctl.h>
#include "linux/spi/spidev.h"
#include "unistd.h"
#include "stdint.h"

#define SPI_NAME "/dev/spidev0.0"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum no_os_spi_mode
 * @brief SPI configuration for clock phase and polarity.
 */
typedef enum no_os_spi_mode {
	/** Data on rising, shift out on falling */
	NO_OS_SPI_MODE_0 = (0 | 0),
	/** Data on falling, shift out on rising */
	NO_OS_SPI_MODE_1 = (0 | NO_OS_SPI_CPHA),
	/** Data on falling, shift out on rising */
	NO_OS_SPI_MODE_2 = (NO_OS_SPI_CPOL | 0),
	/** Data on rising, shift out on falling */
	NO_OS_SPI_MODE_3 = (NO_OS_SPI_CPOL | NO_OS_SPI_CPHA)
} no_os_spi_mode;

/**
 * @enum no_os_spi_bit_order
 * @brief SPI configuration for bit order (MSB/LSB).
 */
typedef enum no_os_spi_bit_order {
	/** Most-significant bit (MSB) first */
	NO_OS_SPI_BIT_ORDER_MSB_FIRST = 0,
	/** Least-significant bit (LSB) first */
	NO_OS_SPI_BIT_ORDER_LSB_FIRST = 1,
} no_os_spi_bit_order;

/**
 * @struct no_os_spi_msg_list
 * @brief List item describing a SPI transfer
 */
struct no_os_spi_msg {
	/** Buffer with data to send. If NULL, 0x00 will be sent */
	uint8_t			*tx_buff;
	/** Buffer where to store data. If NULL, incoming data won't be saved */
	uint8_t			*rx_buff;
	/** Length of buffers. Must have equal size. */
	uint32_t		bytes_number;
	/** If set, CS will be deasserted after the transfer */
	uint8_t			cs_change;
};

/**
 * @struct no_os_spi_platform_ops
 * @brief Structure holding SPI function pointers that point to the platform
 * specific function
 */
struct no_os_spi_platform_ops ;

/**
 * @struct no_os_spi_init_param
 * @brief Structure holding the parameters for SPI initialization
 */
typedef struct no_os_spi_init_param {
	/** Device ID */
	uint32_t	device_id;
	/** maximum transfer speed */
	uint32_t	max_speed_hz;
	/** SPI chip select */
	uint8_t		chip_select;
	/** SPI mode */
	enum no_os_spi_mode	mode;
	/** SPI bit order */
	enum no_os_spi_bit_order	bit_order;
	const struct no_os_spi_platform_ops *platform_ops;
	/**  SPI extra parameters (device specific) */
	void		*extra;
} no_os_spi_init_param;

/**
 * @struct no_os_spi_desc
 * @brief Structure holding SPI descriptor.
 */
typedef struct no_os_spi_desc {
	/** Device ID */
	uint32_t	device_id;
	/** maximum transfer speed */
	uint32_t	max_speed_hz;
	/** SPI chip select */
	uint8_t		chip_select;
	/** SPI mode */
	enum no_os_spi_mode	mode;
	/** SPI bit order */
	enum no_os_spi_bit_order	bit_order;
	const struct no_os_spi_platform_ops *platform_ops;
	/**  SPI extra parameters (device specific) */
	
	int32_t fd_spi;
} no_os_spi_desc;

/**
 * @struct no_os_spi_platform_ops
 * @brief Structure holding SPI function pointers that point to the platform
 * specific function
 */
struct no_os_spi_platform_ops {
	/** SPI initialization function pointer */
	int32_t (*init)(struct no_os_spi_desc **, const struct no_os_spi_init_param *);
	/** SPI write/read function pointer */
	int32_t (*write_and_read)(struct no_os_spi_desc *, uint8_t *, uint16_t);
	/** Iterate over the spi_msg array and send all messages at once */
	int32_t (*transfer)(struct no_os_spi_desc *, struct no_os_spi_msg *, uint32_t);
	/** SPI remove function pointer */
	int32_t (*remove)(struct no_os_spi_desc *);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize the SPI communication peripheral. */
int32_t no_os_spi_init(struct no_os_spi_desc **desc,
		       const struct no_os_spi_init_param *param);

/* Free the resources allocated by no_os_spi_init(). */
int32_t no_os_spi_remove(struct no_os_spi_desc *desc);

/* Write and read data to/from SPI. */
int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc,
				 uint8_t *data,
				 uint16_t bytes_number);



#endif // _NO_OS_SPI_H_
