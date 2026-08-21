#include "m300_noos_platform.hpp"

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <new>

extern "C" {
#include "no_os_error.h"
#include "no_os_spi.h"
}

using namespace sdr::driver;

namespace {
thread_local m300_noos_context* g_active_context = nullptr;

struct m300_gpio_desc
{
    uint8_t direction = NO_OS_GPIO_IN;
};

uint32_t ad9361_spi_addr_from_cmd(const uint8_t* data)
{
    const uint16_t cmd = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return cmd & 0x03ffu;
}

bool ad9361_spi_is_write(const uint8_t* data)
{
    const uint16_t cmd = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return (cmd & 0x8000u) != 0u;
}

uint32_t ad9361_spi_count(const uint8_t* data)
{
    const uint16_t cmd = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return ((cmd >> 12) & 0x7u) + 1u;
}

int32_t require_context(m300_noos_context** context)
{
    *context = m300_noos_get_active_context();
    return *context ? 0 : -ENODEV;
}
}

m300_noos_context::m300_noos_context(const m300_xdma_ctrl::sptr& ctrl)
    : _ctrl(ctrl)
{
    if (_ctrl) {
        try {
            _gpio_out = _ctrl->read_gpio_out(1.0);
            _gpio_oe = _ctrl->read_gpio_oe(1.0);
        } catch (...) {
            _gpio_out = 0x00000001u;
            _gpio_oe = 0x000001ffu;
        }
    }
}

int32_t m300_noos_context::spi_transfer(uint8_t* data, uint16_t bytes_number)
{
    if (!_ctrl || !data || bytes_number < 3) {
        return -EINVAL;
    }

    const bool is_write = ad9361_spi_is_write(data);
    const uint32_t reg = ad9361_spi_addr_from_cmd(data);
    const uint32_t count = ad9361_spi_count(data);
    if (bytes_number < count + 2u) {
        return -EINVAL;
    }

    try {
        const bool use_burst_write = std::getenv("M300_AD9361_SPI_BURST") != nullptr;
        const bool trace_spi = std::getenv("M300_AD9361_SPI_TRACE") != nullptr;
        if (trace_spi && is_write && count > 1u) {
            std::cout << "ad9361_spi_multi_write mode="
                      << (use_burst_write ? "burst" : "split")
                      << " start=0x" << std::hex
                      << std::setw(3) << std::setfill('0') << reg;
            for (uint32_t i = 0; i < count; ++i) {
                const uint16_t addr = static_cast<uint16_t>((reg - i) & 0x03ffu);
                std::cout << " [0x" << std::setw(3) << addr << "]=0x"
                          << std::setw(2) << static_cast<unsigned>(data[2u + i]);
            }
            std::cout << std::dec << "\n";
        }

        if (is_write && count > 1u && use_burst_write) {
            (void)_ctrl->ad9361_spi_write_burst(static_cast<uint16_t>(reg & 0x03ffu),
                                                data + 2u,
                                                count,
                                                1.0);
        } else {
            for (uint32_t i = 0; i < count; ++i) {
                const uint16_t addr = static_cast<uint16_t>((reg - i) & 0x03ffu);
                if (is_write) {
                    (void)_ctrl->ad9361_spi_write(addr, data[2u + i], 1.0);
                } else {
                    data[2u + i] = _ctrl->ad9361_spi_read(addr, 1.0);
                }
            }
        }
    } catch (...) {
        return -EIO;
    }

    return 0;
}

int32_t m300_noos_context::axi_read(uint32_t base, uint32_t offset, uint32_t* data)
{
    if (!_ctrl || !data) {
        return -EINVAL;
    }

    try {
        *data = _ctrl->read_axi(base + offset, 1.0);
    } catch (...) {
        return -EIO;
    }

    return 0;
}

int32_t m300_noos_context::axi_write(uint32_t base, uint32_t offset, uint32_t data)
{
    if (!_ctrl) {
        return -EINVAL;
    }

    try {
        (void)_ctrl->write_axi(base + offset, data, 1.0);
    } catch (...) {
        return -EIO;
    }

    return 0;
}

int32_t m300_noos_context::gpio_set_direction(uint32_t number, bool output, bool value)
{
    if (!_ctrl || number > 31u) {
        return -EINVAL;
    }

    const uint32_t mask = 1u << number;
    if (output) {
        _gpio_oe |= mask;
        if (value) {
            _gpio_out |= mask;
        } else {
            _gpio_out &= ~mask;
        }
    } else {
        _gpio_oe &= ~mask;
    }

    try {
        (void)_ctrl->write_gpio_out(_gpio_out, 1.0);
        (void)_ctrl->write_gpio_oe(_gpio_oe, 1.0);
    } catch (...) {
        return -EIO;
    }

    return 0;
}

int32_t m300_noos_context::gpio_set_value(uint32_t number, bool value)
{
    if (!_ctrl || number > 31u) {
        return -EINVAL;
    }

    const uint32_t mask = 1u << number;
    if (value) {
        _gpio_out |= mask;
    } else {
        _gpio_out &= ~mask;
    }

    try {
        (void)_ctrl->write_gpio_out(_gpio_out, 1.0);
    } catch (...) {
        return -EIO;
    }

    return 0;
}

int32_t m300_noos_context::gpio_get_value(uint32_t number, bool* value)
{
    if (!_ctrl || !value || number > 31u) {
        return -EINVAL;
    }

    try {
        const uint32_t gpio_in = _ctrl->read_gpio_in(1.0);
        *value = ((gpio_in >> number) & 0x1u) != 0u;
    } catch (...) {
        return -EIO;
    }

    return 0;
}

void sdr::driver::m300_noos_set_active_context(m300_noos_context* context)
{
    g_active_context = context;
}

m300_noos_context* sdr::driver::m300_noos_get_active_context()
{
    return g_active_context;
}

extern "C" int32_t no_os_spi_init(struct no_os_spi_desc** desc,
                                  const struct no_os_spi_init_param* param)
{
    if (!desc || !param) {
        return -EINVAL;
    }

    no_os_spi_desc* descriptor =
        static_cast<no_os_spi_desc*>(std::calloc(1, sizeof(*descriptor)));
    if (!descriptor) {
        return -ENOMEM;
    }

    descriptor->device_id = param->device_id;
    descriptor->max_speed_hz = param->max_speed_hz;
    descriptor->chip_select = param->chip_select;
    descriptor->mode = param->mode;
    descriptor->bit_order = param->bit_order;
    descriptor->platform_ops = param->platform_ops;
    descriptor->fd_spi = -1;
    *desc = descriptor;
    return 0;
}

extern "C" int32_t no_os_spi_remove(struct no_os_spi_desc* desc)
{
    std::free(desc);
    return 0;
}

extern "C" int32_t no_os_spi_write_and_read(struct no_os_spi_desc* desc,
                                            uint8_t* data,
                                            uint16_t bytes_number)
{
    (void)desc;
    m300_noos_context* context = nullptr;
    const int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }
    return context->spi_transfer(data, bytes_number);
}

extern "C" int32_t no_os_axi_io_read(uint32_t base, uint32_t offset, uint32_t* data)
{
    m300_noos_context* context = nullptr;
    const int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }
    return context->axi_read(base, offset, data);
}

extern "C" int32_t no_os_axi_io_write(uint32_t base, uint32_t offset, uint32_t data)
{
    m300_noos_context* context = nullptr;
    const int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }
    return context->axi_write(base, offset, data);
}

extern "C" int32_t m300_gpio_get(struct no_os_gpio_desc** desc,
                                 const struct no_os_gpio_init_param* param)
{
    if (!desc || !param || param->number < 0) {
        return -EINVAL;
    }

    no_os_gpio_desc* descriptor =
        static_cast<no_os_gpio_desc*>(std::calloc(1, sizeof(*descriptor)));
    if (!descriptor) {
        return -ENOMEM;
    }

    m300_gpio_desc* extra = new (std::nothrow) m300_gpio_desc();
    if (!extra) {
        std::free(descriptor);
        return -ENOMEM;
    }

    descriptor->port = param->port;
    descriptor->number = param->number;
    descriptor->pull = param->pull;
    descriptor->platform_ops = param->platform_ops;
    descriptor->extra = extra;
    *desc = descriptor;
    return 0;
}

extern "C" int32_t m300_gpio_get_optional(struct no_os_gpio_desc** desc,
                                          const struct no_os_gpio_init_param* param)
{
    if (!desc) {
        return -EINVAL;
    }
    if (!param || param->number < 0) {
        *desc = nullptr;
        return 0;
    }
    return m300_gpio_get(desc, param);
}

extern "C" int32_t m300_gpio_remove(struct no_os_gpio_desc* desc)
{
    if (!desc) {
        return 0;
    }
    delete static_cast<m300_gpio_desc*>(desc->extra);
    std::free(desc);
    return 0;
}

extern "C" int32_t m300_gpio_direction_input(struct no_os_gpio_desc* desc)
{
    if (!desc) {
        return -EINVAL;
    }

    m300_noos_context* context = nullptr;
    int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }

    ret = context->gpio_set_direction(static_cast<uint32_t>(desc->number), false, false);
    if (ret == 0) {
        static_cast<m300_gpio_desc*>(desc->extra)->direction = NO_OS_GPIO_IN;
    }
    return ret;
}

extern "C" int32_t m300_gpio_direction_output(struct no_os_gpio_desc* desc,
                                              uint8_t value)
{
    if (!desc) {
        return -EINVAL;
    }

    m300_noos_context* context = nullptr;
    int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }

    ret = context->gpio_set_direction(static_cast<uint32_t>(desc->number),
                                      true, value != 0);
    if (ret == 0) {
        static_cast<m300_gpio_desc*>(desc->extra)->direction = NO_OS_GPIO_OUT;
    }
    return ret;
}

extern "C" int32_t m300_gpio_get_direction(struct no_os_gpio_desc* desc,
                                           uint8_t* direction)
{
    if (!desc || !direction) {
        return -EINVAL;
    }
    *direction = static_cast<m300_gpio_desc*>(desc->extra)->direction;
    return 0;
}

extern "C" int32_t m300_gpio_set_value(struct no_os_gpio_desc* desc,
                                       uint8_t value)
{
    if (!desc) {
        return -EINVAL;
    }

    m300_noos_context* context = nullptr;
    const int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }
    return context->gpio_set_value(static_cast<uint32_t>(desc->number), value != 0);
}

extern "C" int32_t m300_gpio_get_value(struct no_os_gpio_desc* desc,
                                       uint8_t* value)
{
    if (!desc || !value) {
        return -EINVAL;
    }

    m300_noos_context* context = nullptr;
    const int32_t ret = require_context(&context);
    if (ret < 0) {
        return ret;
    }

    bool high = false;
    const int32_t get_ret =
        context->gpio_get_value(static_cast<uint32_t>(desc->number), &high);
    if (get_ret < 0) {
        return get_ret;
    }
    *value = high ? NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW;
    return 0;
}

extern "C" const struct no_os_gpio_platform_ops m300_gpio_ops = {
    m300_gpio_get,
    m300_gpio_get_optional,
    m300_gpio_remove,
    m300_gpio_direction_input,
    m300_gpio_direction_output,
    m300_gpio_get_direction,
    m300_gpio_set_value,
    m300_gpio_get_value
};
