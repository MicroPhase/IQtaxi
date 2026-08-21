#ifndef SOAPY_M300_NOOS_PLATFORM_HPP
#define SOAPY_M300_NOOS_PLATFORM_HPP

#include <cstdint>
#include <memory>

#include "m300_xdma_ctrl.hpp"

extern "C" {
#include "no_os_gpio.h"
}

namespace sdr { namespace driver {

class m300_noos_context
{
public:
    explicit m300_noos_context(const m300_xdma_ctrl::sptr& ctrl);

    int32_t spi_transfer(uint8_t* data, uint16_t bytes_number);
    int32_t axi_read(uint32_t base, uint32_t offset, uint32_t* data);
    int32_t axi_write(uint32_t base, uint32_t offset, uint32_t data);
    int32_t gpio_set_direction(uint32_t number, bool output, bool value);
    int32_t gpio_set_value(uint32_t number, bool value);
    int32_t gpio_get_value(uint32_t number, bool* value);

private:
    m300_xdma_ctrl::sptr _ctrl;
    uint32_t _gpio_out = 0x00000001u;
    uint32_t _gpio_oe = 0x000001ffu;
};

void m300_noos_set_active_context(m300_noos_context* context);
m300_noos_context* m300_noos_get_active_context();

}} // namespace sdr::driver

extern "C" const struct no_os_gpio_platform_ops m300_gpio_ops;

#endif // SOAPY_M300_NOOS_PLATFORM_HPP
