#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#include <stdbool.h>

/**
 * Separate file for GPIO handling to avoid including MSP430.h directly
 * in the sensor drivers to make them more portable.
 */

typedef enum
{
    GPIO_XSHUT_FIRST,
    GPIO_XSHUT_SECOND,
    GPIO_XSHUT_THIRD,
} gpio_t;

void hal_gpio_init(void);
void hal_gpio_set_output(gpio_t gpio, bool enable);

#endif /* GPIO_H */