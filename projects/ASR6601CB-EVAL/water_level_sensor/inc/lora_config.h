#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tremo_gpio.h"

#define CONFIG_LORA_RFSW_CTRL_GPIOX GPIOD
#define CONFIG_LORA_RFSW_CTRL_PIN   GPIO_PIN_11

#define CONFIG_LORA_RFSW_VDD_GPIOX GPIOA
#define CONFIG_LORA_RFSW_VDD_PIN   GPIO_PIN_10

#define CONFIG_WATER_SENSOR_1_GPIOX GPIOD
#define CONFIG_WATER_SENSOR_1_PIN   GPIO_PIN_11

#define CONFIG_WATER_SENSOR_2_GPIOX GPIOD
#define CONFIG_WATER_SENSOR_2_PIN   GPIO_PIN_11

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
