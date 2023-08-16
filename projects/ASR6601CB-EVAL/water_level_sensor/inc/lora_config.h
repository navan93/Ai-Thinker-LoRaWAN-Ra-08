#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tremo_gpio.h"

#define CONFIG_LORA_RFSW_CTRL_GPIOX        GPIOD
#define CONFIG_LORA_RFSW_CTRL_PIN          GPIO_PIN_11

#define CONFIG_LORA_RFSW_VDD_GPIOX         GPIOA
#define CONFIG_LORA_RFSW_VDD_PIN           GPIO_PIN_10

//IO9
#define CONFIG_WATER_SENSOR_1_GPIOX        GPIOA
#define CONFIG_WATER_SENSOR_1_PIN          GPIO_PIN_9

//IO8
#define CONFIG_WATER_SENSOR_2_GPIOX        GPIOA
#define CONFIG_WATER_SENSOR_2_PIN          GPIO_PIN_11

//IO5
#define CONFIG_WATER_SENSOR_EN_GPIOX       GPIOA
#define CONFIG_WATER_SENSOR_EN_PIN         GPIO_PIN_5

//IO
#define CONFIG_WATER_SENSOR_HX710B_SCK_PIN GPIO_PIN_7
#define CONFIG_WATER_SENSOR_HX710B_OUT_PIN GPIO_PIN_6

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
