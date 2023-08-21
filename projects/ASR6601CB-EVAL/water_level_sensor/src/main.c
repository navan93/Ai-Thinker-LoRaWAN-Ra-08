#include <stdio.h>
#include <stdarg.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_regs.h"
#include "tremo_rtc.h"
#include "tremo_rcc.h"
#include "tremo_pwr.h"
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "lora_config.h"
#include "tremo_delay.h"
#include "rtc-board.h"
#include "tremo_adc.h"




extern int app_start(void);

void uart_log_init(void)
{
    // uart0
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    /* uart config struct init */
    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_115200;
    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
    printf("\r\n<----LoRa Water Sensor Logs---->\r\n");
}

void board_gpio_init(void)
{
    gpio_set_iomux(CONFIG_WATER_SENSOR_1_GPIOX, CONFIG_WATER_SENSOR_1_PIN, 0);
    gpio_set_iomux(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_2_PIN, 0);
    gpio_set_iomux(CONFIG_WATER_SENSOR_EN_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, 0);
    gpio_set_iomux(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, 0);
    gpio_set_iomux(GPIOA, CONFIG_WATER_SENSOR_HX710B_OUT_PIN, 0);
    gpio_init(CONFIG_WATER_SENSOR_1_GPIOX, CONFIG_WATER_SENSOR_1_PIN, GPIO_MODE_INPUT_PULL_UP);
    gpio_init(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_2_PIN, GPIO_MODE_INPUT_PULL_UP);
    gpio_init(CONFIG_WATER_SENSOR_EN_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_MODE_OUTPUT_PP_LOW);
    gpio_init(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_MODE_INPUT_PULL_DOWN);
    gpio_init(GPIOA, CONFIG_WATER_SENSOR_HX710B_OUT_PIN, GPIO_MODE_INPUT_FLOATING);
    gpio_config_drive_capability(CONFIG_WATER_SENSOR_EN_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_DRIVE_CAPABILITY_8MA);

    gpio_write(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_LEVEL_LOW);
}

void board_init()
{
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_oscillator(RCC_OSC_XO32K, true);

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);


    board_gpio_init();

    delay_ms(100);
    pwr_xo32k_lpm_cmd(true);

    uart_log_init();

    RtcInit();
}


int main(void)
{
    // Target board initialization
    board_init();

    app_start();
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
