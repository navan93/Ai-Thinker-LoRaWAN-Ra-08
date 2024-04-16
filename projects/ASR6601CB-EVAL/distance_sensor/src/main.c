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
#include "tremo_i2c.h"

extern int app_start(void);

void uart_log_init(void)
{
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);

    // uart0
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

    /* uart config struct init */
    uart_config_t uart_config;
    uart_config_init(&uart_config);

    uart_config.baudrate = UART_BAUDRATE_115200;
    uart_init(CONFIG_DEBUG_UART, &uart_config);
    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
    printf("\r\n<----LoRa VL53L0X Distance Sensor Logs---->\r\n");
}

void board_i2c_init(void)
{
    i2c_config_t config;

    // enable the clk
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_I2C0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

    // set iomux
    gpio_set_iomux(GPIOA, GPIO_PIN_14, 3);
    gpio_set_iomux(GPIOA, GPIO_PIN_15, 3);

    // init
    i2c_config_init(&config);
    i2c_init(I2C0, &config);
    i2c_cmd(I2C0, true);
}

void board_init()
{
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_oscillator(RCC_OSC_XO32K, true);

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);

    delay_ms(100);
    pwr_xo32k_lpm_cmd(true);

    uart_log_init();
    board_i2c_init();

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
