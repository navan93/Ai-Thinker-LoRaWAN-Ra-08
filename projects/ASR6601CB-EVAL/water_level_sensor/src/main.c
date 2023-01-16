#include <stdio.h>
#include <stdarg.h>
#include "tremo_regs.h"
#include "tremo_rtc.h"
#include "tremo_rcc.h"
#include "tremo_pwr.h"
#include "tremo_uart.h"
#include "tremo_gpio.h"

static uint8_t rtc_wakeup_intr_flag;

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
}

void rtc_cyc()
{
    /* NVIC config */
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 2);

    rtc_config_cyc_max(32768);
    rtc_config_cyc_wakeup(ENABLE);
    rtc_cyc_cmd(true);
    rtc_config_interrupt(RTC_CYC_IT, ENABLE);
}

void rtc_IRQHandler(void)
{
    uint8_t intr_stat;

    intr_stat = rtc_get_status(RTC_CYC_SR);
    if (intr_stat == true) {
        rtc_config_interrupt(RTC_CYC_IT, DISABLE);
        rtc_set_status(RTC_CYC_SR, false);
        rtc_config_interrupt(RTC_CYC_IT, ENABLE);
        rtc_wakeup_intr_flag = 1;
    }
}

int main(void)
{
    rtc_wakeup_intr_flag = 0;

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_AFEC, true);

    // enable the clk
    rcc_enable_oscillator(RCC_OSC_XO32K, true);

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);

    uart_log_init();

    rtc_cyc();

    printf("Hello World\r\n");

    /* Infinite loop */
    while (1) {
        // pwr_deepsleep_wfi(PWR_LP_MODE_STOP3);
        if(rtc_wakeup_intr_flag == 1) {
            rtc_wakeup_intr_flag = 0;
            printf("RTC Interrupt\r\n");
        }
     }
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
