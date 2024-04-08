#include <stdio.h>
#include "tremo_regs.h"
#include "tremo_adc.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_uart.h"

#define ADC_DATA_NUM 14
uint16_t adc_data_1[ADC_DATA_NUM] = {0};
uint16_t adc_data_2[ADC_DATA_NUM] = {0};
float calibrated_sample_1[ADC_DATA_NUM] = {0.0};
float calibrated_sample_2[ADC_DATA_NUM] = {0.0};

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

void adc_single_mode_test(void)
{
    uint16_t i;
    gpio_t *gpiox;
    uint32_t pin;
    float gain_value;
    float dco_value;

    adc_get_calibration_value(false, &gain_value, &dco_value);

    gpiox = GPIOA;
    pin = GPIO_PIN_8;
    gpio_init(gpiox, pin, GPIO_MODE_ANALOG);
    pin = GPIO_PIN_11;
    gpio_init(gpiox, pin, GPIO_MODE_ANALOG);

    adc_init();

    adc_config_clock_division(20); //sample frequence 150K

    adc_config_sample_sequence(0, 0);
    adc_config_sample_sequence(1, 2);
    adc_config_sample_sequence(2, 3);
    adc_config_sample_sequence(3, 4);
    adc_config_sample_sequence(4, 5);
    adc_config_sample_sequence(5, 6);
    adc_config_sample_sequence(6, 7);
    adc_config_sample_sequence(7, 8);
    adc_config_sample_sequence(8, 9);
    adc_config_sample_sequence(9, 10);
    adc_config_sample_sequence(10, 11);
    adc_config_sample_sequence(11, 12);
    adc_config_sample_sequence(12, 13);
    adc_config_sample_sequence(13, 14);
    adc_config_sample_sequence(14, 15);

    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);

    adc_enable(true);

    adc_start(true);

    for (i = 0; i < ADC_DATA_NUM; i++)
    {
        while(!adc_get_interrupt_status(ADC_ISR_EOC));
        adc_data_1[i] = adc_get_data();
        // while(!adc_get_interrupt_status(ADC_ISR_EOC));
        // adc_data_2[i] = adc_get_data();
        // while(!adc_get_interrupt_status(ADC_ISR_EOC));
        // (void)adc_get_data();
        // adc_start(true);
    }

    adc_start(false);
    adc_enable(false);

    for (i = 0; i < ADC_DATA_NUM; i++)
    {//calibration sample value
        calibrated_sample_1[i] = ((1.2/4096) * adc_data_1[i] - dco_value) / gain_value;
        // calibrated_sample_2[i] = ((1.2/4096) * adc_data_2[i] - dco_value) / gain_value;
        printf("chan %u: %f\r\n", i+1, calibrated_sample_1[i]);
    }
}

int main(void)
{
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);

    uart_log_init();

    printf("ADC Test Start!\r\n");

    adc_single_mode_test();

    /* Infinite loop */
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
