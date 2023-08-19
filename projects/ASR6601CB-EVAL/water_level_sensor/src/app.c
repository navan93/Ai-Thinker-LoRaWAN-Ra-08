/*!
 * \file      main.c
 *
 * \brief     Water level sensor app using 2 point contact sensors
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Navaneeth Bhardwaj G
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"
#include "tremo_pwr.h"
#include "tremo_gpio.h"
#include "lora_config.h"
#include "hx710b.h"
#include "tremo_adc.h"
#include "tremo_delay.h"


#define RF_FREQUENCY                                865000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       10         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        12        // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define SLEEP_TIMEOUT_VALUE                         60 * 1000      //ms
#define VBAT_FACTOR                                 3.06f

typedef enum
{
    LOWPOWER,
    MEASURE_WATER_LEVEL,
    TX,
    TX_TIMEOUT
}States_t;

typedef struct {
    States_t State;
    TimerEvent_t SleepTimeoutTimer;
    RadioEvents_t RadioEvents;
}app_sm_t;

typedef struct message_fields{
    uint32_t ChipId[2];
    uint16_t fw_ver;
    uint16_t hw_ver;
    float battery_voltage;   //32bits
    uint32_t hx710b_val_raw;
    uint8_t  sensor_value_raw;
    uint8_t  sensor_value_mask;
}tx_message_t;

static app_sm_t m_app_sm;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Sleep timeout event
 */
void SleepTimeoutIrq( void );

uint8_t contact_sensor_read(void);
void update_sensor_vals(tx_message_t*);

float read_vbat(void)
{
    gpio_t *gpiox;
    uint32_t pin;
    float gain_value = 1.188f;
    float dco_value = -0.107f;
    uint16_t adc_data_1;
    float calibrated_sample_1;

    // adc_get_calibration_value(false, &gain_value, &dco_value);
    // gain_value = 1.0f;


    //ADC_IN1
    gpiox = GPIOA;
    pin = GPIO_PIN_8;
    gpio_init(gpiox, pin, GPIO_MODE_ANALOG);

    adc_enable_vbat31(true);

    adc_init();
    delay_us(1000);

    adc_config_clock_division(20); //sample frequence 150K


    adc_config_sample_sequence(0, 15);

    adc_config_conv_mode(ADC_CONV_MODE_DISCONTINUE);

    adc_enable(true);

	adc_start(true);
    while(!adc_get_interrupt_status(ADC_ISR_EOC));
    (void)adc_get_data();

    adc_start(true);
    while(!adc_get_interrupt_status(ADC_ISR_EOC));
    adc_data_1 = adc_get_data();

    adc_start(false);
    adc_enable(false);
    //calibration sample value
    calibrated_sample_1 = ((1.2/4096) * adc_data_1 - dco_value) / gain_value;
    calibrated_sample_1 *= VBAT_FACTOR;
    printf("vbat_adc: %d, vbat_calibrated: %fV, gain: %f, offset: %f\r\n",adc_data_1, calibrated_sample_1, gain_value, dco_value);

    return calibrated_sample_1;
}

/**
 * Main application entry point.
 */
int app_start(void)
{
    uint32_t random;

    tx_message_t tx_message = {
        .fw_ver                 = 0x0100, // v1.0
        .hw_ver                 = 0x0101, // v1.1
        .battery_voltage        = 0.0,
        .sensor_value_mask      = 3,     //1 bit represnting 1 contact, lsb being the lowest point contact
        .sensor_value_raw       = 0,
        .hx710b_val_raw         = 0
    };
    (void)system_get_chip_id(tx_message.ChipId);

    printf("Water Level Sensor Start!\r\n");

    memset(&m_app_sm, 0, sizeof(m_app_sm));

    m_app_sm.State = MEASURE_WATER_LEVEL;

    // Radio initialization
    m_app_sm.RadioEvents.TxDone    = OnTxDone;
    m_app_sm.RadioEvents.TxTimeout = OnTxTimeout;
    m_app_sm.RadioEvents.RxDone    = NULL;
    m_app_sm.RadioEvents.RxTimeout = NULL;
    m_app_sm.RadioEvents.RxError   = NULL;

    Radio.Init( &m_app_sm.RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 1000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    TimerInit( &m_app_sm.SleepTimeoutTimer, SleepTimeoutIrq );

    printf("Starting App\r\n");
    hx710b_set_pd(1);

    while( 1 )
    {
        switch(m_app_sm.State)
        {
        case TX:
            // Send the next PING frame

            srand(tx_message.ChipId[0]);
            random = ( rand() + 1 ) % 90;
            DelayMs( random );
            Radio.Send((uint8_t*)&tx_message, sizeof(tx_message_t));
            printf("Sent: Tx Message %dB\r\n", sizeof(tx_message_t));
            printf("ChipId_L: %lx, ChipId_H: %lx, fw_ver: %x, hw_ver: %x, vbat: %f, hx710b_raw: %lu, sensor_value_raw: %d, sensor_value_mask: %x\r\n",
                tx_message.ChipId[0],
                tx_message.ChipId[1],
                tx_message.fw_ver,
                tx_message.hw_ver,
                tx_message.battery_voltage,
                tx_message.hx710b_val_raw,
                tx_message.sensor_value_raw,
                tx_message.sensor_value_mask
            );
            m_app_sm.State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            m_app_sm.State = LOWPOWER;
            break;
        case MEASURE_WATER_LEVEL:
            update_sensor_vals(&tx_message);
            m_app_sm.State = TX;
            break;
        case LOWPOWER:
        default:
            // Set low power
            TimerLowPowerHandler();
            break;
        }

        // Process Radio IRQ
        Radio.IrqProcess();
    }
}

void OnTxDone( void )
{
    // printf("OnTxDone\r\n");
    Radio.Sleep( );
    TimerSetValue(&m_app_sm.SleepTimeoutTimer, SLEEP_TIMEOUT_VALUE);
    TimerStart(&m_app_sm.SleepTimeoutTimer);
    m_app_sm.State = LOWPOWER;
}

void OnTxTimeout( void )
{
    // printf("OnTxTimeout\r\n");
    Radio.Sleep( );
    m_app_sm.State = TX_TIMEOUT;
}

void SleepTimeoutIrq( void )
{
    // printf("SleepTimeoutIrq\r\n");
    TimerStop(&m_app_sm.SleepTimeoutTimer);
    m_app_sm.State = MEASURE_WATER_LEVEL;
}

uint8_t contact_sensor_read(void)
{
    uint8_t sensor_value_raw = 0;
    gpio_write(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_LEVEL_HIGH);
    DelayMs(100);
    //Invert because input is fed from an open drain transistor
    sensor_value_raw  = !gpio_read(CONFIG_WATER_SENSOR_1_GPIOX, CONFIG_WATER_SENSOR_1_PIN);
    sensor_value_raw |= !gpio_read(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_2_PIN) << 1;
    gpio_write(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_LEVEL_LOW);
    return sensor_value_raw;
}

void update_sensor_vals(tx_message_t *p_tx_message)
{
    // printf("HX710B: %lu \r\n", hx710b_read_pressure_raw());
    // printf("HX710B: %f cm\r\n", hx710b_read_water_cm());
    // printf("HX710B: %f pa\r\n", hx710b_read_pascal());
    p_tx_message->hx710b_val_raw   = hx710b_read_pressure_raw();
    p_tx_message->sensor_value_raw = contact_sensor_read();
    p_tx_message->battery_voltage  = read_vbat();
    uint32_t hx710b_temp = hx710b_read_temperature();
    printf("HX710B Temp: %lu\r\n", hx710b_temp);
}