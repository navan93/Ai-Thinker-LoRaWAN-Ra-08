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

typedef enum
{
    LOWPOWER,
    MEASURE_WATER_LEVEL,
    TX,
    TX_TIMEOUT
}States_t;

typedef enum
{
    SENSOR_OK,
    SENSOR_INVALID_VAL
}sensor_err_t;

typedef struct {
    States_t State;
    TimerEvent_t SleepTimeoutTimer;
    uint32_t ChipId[2];
    RadioEvents_t RadioEvents;
    sensor_err_t sensor_error_status;
    uint8_t water_level_percentage;
    uint8_t sensor_value_raw;
}app_sm_t;


typedef union {
    struct message_fields{
        uint16_t device_id;
        uint16_t fw_ver;
        uint16_t hw_ver;
        uint8_t  water_level_percentage;
        uint8_t  sensor_value_raw;
        uint16_t battery_voltage_mv;
        sensor_err_t  error_status;
    }fields;
    uint8_t buffer[12];
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

void contact_sensor_read(void);
void measure_water_level(void);

/**
 * Main application entry point.
 */
int app_start(void)
{
    tx_message_t tx_message = {
        .fields.device_id              = 1,
        .fields.fw_ver                 = 0x0005, // v0.5
        .fields.hw_ver                 = 0x0100, // v1.0
        .fields.battery_voltage_mv     = 3600,
        .fields.water_level_percentage = 0,
        .fields.sensor_value_raw       = 0
    };

    // printf("Water Level Sensor Start!\r\n");

    memset(&m_app_sm, 0, sizeof(m_app_sm));

    m_app_sm.State = MEASURE_WATER_LEVEL;

    (void)system_get_chip_id(m_app_sm.ChipId);

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

    while( 1 )
    {
        switch(m_app_sm.State)
        {
        case TX:
            // Send the next PING frame
            // uint32_t random;
            // srand( *ChipId );
            // random = ( rand() + 1 ) % 90;
            // DelayMs( random );
            Radio.Send(tx_message.buffer, sizeof(tx_message_t));
            // printf("Sent: Tx Message %dB\r\n", sizeof(tx_message_t));
            m_app_sm.State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            m_app_sm.State = LOWPOWER;
            break;
        case MEASURE_WATER_LEVEL:
            measure_water_level();
            tx_message.fields.water_level_percentage = m_app_sm.water_level_percentage;
            tx_message.fields.sensor_value_raw = m_app_sm.sensor_value_raw;
            tx_message.fields.error_status = m_app_sm.sensor_error_status;
            m_app_sm.State = TX;
            // printf("Water Level: %d\r\n", tx_message.fields.water_level_percentage);
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

void contact_sensor_read(void)
{
    gpio_write(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_LEVEL_HIGH);
    DelayMs(100);
    m_app_sm.sensor_value_raw  = gpio_read(CONFIG_WATER_SENSOR_1_GPIOX, CONFIG_WATER_SENSOR_1_PIN);
    m_app_sm.sensor_value_raw |= gpio_read(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_2_PIN) << 1;
    gpio_write(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_EN_PIN, GPIO_LEVEL_LOW);
}

void measure_water_level(void)
{
    contact_sensor_read();

    switch(m_app_sm.sensor_value_raw) {
        case 0x0003:
            m_app_sm.water_level_percentage = 0;
            m_app_sm.sensor_error_status = SENSOR_OK;
            break;
        case 0x0001:
            m_app_sm.water_level_percentage = 50;
            m_app_sm.sensor_error_status = SENSOR_OK;
            break;
        case 0x0000:
            m_app_sm.water_level_percentage = 100;
            m_app_sm.sensor_error_status = SENSOR_OK;
            break;
        default:
            m_app_sm.water_level_percentage = 80; //Send 80% in error conditions to prevent false motor running
            m_app_sm.sensor_error_status = SENSOR_INVALID_VAL;
            break;
    }
}