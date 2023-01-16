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

typedef enum
{
    LOWPOWER,
    MEASURE_WATER_LEVEL,
    TX,
    TX_TIMEOUT
}States_t;

typedef union {
    struct message_fields{
        uint16_t device_id;
        uint16_t fw_ver;
        uint16_t hw_ver;
        uint8_t  water_level_percentage;
        uint8_t  sensor_value_raw;
        uint16_t battery_voltage_mv;
        uint8_t  error_status;
    }fields;
    uint8_t buffer[11];
}tx_message_t;

#define BUFFER_SIZE                                 sizeof(tx_message_t) // Define the payload size here
#define SLEEP_TIMEOUT_VALUE                         7000

static volatile States_t State = TX;
static uint32_t ChipId[2] = {0};
static TimerEvent_t SleepTimeoutTimer;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

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

uint16_t get_sensor_value(void);
uint8_t measure_water_level(void);

/**
 * Main application entry point.
 */
int app_start( void )
{
    bool isMaster = true;
    uint8_t i;
    uint32_t random;
    tx_message_t tx_message = {
        .fields.device_id              = 1,
        .fields.fw_ver                 = 0x0005, // v0.5
        .fields.hw_ver                 = 0x0100, // v1.0
        .fields.battery_voltage_mv     = 3600,
        .fields.water_level_percentage = 0,
        .fields.sensor_value_raw       = 0
    };

    printf("PingPong test Start!\r\n");

    (void)system_get_chip_id(ChipId);

    // Radio initialization
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = NULL;
    RadioEvents.RxTimeout = NULL;
    RadioEvents.RxError   = NULL;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 1000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    TimerInit( &SleepTimeoutTimer, SleepTimeoutIrq );

    while( 1 )
    {
        switch( State )
        {
        case TX:
            // Send the next PING frame
            // srand( *ChipId );
            // random = ( rand() + 1 ) % 90;
            // DelayMs( random );
            Radio.Send(tx_message.buffer, sizeof(tx_message_t));
            printf("Sent: Tx Message\r\n");
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            State = LOWPOWER;
            break;
        case MEASURE_WATER_LEVEL:
            tx_message.fields.water_level_percentage = measure_water_level();
            tx_message.fields.sensor_value_raw = get_sensor_value();
            State = TX;
            printf("Water Level: %d\%", tx_message.fields.water_level_percentage);
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
    printf("OnTxDone\r\n");
    Radio.Sleep( );
    TimerSetValue(&SleepTimeoutTimer, SLEEP_TIMEOUT_VALUE);
    TimerStart(&SleepTimeoutTimer);
    State = LOWPOWER;
}

void OnTxTimeout( void )
{
    printf("OnTxTimeout\r\n");
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void SleepTimeoutIrq( void )
{
    printf("SleepTimeoutIrq\r\n");
    TimerStop(&SleepTimeoutTimer);
    State = MEASURE_WATER_LEVEL;
}

uint16_t get_sensor_value(void)
{
    uint16_t sensor_val;
    sensor_val  = gpio_read(CONFIG_WATER_SENSOR_1_GPIOX, CONFIG_WATER_SENSOR_1_PIN);
    sensor_val |= gpio_read(CONFIG_WATER_SENSOR_2_GPIOX, CONFIG_WATER_SENSOR_2_PIN) << 1;
    return sensor_val;
}

uint8_t measure_water_level(void)
{
    uint16_t sensor_val = get_sensor_value();
    uint8_t water_level;

    switch(sensor_val) {
        case 0x0000:
            water_level = 0;
            break;
        case 0x0001:
            water_level = 50;
            break;
        case 0x0003:
            water_level = 100;
            break;
        default: 
            water_level = 100; //Send 100% in error conditions to prevent false motor running
            break;
    }
    return water_level;
}