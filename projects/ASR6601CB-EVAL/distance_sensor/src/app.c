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
 *
 * \TODO:
 *  1. Add timeouts to I2C comms
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
#include "tremo_adc.h"
#include "tremo_delay.h"
#include "tremo_i2c.h"
#include "vl53l0x.h"
#include "hal_i2c.h"
#include "sx126x-board.h"



#define RF_FREQUENCY                                865000000 // Hz
#define TX_OUTPUT_POWER                             1         // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        12        // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define SLEEP_TIMEOUT_VALUE                         (10 * 1000)   //ms
#define VBAT_FACTOR                                 3
#define VBAT_UVLO_LEVEL                             2.85f        // V
#define VBAT_UVLO_LEVEL_EXIT                        3.0f        // V
#define DISTANCE_MIN_DELTA                          50          // mm
#define KEEP_ALIVE_TIMEOUT_MS                       (30 * 60 * 1000) // ms
// #define KEEP_ALIVE_TIMEOUT_COUNT                    (KEEP_ALIVE_TIMEOUT_MS/SLEEP_TIMEOUT_VALUE)
#define KEEP_ALIVE_TIMEOUT_COUNT                    0

typedef enum
{
    LOWPOWER,
    TX,
    TX_TIMEOUT,
    MEASURE_DISTANCE
}States_t;

typedef struct {
    States_t State;
    TimerEvent_t SleepTimeoutTimer;
    RadioEvents_t RadioEvents;
    uint16_t prev_distance;
    uint16_t time_since_last_tx;
    float battery_voltage;
}app_sm_t;

typedef struct message_fields{
    uint32_t ChipId[2];
    uint16_t fw_ver;
    uint16_t hw_ver;
    float battery_voltage;   //32bits
    uint16_t distance_mm;
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


float read_vbat(void)
{
    gpio_t *gpiox;
    uint32_t pin;
    float gain_value = 0.95f;
    float dco_value = 0.1f;
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
    calibrated_sample_1 = ((((1.2f*VBAT_FACTOR)/4096) * adc_data_1) - dco_value) * gain_value;
    // calibrated_sample_1 *= VBAT_FACTOR;
    // calibrated_sample_1 = (adc_data_1 - dco_value) / gain_value;
    printf("vbat_adc: %d, vbat_calibrated: %fV, gain: %f, offset: %f\r\n",adc_data_1, calibrated_sample_1, gain_value, dco_value);

    return calibrated_sample_1;
}

// static uint8_t i2c_read(uint8_t reg_addr)
// {
//     uint8_t reg_data;

//     // start
//     i2c_master_send_start(I2C0, 0x29, I2C_WRITE);
//     i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
//     while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

//     // send data
//     i2c_send_data(I2C0, reg_addr);
//     i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
//     while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

//     // send start for read
//     i2c_master_send_start(I2C0, 0x29, I2C_READ);
//     i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
//     while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);

//     // read data
//     i2c_set_receive_mode(I2C0, I2C_NAK);
//     while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
//     i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
//     reg_data = i2c_receive_data(I2C0);

//     // stop
//     i2c_master_send_stop(I2C0);

//     return reg_data;
// }

/**
 * Main application entry point.
 */
int app_start(void)
{
    uint32_t random;
    uint8_t tx_msg[100];
    // uint8_t vl53l0x_reg_data;
    bool vl53l0x_init_done = false;
    uint16_t distance_curr=0;

    int8_t tx_powers[] = {-21, -18, -15, -12, -9, -6, -3, -1, 0, 3, 6, 9, 12, 15, 18, 21, 22};
    uint8_t tx_power_indx = 0;

    tx_message_t tx_message = {
        .fw_ver                  = 0x0102, // v1.2
        .hw_ver                  = 0x0102, // v1.2
        .battery_voltage         = 0.0f,
    };
    (void)system_get_chip_id(tx_message.ChipId);
    tx_message.ChipId[0] = 0x1234;

    printf("Distance Sensor Start!\r\n");

    memset(&m_app_sm, 0, sizeof(m_app_sm));

    m_app_sm.State = MEASURE_DISTANCE;

    // Radio initialization
    m_app_sm.RadioEvents.TxDone    = OnTxDone;
    m_app_sm.RadioEvents.TxTimeout = OnTxTimeout;
    m_app_sm.RadioEvents.RxDone    = NULL;
    m_app_sm.RadioEvents.RxTimeout = NULL;
    m_app_sm.RadioEvents.RxError   = NULL;

    // Internal variables
    m_app_sm.prev_distance = VL53L0X_OUT_OF_RANGE;
    m_app_sm.time_since_last_tx = KEEP_ALIVE_TIMEOUT_COUNT;

    Radio.Init( &m_app_sm.RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 10000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    TimerInit( &m_app_sm.SleepTimeoutTimer, SleepTimeoutIrq );

    // bool rc = vl53l0x_init();
    // if(rc) {
    //     vl53l0x_init_done = true;
    //     printf("VL53L0X Initialised\n");
    // }


    printf("Starting App\r\n");
    printf("Tx Message size: %d Bytes\r\n", sizeof(tx_msg));
    printf("Radio time on air: %ld ms\r\n", Radio.TimeOnAir(MODEM_LORA, sizeof(tx_msg)));

    while( 1 )
    {
        switch(m_app_sm.State)
        {
        case TX:
            if(m_app_sm.time_since_last_tx >= KEEP_ALIVE_TIMEOUT_COUNT || abs(m_app_sm.prev_distance-distance_curr)>50) {
                // Send the next PING frame
                srand(tx_message.ChipId[0]);
                random = ( rand() + 1 ) % 90;
                DelayMs( random );
                m_app_sm.prev_distance = distance_curr;
                tx_message.distance_mm = m_app_sm.prev_distance;
                SX126xSetRfTxPower(tx_powers[tx_power_indx]);
                printf("Send Tx at %i dBm\r\n", tx_powers[tx_power_indx]);
                Radio.Send(tx_msg, sizeof(tx_msg));
                printf("ChipId_L: 0x%lx, ChipId_H: 0x%lx, fw_ver: 0x%x, hw_ver: 0x%x, vbat: %f V, distance: %d mm\r\n",
                    tx_message.ChipId[0],
                    tx_message.ChipId[1],
                    tx_message.fw_ver,
                    tx_message.hw_ver,
                    tx_message.battery_voltage,
                    tx_message.distance_mm
                );
                m_app_sm.time_since_last_tx = 0;
                if(++tx_power_indx >= sizeof(tx_powers)) tx_power_indx = 0;
            } else {
                m_app_sm.time_since_last_tx++;
            }
            m_app_sm.State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            printf("Tx timed out!\r\n");
            m_app_sm.State = LOWPOWER;
            break;
        case MEASURE_DISTANCE:
            m_app_sm.battery_voltage = read_vbat();
            m_app_sm.State = LOWPOWER;
            if(m_app_sm.battery_voltage <= VBAT_UVLO_LEVEL) {
                break;
            } else if(m_app_sm.battery_voltage >= VBAT_UVLO_LEVEL_EXIT) {
                m_app_sm.State = TX;
                if(vl53l0x_init_done) {
                    vl53l0x_read_range_single(VL53L0X_IDX_FIRST, &distance_curr);
                    printf("Measure VL53L0X distance: %d mm\n", distance_curr);
                } else {
                    printf("VL53L0X Unavailable\r\n");
                    tx_message.distance_mm = 0;
                }
            }
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
    Radio.Sleep( );
    m_app_sm.State = TX_TIMEOUT;
}

void SleepTimeoutIrq( void )
{
    // printf("SleepTimeoutIrq\r\n");
    TimerStop(&m_app_sm.SleepTimeoutTimer);
    m_app_sm.State = MEASURE_DISTANCE;
}

