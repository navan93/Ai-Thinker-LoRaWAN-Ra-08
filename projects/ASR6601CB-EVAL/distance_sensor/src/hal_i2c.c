#include "hal_i2c.h"
#include "tremo_i2c.h"
#include <stdio.h>


#define DEFAULT_SLAVE_ADDRESS (0x29)

typedef enum
{
    ADDR_SIZE_8BIT,
    ADDR_SIZE_16BIT
} addr_size_t;

typedef enum
{
    REG_SIZE_8BIT,
    REG_SIZE_16BIT,
    REG_SIZE_32BIT
} reg_size_t;

uint8_t m_dev_addr = 0x29;

static bool start_transfer(addr_size_t addr_size, uint16_t addr)
{
    bool success = false;
    uint8_t addr_buf;


    /* Note, when master is TX, we must write to TXBUF before waiting for UCTXSTT */
    switch (addr_size) {
    case ADDR_SIZE_8BIT:
        addr_buf = (uint8_t)(addr & 0xFF);
        break;
    case ADDR_SIZE_16BIT:
        addr_buf = (uint8_t)((addr >> 8) & 0xFF); /* Send the most significant byte of the 16-bit address */
        break;
    }

    i2c_master_send_start(I2C0, m_dev_addr, I2C_WRITE); /* Set up master as TX and send start condition */
    /* Wait for start condition to be sent */
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);
    success = true;
    if (success) {
        // send data
        i2c_send_data(I2C0, addr_buf);
        i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);
    }

    if (success) {
        switch (addr_size) {
        case ADDR_SIZE_8BIT:
            break;
        case ADDR_SIZE_16BIT:
            addr_buf = addr & 0xFF;
            // send data
            i2c_send_data(I2C0, addr_buf);
            i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
            while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);
            success = (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) == SET);
            break;
        }
    }
    return success;
}

static void stop_transfer()
{
    i2c_master_send_stop(I2C0); /* Send stop condition */
     /* Wait for stop condition to be sent */
}

/* Read a register of size reg_size at address addr.
 * NOTE: The bytes are read from MSB to LSB. */
static bool read_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint8_t *data)
{
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    /* Address sent, now configure as receiver and get the data */
    i2c_master_send_start(I2C0, m_dev_addr, I2C_READ);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET);
    success = true;
    if (success) {
        switch (reg_size) {
        case REG_SIZE_8BIT:
            break;
        case REG_SIZE_16BIT:
            /* Bytes are read from most to least significant */
            /* Wait for byte before reading the buffer */
            i2c_set_receive_mode(I2C0, I2C_ACK);
            while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
            i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
            data[1] = i2c_receive_data(I2C0);
            break;
        case REG_SIZE_32BIT:
            /* Bytes are read from most to least significant */
            i2c_set_receive_mode(I2C0, I2C_ACK);
            while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
            i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
            data[3] = i2c_receive_data(I2C0);

            i2c_set_receive_mode(I2C0, I2C_ACK);
            while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
            i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
            data[2] = i2c_receive_data(I2C0);

            i2c_set_receive_mode(I2C0, I2C_ACK);
            while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
            i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
            data[1] = i2c_receive_data(I2C0);
            break;
        }
        i2c_set_receive_mode(I2C0, I2C_NAK);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
        i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
        data[0] = i2c_receive_data(I2C0);
        stop_transfer();
    }

    return success;
}

static bool read_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count)
{
    bool success = false;
    bool transfer_stopped = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    /* Address sent, now configure as receiver and get the data */
    i2c_master_send_start(I2C0, m_dev_addr, I2C_READ);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET); /* Wait for start byte to be sent */
    success = true;
    if (success) {
        for (int i = 0; i < byte_count; i++) {
            if (i + 1 == byte_count) {
            //     stop_transfer();
            //     transfer_stopped = true;
                i2c_set_receive_mode(I2C0, I2C_NAK);
            } else i2c_set_receive_mode(I2C0, I2C_ACK);
            success = true;
            if (success) {
                // i2c_set_receive_mode(I2C0, I2C_ACK);
                while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET);
                i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
                bytes[i] = i2c_receive_data(I2C0);
            } else {
                break;
            }
        }
    }
    if (!transfer_stopped) {
        stop_transfer();
    }

    return success;
}

bool hal_i2c_read_addr8_data8(uint8_t addr, uint8_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, data);
}

bool hal_i2c_read_addr8_data16(uint8_t addr, uint16_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool hal_i2c_read_addr16_data8(uint16_t addr, uint8_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, data);
}

bool hal_i2c_read_addr16_data16(uint16_t addr, uint16_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, (uint8_t *)data);
}

bool hal_i2c_read_addr8_data32(uint16_t addr, uint32_t *data)
{
    return read_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool hal_i2c_read_addr16_data32(uint16_t addr, uint32_t *data)
{
    return read_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_32BIT, (uint8_t *)data);
}

bool hal_i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return read_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

/* Write data to a register of size reg_size at address addr.
 * NOTE: Writes the most significant byte (MSB) first. */
static bool write_reg(addr_size_t addr_size, uint16_t addr, reg_size_t reg_size, uint16_t data)
{
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    switch (reg_size) {
    case REG_SIZE_8BIT:
        success = true;
        break;
    case REG_SIZE_16BIT:
        // send data
        i2c_send_data(I2C0, ((data >> 8) & 0xFF)); /* Start with the most significant byte */
        i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET); /* Wait for byte to be sent */
        success = true;
        break;
    case REG_SIZE_32BIT:
        /* Not supported */
        return false;
    }

    if (success) {
        i2c_send_data(I2C0, (0xFF & data)); /* Send the least significant byte */
        i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET); /* Wait for byte to be sent */
        success = true;
    }

    stop_transfer();
    return success;
}

static bool write_reg_bytes(addr_size_t addr_size, uint16_t addr, uint8_t *bytes, uint16_t byte_count)
{
    bool success = false;

    if (!start_transfer(addr_size, addr)) {
        return false;
    }

    for (uint16_t i = 0; i < byte_count; i++) {
        i2c_send_data(I2C0, bytes[i]);
        i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET); /* Wait for byte to be sent */
        success = true;
        if (!success) {
            break;
        }
    }

    stop_transfer();
    return success;
}

bool hal_i2c_write_addr8_data8(uint8_t addr, uint8_t value)
{
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_8BIT, value);
}

bool hal_i2c_write_addr8_data16(uint8_t addr, uint16_t value)
{
    return write_reg(ADDR_SIZE_8BIT, addr, REG_SIZE_16BIT, value);
}

bool hal_i2c_write_addr16_data8(uint16_t addr, uint8_t value)
{
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_8BIT, value);
}

bool hal_i2c_write_addr16_data16(uint16_t addr, uint16_t value)
{
    return write_reg(ADDR_SIZE_16BIT, addr, REG_SIZE_16BIT, value);
}
bool hal_i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    return write_reg_bytes(ADDR_SIZE_8BIT, start_addr, bytes, byte_count);
}

void hal_i2c_set_slave_address(uint8_t addr)
{
    m_dev_addr = addr;
}

void hal_i2c_init()
{
    /* Pinmux P1.6 (SCL) and P1.7 (SDA) to I2C peripheral  */
    // P1SEL |= BIT6 + BIT7;
    // P1SEL2 |= BIT6 + BIT7;

    // UCB0CTL1 |= UCSWRST; /* Enable SW reset */
    // UCB0CTL0 = UCMST + UCSYNC + UCMODE_3; /* Single master, synchronous mode, I2C mode */
    // UCB0CTL1 |= UCSSEL_2; /* SMCLK */
    // UCB0BR0 = 10; /* SMCLK / 10 = ~100kHz */
    // UCB0BR1 = 0;
    // UCB0CTL1 &= ~UCSWRST; /* Clear SW */
    hal_i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS);
}