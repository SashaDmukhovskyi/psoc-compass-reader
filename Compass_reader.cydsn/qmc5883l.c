/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/**
 * @file qmc5883l.c
 * @brief Sensor initialization and blocking I2C register access.
 */

#include "project.h"
#include "qmc5883l.h"
#include <stddef.h>

/* Sensor address and registers. */
#define QMC5883L_ADDRESS          (0x0Du)
#define QMC5883L_DATA_START_REG   (0x00u)
#define QMC5883L_DATA_BYTE_COUNT  (6u)
#define QMC5883L_CONTROL_1_REG    (0x09u)
#define QMC5883L_CONTROL_2_REG    (0x0Au)
#define QMC5883L_SET_RESET_REG    (0x0Bu)

/* Sensor configuration values. */
#define QMC5883L_SOFT_RESET       (0x80u)
#define QMC5883L_INTERRUPT_ENABLE (0x00u)
#define QMC5883L_SET_RESET_PERIOD (0x01u)

/* OSR 512, +/-2 G, 200 Hz, continuous measurement. */
#define QMC5883L_CONFIG_VALUE     (0x0Du)

/* I2C operation settings. */
#define I2C_WRITE_MODE            (0u)
#define I2C_READ_MODE             (1u)
#define I2C_ACK                   (0u)
#define I2C_NACK                  (1u)
#define I2C_STATUS_OK             (0u)
#define I2C_TIMEOUT_MS            (100u)


/* Writes one register. Returns 1 on success, 0 on failure. */
uint8 QMC5883L_WriteRegister(uint8 registerAddress, uint8 value)
{
    uint32 status;

    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        registerAddress, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(value, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    return (status == I2C_STATUS_OK) ? 1u : 0u;
}


/* Reads one register. Returns 1 on success, 0 on failure. */
uint8 QMC5883L_ReadRegister(uint8 registerAddress, uint8 *value)
{
    uint32 status;

    if (value == NULL)
    {
        return 0u;
    }

    /* Select the register using a write transaction. */
    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        registerAddress, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    /* Switch to reading without releasing the bus. */
    status = I2C_1_I2CMasterSendRestart(
        QMC5883L_ADDRESS, I2C_READ_MODE, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    /* A single requested byte is also the final byte: send NACK. */
    status = I2C_1_I2CMasterReadByte(
        I2C_NACK, value, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    return (status == I2C_STATUS_OK) ? 1u : 0u;
}


/* Reads raw signed X/Y/Z values. Updates data only after I2C success. */
uint8 QMC5883L_ReadData(QMC5883L_Data *data)
{
    uint8 rawData[QMC5883L_DATA_BYTE_COUNT];
    uint8 index;
    uint8 response;
    uint32 status;

    if (data == NULL)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        QMC5883L_DATA_START_REG, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendRestart(
        QMC5883L_ADDRESS, I2C_READ_MODE, I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    for (index = 0u; index < QMC5883L_DATA_BYTE_COUNT; index++)
    {
        /* ACK for the first five bytes, NACK for the last byte. */
        response = (index == QMC5883L_DATA_BYTE_COUNT - 1u)
            ? I2C_NACK
            : I2C_ACK;

        status = I2C_1_I2CMasterReadByte(
            response, &rawData[index], I2C_TIMEOUT_MS);

        if (status != I2C_STATUS_OK)
        {
            (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
            return 0u;
        }
    }

    status = I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    /* Combine each low/high byte pair into a signed 16-bit value. */
    data->x = (int16)(((uint16)rawData[1] << 8u) | rawData[0]);
    data->y = (int16)(((uint16)rawData[3] << 8u) | rawData[2]);
    data->z = (int16)(((uint16)rawData[5] << 8u) | rawData[4]);

    return 1u;
}


/* Resets and configures the sensor. Returns 1 on success. */
uint8 QMC5883L_Initialize(void)
{
    if (QMC5883L_WriteRegister(
            QMC5883L_CONTROL_2_REG,
            QMC5883L_SOFT_RESET) == 0u)
    {
        return 0u;
    }

    CyDelay(10u);

    if (QMC5883L_WriteRegister(
            QMC5883L_SET_RESET_REG,
            QMC5883L_SET_RESET_PERIOD) == 0u)
    {
        return 0u;
    }

    if (QMC5883L_WriteRegister(
            QMC5883L_CONTROL_2_REG,
            QMC5883L_INTERRUPT_ENABLE) == 0u)
    {
        return 0u;
    }

    if (QMC5883L_WriteRegister(
            QMC5883L_CONTROL_1_REG,
            QMC5883L_CONFIG_VALUE) == 0u)
    {
        return 0u;
    }

    return 1u;
}
/* [] END OF FILE */
