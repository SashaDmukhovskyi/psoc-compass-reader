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
#include "project.h"
#include "qmc5883l.h"

/* QMC5883L I2C settings */
#define QMC5883L_ADDRESS           (0x0Du)
#define QMC5883L_CHIP_ID_REG       (0x0Du)

/* QMC5883L data registers */
#define QMC5883L_DATA_START_REG    (0x00u)
#define QMC5883L_DATA_BYTE_COUNT   (6u)

/* QMC5883L configuration registers */
#define QMC5883L_CONTROL_1_REG     (0x09u)
#define QMC5883L_CONTROL_2_REG     (0x0Au)
#define QMC5883L_SET_RESET_REG     (0x0Bu)

/* QMC5883L configuration values */
#define QMC5883L_SOFT_RESET        (0x80u)
#define QMC5883L_INTERRUPT_ENABLE  (0x00u)
#define QMC5883L_SET_RESET_PERIOD  (0x01u)

/* OSR 512, +/-2 G, 50 Hz, continuous measurement */
#define QMC5883L_CONFIG_VALUE      (0x05u)

/* I2C settings */
#define I2C_WRITE_MODE             (0u)
#define I2C_READ_MODE              (1u)
#define I2C_ACK                    (0u)
#define I2C_NACK                   (1u)
#define I2C_STATUS_OK              (0u)
#define I2C_TIMEOUT_MS             (100u)


/** @brief Writes one byte to a QMC5883L register. */
uint8 QMC5883L_WriteRegister(uint8 registerAddress, uint8 value)
{
    uint32 status;

    status = I2C_1_I2CMasterSendStart(QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }
    
    status = I2C_1_I2CMasterWriteByte(registerAddress, I2C_TIMEOUT_MS);
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

/** @brief Reads one byte from QMC5883L register **/
uint8 QMC5883L_ReadRegister(uint8 registerAddress, uint8 *value)
{
    uint32 status;
    
    status = I2C_1_I2CMasterSendStart(QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }
    
    status = I2C_1_I2CMasterWriteByte(registerAddress, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }
    
    status = I2C_1_I2CMasterSendRestart(QMC5883L_ADDRESS, I2C_READ_MODE, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }
    
    status = I2C_1_I2CMasterReadByte(I2C_NACK, value, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }
    
    status = I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
    
    return (status == I2C_STATUS_OK) ? 1u : 0u; 
}

uint8 QMC5883L_ReadData(QMC5883L_Data *data)
{
    uint8 rawData[QMC5883L_DATA_BYTE_COUNT];
    uint8 index;
    uint8 response;
    uint32 status;
    
    status = I2C_1_I2CMasterSendStart(QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }
    
    status = I2C_1_I2CMasterWriteByte(QMC5883L_DATA_START_REG, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u; 
    }
    
    status = I2C_1_I2CMasterSendRestart(QMC5883L_ADDRESS, I2C_READ_MODE, I2C_TIMEOUT_MS);
    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }
    
    for (index = 0; index < QMC5883L_DATA_BYTE_COUNT; index++)
    {
        if (index == (QMC5883L_DATA_BYTE_COUNT - 1u))
        {
            response = I2C_NACK;
        }
        else
        {
            response = I2C_ACK;
        }
        
        status = I2C_1_I2CMasterReadByte(response, &rawData[index], I2C_TIMEOUT_MS);
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
    
    data->x =(int16)(((uint16)rawData[1] << 8u) | rawData[0]);
    data->y =(int16)(((uint16)rawData[3] << 8u) | rawData[2]);
    data->z =(int16)(((uint16)rawData[5] << 8u) | rawData[4]);
    
    return 1u;
}

uint8 QMC5883L_Initialize(void)
{
    uint8 status;

    status = QMC5883L_WriteRegister(QMC5883L_CONTROL_2_REG, QMC5883L_SOFT_RESET);
    if (status == 0u)
    {
        return 0u;
    }

    CyDelay(10u);

    status = QMC5883L_WriteRegister(QMC5883L_SET_RESET_REG, QMC5883L_SET_RESET_PERIOD);
    if (status == 0u)
    {
        return 0u;
    }

    status = QMC5883L_WriteRegister(QMC5883L_CONTROL_2_REG, QMC5883L_INTERRUPT_ENABLE);
    if (status == 0u)
    {
        return 0u;
    }

    status = QMC5883L_WriteRegister(QMC5883L_CONTROL_1_REG, QMC5883L_CONFIG_VALUE);
    if (status == 0u)
    {
        return 0u;
    }

    return 1u;
}
/* [] END OF FILE */
