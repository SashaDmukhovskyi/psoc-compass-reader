/* ========================================
 *
 * Compass reader using QMC5883L
 *
 * ========================================
 */

/**
 * @file main.c
 * @brief QMC5883L compass reader using I2C, DRDY interrupt and UART.
 */

#include "project.h"
#include <stdio.h>


/* QMC5883L I2C settings */
#define QMC5883L_ADDRESS               (0x0Du)
#define QMC5883L_CHIP_ID_REG           (0x0Du)

/* QMC5883L data registers */
#define QMC5883L_DATA_START_REG        (0x00u)
#define QMC5883L_DATA_BYTE_COUNT       (6u)

/* QMC5883L configuration registers */
#define QMC5883L_CONTROL_1_REG         (0x09u)
#define QMC5883L_CONTROL_2_REG         (0x0Au)
#define QMC5883L_SET_RESET_REG         (0x0Bu)

/* QMC5883L configuration values */
#define QMC5883L_SOFT_RESET            (0x80u)
#define QMC5883L_INTERRUPT_ENABLE      (0x00u)
#define QMC5883L_SET_RESET_PERIOD      (0x01u)

/* OSR 512, +/-2 G, 50 Hz, continuous measurement */
#define QMC5883L_CONFIG_VALUE          (0x05u)

/* I2C settings */
#define I2C_WRITE_MODE                 (0u)
#define I2C_READ_MODE                  (1u)
#define I2C_ACK                        (0u)
#define I2C_NACK                       (1u)
#define I2C_STATUS_OK                  (0u)
#define I2C_TIMEOUT_MS                 (100u)

/* Higher values provide stronger smoothing but slower response. */
#define IIR_FILTER_DIVISOR             (8)


typedef struct
{
    int16 x;
    int16 y;
    int16 z;
} QMC5883L_Data;

typedef struct
{
    int32 x;
    int32 y;
    int32 z;
    uint8 initialized;
} IIR_Filter;

/* Set by the DRDY interrupt and processed in the main loop. */
volatile uint8 dataReady = 0u;


/**
 * @brief Writes one byte to a QMC5883L register.
 *
 * @retval 1u Write completed successfully.
 * @retval 0u I2C communication failed.
 */
uint8 QMC5883L_WriteRegister(uint8 registerAddress, uint8 value)
{
    uint32 status;

    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS,
        I2C_WRITE_MODE,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        registerAddress,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        value,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    return (status == I2C_STATUS_OK) ? 1u : 0u;
}


/**
 * @brief Reads one byte from a QMC5883L register.
 *
 * @retval 1u Read completed successfully.
 * @retval 0u I2C communication failed.
 */
uint8 QMC5883L_ReadRegister(uint8 registerAddress, uint8 *value)
{
    uint32 status;

    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS,
        I2C_WRITE_MODE,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        registerAddress,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendRestart(
        QMC5883L_ADDRESS,
        I2C_READ_MODE,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterReadByte(
        I2C_NACK,
        value,
        I2C_TIMEOUT_MS
    );

    (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    return (status == I2C_STATUS_OK) ? 1u : 0u;
}


/**
 * @brief Reads raw X, Y and Z measurements from the QMC5883L.
 *
 * @retval 1u Measurement was read successfully.
 * @retval 0u I2C communication failed.
 */
uint8 QMC5883L_ReadData(QMC5883L_Data *data)
{
    uint8 rawData[QMC5883L_DATA_BYTE_COUNT];
    uint8 index;
    uint8 response;
    uint32 status;

    status = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS,
        I2C_WRITE_MODE,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        return 0u;
    }

    status = I2C_1_I2CMasterWriteByte(
        QMC5883L_DATA_START_REG,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    status = I2C_1_I2CMasterSendRestart(
        QMC5883L_ADDRESS,
        I2C_READ_MODE,
        I2C_TIMEOUT_MS
    );

    if (status != I2C_STATUS_OK)
    {
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
        return 0u;
    }

    for (index = 0u; index < QMC5883L_DATA_BYTE_COUNT; index++)
    {
        if (index == (QMC5883L_DATA_BYTE_COUNT - 1u))
        {
            response = I2C_NACK;
        }
        else
        {
            response = I2C_ACK;
        }

        status = I2C_1_I2CMasterReadByte(
            response,
            &rawData[index],
            I2C_TIMEOUT_MS
        );

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

    data->x = (int16)(
        ((uint16)rawData[1] << 8u) |
        rawData[0]
    );

    data->y = (int16)(
        ((uint16)rawData[3] << 8u) |
        rawData[2]
    );

    data->z = (int16)(
        ((uint16)rawData[5] << 8u) |
        rawData[4]
    );

    return 1u;
}

/**
 * @brief Updates the IIR filter using a new sensor measurement.
 *
 * @param[in,out] filter      Current state of the IIR filter.
 * @param[in]     measurement New raw magnetometer measurement.
 */
void IIR_FilterUpdate(
    IIR_Filter *filter,
    const QMC5883L_Data *measurement)
{
    if (filter->initialized == 0u)
    {
        filter->x = measurement->x;
        filter->y = measurement->y;
        filter->z = measurement->z;
        filter->initialized = 1u;
        
        return;
    }
        filter->x +=
            ((int32)measurement->x - filter->x) /
            IIR_FILTER_DIVISOR;

        filter->y +=
            ((int32)measurement->y - filter->y) /
            IIR_FILTER_DIVISOR;

        filter->z +=
            ((int32)measurement->z - filter->z) /
            IIR_FILTER_DIVISOR;
}

/**
 * @brief Configures the QMC5883L for continuous measurements.
 *
 * @retval 1u Configuration completed successfully.
 * @retval 0u A register write failed.
 */
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


/**
 * @brief Handles the QMC5883L data-ready interrupt.
 */
CY_ISR(DRDY_handler)
{
    (void)DRDY_ClearInterrupt();
    dataReady = 1u;
}


/**
 * @brief Initializes and runs the compass reader.
 */
int main(void)
{
    uint8 chipId;
    uint32 i2cStatus;
    QMC5883L_Data measurement;
    IIR_Filter filter = {0};
    char output[140];

    UART_1_Start();
    I2C_1_Start();

    (void)DRDY_ClearInterrupt();
    isr_DRDY_StartEx(DRDY_handler);

    CyGlobalIntEnable;

    UART_1_UartPutString("Compass reader started\r\n");

    i2cStatus = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS,
        I2C_WRITE_MODE,
        I2C_TIMEOUT_MS
    );

    if (i2cStatus == I2C_STATUS_OK)
    {
        UART_1_UartPutString(
            "Device found at 0x0D\r\n"
        );
    }
    else
    {
        UART_1_UartPutString(
            "No response at 0x0D\r\n"
        );
    }

    (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    if (QMC5883L_ReadRegister(
            QMC5883L_CHIP_ID_REG,
            &chipId) != 0u)
    {
        if (chipId == 0xFFu)
        {
            UART_1_UartPutString(
                "QMC5883L chip ID confirmed: 0xFF\r\n"
            );

            if (QMC5883L_Initialize() != 0u)
            {
                UART_1_UartPutString(
                    "QMC5883L initialized\r\n"
                );
            }
            else
            {
                UART_1_UartPutString(
                    "QMC5883L initialization failed\r\n"
                );
            }
        }
        else
        {
            UART_1_UartPutString(
                "Unexpected chip ID\r\n"
            );
        }
    }
    else
    {
        UART_1_UartPutString(
            "Failed to read chip ID register\r\n"
        );
    }

    for (;;)
    {
        if (dataReady != 0u)
        {
            dataReady = 0u;

            if (QMC5883L_ReadData(&measurement) != 0u)
            {
                IIR_FilterUpdate(&filter, &measurement);
                
                sprintf(
                    output,
                    "RAW X:%d Y:%d Z:%d | IIR X:%ld Y:%ld Z:%ld\r\n",
                    (int)measurement.x,
                    (int)measurement.y,
                    (int)measurement.z,
                    (long)filter.x,
                    (long)filter.y,
                    (long)filter.z                    
                );

                UART_1_UartPutString(output);
            }
            else
            {
                UART_1_UartPutString(
                    "Failed to read sensor data\r\n"
                );
            }
        }
    }
}

/* [] END OF FILE */