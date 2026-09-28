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


/* QMC5883L I2C settings */
#define QMC5883L_ADDRESS               (0x0Du)
#define QMC5883L_CHIP_ID_REG           (0x0Du)

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
#define I2C_NACK                       (1u)
#define I2C_STATUS_OK                  (0u)
#define I2C_TIMEOUT_MS                 (100u)


/* Set by the DRDY interrupt and processed in the main loop. */
volatile uint8 dataReady = 0u;


/**
 * @brief Writes one byte to a QMC5883L register.
 *
 * @param[in] registerAddress Destination register address.
 * @param[in] value           Value to write.
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
 * @param[in]  registerAddress Register address to read.
 * @param[out] value           Destination for the received byte.
 *
 * @retval 1u Read completed successfully.
 * @retval 0u I2C communication failed.
 */
uint8 QMC5883L_ReadRegister(uint8 registerAddress, uint8 *value)
{
    uint32 status;

    /*
     * Write the register address first to set the sensor's
     * internal register pointer.
     */
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

    /* Repeated START changes direction without releasing the bus. */
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

    /* NACK indicates that this is the final requested byte. */
    status = I2C_1_I2CMasterReadByte(
        I2C_NACK,
        value,
        I2C_TIMEOUT_MS
    );

    (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    return (status == I2C_STATUS_OK) ? 1u : 0u;
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
 * @brief Initializes the compass reader application.
 *
 * @return This function does not return.
 */
int main(void)
{
    uint8 chipId;
    uint32 i2cStatus;

    UART_1_Start();
    I2C_1_Start();

    /* Clear an old pending event before enabling the interrupt. */
    (void)DRDY_ClearInterrupt();
    isr_DRDY_StartEx(DRDY_handler);

    CyGlobalIntEnable;

    UART_1_UartPutString("Compass reader started\r\n");

    /* Check whether a device acknowledges address 0x0D. */
    i2cStatus = I2C_1_I2CMasterSendStart(
        QMC5883L_ADDRESS,
        I2C_WRITE_MODE,
        I2C_TIMEOUT_MS
    );

    if (i2cStatus == I2C_STATUS_OK)
    {
        UART_1_UartPutString("Device found at 0x0D\r\n");
    }
    else
    {
        UART_1_UartPutString("No response at 0x0D\r\n");
    }

    (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);

    /* Verify that the responding device is a QMC5883L. */
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

            UART_1_UartPutString(
                "DRDY interrupt received\r\n"
            );
        }
    }
}

/* [] END OF FILE */