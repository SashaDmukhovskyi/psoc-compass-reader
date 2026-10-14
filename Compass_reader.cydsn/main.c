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
#include "qmc5883l.h"
#include "compass.h"
#include <stdio.h>
#include <math.h>

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

#define PI                             (3.14159265358979323846)
#define RADIANS_TO_DEGREES             (180.0 / PI)

/* Set by the DRDY interrupt and processed in the main loop. */
volatile uint8 dataReady = 0u;

/** @brief Handles the QMC5883L data-ready interrupt. **/
CY_ISR(DRDY_handler)
{
    (void)DRDY_ClearInterrupt();
    dataReady = 1u;
}

/** @brief Initializes and runs the compass reader. **/
int main(void)
{
    uint16 filterDivisor = 8u;
    uint8 chipId;
    uint32 i2cStatus;
    QMC5883L_Data measurement;
    IIR_Filter filter = {0};
    uint16 heading;
    char output[160];

    UART_1_Start();
    I2C_1_Start();

    (void)DRDY_ClearInterrupt();
    isr_DRDY_StartEx(DRDY_handler);

    CyGlobalIntEnable;

    UART_1_UartPutString("[INFO] Compass reader started\r\n");

    i2cStatus = I2C_1_I2CMasterSendStart(QMC5883L_ADDRESS, I2C_WRITE_MODE, I2C_TIMEOUT_MS);

    if (i2cStatus == I2C_STATUS_OK)
    {
        UART_1_UartPutString("[INFO] Device found at 0x0D\r\n");
        (void)I2C_1_I2CMasterSendStop(I2C_TIMEOUT_MS);
    }
    else
    {
        UART_1_UartPutString("[ERROR] No response at 0x0D\r\n");
    }

    if (QMC5883L_ReadRegister(QMC5883L_CHIP_ID_REG, &chipId) != 0u)
    {
        if (chipId == 0xFFu)
        {
            UART_1_UartPutString("[INFO] QMC5883L chip ID confirmed: 0xFF\r\n");

            if (QMC5883L_Initialize() != 0u)
            {
                UART_1_UartPutString("[INFO] QMC5883L initialized\r\n");
            }
            else
            {
                UART_1_UartPutString("[ERROR] QMC5883L initialization failed\r\n");
            }
        }
        else
        {
            UART_1_UartPutString("[ERROR] Unexpected chip ID\r\n");
        }
    }
    else
    {
        UART_1_UartPutString("[ERROR] Failed to read chip ID register\r\n");
    }

    for (;;)
    {
        if (dataReady != 0u)
        {
            dataReady = 0u;

            if (QMC5883L_ReadData(&measurement) != 0u)
            {
                IIR_FilterUpdate(&filter, &measurement, filterDivisor);

                heading = QMC5883L_CalculateHeading(&filter);

                sprintf(output, "[DATA] RAW X:%d Y:%d Z:%d | IIR X:%ld Y:%ld Z:%ld | Heading:%u deg\r\n", (int)measurement.x, (int)measurement.y, (int)measurement.z, (long)filter.x, (long)filter.y, (long)filter.z, (unsigned int)heading);

                UART_1_UartPutString(output);
            }
            else
            {
                UART_1_UartPutString("[ERROR] Failed to read sensor data\r\n");
            }
        }
    }
}

/* [] END OF FILE */