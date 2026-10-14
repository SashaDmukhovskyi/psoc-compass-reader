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
#include "benchmark.h"
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




/** @brief Checks both heading algorithms using predefined vectors. */
static void RunHeadingTests(void)
{
    static const int16 cases[][2] = {
        {1000, 0},
        {1000, 1000},
        {0, 1000},
        {-1000, 1000},
        {-1000, 0},
        {-1000, -1000},
        {0, -1000},
        {1000, -1000},
        {1000, 414},
        {1000, 415},
        {1000, -1}
    };

    IIR_Filter testFilter = {0};
    uint8 index;
    double doubleDegrees;
    double fixedDegrees;
    double error;
    uint32 doubleCentiDeg;
    uint32 fixedCentiDeg;
    uint32 errorMilliDeg;
    char text[160];

    for (index = 0u; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        testFilter.x = cases[index][0];
        testFilter.y = cases[index][1];

        doubleDegrees = Compass_CalculateHeadingDoubleRadians(&testFilter) * RADIANS_TO_DEGREES;
        fixedDegrees = (double)Compass_CalculateHeadingFixed(&testFilter) * RADIANS_TO_DEGREES / 65536.0;

        error = fabs(doubleDegrees - fixedDegrees);

        if (error > 180.0)
        {
            error = 360.0 - error;
        }

        doubleCentiDeg = (uint32)(doubleDegrees * 100.0);
        fixedCentiDeg = (uint32)(fixedDegrees * 100.0);
        errorMilliDeg = (uint32)(error * 1000.0 + 0.5);

        sprintf(text, "[TEST] X:%d Y:%d | Double:%lu.%02lu | Fixed:%lu.%02lu | Error:%lu.%03lu deg\r\n", (int)cases[index][0], (int)cases[index][1], (unsigned long)(doubleCentiDeg / 100u), (unsigned long)(doubleCentiDeg % 100u), (unsigned long)(fixedCentiDeg / 100u), (unsigned long)(fixedCentiDeg % 100u), (unsigned long)(errorMilliDeg / 1000u), (unsigned long)(errorMilliDeg % 1000u));
        UART_1_UartPutString(text);
    }

    testFilter.x = 0;
    testFilter.y = 0;

    sprintf(text, "[TEST] Zero vector: Fixed=%ld; direction undefined\r\n", (long)Compass_CalculateHeadingFixed(&testFilter));
    UART_1_UartPutString(text);
}

/** @brief Repeats timing measurements on an unchanged vector. */
static void RunTimingTest(void)
{
    IIR_Filter input = {1000, 414, 0, 1u};
    uint8 index;
    uint8 interruptState;
    uint32 start;
    uint32 end;
    uint32 doubleCycles;
    uint32 fixedCycles;
    double doubleResult;
    int32 fixedResult;
    volatile double doubleSink;
    volatile int32 fixedSink;
    char text[160];

    for (index = 0u; index < 16u; index++)
    {
        interruptState = CyEnterCriticalSection();

        start = Benchmark_ReadCounter();
        doubleResult = Compass_CalculateHeadingDoubleRadians(&input);
        end = Benchmark_ReadCounter();
        doubleCycles = Benchmark_ElapsedCycles(start, end);

        start = Benchmark_ReadCounter();
        fixedResult = Compass_CalculateHeadingFixed(&input);
        end = Benchmark_ReadCounter();
        fixedCycles = Benchmark_ElapsedCycles(start, end);

        CyExitCriticalSection(interruptState);

        doubleSink = doubleResult;
        fixedSink = fixedResult;

        sprintf(text, "[BENCH] Run:%u X:%ld Y:%ld | Double:%lu | Fixed:%lu cycles\r\n", (unsigned int)index, (long)input.x, (long)input.y, (unsigned long)doubleCycles, (unsigned long)fixedCycles);
        UART_1_UartPutString(text);
    }

    (void)doubleSink;
    (void)fixedSink;
}















/** @brief Initializes and runs the compass reader. **/
int main(void)
{
    uint16 filterDivisor = 8u;
    uint8 chipId;
    uint32 i2cStatus;
    QMC5883L_Data measurement;
    IIR_Filter filter = {0};
    int32 headingFixed;
    uint32 headingFixedCentiDeg;
    char output[160];
    double headingDouble;
    double headingFixedDegrees;
    double headingError;
    uint32 headingDoubleCentiDeg;
    uint32 headingErrorMilliDeg;
    uint32 startCycles;
    uint32 endCycles;
    uint32 doubleCycles;
    uint32 fixedCycles;
    uint8 interruptState;
    uint32 counterOverhead;

    UART_1_Start();
    I2C_1_Start();
    Benchmark_Initialize();

    (void)DRDY_ClearInterrupt();
    isr_DRDY_StartEx(DRDY_handler);

    CyGlobalIntEnable;
    
    
    RunHeadingTests(); //////
    RunTimingTest();

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

                interruptState = CyEnterCriticalSection();
                
                startCycles = Benchmark_ReadCounter();
                endCycles = Benchmark_ReadCounter();
                counterOverhead = Benchmark_ElapsedCycles(startCycles, endCycles);

                startCycles = Benchmark_ReadCounter();
                headingDouble = Compass_CalculateHeadingDoubleRadians(&filter);
                endCycles = Benchmark_ReadCounter();
                doubleCycles = Benchmark_ElapsedCycles(startCycles, endCycles);

                startCycles = Benchmark_ReadCounter();
                headingFixed = Compass_CalculateHeadingFixed(&filter);
                endCycles = Benchmark_ReadCounter();
                fixedCycles = Benchmark_ElapsedCycles(startCycles, endCycles);

                CyExitCriticalSection(interruptState);
                
                headingDouble *= RADIANS_TO_DEGREES;

                headingFixedDegrees = (double)headingFixed * (180.0 / PI) / 65536.0;
                headingFixedCentiDeg = (uint32)(headingFixedDegrees * 100.0);

                headingError = fabs(headingDouble - headingFixedDegrees);

                if (headingError > 180.0)
                {
                    headingError = 360.0 - headingError;
                }

                headingDoubleCentiDeg = (uint32)(headingDouble * 100.0);
                headingErrorMilliDeg = (uint32)(headingError * 1000.0 + 0.5);

                sprintf(output, "[DATA] Double:%lu.%02lu deg | Fixed:%lu.%02lu deg | Error:%lu.%03lu deg\r\n",
                (unsigned long)(headingDoubleCentiDeg / 100u), (unsigned long)(headingDoubleCentiDeg % 100u),
                (unsigned long)(headingFixedCentiDeg / 100u), (unsigned long)(headingFixedCentiDeg % 100u),
                (unsigned long)(headingErrorMilliDeg / 1000u), (unsigned long)(headingErrorMilliDeg % 1000u));
                UART_1_UartPutString(output);

                sprintf(output, "[TIME] Double:%lu cycles | Fixed:%lu cycles | Overhead:%lu cycles\r\n",
                (unsigned long)doubleCycles, (unsigned long)fixedCycles, (unsigned long)counterOverhead);
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