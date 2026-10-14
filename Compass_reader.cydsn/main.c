/* ========================================
 *
 * Compass reader using QMC5883L
 *
 * ========================================
 */
/**
 * @file main.c
 * @brief Reads the magnetometer and compares double and fixed-point headings.
 */

#include "project.h"
#include "qmc5883l.h"
#include "compass.h"
#include "benchmark.h"
#include <stdio.h>
#include <math.h>

/* Change these settings, rebuild and program the board. */
#define FILTER_DIVISOR       (8u)
#define OUTPUT_PERIOD_MS     (1000u)
#define RUN_STARTUP_TESTS     (0u)

#define CHIP_ID_REGISTER     (0x0Du)
#define EXPECTED_CHIP_ID     (0xFFu)

#define PI                   (3.14159265358979323846)
#define RADIANS_TO_DEGREES   (180.0 / PI)
#define FIXED_SCALE          (65536.0)
#define CPU_FREQUENCY_HZ     (CYDEV_BCLK__SYSCLK__HZ)
#define OUTPUT_PERIOD_CYCLES ((CPU_FREQUENCY_HZ / 1000u) * OUTPUT_PERIOD_MS)

#if FILTER_DIVISOR < 1u || FILTER_DIVISOR > 65535u
#error FILTER_DIVISOR must be between 1 and 65535.
#endif

#if OUTPUT_PERIOD_MS < 1u || OUTPUT_PERIOD_MS > 1000u
#error OUTPUT_PERIOD_MS must be between 1 and 1000.
#endif

static volatile uint8 dataReady = 0u;


/** @brief Signals that a new sensor measurement is available. */
CY_ISR(DRDY_handler)
{
    (void)DRDY_ClearInterrupt();
    dataReady = 1u;
}


/** @brief Measures both algorithms and prints their results outside the timed interval. */
static void PrintComparison(const char *label, const IIR_Filter *filter)
{
    uint8 interruptState;
    uint32 start;
    uint32 end;
    uint32 doubleCycles;
    uint32 fixedCycles;
    uint32 overhead;
    int32 fixedRadians;
    double doubleRadians;
    double doubleDegrees;
    double fixedDegrees;
    double error;
    uint32 doubleCentiDeg;
    uint32 fixedCentiDeg;
    uint32 errorMilliDeg;
    char output[256];

    interruptState = CyEnterCriticalSection();

    start = Benchmark_ReadCounter();
    end = Benchmark_ReadCounter();
    overhead = Benchmark_ElapsedCycles(start, end);

    start = Benchmark_ReadCounter();
    doubleRadians = Compass_CalculateHeadingDoubleRadians(filter);
    end = Benchmark_ReadCounter();
    doubleCycles = Benchmark_ElapsedCycles(start, end);

    start = Benchmark_ReadCounter();
    fixedRadians = Compass_CalculateHeadingFixed(filter);
    end = Benchmark_ReadCounter();
    fixedCycles = Benchmark_ElapsedCycles(start, end);

    CyExitCriticalSection(interruptState);

    doubleDegrees = doubleRadians * RADIANS_TO_DEGREES;
    fixedDegrees = (double)fixedRadians * RADIANS_TO_DEGREES / FIXED_SCALE;

    error = fabs(doubleDegrees - fixedDegrees);

    if (error > 180.0)
    {
        error = 360.0 - error;
    }

    doubleCentiDeg = (uint32)(doubleDegrees * 100.0);
    fixedCentiDeg = (uint32)(fixedDegrees * 100.0);
    errorMilliDeg = (uint32)(error * 1000.0 + 0.5);

    (void)snprintf(output, sizeof(output), "[%s] X:%ld Y:%ld Z:%ld | Double:%lu.%02lu deg | Fixed:%lu.%02lu deg | Error:%lu.%03lu deg\r\n", label, (long)filter->x, (long)filter->y, (long)filter->z, (unsigned long)(doubleCentiDeg / 100u), (unsigned long)(doubleCentiDeg % 100u), (unsigned long)(fixedCentiDeg / 100u), (unsigned long)(fixedCentiDeg % 100u), (unsigned long)(errorMilliDeg / 1000u), (unsigned long)(errorMilliDeg % 1000u));
    UART_1_UartPutString(output);

    (void)snprintf(output, sizeof(output), "[TIME] Double:%lu cycles | Fixed:%lu cycles | Overhead:%lu cycles\r\n", (unsigned long)doubleCycles, (unsigned long)fixedCycles, (unsigned long)overhead);
    UART_1_UartPutString(output);
}


/** @brief Checks predefined directions and repeats one timing case. */
static void RunStartupTests(void)
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

    IIR_Filter input = {0};
    uint32 index;
    char output[100];

    UART_1_UartPutString("[INFO] Heading tests\r\n");

    for (index = 0u; index < (sizeof(cases) / sizeof(cases[0])); index++)
    {
        input.x = cases[index][0];
        input.y = cases[index][1];
        PrintComparison("TEST", &input);
    }

    input.x = 0;
    input.y = 0;

    (void)snprintf(output, sizeof(output), "[TEST] Zero vector: Fixed=%ld; direction undefined\r\n", (long)Compass_CalculateHeadingFixed(&input));
    UART_1_UartPutString(output);

    input.x = 1000;
    input.y = 414;

    UART_1_UartPutString("[INFO] Timing test: 16 repetitions, X=1000 Y=414\r\n");

    for (index = 0u; index < 16u; index++)
    {
        PrintComparison("BENCH", &input);
    }
}


/** @brief Initializes the sensor and processes data-ready events. */
int main(void)
{
    uint8 chipId;
    uint8 interruptState;
    uint8 ready;
    uint32 previousCounter;
    uint32 currentCounter;
    uint32 elapsed;
    uint32 outputElapsed = 0u;
    QMC5883L_Data measurement;
    IIR_Filter filter = {0};
    uint32 readCount = 0u;
    uint32 rateElapsed = 0u;
    char rateOutput[100];

    UART_1_Start();
    I2C_1_Start();
    Benchmark_Initialize();

    CyGlobalIntEnable;

    UART_1_UartPutString("[INFO] Compass reader started\r\n");

    if (RUN_STARTUP_TESTS != 0u)
    {
        RunStartupTests();
    }

    if (QMC5883L_ReadRegister(CHIP_ID_REGISTER, &chipId) == 0u)
    {
        UART_1_UartPutString("[ERROR] Failed to read chip ID\r\n");

        for (;;)
        {
            CyDelay(100u);
        }
    }

    if (chipId != EXPECTED_CHIP_ID)
    {
        UART_1_UartPutString("[ERROR] Unexpected chip ID\r\n");

        for (;;)
        {
            CyDelay(100u);
        }
    }

    UART_1_UartPutString("[INFO] QMC5883L-compatible chip ID: 0xFF\r\n");

    (void)DRDY_ClearInterrupt();
    isr_DRDY_StartEx(DRDY_handler);

    if (QMC5883L_Initialize() == 0u)
    {
        UART_1_UartPutString("[ERROR] Sensor initialization failed\r\n");

        for (;;)
        {
            CyDelay(100u);
        }
    }

    UART_1_UartPutString("[INFO] Sensor initialized\r\n");
    if (QMC5883L_ReadRegister(0x09u, &chipId) != 0u)
    {
        if (chipId == 0x0Du)
        {
            UART_1_UartPutString("[INFO] Configuration confirmed: 200 Hz\r\n");
        }
        else
        {
            UART_1_UartPutString("[ERROR] Unexpected sensor configuration\r\n");
        }
    }
    else
    {
        UART_1_UartPutString("[ERROR] Failed to verify sensor configuration\r\n");
}

    previousCounter = Benchmark_ReadCounter();

    for (;;)
    {
        /* Accumulate elapsed cycles; each poll must be within one SysTick period. */
        currentCounter = Benchmark_ReadCounter();
        elapsed = Benchmark_ElapsedCycles(previousCounter, currentCounter);
        previousCounter = currentCounter;
        
        
        
        rateElapsed += elapsed;

        if (rateElapsed >= CPU_FREQUENCY_HZ)
        {
            (void)snprintf(rateOutput, sizeof(rateOutput), "[RATE] Reads:%lu | Window:%lu ms\r\n", (unsigned long)readCount, (unsigned long)(rateElapsed / (CPU_FREQUENCY_HZ / 1000u)));
            UART_1_UartPutString(rateOutput);

            readCount = 0u;
            rateElapsed = 0u;
        }





        if (outputElapsed < OUTPUT_PERIOD_CYCLES)
        {
            if (elapsed >= (OUTPUT_PERIOD_CYCLES - outputElapsed))
            {
                outputElapsed = OUTPUT_PERIOD_CYCLES;
            }
            else
            {
                outputElapsed += elapsed;
            }
        }

        /* Read and clear the flag together to avoid losing an interrupt between them. */
        interruptState = CyEnterCriticalSection();
        ready = dataReady;
        dataReady = 0u;
        CyExitCriticalSection(interruptState);

        if (ready == 0u)
        {
            continue;
        }

        if (QMC5883L_ReadData(&measurement) == 0u)
        {
            UART_1_UartPutString("[ERROR] Failed to read sensor data\r\n");
            continue;
        }
        
        readCount++;
        IIR_FilterUpdate(&filter, &measurement, FILTER_DIVISOR);

        IIR_FilterUpdate(&filter, &measurement, FILTER_DIVISOR);

        if (outputElapsed >= OUTPUT_PERIOD_CYCLES)
        {
            outputElapsed = 0u;
            PrintComparison("DATA", &filter);
        }
    }
}
/* [] END OF FILE */