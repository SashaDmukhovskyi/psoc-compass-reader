/* ========================================
 *
 * Compass reader using QMC5883L
 *
 * ========================================
 */
/**
 * @file main.c
 * @brief Sensor initialization, DRDY handling and measurement processing.
 */

#include "project.h"
#include "qmc5883l.h"
#include "compass.h"
#include "benchmark.h"
#include "compass_tests.h"
#include <stdio.h>

/* Application settings. Changes require rebuilding and programming. */
#define FILTER_DIVISOR       (8u)
#define OUTPUT_PERIOD_MS     (1000u)
#define RUN_STARTUP_TESTS     (1u)

/* Sensor identification and configuration readback. */
#define CHIP_ID_REGISTER     (0x0Du)
#define EXPECTED_CHIP_ID     (0xFFu)
#define CONTROL_1_REGISTER   (0x09u)
#define EXPECTED_CONFIG      (0x0Du)

/* SysTick uses the CPU clock. */
#define CPU_FREQUENCY_HZ     (CYDEV_BCLK__SYSCLK__HZ)
#define CPU_CYCLES_PER_MS    (CPU_FREQUENCY_HZ / 1000u)
#define OUTPUT_PERIOD_CYCLES (CPU_CYCLES_PER_MS * OUTPUT_PERIOD_MS)

#if FILTER_DIVISOR < 1u || FILTER_DIVISOR > 65535u
#error FILTER_DIVISOR must be between 1 and 65535.
#endif

#if OUTPUT_PERIOD_MS < 1u || OUTPUT_PERIOD_MS > 1000u
#error OUTPUT_PERIOD_MS must be between 1 and 1000.
#endif

/* Shared between the interrupt handler and main loop. */
static volatile uint8 dataReady = 0u;


/* Signal readiness; sensor reading is performed in the main loop. */
CY_ISR(DRDY_handler)
{
    (void)DRDY_ClearInterrupt();
    dataReady = 1u;
}


int main(void)
{
    uint8 chipId;
    uint8 configuration;
    uint8 interruptState;
    uint8 ready;

    uint32 previousCounter;
    uint32 currentCounter;
    uint32 elapsed;
    uint32 outputElapsed = 0u;
    uint32 rateElapsed = 0u;
    uint32 readCount = 0u;

    QMC5883L_Data measurement;
    IIR_Filter filter = {0};

    char rateOutput[100];

    UART_1_Start();
    I2C_1_Start();
    Benchmark_Initialize();

    CyGlobalIntEnable;

    UART_1_UartPutString("[INFO] Compass reader started\r\n");

    /* Run numerical and timing tests before configuring the sensor. */
    if (RUN_STARTUP_TESTS != 0u)
    {
        RunStartupTests();
        RunArithmeticTests();
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

    UART_1_UartPutString(
        "[INFO] QMC5883L-compatible chip ID: 0xFF\r\n");

    /* Register the handler before enabling sensor measurements. */
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

    /* Report configuration verification; processing continues on mismatch. */
    if (QMC5883L_ReadRegister(
            CONTROL_1_REGISTER, &configuration) != 0u)
    {
        if (configuration == EXPECTED_CONFIG)
        {
            UART_1_UartPutString(
                "[INFO] Configuration confirmed: 200 Hz\r\n");
        }
        else
        {
            UART_1_UartPutString(
                "[ERROR] Unexpected sensor configuration\r\n");
        }
    }
    else
    {
        UART_1_UartPutString(
            "[ERROR] Failed to verify sensor configuration\r\n");
    }

    previousCounter = Benchmark_ReadCounter();

    for (;;)
    {
        /* Poll within one full SysTick period to track elapsed cycles. */
        currentCounter = Benchmark_ReadCounter();
        elapsed = Benchmark_ElapsedCycles(
            previousCounter, currentCounter);
        previousCounter = currentCounter;

        /* Count successful reads over approximately one second. */
        rateElapsed += elapsed;

        if (rateElapsed >= CPU_FREQUENCY_HZ)
        {
            (void)snprintf(rateOutput, sizeof(rateOutput),
                "[RATE] Reads:%lu | Window:%lu ms\r\n",
                (unsigned long)readCount,
                (unsigned long)(rateElapsed / CPU_CYCLES_PER_MS));

            UART_1_UartPutString(rateOutput);

            readCount = 0u;
            rateElapsed = 0u;
        }

        /* Saturate the output timer until a measurement is available. */
        if (outputElapsed < OUTPUT_PERIOD_CYCLES)
        {
            if (elapsed >= OUTPUT_PERIOD_CYCLES - outputElapsed)
            {
                outputElapsed = OUTPUT_PERIOD_CYCLES;
            }
            else
            {
                outputElapsed += elapsed;
            }
        }

        /* Take and clear the flag atomically.
         * Multiple DRDY events can merge into one pending flag.
         */
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
            UART_1_UartPutString(
                "[ERROR] Failed to read sensor data\r\n");
            continue;
        }

        readCount++;
        IIR_FilterUpdate(&filter, &measurement, FILTER_DIVISOR);

        /* Output frequency is independent of the sensor configuration. */
        if (outputElapsed >= OUTPUT_PERIOD_CYCLES)
        {
            outputElapsed = 0u;
            PrintComparison("DATA", &filter);
        }
    }
}
/* [] END OF FILE */