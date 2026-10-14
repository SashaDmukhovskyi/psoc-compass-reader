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
 * @file compass_tests.c
 * @brief Heading accuracy tests, timing measurements and comparison output.
 */

#include "project.h"
#include "compass_tests.h"
#include "benchmark.h"
#include <stdio.h>
#include <math.h>

#define PI                 (3.14159265358979323846)
#define RADIANS_TO_DEGREES (180.0 / PI)
#define FIXED_SCALE        (65536.0)


/** @brief Measures three algorithms and prints results outside the timed interval. */
void PrintComparison(const char *label, const IIR_Filter *filter)
{
    uint8 interruptState;
    uint32 start;
    uint32 end;
    uint32 doubleCycles;
    uint32 fixedCycles;
    uint32 libraryCycles;
    uint32 overhead;
    int32 fixedRadians;
    int32 libraryRadians;
    double doubleRadians;
    double doubleDegrees;
    double fixedDegrees;
    double libraryDegrees;
    double fixedError;
    double libraryError;
    uint32 doubleCentiDeg;
    uint32 fixedCentiDeg;
    uint32 libraryCentiDeg;
    uint32 fixedErrorMilliDeg;
    uint32 libraryErrorMilliDeg;
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

    start = Benchmark_ReadCounter();
    libraryRadians = Compass_CalculateHeadingLibrary(filter);
    end = Benchmark_ReadCounter();
    libraryCycles = Benchmark_ElapsedCycles(start, end);

    CyExitCriticalSection(interruptState);

    doubleDegrees = doubleRadians * RADIANS_TO_DEGREES;
    fixedDegrees = (double)fixedRadians * RADIANS_TO_DEGREES / FIXED_SCALE;
    libraryDegrees = (double)libraryRadians * RADIANS_TO_DEGREES / FIXED_SCALE;

    fixedError = fabs(doubleDegrees - fixedDegrees);
    libraryError = fabs(doubleDegrees - libraryDegrees);

    if (fixedError > 180.0)
    {
        fixedError = 360.0 - fixedError;
    }

    if (libraryError > 180.0)
    {
        libraryError = 360.0 - libraryError;
    }

    doubleCentiDeg = (uint32)(doubleDegrees * 100.0);
    fixedCentiDeg = (uint32)(fixedDegrees * 100.0);
    libraryCentiDeg = (uint32)(libraryDegrees * 100.0);
    fixedErrorMilliDeg = (uint32)(fixedError * 1000.0 + 0.5);
    libraryErrorMilliDeg = (uint32)(libraryError * 1000.0 + 0.5);

    (void)snprintf(output, sizeof(output), "[%s] X:%ld Y:%ld Z:%ld | Double:%lu.%02lu deg | Series:%lu.%02lu deg | Library:%lu.%02lu deg\r\n", label, (long)filter->x, (long)filter->y, (long)filter->z, (unsigned long)(doubleCentiDeg / 100u), (unsigned long)(doubleCentiDeg % 100u), (unsigned long)(fixedCentiDeg / 100u), (unsigned long)(fixedCentiDeg % 100u), (unsigned long)(libraryCentiDeg / 100u), (unsigned long)(libraryCentiDeg % 100u));
    UART_1_UartPutString(output);

    (void)snprintf(output, sizeof(output), "[ERROR-DEG] Series:%lu.%03lu | Library:%lu.%03lu\r\n", (unsigned long)(fixedErrorMilliDeg / 1000u), (unsigned long)(fixedErrorMilliDeg % 1000u), (unsigned long)(libraryErrorMilliDeg / 1000u), (unsigned long)(libraryErrorMilliDeg % 1000u));
    UART_1_UartPutString(output);

    (void)snprintf(output, sizeof(output), "[TIME] Double:%lu | Series:%lu | Library:%lu | Overhead:%lu cycles\r\n", (unsigned long)doubleCycles, (unsigned long)fixedCycles, (unsigned long)libraryCycles, (unsigned long)overhead);
    UART_1_UartPutString(output);
}


/** @brief Checks predefined directions and repeats one timing case. */
void RunStartupTests(void)
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

    (void)snprintf(output, sizeof(output), "[TEST] Zero vector: Series=%ld Library=%ld; direction undefined\r\n", (long)Compass_CalculateHeadingFixed(&input), (long)Compass_CalculateHeadingLibrary(&input));
    UART_1_UartPutString(output);

    input.x = 1000;
    input.y = 414;

    UART_1_UartPutString("[INFO] Timing test: 16 repetitions, X=1000 Y=414\r\n");

    for (index = 0u; index < 16u; index++)
    {
        PrintComparison("BENCH", &input);
    }
}

/* Measures a batch, including loop and volatile memory access overhead. */
#define MEASURE_ARITHMETIC(label, expression)                         \
    do                                                               \
    {                                                                \
        interruptState = CyEnterCriticalSection();                   \
        start = Benchmark_ReadCounter();                             \
        for (index = 0u; index < repetitions; index++)                \
        {                                                            \
            expression;                                              \
        }                                                            \
        end = Benchmark_ReadCounter();                               \
        CyExitCriticalSection(interruptState);                       \
        cycles = Benchmark_ElapsedCycles(start, end);                \
        (void)snprintf(output, sizeof(output),                        \
            "[ARITH] %s: %lu cycles / %lu iterations\r\n",            \
            label, (unsigned long)cycles,                            \
            (unsigned long)repetitions);                             \
        UART_1_UartPutString(output);                                 \
    } while (0)


/** @brief Compares batches of unsigned 32-bit and 64-bit arithmetic. */
void RunArithmeticTests(void)
{
    const uint32 repetitions = 1000u;
    uint8 interruptState;
    uint32 start;
    uint32 end;
    uint32 cycles;
    uint32 index;
    char output[100];

    volatile uint32_t a32 = 123456u;
    volatile uint32_t b32 = 37u;
    volatile uint32_t result32 = 0u;

    volatile uint64_t a64 = 5000000000ULL;
    volatile uint64_t b64 = 37ULL;
    volatile uint64_t result64 = 0ULL;

    UART_1_UartPutString("[INFO] Arithmetic tests: loop and memory overhead included\r\n");

    MEASURE_ARITHMETIC("ADD32", result32 = a32 + b32);
    MEASURE_ARITHMETIC("ADD64", result64 = a64 + b64);

    MEASURE_ARITHMETIC("SUB32", result32 = a32 - b32);
    MEASURE_ARITHMETIC("SUB64", result64 = a64 - b64);

    MEASURE_ARITHMETIC("MUL32", result32 = a32 * b32);
    MEASURE_ARITHMETIC("MUL64", result64 = a64 * b64);

    (void)result32;
    (void)result64;
}

#undef MEASURE_ARITHMETIC
/* [] END OF FILE */
