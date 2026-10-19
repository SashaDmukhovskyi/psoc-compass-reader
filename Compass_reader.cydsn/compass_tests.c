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
 * @brief Heading comparison and 32/64-bit arithmetic tests.
 */

#include "project.h"
#include "compass_tests.h"
#include "benchmark.h"
#include <stdio.h>
#include <stdint.h>

#define PI               (3.14159265358979323846)
#define FIXED_PI         (205887LL)
#define HALF_CIRCLE_MDEG (180000L)
#define FULL_CIRCLE_MDEG (360000L)


/* Q16.16 radians -> millidegrees using integer arithmetic. */
static int32 FixedRadiansToMilliDegrees(int32 radians)
{
    int64 numerator = (int64)radians * HALF_CIRCLE_MDEG;

    return (int32)((numerator + FIXED_PI / 2LL) / FIXED_PI)
           % FULL_CIRCLE_MDEG;
}


/* Measures two algorithms; conversion and output are not timed. */
void PrintComparison(const char *label, const IIR_Filter *filter)
{
    uint8 interruptState;
    uint32 start;
    uint32 end;
    uint32 doubleCycles;
    uint32 fixedCycles;
    uint32 overhead;

    double doubleRadians;
    int32 fixedRadians;
    int32 doubleMilliDeg;
    int32 fixedMilliDeg;
    int32 errorMilliDeg;

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

    /* Convert the double baseline once, then work with integers. */
    doubleMilliDeg =
        (int32)(doubleRadians * (180000.0 / PI) + 0.5);
    doubleMilliDeg %= FULL_CIRCLE_MDEG;

    /* The series result remains integer throughout. */
    fixedMilliDeg = FixedRadiansToMilliDegrees(fixedRadians);

    errorMilliDeg = doubleMilliDeg - fixedMilliDeg;

    if (errorMilliDeg < 0)
    {
        errorMilliDeg = -errorMilliDeg;
    }

    if (errorMilliDeg > HALF_CIRCLE_MDEG)
    {
        errorMilliDeg = FULL_CIRCLE_MDEG - errorMilliDeg;
    }

    (void)snprintf(output, sizeof(output),
        "[%s] X:%ld Y:%ld Z:%ld"
        " | Double:%ld.%03ld deg"
        " | Series:%ld.%03ld deg\r\n",
        label,
        (long)filter->x,
        (long)filter->y,
        (long)filter->z,
        (long)(doubleMilliDeg / 1000L),
        (long)(doubleMilliDeg % 1000L),
        (long)(fixedMilliDeg / 1000L),
        (long)(fixedMilliDeg % 1000L));

    UART_1_UartPutString(output);

    (void)snprintf(output, sizeof(output),
        "[ERROR-DEG] Series:%ld.%03ld\r\n",
        (long)(errorMilliDeg / 1000L),
        (long)(errorMilliDeg % 1000L));

    UART_1_UartPutString(output);

    (void)snprintf(output, sizeof(output),
        "[TIME] Double:%lu | Series:%lu | Overhead:%lu cycles\r\n",
        (unsigned long)doubleCycles,
        (unsigned long)fixedCycles,
        (unsigned long)overhead);

    UART_1_UartPutString(output);
}


/* Checks predefined directions and repeats one timing case. */
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

    for (index = 0u;
         index < sizeof(cases) / sizeof(cases[0]);
         index++)
    {
        input.x = cases[index][0];
        input.y = cases[index][1];
        PrintComparison("TEST", &input);
    }

    input.x = 0;
    input.y = 0;

    (void)snprintf(output, sizeof(output),
        "[TEST] Zero vector: Series=%ld; direction undefined\r\n",
        (long)Compass_CalculateHeadingFixed(&input));

    UART_1_UartPutString(output);

    input.x = 1000;
    input.y = 414;

    UART_1_UartPutString(
        "[INFO] Timing test: 16 repetitions, X=1000 Y=414\r\n");

    for (index = 0u; index < 16u; index++)
    {
        PrintComparison("BENCH", &input);
    }
}


/* Batch timings include loop and volatile memory access overhead. */
#define MEASURE_ARITHMETIC(label, expression)                       \
    do                                                             \
    {                                                              \
        interruptState = CyEnterCriticalSection();                 \
        start = Benchmark_ReadCounter();                           \
        for (index = 0u; index < repetitions; index++)              \
        {                                                          \
            expression;                                            \
        }                                                          \
        end = Benchmark_ReadCounter();                             \
        CyExitCriticalSection(interruptState);                     \
        cycles = Benchmark_ElapsedCycles(start, end);              \
        (void)snprintf(output, sizeof(output),                      \
            "[ARITH] %s: %lu cycles / %lu iterations\r\n",          \
            label, (unsigned long)cycles,                          \
            (unsigned long)repetitions);                           \
        UART_1_UartPutString(output);                              \
    } while (0)


/* Compares unsigned 32-bit and 64-bit arithmetic. */
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

    UART_1_UartPutString(
        "[INFO] Arithmetic tests: loop and memory overhead included\r\n");

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
