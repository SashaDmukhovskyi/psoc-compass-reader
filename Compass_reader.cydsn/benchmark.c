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
#include "benchmark.h"

#define BENCHMARK_COUNTER_MASK (0x00FFFFFFu)

/** @brief Starts a free-running SysTick counter clocked by SYSCLK. */
void Benchmark_Initialize(void)
{
    CySysTickInit();
    CySysTickStop();
    CySysTickSetClockSource(CY_SYS_SYST_CSR_CLK_SRC_SYSCLK);
    CySysTickSetReload(BENCHMARK_COUNTER_MASK);
    CySysTickClear();
    CySysTickEnable();
    CySysTickDisableInterrupt();
}

/** @brief Returns the current down-counter value. */
uint32 Benchmark_ReadCounter(void)
{
    return CySysTickGetValue();
}

/** @brief Returns elapsed cycles for an interval shorter than one counter period. */
uint32 Benchmark_ElapsedCycles(uint32 start, uint32 end)
{
    return (start - end) & BENCHMARK_COUNTER_MASK;
}
/* [] END OF FILE */
