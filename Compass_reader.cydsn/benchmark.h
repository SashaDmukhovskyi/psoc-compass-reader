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
 * @file benchmark.h
 * @brief CPU cycle measurement interface.
 */

#ifndef BENCHMARK_H
#define BENCHMARK_H

#include "cytypes.h"

void Benchmark_Initialize(void);

uint32 Benchmark_ReadCounter(void);

/* Interval must be shorter than 2^24 cycles:
 * approximately 349.5 ms at 48 MHz.
 */
uint32 Benchmark_ElapsedCycles(uint32 start, uint32 end);

#endif /* BENCHMARK_H */
/* [] END OF FILE */
