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
#ifndef BENCHMARK_H
#define BENCHMARK_H

#include "cytypes.h"

void Benchmark_Initialize(void);
uint32 Benchmark_ReadCounter(void);
uint32 Benchmark_ElapsedCycles(uint32 start, uint32 end);

#endif
/* [] END OF FILE */
