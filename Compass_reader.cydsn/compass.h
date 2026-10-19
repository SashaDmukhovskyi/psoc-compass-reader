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
 * @file compass.h
 * @brief Filter state and heading calculation interface.
 */

#ifndef COMPASS_H
#define COMPASS_H

#include "qmc5883l.h"

/* Filtered sensor coordinates, stored as ordinary signed integers. */
typedef struct
{
    int32 x;
    int32 y;
    int32 z;
    uint8 initialized;
} IIR_Filter;

/* A zero divisor leaves the filter unchanged. */
void IIR_FilterUpdate(
    IIR_Filter *filter,
    const QMC5883L_Data *measurement,
    uint16 filterDivisor);

/* Reference calculation using double atan2(); returns radians. */
double Compass_CalculateHeadingDoubleRadians(
    const IIR_Filter *filter);

/* Integer series calculation; returns radians scaled by 65536. */
int32 Compass_CalculateHeadingFixed(
    const IIR_Filter *filter);

#endif /* COMPASS_H */
/* [] END OF FILE */
