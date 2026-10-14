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
 * @file compass.c
 * @brief Integer IIR filtering and double/fixed-point heading calculations.
 */

#include "compass.h"
#include "fix16.h"
#include <math.h>

#define PI                (3.14159265358979323846)

#define FIXED_ONE         (65536L)
#define FIXED_PI          (205887L)
#define FIXED_PI_OVER_2   (102944L)
#define FIXED_PI_OVER_4   (51472L)
#define FIXED_TWO_PI      (411775L)
#define FIXED_ATAN_LIMIT  (27146L)
#define ATAN_SERIES_TERMS (12u)


/** @brief Multiplies fixed-point values using a 64-bit intermediate. */
static int32 Fixed_Multiply(int32 a, int32 b)
{
    return (int32)(((int64)a * (int64)b) / FIXED_ONE);
}


/** @brief Evaluates the atan series for a reduced argument, |t| <= 0.4143. */
static int32 Fixed_AtanSeries(int32 t)
{
    int32 square = Fixed_Multiply(t, t);
    int32 power = t;
    int32 result = t;
    uint8 index;

    /* t - t^3/3 + t^5/5 - t^7/7 + ... */
    for (index = 1u; index < ATAN_SERIES_TERMS; index++)
    {
        power = -Fixed_Multiply(power, square);
        result += power / (int32)(2u * index + 1u);
    }

    return result;
}


/** @brief Calculates atan(t) for a fixed-point argument from 0 to 1. */
static int32 Fixed_AtanUnit(int32 t)
{
    int32 reduced;

    if (t <= FIXED_ATAN_LIMIT)
    {
        return Fixed_AtanSeries(t);
    }

    /* atan(t) = pi/4 + atan((t - 1) / (t + 1)). */
    reduced = (int32)(((int64)(t - FIXED_ONE) * FIXED_ONE) / (t + FIXED_ONE));

    return FIXED_PI_OVER_4 + Fixed_AtanSeries(reduced);
}


/** @brief Updates all three filtered axes; a zero divisor leaves the state unchanged. */
void IIR_FilterUpdate(IIR_Filter *filter, const QMC5883L_Data *measurement, uint16 filterDivisor)
{
    if (filterDivisor == 0u)
    {
        return;
    }

    if (filter->initialized == 0u)
    {
        filter->x = measurement->x;
        filter->y = measurement->y;
        filter->z = measurement->z;
        filter->initialized = 1u;
        return;
    }

    filter->x += ((int32)measurement->x - filter->x) / (int32)filterDivisor;
    filter->y += ((int32)measurement->y - filter->y) / (int32)filterDivisor;
    filter->z += ((int32)measurement->z - filter->z) / (int32)filterDivisor;
}


/** @brief Returns the double baseline angle in radians from 0 to 2*pi. */
double Compass_CalculateHeadingDoubleRadians(const IIR_Filter *filter)
{
    double angle = atan2((double)filter->y, (double)filter->x);

    if (angle < 0.0)
    {
        angle += 2.0 * PI;
    }

    return angle;
}


/** @brief Returns the angle in fixed-point radians; zero vector returns an arbitrary 0. */
int32 Compass_CalculateHeadingFixed(const IIR_Filter *filter)
{
    int64 x = filter->x;
    int64 y = filter->y;
    int64 absX = (x < 0) ? -x : x;
    int64 absY = (y < 0) ? -y : y;
    int32 ratio;
    int32 angle;

    if ((absX == 0) && (absY == 0))
    {
        return 0;
    }

    /* Divide the smaller magnitude by the larger to keep the ratio within [0, 1]. */
    if (absX >= absY)
    {
        ratio = (int32)((absY * FIXED_ONE) / absX);
        angle = Fixed_AtanUnit(ratio);
    }
    else
    {
        ratio = (int32)((absX * FIXED_ONE) / absY);
        angle = FIXED_PI_OVER_2 - Fixed_AtanUnit(ratio);
    }

    if (x < 0)
    {
        angle = FIXED_PI - angle;
    }

    if (y < 0)
    {
        angle = FIXED_TWO_PI - angle;
    }

    if (angle >= FIXED_TWO_PI)
    {
        angle -= FIXED_TWO_PI;
    }

    return angle;
}

/** @brief Returns the libfixmath angle in Q16.16 radians; zero vector returns 0. */
int32 Compass_CalculateHeadingLibrary(const IIR_Filter *filter)
{
    fix16_t angle;

    if ((filter->x == 0) && (filter->y == 0))
    {
        return 0;
    }

    angle = fix16_atan2((fix16_t)filter->y, (fix16_t)filter->x);

    if (angle < 0)
    {
        angle += 2 * fix16_pi;
    }

    return (int32)angle;
}
/* [] END OF FILE */
