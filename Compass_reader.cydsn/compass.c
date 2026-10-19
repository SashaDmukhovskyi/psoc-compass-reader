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
 * @brief IIR filter and double/integer heading calculations.
 */

#include "compass.h"
#include <math.h>

#define PI                (3.14159265358979323846)

/* Q16.16 constants: real value multiplied by 65536. */
#define FIXED_ONE         (65536L)
#define FIXED_PI          (205887L)
#define FIXED_PI_OVER_2   (102944L)
#define FIXED_PI_OVER_4   (51472L)
#define FIXED_TWO_PI      (411775L)

/* Approximately tan(pi/8) in Q16.16. */
#define FIXED_ATAN_LIMIT  (27146L)
#define ATAN_SERIES_TERMS (12u)


/* Q16.16 multiplication with a 64-bit intermediate product. */
static int32 Fixed_Multiply(int32 a, int32 b)
{
    return (int32)(((int64)a * (int64)b) / FIXED_ONE);
}


/* atan(t) = t - t^3/3 + t^5/5 - ... for |t| <= approximately 0.4143. */
static int32 Fixed_AtanSeries(int32 t)
{
    int32 square = Fixed_Multiply(t, t);
    int32 power = t;
    int32 result = t;
    uint8 index;

    for (index = 1u; index < ATAN_SERIES_TERMS; index++)
    {
        power = -Fixed_Multiply(power, square);
        result += power / (int32)(2u * index + 1u);
    }

    return result;
}


/* Calculates atan(t) for a Q16.16 argument in [0, 1]. */
static int32 Fixed_AtanUnit(int32 t)
{
    int32 reduced;

    if (t <= FIXED_ATAN_LIMIT)
    {
        return Fixed_AtanSeries(t);
    }

    /* Reduce the argument:
     * atan(t) = pi/4 + atan((t - 1) / (t + 1)).
     */
    reduced = (int32)(
        ((int64)(t - FIXED_ONE) * FIXED_ONE) /
        (t + FIXED_ONE));

    return FIXED_PI_OVER_4 + Fixed_AtanSeries(reduced);
}


/* Updates the integer filter state using one sensor measurement. */
void IIR_FilterUpdate(
    IIR_Filter *filter,
    const QMC5883L_Data *measurement,
    uint16 filterDivisor)
{
    if (filterDivisor == 0u)
    {
        return;
    }

    /* Initialize directly from the first measurement. */
    if (filter->initialized == 0u)
    {
        filter->x = measurement->x;
        filter->y = measurement->y;
        filter->z = measurement->z;
        filter->initialized = 1u;
        return;
    }

    /* Integer division truncates small differences toward zero. */
    filter->x +=
        ((int32)measurement->x - filter->x) / (int32)filterDivisor;

    filter->y +=
        ((int32)measurement->y - filter->y) / (int32)filterDivisor;

    filter->z +=
        ((int32)measurement->z - filter->z) / (int32)filterDivisor;
}


/* Double-precision reference angle, normalized to [0, 2*pi). */
double Compass_CalculateHeadingDoubleRadians(const IIR_Filter *filter)
{
    double angle = atan2((double)filter->y, (double)filter->x);

    if (angle < 0.0)
    {
        angle += 2.0 * PI;
    }

    return angle;
}


/* Integer heading calculation; returns Q16.16 radians. */
int32 Compass_CalculateHeadingFixed(const IIR_Filter *filter)
{
    /* Promote before negation to safely handle negative int32 values. */
    int64 x = filter->x;
    int64 y = filter->y;
    int64 absX = (x < 0) ? -x : x;
    int64 absY = (y < 0) ? -y : y;

    int32 ratio;
    int32 angle;

    /* The zero vector has no direction; return 0 by convention. */
    if ((absX == 0) && (absY == 0))
    {
        return 0;
    }

    /* Keep the ratio in [0, 1] for the arctangent calculation. */
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

    /* Restore the quadrant using the original coordinate signs. */
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
/* [] END OF FILE */
