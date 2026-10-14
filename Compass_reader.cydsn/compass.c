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
#include "compass.h"
#include <math.h>

#define PI                 (3.14159265358979323846)
#define RADIANS_TO_DEGREES (180.0 / PI)

#define FIXED_ONE (65536L)

#define FIXED_PI_OVER_4   (51472L)
#define FIXED_ATAN_LIMIT  (27146L)

#define FIXED_PI          (205887L)
#define FIXED_PI_OVER_2   (102944L)
#define FIXED_TWO_PI      (411775L)

/** @brief Multiplies two fixed-point values with 16 fractional bits. */
static int32 Fixed_Multiply(int32 a, int32 b)
{
    return (int32)(((int64)a * (int64)b) / FIXED_ONE);
}

/** @brief Approximates atan(t) in radians for |t| <= 0.4143, using fixed-point arithmetic. */
static int32 Fixed_AtanSeries(int32 t)
{
    int32 square = Fixed_Multiply(t, t);
    int32 power = t;
    int32 result = t;
    uint8 index;

    for (index = 1u; index < 12u; index++)
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

    reduced = (int32)(((int64)(t - FIXED_ONE) * FIXED_ONE) / (t + FIXED_ONE));

    return FIXED_PI_OVER_4 + Fixed_AtanSeries(reduced);
}

/** @brief Calculates the XY angle in fixed-point radians; returns 0 for a zero vector. */
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



/** @brief Updates the IIR filter using a new sensor measurement. **/
void IIR_FilterUpdate(IIR_Filter *filter, const QMC5883L_Data *measurement, uint16 filterDivisor)
{
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

/** @brief Calculates the compass heading from filtered X and Y values. **/

 
uint16 QMC5883L_CalculateHeading(
    const IIR_Filter *filter)
{
    double heading;

    heading = atan2(
        (double)filter->y,
        (double)filter->x
    );

    heading *= RADIANS_TO_DEGREES;

    if (heading < 0.0)
    {
        heading += 360.0;
    }

    return (uint16)heading;
}

/** @brief Returns the uncalibrated XY angle in double-precision degrees. */
double Compass_CalculateHeadingDouble(const IIR_Filter *filter)
{
    double angle = atan2((double)filter->y, (double)filter->x) * RADIANS_TO_DEGREES;

    if (angle < 0.0)
    {
        angle += 360.0;
    }

    return angle;
}

/** @brief Returns the XY angle in double-precision radians from 0 to 2*pi. */
double Compass_CalculateHeadingDoubleRadians(const IIR_Filter *filter)
{
    double angle = atan2((double)filter->y, (double)filter->x);

    if (angle < 0.0)
    {
        angle += 2.0 * PI;
    }

    return angle;
}
/* [] END OF FILE */
