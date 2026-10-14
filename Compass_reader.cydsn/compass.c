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
/* [] END OF FILE */
