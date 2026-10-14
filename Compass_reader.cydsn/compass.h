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
#ifndef COMPASS_H
#define COMPASS_H

#include "qmc5883l.h"

typedef struct
{
    int32 x;
    int32 y;
    int32 z;
    uint8 initialized;
} IIR_Filter;


void IIR_FilterUpdate(
    IIR_Filter *filter,
    const QMC5883L_Data *measurement,
    uint16 filterDivisor);

uint16 QMC5883L_CalculateHeading(const IIR_Filter *filter);
    
#endif
/* [] END OF FILE */
