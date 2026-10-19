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
 * @file qmc5883l.h
 * @brief Raw sensor data and driver interface.
 */

#ifndef QMC5883L_H
#define QMC5883L_H

#include "cytypes.h"

/* Raw signed sensor measurements. */
typedef struct
{
    int16 x;
    int16 y;
    int16 z;
} QMC5883L_Data;

/* All driver functions return 1 on success and 0 on failure. */
uint8 QMC5883L_WriteRegister(uint8 registerAddress, uint8 value);
uint8 QMC5883L_ReadRegister(uint8 registerAddress, uint8 *value);
uint8 QMC5883L_ReadData(QMC5883L_Data *data);
uint8 QMC5883L_Initialize(void);

#endif /* QMC5883L_H */
/* [] END OF FILE */