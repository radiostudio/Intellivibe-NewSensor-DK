/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for the TMP112
temperature sensor driver. Communicates via I2C, supports one-shot
conversion mode, and initialises in shutdown/low-power mode.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef TEMP_H
#define TEMP_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <zephyr/device.h>

//******************************************************************************
// DEFINES
//******************************************************************************
#define TEMP_I2C_ADDR               0x48

/* Register addresses */
#define TEMP_REG_TEMPERATURE        0x00
#define TEMP_REG_CONFIG             0x01
#define TEMP_REG_T_LOW              0x02
#define TEMP_REG_T_HIGH             0x03

/* Config register bits */
#define TEMP_CFG_SHUTDOWN           0x01
#define TEMP_CFG_THERMOSTAT         (1 << 1)
#define TEMP_CFG_POL_ACTIVE_HIGH    (1 << 2)
#define TEMP_CFG_EXTENDED_MODE      (1 << 4)
#define TEMP_CFG_ONESHOT            (1 << 7)

/* Conversion */
#define TEMP_CONV_FACTOR            0.0625f
#define TEMP_CONV_WAIT_MS           30

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int   Temp_Init(void);
int   Temp_Read(float *Temperature);
void  Temp_SetOffset(int8_t Offset);
int   Temp_GetScaledValue(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_H */
