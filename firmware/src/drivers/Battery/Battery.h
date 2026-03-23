/*******************************************************************************
********************************************************************************
Description:
This file defines the macros, register map, and function prototypes for
the LTC3335 coulomb counter driver. Communicates via I2C, reads accumulated
charge to calculate remaining battery percentage.

Part: Analog Devices LTC3335 (retained from old firmware)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef BATTERY_H
#define BATTERY_H

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
#define BATTERY_I2C_ADDR            0x64

/* Register addresses */
#define BATTERY_REG_PRESCALER       0x01
#define BATTERY_REG_ACC_CHARGE      0x03
#define BATTERY_REG_INTERRUPT       0x05

/* Prescaler: 0x0D for 1.2 Ah battery, 250 mA peak current */
#define BATTERY_PRESCALER_VALUE     0x0D

/* Clear interrupt / reset counter */
#define BATTERY_CLEAR_INTERRUPT     0x01

/* Battery specs */
#define BATTERY_QLSB_UAH           7.031f
#define BATTERY_FULL_CAPACITY_MAH   1200.0f
#define BATTERY_FULL_CAPACITY_AH    1.2f

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int     Battery_Init(void);
int     Battery_Read(uint8_t *Percent);
int     Battery_ResetCounter(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
