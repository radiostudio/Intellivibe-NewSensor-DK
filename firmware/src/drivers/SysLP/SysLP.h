/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for system low-power
GPIO management. Handles configuring all GPIOs to low-power state before
sleep and restoring them on wake.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef SYSLP_H
#define SYSLP_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>

//******************************************************************************
// DEFINES
//******************************************************************************

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int  SysLP_Init(void);
void SysLP_EnterSleep(void);
void SysLP_ExitSleep(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSLP_H */
