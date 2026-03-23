/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for UserConfig handling.
Manages system configuration including factory defaults, NVS persistence,
magic number validation, and CRC integrity checking.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef USERCONFIG_H
#define USERCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include "CommonTypes.h"

//******************************************************************************
// DEFINES
//******************************************************************************

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int              UserConfig_Init(void);
SystemSettings_t *UserConfig_GetSettings(void);
int              UserConfig_SaveSettings(void);
void             UserConfig_LoadDefaults(void);
void             UserConfig_PrintSettings(void);
uint16_t         UserConfig_CalculateCrc(const uint8_t *Data, size_t Len);

#ifdef __cplusplus
}
#endif

#endif /* USERCONFIG_H */
