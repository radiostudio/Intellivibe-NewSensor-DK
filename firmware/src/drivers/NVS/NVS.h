/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for NVS (Non-Volatile
Storage) handling. Provides persistent storage for system settings using
the Zephyr NVS subsystem on internal flash.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef NVS_H
#define NVS_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>

//******************************************************************************
// DEFINES
//******************************************************************************
#define NVS_SETTINGS_ID     100
#define NVS_TEST_ID         1

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int NVS_Init(void);
int NVS_Read(uint16_t Id, void *Data, size_t Len);
int NVS_Write(uint16_t Id, const void *Data, size_t Len);

#ifdef __cplusplus
}
#endif

#endif /* NVS_H */
