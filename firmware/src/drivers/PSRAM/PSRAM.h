/*******************************************************************************
********************************************************************************
Description:
This file defines the macros, SPI commands, and function prototypes for
the APS6404L-SQH-ZR 64Mbit SPI PSRAM driver. Used as external memory
for high-speed accelerometer streaming data buffering.

Part: APMemory APS6404L-SQH-ZR (8 MB SPI PSRAM)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef PSRAM_H
#define PSRAM_H

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

/* SPI commands */
#define PSRAM_CMD_READ              0x03
#define PSRAM_CMD_WRITE             0x02
#define PSRAM_CMD_READ_ID           0x9F
#define PSRAM_CMD_RESET_ENABLE      0x66
#define PSRAM_CMD_RESET             0x99

/* Device identification */
#define PSRAM_KNOWN_ID_BYTE3        0x0D
#define PSRAM_KNOWN_ID_BYTE4        0x5D

/* Power gating */
#define PSRAM_VDD_ON                1
#define PSRAM_VDD_OFF               0

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int  PSRAM_Init(void);
void PSRAM_Write(uint32_t Address, uint8_t *Data, uint16_t Len);
void PSRAM_Read(uint32_t Address, uint8_t *Buffer, uint16_t Len);
int  PSRAM_PowerOn(void);
int  PSRAM_PowerOff(void);

#ifdef __cplusplus
}
#endif

#endif /* PSRAM_H */
