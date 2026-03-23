/*******************************************************************************

********************************************************************************
Description:
This file defines the macros and function prototypes for Controller handling.
Includes deep sleep management, RTC functions, EEPROM event storage,
and battery monitoring.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Jan 2026
*******************************************************************************/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include "Common.h"

//******************************************************************************
// DEFINES
//******************************************************************************
#define TIMEDATA_ROWS   3
#define TIMEDATA_COLS   2
#define CELL_STR_MAX    20

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// EXTERNS
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************

bool Controller_Init(void);
bool Controller_LoadConfigurationFromEEPROM(void);
void Controller_LoadDefaultConfiguration(void);
char (*Controller_GetTimeData(void))[TIMEDATA_COLS][CELL_STR_MAX];
char *Controller_GetTimeDataBuffer(uint8_t row, uint8_t col);
void Controller_EnterDeepSleep(void);
void Controller_ExitDeepSleep(void);
bool Controller_CheckIfTimeValid(void);
bool Controller_GetRTCString(char *buffer, size_t bufferSize);
HAL_StatusTypeDef Controller_UpdateRTC(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void Controller_SetDailyAlarm(uint8_t hours, uint8_t minutes, uint8_t seconds);
void Controller_GetAlarmTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
void Controller_PushBatteryEvent(const char *imei, const char *uid);
uint8_t Controller_ReadBatterySOC(void);
uint8_t Controller_GetBatterySOC(void);
void Controller_BuzzerBeep(uint16_t durationMs);
void Controller_BuzzerOn(void);
void Controller_BuzzerOff(void);
void Controller_LEDFlash(uint8_t ledNum, uint16_t durationMs);
bool Controller_IsDataBackedUp(void);
uint16_t Controller_GetEEPROMEventCount(void);
bool Controller_LoadQueueFromEEPROM(const char *imei, const char *uid);
bool Controller_SaveQueueToEEPROM(void);
bool Controller_SaveEventToEEPROM(EC200U_EventData_t *eventData);
bool Controller_SaveTimeDataINEEPROM(void);
bool Controller_SetConfigField(const char *field, const char *value);
bool Controller_GetConfigField(const char *field, char *buf, size_t bufSize);
void Controller_ClearEventData(void);
void Controller_ClearEventTime(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H
