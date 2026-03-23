/*******************************************************************************
********************************************************************************
Description:
This file handles functions for Controller management.
Includes deep sleep, RTC management, EEPROM event storage (AT24C32D 4KB),
and battery monitoring via ADC.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Jan 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include "Controller.h"
#include "main.h"
#include "rtc.h"
#include "adc.h"
#include "EEPROM.h"
#include "tim.h"
#include "usart.h"
#include "tsc.h"
#include "spi.h"
#include "Display.h"
#include "UserConfig.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
// EEPROM Memory Layout for AT24C32D (4096 bytes, 128 pages x 32 bytes):
// Pages 0-1:    Configuration (64 bytes) — stores write/read pointers
// Pages 2-127:  Event data (4032 bytes)

#define EEPROM_TOTAL_SIZE               4096
#define EEPROM_PAGE_SIZE                32
#define EEPROM_TOTAL_PAGES              128

#define EEPROM_CONFIG_START             0x0000  // Pages 0-1 (64 bytes)
#define EEPROM_CONFIG_SIZE              256    // Config struct size (must fit in 256 bytes)

#define EEPROM_POINTERS_START           0x100     // 2 pages for pointers
#define EEPROM_EVENT_DATA_START         0x140  // Pages 11-123 
#define EEPROM_EVENT_DATA_END           0x0F7F

// Compact EEPROM event: only buttonId + rfid + dateTime (no imei/uid)
typedef struct __attribute__((packed)) {
    char dateTime[25];
    char rfid[32];
    uint8_t buttonId;
} EepromEvent_t;  // 58 bytes

#define EEPROM_EVENT_BLOCK_SIZE         60      // Must be >= sizeof(EepromEvent_t) (58 bytes)
#define EEPROM_MAX_EVENTS               ((EEPROM_EVENT_DATA_END - EEPROM_EVENT_DATA_START + 1) / EEPROM_EVENT_BLOCK_SIZE)

// Config page offsets for write/read pointers
#define EEPROM_CONFIG_MAGIC_VALUE       0xDEAD
#define EEPROM_POINTER_MAGIC_VALUE      0xDEAD

// Time data section — last 128 bytes of 4 KB EEPROM (0x0F80-0x0FFF)
// Layout: flat array of TIMEDATA_ROWS*TIMEDATA_COLS strings, each CELL_STR_MAX bytes
#define EEPROM_TIMEDATA_START           0x0F80
#define EEPROM_TIMEDATA_MAGIC_VALUE     0xCAFE
#define EEPROM_TIMEDATA_DATA_OFFSET     2
#define EEPROM_TIMEDATA_SIZE            (TIMEDATA_ROWS * TIMEDATA_COLS * (CELL_STR_MAX))

// Battery ADC: PA0 with 470K/470K voltage divider
// Vbatt = ADC_reading * Vref / 4096 * 2 (divider ratio)
// Full = 6.5V, Empty = 4.5V
#define BATT_FULL_MV                    6500
#define BATT_EMPTY_MV                   4500
#define BATT_ADC_VREF_MV                3300
/******************************************************************************
// Data types
******************************************************************************/
typedef struct __attribute__((packed))
{
  char ServerName[100];
  uint32_t ServerPort;
  char APName[15];
  char APDescription[15];
  uint8_t ShutDownBatLevel;
  uint32_t MagicNumber;
} ConfigParam_t;

typedef struct __attribute__((packed))
{
    uint16_t writePtr;
    uint16_t readPtr;
    bool TimeDataValid;
    uint8_t alarmHours;
    uint8_t alarmMinutes;
    uint8_t alarmSeconds;
    uint32_t MagicNumber;
} EventPtrConfig_t;
typedef struct __attribute__((packed))
{
    char EventTimeData[TIMEDATA_ROWS][TIMEDATA_COLS][CELL_STR_MAX];
    uint32_t MagicNumber;
} EventDataTime_t;
//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static ConfigParam_t m_SystemConfig;
static EventPtrConfig_t m_SystemEventPtr;
static EventDataTime_t m_SystemTimeData;
static uint8_t m_BatterySoc = 0;
//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
extern void SystemClock_Config(void);
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim1;

static uint8_t* Controller_GetConfig(void);
static void Controller_LoadPointersFromEEPROM(void);
static void Controller_SavePointersToEEPROM(void);
static void Controller_SaveConfigurationToEEPROM(void);
static void Controller_LoadTimeDataFromEEPROM(void);
static uint16_t Controller_NextPtr(uint16_t ptr);
static uint32_t Controller_ReadADC(void);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the Controller module. Must be called once at startup before any
other Controller function or display render.

Sequence:
  1. Load system configuration (ConfigParam_t, magic 0xBEEF) from EEPROM.
     If invalid, factory defaults are written (EC200U_SERVER_HOST/PORT).
  2. Load event circular-buffer pointers (EventPtrConfig_t, magic 0xDEAD)
     from EEPROM. If invalid or out-of-range, pointers are reset to 0.
  3. Load the TIMEDATA_ROWS x TIMEDATA_COLS display time-data strings from
     EEPROM starting at EEPROM_TIMEDATA_START (0x1000).
  4. Arm the daily RTC Alarm A (04:00:00 every day).

Event data is not loaded here — call Controller_LoadQueueFromEEPROM() later
when the GSM task is ready to transmit.

Argument(s):
None

Return:
bool - true if EEPROM configuration read succeeded (step 1);
       false if EEPROM_Read failed (remaining steps still execute).

Note(s):
The RTC alarm is armed unconditionally regardless of EEPROM validity.
*******************************************************************************/
bool Controller_Init(void)
{
    bool status = true;
    /* Step 1 — system configuration (server name/port) */
    status = Controller_LoadConfigurationFromEEPROM();
    LOG_INFO(CONTROLLER_LOG, "Config: Server=%s Port=%lu APN=%s Desc=%s",
             m_SystemConfig.ServerName,
             (unsigned long)m_SystemConfig.ServerPort,
             m_SystemConfig.APName,
             m_SystemConfig.APDescription);

    /* Step 2 — event circular-buffer write/read pointers */
    Controller_LoadPointersFromEEPROM();
    LOG_INFO(CONTROLLER_LOG, "Pointers: writePtr=%u readPtr=%u timeDataValid=%d alarm=%02d:%02d:%02d",
             m_SystemEventPtr.writePtr, m_SystemEventPtr.readPtr,
             m_SystemEventPtr.TimeDataValid,
             m_SystemEventPtr.alarmHours, m_SystemEventPtr.alarmMinutes,
             m_SystemEventPtr.alarmSeconds);

    /* Step 3 — display time-data strings */
    Controller_LoadTimeDataFromEEPROM();
    for (int r = 0; r < TIMEDATA_ROWS; r++)
    {
        LOG_INFO(CONTROLLER_LOG, "TimeData[%d]: \"%s\" | \"%s\"",
                 r,
                 m_SystemTimeData.EventTimeData[r][0],
                 m_SystemTimeData.EventTimeData[r][1]);
    }

    /* Step 4 — arm daily RTC alarm from stored EEPROM values */
    Controller_SetDailyAlarm(m_SystemEventPtr.alarmHours,
                             m_SystemEventPtr.alarmMinutes,
                             m_SystemEventPtr.alarmSeconds);

    return status;
}
/*******************************************************************************
Description:
Load system configuration from EEPROM (starting at EEPROM_CONFIG_START 0x0000).
Reads the ConfigParam_t structure and validates the MagicNumber field against
EEPROM_CONFIG_MAGIC_VALUE (0xBEEF).
If read OK + valid magic   : logs success.
If read OK + invalid magic : logs warning, calls Controller_LoadDefaultConfiguration()
                             to write factory defaults.
If EEPROM_Read fails       : returns false immediately.

Argument(s):
None

Return:
bool - true if EEPROM_Read succeeded (magic validation may still have failed);
       false if EEPROM_Read returned an error.

Note(s):
Called from Controller_Init() during startup.
*******************************************************************************/
bool Controller_LoadConfigurationFromEEPROM(void)
{
    bool status = false;
    if(EEPROM_Read(EEPROM_CONFIG_START , (uint8_t *)&m_SystemConfig, sizeof(m_SystemConfig)) == EEPROM_OK)
    {      
        status = true;  
        if (m_SystemConfig.MagicNumber == EEPROM_CONFIG_MAGIC_VALUE)
        {
            LOG_INFO(CONTROLLER_LOG, "EEPROM configuration valid");
        }
        else
        {
            LOG_WARN(CONTROLLER_LOG, "EEPROM configuration invalid-loading default values");
            Controller_LoadDefaultConfiguration();
        }
    }
    return status;
}
/*******************************************************************************
Description:
Populate m_SystemConfig with factory-default values and write them to EEPROM.
Sets ServerName to EC200U_SERVER_HOST, ServerPort to EC200U_SERVER_PORT, and
MagicNumber to EEPROM_CONFIG_MAGIC_VALUE (0xBEEF).
Calls Controller_SaveConfigurationToEEPROM() to persist the struct to EEPROM.

Argument(s):
None

Return:
None

Note(s):
Called when EEPROM config magic is invalid (first boot or corrupted data).
*******************************************************************************/
void Controller_LoadDefaultConfiguration(void)
{
    memset(&m_SystemConfig, 0, sizeof(m_SystemConfig));
    strcpy(m_SystemConfig.ServerName, EC200U_SERVER_HOST);
    m_SystemConfig.ServerPort = EC200U_SERVER_PORT;
    m_SystemConfig.ShutDownBatLevel = 20;
    m_SystemConfig.MagicNumber = EEPROM_CONFIG_MAGIC_VALUE;
    Controller_SaveConfigurationToEEPROM();
}

/*******************************************************************************
Description:
Return a pointer to the internal TIMEDATA_ROWS x TIMEDATA_COLS time-data
string array populated by Controller_Init(). The caller accesses strings as
buf[row][col] where each element is a char* into the internal backing buffer.
No string data is copied.

Argument(s):
None

Return:
char *(*)[TIMEDATA_COLS] - Pointer to the first row of m_EventTimeDataBuf.
                           Elements are NULL if no valid data was loaded.

Note(s):
Call Controller_Init() before calling this function.
The returned pointer is valid for the lifetime of the module.
*******************************************************************************/
char (*Controller_GetTimeData(void))[TIMEDATA_COLS][CELL_STR_MAX]
{
    return m_SystemTimeData.EventTimeData;
}

/*******************************************************************************
Description:
Return a pointer to a single cell string within the time-data buffer.
Provides direct access to m_EventTimeDataBuf[row][col] for reading or writing.

Argument(s):
row - Row index (0 .. TIMEDATA_ROWS-1)
col - Column index (0 .. TIMEDATA_COLS-1)

Return:
char* - Pointer to the null-terminated string (up to CELL_STR_MAX bytes);
        NULL if row or col is out of range.
*******************************************************************************/
char *Controller_GetTimeDataBuffer(uint8_t row, uint8_t col)
{
    if (row >= TIMEDATA_ROWS || col >= TIMEDATA_COLS) return NULL;
    return m_SystemTimeData.EventTimeData[row][col];
}

/*******************************************************************************
Description:
Enter deep sleep mode (Stop mode). Suspends the FreeRTOS scheduler and puts
the microcontroller in Stop mode with low power regulator. Execution resumes
after the WFI instruction when a wakeup event occurs (RTC Alarm, EXTI).

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_EnterDeepSleep(void)
{
    /* Check if battery is at or below shutdown level */
    bool shutdown = (m_SystemConfig.ShutDownBatLevel > 0) &&
                    (m_BatterySoc <= m_SystemConfig.ShutDownBatLevel);

    if (shutdown)
    {
        /* Wake display, show SYSTEM SHUTDOWN, then sleep display */
        Display_Wakeup();
        Display_Clear();

        const char *msg = "SYSTEM SHUTDOWN";
        UWORD textW = (UWORD)(strlen(msg) * Font24.Width);
        UWORD textH = Font24.Height;
        UWORD x = (EPD_3IN52_HEIGHT - textW) / 2;
        UWORD y = (EPD_3IN52_WIDTH - textH) / 2;
        Paint_DrawString_EN(x, y, msg, &Font24, BLACK, WHITE);

        Display_Update();
        Display_Sleep();

        /* Disable RTC Alarm so it cannot wake the MCU */
        HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
        HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);

        /* Disable proximity interrupt so touch cannot wake the MCU */
        HAL_NVIC_DisableIRQ(PROX_INT_EXTI_IRQn);
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* Set I2C1 (EEPROM) pins to analog mode */
    gpio.Pin = EEPROM_SCL1_Pin;
    HAL_GPIO_Init(EEPROM_SCL1_GPIO_Port, &gpio);

    gpio.Pin = EEPROM_SDA1_Pin;
    HAL_GPIO_Init(EEPROM_SDA1_GPIO_Port, &gpio);

    /* De-init USART2 (Debug) and set its pins to analog mode */
    HAL_UART_DeInit(&huart2);

    /* USART2 (Debug): PA2 TX, PA3 RX */
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* De-init TSC and set all touch pins to analog mode */
    HAL_TSC_DeInit(&htsc);

    /* TSC GPIOB: PB14 (guard sampling), PB15 (guard ring) */
    gpio.Pin = TOUCH_GUARD_SAMPLING_Pin | TOUCH_GUARD_RING_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* TSC GPIOC: PC6 (G4 sampling), PC7-PC9 (G4 channels), PC10-PC12 (G3 channels) */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9
             | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* TSC GPIOA: PA15 (G3 sampling) */
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* BATT_SWITCH (PB12) to analog */
    gpio.Pin = BATT_SWITCH_Pin;
    HAL_GPIO_Init(BATT_SWITCH_GPIO_Port, &gpio);

    /* BUZZER (PA11, TIM1_CH4) to analog */
    gpio.Pin = BUZZER_Pin;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &gpio);

    vTaskSuspendAll();
    HAL_SuspendTick();                  /* disable SysTick so it doesn't wake us */

    /* Disable peripheral interrupts that would spuriously wake from Stop mode.
       Only RTC_Alarm and PROX_INT (EXTI15_10) should be able to wake the MCU. */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_DisableIRQ(TSC_IRQn);

    /* Disable debug clocks in Stop mode so SWD doesn't wake us */
    DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/*******************************************************************************
Description:
Exit deep sleep mode. Reconfigures system clock (HSI/PLL lost during Stop mode)
and resumes the FreeRTOS scheduler.

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_ExitDeepSleep(void)
{
    SystemClock_Config();
    HAL_ResumeTick();                   /* re-enable SysTick */

    /* Re-enable peripheral interrupts disabled before Stop mode */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* Re-init USART2 (Debug) */
    MX_USART2_UART_Init();

    /* Re-init DMA channel, re-link to USART2, and restart DMA receive + IDLE */
    UserConfig_RearmDMA();

    /* Re-enable USART2 + DMA IRQs */
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* Disable PROX_INT EXTI before TSC pin reconfiguration.
       MX_TSC_Init() transitions PC6-PC12 from analog → AF, coupling noise
       onto adjacent PC13. With the IRQ enabled the EXTI ISR would fire and
       give a spurious TouchScanSemaphore token before we can clear it. */
    HAL_NVIC_DisableIRQ(PROX_INT_EXTI_IRQn);

    /* Re-init TSC (restores all touch pins to AF mode, enables clock and IRQ) */
    MX_TSC_Init();
    HAL_NVIC_EnableIRQ(TSC_IRQn);

    /* Reconfigure I2C1 (EEPROM) pins back to alternate function */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;

    gpio.Pin = EEPROM_SCL1_Pin;
    HAL_GPIO_Init(EEPROM_SCL1_GPIO_Port, &gpio);

    gpio.Pin = EEPROM_SDA1_Pin;
    HAL_GPIO_Init(EEPROM_SDA1_GPIO_Port, &gpio);

    /* Restore BATT_SWITCH (PB12) as output push-pull, default LOW */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;

    gpio.Pin = BATT_SWITCH_Pin;
    HAL_GPIO_Init(BATT_SWITCH_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(BATT_SWITCH_GPIO_Port, BATT_SWITCH_Pin, GPIO_PIN_SET);

    /* Restore BUZZER (PA11) as TIM1_CH4 AF push-pull */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF1_TIM1;

    gpio.Pin = BUZZER_Pin;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &gpio);

    /* Clear any latched EXTI edge from GPIO reconfiguration, then re-enable */
    __HAL_GPIO_EXTI_CLEAR_IT(PROX_INT_Pin);
    HAL_NVIC_ClearPendingIRQ(PROX_INT_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(PROX_INT_EXTI_IRQn);

    xTaskResumeAll();
}

/*******************************************************************************
Description:
Read current RTC into a formatted string "20YY/MM/DD,HH:MM:SS"

Arguments:
buffer      - Output buffer for the formatted string
bufferSize  - Size of the buffer (must be >= 20)

Return:
bool - true if successful
*******************************************************************************/
bool Controller_GetRTCString(char *buffer, size_t bufferSize)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if (buffer == NULL || bufferSize < 20)
    {
        return false;
    }

    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        return false;
    }

    // Must read date after time to unlock RTC shadow registers
    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        return false;
    }

    snprintf(buffer, bufferSize, "%02d/%02d/20%02d-%02d:%02d",
        sDate.Date, sDate.Month,sDate.Year,
             sTime.Hours, sTime.Minutes);

    return true;
}

/*******************************************************************************
Description:
Check whether a valid RTC time has been received from the network and stored.
Returns the TimeDataValid flag from the event pointer config struct.

Argument(s):
None

Return:
bool - true if a valid network time has been applied to the RTC;
       false if RTC has never been synchronised.
*******************************************************************************/
bool Controller_CheckIfTimeValid(void)
{
    return m_SystemEventPtr.TimeDataValid;
}

/*******************************************************************************
Description:
Return a pointer to the internal system configuration struct (m_SystemConfig)
cast as a byte array. Caller may use this to inspect or transmit the config.

Argument(s):
None

Return:
uint8_t* - Pointer to m_SystemConfig cast as byte array.
*******************************************************************************/
static uint8_t* Controller_GetConfig(void)
{
    return (uint8_t*)&m_SystemConfig;
}

/*******************************************************************************
Description:
Update RTC time and date from parsed network time.

Arguments:
time - Pointer to RTC time structure
date - Pointer to RTC date structure

Return:
HAL_StatusTypeDef - HAL_OK on success
*******************************************************************************/
HAL_StatusTypeDef Controller_UpdateRTC(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
    HAL_StatusTypeDef status;

    if (time == NULL || date == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
    if(status == HAL_OK)
    {
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
        m_SystemEventPtr.TimeDataValid = true;
        Controller_SavePointersToEEPROM();
        Display_Date();
        Display_Update();
    }
    return status;
}

/*******************************************************************************
Description:
Configure and arm RTC Alarm A to fire daily at 00:00:00.
AlarmMask = RTC_ALARMMASK_DATEWEEKDAY so the date/weekday field is ignored
and the alarm repeats every day. Uses HAL_RTC_SetAlarm_IT to enable the
alarm interrupt (ALRAIE). NVIC routing is enabled in MX_RTC_Init().

Argument(s):
hours   - Alarm hour (0-23)
minutes - Alarm minute (0-59)

Return:
None
*******************************************************************************/
void Controller_SetDailyAlarm(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    (void)seconds;  /* ignore caller's seconds — always force to 50 */
    RTC_AlarmTypeDef sAlarm = {0};

    sAlarm.AlarmTime.Hours = hours;
    sAlarm.AlarmTime.Minutes = minutes;
    sAlarm.AlarmTime.Seconds = 50;
    sAlarm.AlarmTime.SubSeconds = 0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY; // Mask day → fires every day
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        LOG_ERROR(CONTROLLER_LOG, "HAL_RTC_SetAlarm_IT failed");
    }
    else
    {
        LOG_INFO(CONTROLLER_LOG, "Daily alarm armed for %02d:%02d:%02d",
                 sAlarm.AlarmTime.Hours, sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds);
    }
}

/*******************************************************************************
Description:
Retrieve the stored daily alarm time from the event pointer config struct.

Argument(s):
hours   - Output pointer for alarm hour (0-23), or NULL to skip
minutes - Output pointer for alarm minute (0-59), or NULL to skip
seconds - Output pointer for alarm second (0-59), or NULL to skip

Return:
None
*******************************************************************************/
void Controller_GetAlarmTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    if (hours != NULL)   *hours   = m_SystemEventPtr.alarmHours;
    if (minutes != NULL) *minutes = m_SystemEventPtr.alarmMinutes;
    if (seconds != NULL) *seconds = m_SystemEventPtr.alarmSeconds;
}

/*******************************************************************************
Description:
Read the current battery SOC and push a battery-status event onto
g_eventDataQueue with buttonId=0 and RFID field set to "BS : <soc>".

Argument(s):
imei - Device IMEI string to include in the event
uid  - Device UID string to include in the event

Return:
None
*******************************************************************************/
void Controller_PushBatteryEvent(const char *imei, const char *uid)
{
    Controller_ReadBatterySOC();
    uint8_t bat = Controller_GetBatterySOC();

    EC200U_EventData_t eventData;
    memset(&eventData, 0, sizeof(eventData));
    Controller_GetRTCString(eventData.dateTime, sizeof(eventData.dateTime));
    strncpy(eventData.imei, imei, sizeof(eventData.imei) - 1);
    strncpy(eventData.uid, uid, sizeof(eventData.uid) - 1);
    snprintf(eventData.rfid, sizeof(eventData.rfid), "BS : %d", bat);
    eventData.buttonId = 0;  /* 0 = battery status, not a button press */

    LOG_INFO(CONTROLLER_LOG, "Pushing battery event: %s, SOC=%d%%", eventData.dateTime, bat);
    xQueueSend(g_eventDataQueue, &eventData, 0);
}

/*******************************************************************************
Description:
Read battery state-of-charge from ADC (PA0).
470K/470K voltage divider: Vbatt = ADC * Vref / 4096 * 2
Maps voltage to 0-100% SOC for 3xAA batteries.

Return:
uint8_t - State of charge 0-100
*******************************************************************************/
uint8_t Controller_ReadBatterySOC(void)
{
    uint32_t adcValue = 0;
    uint32_t batteryMv = 0;
    uint8_t soc = 0;

    // Enable battery switch
    HAL_GPIO_WritePin(BATT_SWITCH_GPIO_Port, BATT_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    adcValue = Controller_ReadADC();
    // Calculate battery voltage: ADC * Vref / 4096 * 2 (voltage divider)
    batteryMv = (adcValue * BATT_ADC_VREF_MV * 2) / 4096;

    // Map to SOC percentage
    if (batteryMv >= BATT_FULL_MV)
    {
        soc = 100;
    }
    else if (batteryMv <= BATT_EMPTY_MV)
    {
        soc = 0;
    }
    else
    {
        soc = (uint8_t)(((batteryMv - BATT_EMPTY_MV) * 100) / (BATT_FULL_MV - BATT_EMPTY_MV));
    }
    LOG_INFO(CONTROLLER_LOG, "Battery voltage: %lu mV, SOC: %u%%, adc:%lu", batteryMv, soc, adcValue);
    HAL_GPIO_WritePin(BATT_SWITCH_GPIO_Port, BATT_SWITCH_Pin, GPIO_PIN_RESET);
    m_BatterySoc = soc;
    return soc;
}

/*******************************************************************************
Description:
Return the last cached battery state-of-charge value without performing
a new ADC reading. Call Controller_ReadBatterySOC() first to refresh.

Argument(s):
None

Return:
uint8_t - Cached SOC percentage (0-100)
*******************************************************************************/
uint8_t Controller_GetBatterySOC(void)
{
    return m_BatterySoc;
}

/*******************************************************************************
Description:
Buzzer beep using TIM1 Channel 4 PWM output (PA11).

Arguments:
durationMs - Duration of beep in milliseconds

Return:
None
*******************************************************************************/
void Controller_BuzzerBeep(uint16_t durationMs)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    vTaskDelay(pdMS_TO_TICKS(durationMs));
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

/*******************************************************************************
Description:
Turn the buzzer on by enabling TIM1 Channel 4 output and starting the counter.
Call Controller_BuzzerOff() to silence.

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_BuzzerOn(void)
{
    htim1.Instance->CCER |= TIM_CCER_CC4E;    /* Enable CH4 output compare   */
    htim1.Instance->BDTR |= TIM_BDTR_MOE;     /* Main Output Enable (TIM1)   */
    htim1.Instance->CR1  |= TIM_CR1_CEN;      /* Start counter               */
}

/*******************************************************************************
Description:
Turn the buzzer off by disabling TIM1 Channel 4 output compare.

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_BuzzerOff(void)
{
    htim1.Instance->CCER &= ~TIM_CCER_CC4E;   /* Disable CH4 output compare  */
}

/*******************************************************************************
Description:
Flash an LED briefly.

Arguments:
ledNum      - 1 for LED1 (PB5), 2 for LED2 (PB6)
durationMs  - Duration of flash in milliseconds

Return:
None
*******************************************************************************/
void Controller_LEDFlash(uint8_t ledNum, uint16_t durationMs)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    if (ledNum == 1)
    {
        port = LED1_GPIO_Port;
        pin = LED1_Pin;
    }
    else if (ledNum == 2)
    {
        port = LED2_GPIO_Port;
        pin = LED2_Pin;
    }
    else
    {
        return;
    }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(durationMs));
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/*******************************************************************************
Description:
Check if there is backed-up event data in EEPROM (write pointer != read pointer).

Return:
bool - true if data is backed up
*******************************************************************************/
bool Controller_IsDataBackedUp(void)
{
  if(m_SystemEventPtr.writePtr != m_SystemEventPtr.readPtr)
  {
    return true;
  }
  return false;
}

/*******************************************************************************
Description:
Return the number of events currently stored in the EEPROM circular buffer
by computing the distance between the write and read pointers.

Argument(s):
None

Return:
uint16_t - Number of pending events (0 to EEPROM_MAX_EVENTS-1)
*******************************************************************************/
uint16_t Controller_GetEEPROMEventCount(void)
{
  if (m_SystemEventPtr.writePtr >= m_SystemEventPtr.readPtr)
    return m_SystemEventPtr.writePtr - m_SystemEventPtr.readPtr;
  else
    return EEPROM_MAX_EVENTS - m_SystemEventPtr.readPtr + m_SystemEventPtr.writePtr;
}

/*******************************************************************************
Description:
Load all backed-up events from EEPROM into g_eventDataQueue.
Advances the read pointer as events are loaded.

Return:
bool - true if events loaded successfully (or no events to load)
*******************************************************************************/
bool Controller_LoadQueueFromEEPROM(const char *imei, const char *uid)
{
    EepromEvent_t compact;
    EC200U_EventData_t eventData;

    while (m_SystemEventPtr.readPtr != m_SystemEventPtr.writePtr)
    {
        uint32_t address = EEPROM_EVENT_DATA_START +
                           (m_SystemEventPtr.readPtr * EEPROM_EVENT_BLOCK_SIZE);

        memset(&compact, 0, sizeof(compact));

        if (EEPROM_Read(address, (uint8_t *)&compact, sizeof(EepromEvent_t)) != EEPROM_OK)
        {
            return false;
        }

        // Reconstruct full event from compact data + current imei/uid
        memset(&eventData, 0, sizeof(eventData));
        memcpy(eventData.dateTime, compact.dateTime, sizeof(eventData.dateTime));
        memcpy(eventData.rfid, compact.rfid, sizeof(eventData.rfid));
        eventData.buttonId = compact.buttonId;
        strncpy(eventData.imei, imei, sizeof(eventData.imei) - 1);
        strncpy(eventData.uid, uid, sizeof(eventData.uid) - 1);

        if (xQueueSend(g_eventDataQueue, &eventData, 0) != pdTRUE)
        {
            // Queue full — stop loading, leave remaining in EEPROM
            Controller_SavePointersToEEPROM();
            return false;
        }

        m_SystemEventPtr.readPtr = Controller_NextPtr(m_SystemEventPtr.readPtr);
    }

    Controller_SavePointersToEEPROM();
    return true;
}

/*******************************************************************************
Description:
Save a single event to EEPROM at the write pointer position.

Arguments:
eventData - Pointer to event data to save

Return:
bool - true if successful
*******************************************************************************/
bool Controller_SaveEventToEEPROM(EC200U_EventData_t *eventData)
{
    if (eventData == NULL) return false;

    uint16_t nextWr = Controller_NextPtr(m_SystemEventPtr.writePtr);

    // Check if EEPROM is full (circular buffer wrap)
    if (nextWr == m_SystemEventPtr.readPtr)
    {
        return false;  // EEPROM full
    }

    // Store only the compact fields (no imei/uid)
    EepromEvent_t compact;
    memcpy(compact.dateTime, eventData->dateTime, sizeof(compact.dateTime));
    memcpy(compact.rfid, eventData->rfid, sizeof(compact.rfid));
    compact.buttonId = eventData->buttonId;

    uint32_t address = EEPROM_EVENT_DATA_START +
                       (m_SystemEventPtr.writePtr * EEPROM_EVENT_BLOCK_SIZE);

    if (EEPROM_Write(address, sizeof(EepromEvent_t), (uint8_t *)&compact) != EEPROM_OK)
    {
        return false;
    }

    m_SystemEventPtr.writePtr = nextWr;
    Controller_SavePointersToEEPROM();

    return true;
}

/*******************************************************************************
Description:
Drain all events from g_eventDataQueue and save them to EEPROM.

Return:
bool - true if all events saved successfully
*******************************************************************************/
bool Controller_SaveQueueToEEPROM(void)
{
    EC200U_EventData_t eventData;

    while (xQueueReceive(g_eventDataQueue, &eventData, 0) == pdTRUE)
    {
        if (!Controller_SaveEventToEEPROM(&eventData))
        {
            // Failed to save — put it back
            xQueueSendToFront(g_eventDataQueue, &eventData, 0);
            return false;
        }
    }

    return true;
}


/*******************************************************************************
Description:
Write the m_EventTimeDataBuf array to EEPROM at EEPROM_TIMEDATA_START (0x1000).
Stores the entire TIMEDATA_ROWS x TIMEDATA_COLS x CELL_STR_MAX block so that
Controller_LoadTimeDataFromEEPROM() can restore it on next boot.

Argument(s):
None

Return:
bool - true if EEPROM_Write succeeded; false on write error.
*******************************************************************************/
bool Controller_SaveTimeDataINEEPROM(void)
{
    if (EEPROM_Write(EEPROM_TIMEDATA_START, sizeof(m_SystemTimeData), (uint8_t *)&m_SystemTimeData) != EEPROM_OK)
    {
        LOG_ERROR(CONTROLLER_LOG, "Failed to save time data to EEPROM");
        return false;
    }
    return true;
}

/*******************************************************************************
Description:
Set a configuration field by name. Saves the updated config to EEPROM.
Supported fields: ServerName, ServerPort, APName, APDescription.

Argument(s):
field - Field name (case-sensitive, must match struct member name)
value - String value to set (numeric fields are converted from string)

Return:
bool - true if field was recognised and set; false if unknown field.
*******************************************************************************/
bool Controller_SetConfigField(const char *field, const char *value)
{
    if (field == NULL || value == NULL) return false;

    if (strcmp(field, "ServerName") == 0)
    {
        memset(m_SystemConfig.ServerName, 0, sizeof(m_SystemConfig.ServerName));
        strncpy(m_SystemConfig.ServerName, value, sizeof(m_SystemConfig.ServerName) - 1);
    }
    else if (strcmp(field, "ServerPort") == 0)
    {
        m_SystemConfig.ServerPort = (uint32_t)atoi(value);
    }
    else if (strcmp(field, "APName") == 0)
    {
        memset(m_SystemConfig.APName, 0, sizeof(m_SystemConfig.APName));
        strncpy(m_SystemConfig.APName, value, sizeof(m_SystemConfig.APName) - 1);
    }
    else if (strcmp(field, "APDescription") == 0)
    {
        memset(m_SystemConfig.APDescription, 0, sizeof(m_SystemConfig.APDescription));
        strncpy(m_SystemConfig.APDescription, value, sizeof(m_SystemConfig.APDescription) - 1);
    }
    else if (strcmp(field, "ShutDownBatLevel") == 0)
    {
        m_SystemConfig.ShutDownBatLevel = (uint8_t)atoi(value);
    }
    else if (strcmp(field, "RTCALARM") == 0)
    {
        unsigned int h, m, s;
        if (sscanf(value, "%u:%u:%u", &h, &m, &s) != 3 ||
            h > 23 || m > 59 || s > 59)
        {
            return false;
        }
        m_SystemEventPtr.alarmHours   = (uint8_t)h;
        m_SystemEventPtr.alarmMinutes = (uint8_t)m;
        m_SystemEventPtr.alarmSeconds = (uint8_t)s;
        Controller_SavePointersToEEPROM();
        Controller_SetDailyAlarm((uint8_t)h, (uint8_t)m, (uint8_t)s);
        return true;
    }
    else
    {
        return false;
    }

    Controller_SaveConfigurationToEEPROM();
    return true;
}

/*******************************************************************************
Description:
Get a configuration field value by name.
Supported fields: ServerName, ServerPort, APName, APDescription.

Argument(s):
field   - Field name (case-sensitive, must match struct member name)
buf     - Output buffer for the field value as a string
bufSize - Size of the output buffer

Return:
bool - true if field was recognised; false if unknown field.
*******************************************************************************/
bool Controller_GetConfigField(const char *field, char *buf, size_t bufSize)
{
    if (field == NULL || buf == NULL || bufSize == 0) return false;

    if (strcmp(field, "ServerName") == 0)
    {
        strncpy(buf, m_SystemConfig.ServerName, bufSize - 1);
        buf[bufSize - 1] = '\0';
    }
    else if (strcmp(field, "ServerPort") == 0)
    {
        snprintf(buf, bufSize, "%lu", (unsigned long)m_SystemConfig.ServerPort);
    }
    else if (strcmp(field, "APName") == 0)
    {
        strncpy(buf, m_SystemConfig.APName, bufSize - 1);
        buf[bufSize - 1] = '\0';
    }
    else if (strcmp(field, "APDescription") == 0)
    {
        strncpy(buf, m_SystemConfig.APDescription, bufSize - 1);
        buf[bufSize - 1] = '\0';
    }
    else if (strcmp(field, "ShutDownBatLevel") == 0)
    {
        snprintf(buf, bufSize, "%u", m_SystemConfig.ShutDownBatLevel);
    }
    else if (strcmp(field, "RTCALARM") == 0)
    {
        snprintf(buf, bufSize, "%02d:%02d:%02d",
                 m_SystemEventPtr.alarmHours,
                 m_SystemEventPtr.alarmMinutes,
                 m_SystemEventPtr.alarmSeconds);
    }
    else
    {
        return false;
    }

    return true;
}

/*******************************************************************************
Description:
Clear all event data by resetting the circular buffer write and read pointers
to zero. Saves the reset pointers to EEPROM.

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_ClearEventData(void)
{
    m_SystemEventPtr.writePtr = 0;
    m_SystemEventPtr.readPtr = 0;
    m_SystemEventPtr.TimeDataValid = false;
    Controller_SavePointersToEEPROM();
    LOG_INFO(CONTROLLER_LOG, "Event data cleared");
}

/*******************************************************************************
Description:
Clear the event time data buffer and save to EEPROM.

Argument(s):
None

Return:
None
*******************************************************************************/
void Controller_ClearEventTime(void)
{
    memset(m_SystemTimeData.EventTimeData, 0, sizeof(m_SystemTimeData.EventTimeData));
    Controller_SaveTimeDataINEEPROM();
    LOG_INFO(CONTROLLER_LOG, "Event time data cleared");
}

/*******************************************************************************
Description:
EXTI interrupt callback. On PROX_INT (PC13) edge, disables re-entry and
gives TouchScanSemaphore to wake the touch-scan task.

Argument(s):
GPIO_Pin - Bitmask of the GPIO pin that triggered the interrupt

Return:
None
*******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == PROX_INT_Pin)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    HAL_NVIC_DisableIRQ(PROX_INT_EXTI_IRQn);  /* block re-entry until scan completes */
    xSemaphoreGiveFromISR(TouchScanSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/*******************************************************************************
Description:
RTC Alarm A interrupt callback. Gives RTCWakeupSemaphore to signal the
daily wakeup task (e.g. for date update or scheduled GSM transmission).

Argument(s):
hrtc - Pointer to the RTC handle that triggered the alarm

Return:
None
*******************************************************************************/
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Daily alarm: flag date update + trigger RTC wakeup task */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(RTCWakeupSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Load event circular-buffer pointers from EEPROM (EEPROM_POINTERS_START 0x80).
Reads the EventPtrConfig_t structure and validates MagicNumber against
EEPROM_POINTER_MAGIC_VALUE (0xDEAD).
If valid   : retains writePtr and readPtr values from EEPROM.
If invalid : resets writePtr, readPtr to 0 and writes defaults back to EEPROM.
Sanity check: if either pointer is >= EEPROM_MAX_EVENTS it is reset to 0.
If any reset occurred, the corrected struct is saved back to EEPROM.

Argument(s):
None

Return:
None
*******************************************************************************/
static void Controller_LoadPointersFromEEPROM(void)
{
    bool status = false;

    if (EEPROM_Read(EEPROM_POINTERS_START, (uint8_t*)&m_SystemEventPtr, sizeof(m_SystemEventPtr)) == EEPROM_OK)
    {
       if(m_SystemEventPtr.MagicNumber == EEPROM_POINTER_MAGIC_VALUE)
       {
            status = true;
            LOG_INFO(CONTROLLER_LOG, "Some events still to be sent:%d,%d", m_SystemEventPtr.writePtr, m_SystemEventPtr.readPtr);
       }
       else
       {
            m_SystemEventPtr.writePtr = 0;
            m_SystemEventPtr.readPtr = 0;
            m_SystemEventPtr.TimeDataValid = false;
            /* Generate random alarm time using device UID as seed */
            srand(HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetTick());
            m_SystemEventPtr.alarmHours = (uint8_t)(rand() % 24);
            m_SystemEventPtr.alarmMinutes = (uint8_t)(rand() % 60);
            m_SystemEventPtr.alarmSeconds = 0;
            m_SystemEventPtr.MagicNumber = EEPROM_POINTER_MAGIC_VALUE;
            LOG_ERROR(CONTROLLER_LOG, "Magic numbers dont match-resetting pointers");
            LOG_INFO(CONTROLLER_LOG, "Random alarm time set: %02d:%02d:%02d",
                     m_SystemEventPtr.alarmHours, m_SystemEventPtr.alarmMinutes,
                     m_SystemEventPtr.alarmSeconds);
            status = false;            
       }

        // Sanity check
        if (m_SystemEventPtr.writePtr >= EEPROM_MAX_EVENTS)
        {
            m_SystemEventPtr.writePtr = 0;
            LOG_ERROR(CONTROLLER_LOG, "Write pointer exceeds limit-resetting");
            status = false;
        }
        if (m_SystemEventPtr.readPtr >= EEPROM_MAX_EVENTS)
        {
            m_SystemEventPtr.readPtr = 0;
            LOG_ERROR(CONTROLLER_LOG, "Read pointer exceeds limit-resetting");
            status = false;
        }
        if(status == false)
        {
            Controller_SavePointersToEEPROM();
        }
    }
}
/*******************************************************************************
Description:
Write m_SystemConfig (ConfigParam_t) to EEPROM at EEPROM_CONFIG_START (0x0000).

Argument(s):
None

Return:
None
*******************************************************************************/
static void Controller_SaveConfigurationToEEPROM(void)
{
    EEPROM_Write(EEPROM_CONFIG_START, sizeof(m_SystemConfig), (uint8_t *)&m_SystemConfig);
}
/*******************************************************************************
Description:
Write m_SystemEventPtr (EventPtrConfig_t) to EEPROM at EEPROM_POINTERS_START
(0x0080). Persists writePtr, readPtr, TimeDataValid and MagicNumber fields.

Argument(s):
None

Return:
None
*******************************************************************************/
static void Controller_SavePointersToEEPROM(void)
{
    EEPROM_Write(EEPROM_POINTERS_START, sizeof(m_SystemEventPtr), (uint8_t *)&m_SystemEventPtr);
}

/*******************************************************************************
Description:
Load the TIMEDATA_ROWS x TIMEDATA_COLS display time-data strings from EEPROM.

EEPROM layout (starting at EEPROM_TIMEDATA_START = 0x1000):
  Bytes [0-1] : magic 0xCAFE — indicates that valid time data has been stored.
  Bytes [2 .. 2 + TIMEDATA_ROWS*TIMEDATA_COLS*(CELL_STR_MAX) - 1]:
                flat array of null-terminated strings, stored row-major:
                [0][0], [0][1], [1][0], [1][1], [2][0], [2][1]
                Each string occupies exactly (CELL_STR_MAX) bytes.

On success  : m_EventTimeDataBuf is populated with the block read from EEPROM.
On failure  : m_EventTimeDataBuf is zeroed (EEPROM read error).

Argument(s):
None

Return:
None
*******************************************************************************/
static void Controller_LoadTimeDataFromEEPROM(void)
{
    memset(&m_SystemTimeData, 0, sizeof(m_SystemTimeData));

    /* Read time-data block (magic word + string data) from EEPROM */
    if (EEPROM_Read(EEPROM_TIMEDATA_START,(uint8_t *)&m_SystemTimeData, sizeof(m_SystemTimeData)) == EEPROM_OK)
    {
        if(m_SystemTimeData.MagicNumber == EEPROM_TIMEDATA_MAGIC_VALUE)
        {
            LOG_INFO(CONTROLLER_LOG, "Time data loaded from EEPROM");
        }
        else
        {
            LOG_WARN(CONTROLLER_LOG, "No valid time data in EEPROM (magic mismatch)");
            memset(&m_SystemTimeData, 0, sizeof(m_SystemTimeData));  /* Clear buffer on invalid magic */
            m_SystemTimeData.MagicNumber = EEPROM_TIMEDATA_MAGIC_VALUE;  /* Mark as valid for future saves */
            Controller_SaveTimeDataINEEPROM();  /* Save empty data to set magic value in EEPROM */

        }
    }

}

/*******************************************************************************
Description:
Advance a circular buffer pointer by one slot.

Arguments:
ptr - Current pointer value

Return:
uint16_t - Next pointer value (wraps around)
*******************************************************************************/
static uint16_t Controller_NextPtr(uint16_t ptr)
{
    ptr++;
    if (ptr >= EEPROM_MAX_EVENTS)
    {
        ptr = 0;
    }
    return ptr;
}

/*******************************************************************************
Description:
Read ADC1 channel 5 (PA0) using HAL driver.
hadc1 is initialised by MX_ADC1_Init() at startup with ADC_CHANNEL_5 configured.

Return:
uint32_t - 12-bit ADC value (0-4095), or 0 on error
*******************************************************************************/
static uint32_t Controller_ReadADC(void)
{
    uint32_t adcValue = 0;

    HAL_ADC_DeInit(&hadc1);
    MX_ADC1_Init();

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 0;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return adcValue;
}
