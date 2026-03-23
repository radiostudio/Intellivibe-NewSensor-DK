/*******************************************************************************
********************************************************************************
Description:
This file defines common data types used across the IntelliVibe-BT firmware.
Includes sensor data structures, system settings, and BLE-related types.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef COMMONTYPES_H
#define COMMONTYPES_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <stdint.h>
#include <stdbool.h>

//******************************************************************************
// DEFINES
//******************************************************************************
#define RANGE_2G    1
#define RANGE_4G    2
#define RANGE_8G    3
#define RANGE_16G   4
#define RANGE_32G   5
#define RANGE_64G   6

//******************************************************************************
// DATA TYPES
//******************************************************************************
typedef enum
{
    SENSOR_ACCEL = 0,
    SENSOR_MAG,
    SENSOR_TEMP,
    SENSOR_PSRAM,
    SENSOR_AUDIO,
    SENSOR_BATTERY,
    SENSOR_NVS,
    SENSOR_COUNT
} SensorId_t;

typedef struct
{
    uint16_t XData;
    uint16_t YData;
    uint16_t ZData;
} RmsData_t;

typedef struct
{
    float XData;
    float YData;
    float ZData;
} OutputData_t;

typedef struct
{
    uint16_t Min;
    uint16_t Max;
} RpmRange_t;

#define SETTINGS_MAGIC_NUMBER   0xDEAD

typedef struct __attribute__((packed))
{
    uint16_t   MagicNumber;
    uint16_t   DInterval;
    uint16_t   HighSampleRate;
    uint16_t   HighSampleDuration;
    uint16_t   Interval;
    int8_t     LowerCutoff;
    uint8_t    AccelRange;
    uint8_t    MagAxis;
    uint8_t    NoPolePair;
    int8_t     TempOffset;
    RpmRange_t RpmRange;
    bool       EnRPM;
    uint16_t   Crc;
} SystemSettings_t;

typedef struct
{
    volatile bool VibStreamingEn;
    volatile bool MagStreamingEn;
} BLENotifyEn_t;

typedef struct
{
    uint16_t  CompanyCode;
    RmsData_t AccelRms;
    RmsData_t VelRms;
    RmsData_t MagRms;
    uint16_t  Temperature;
    uint16_t  Noise;
    uint16_t  BatLevel;
} AdvMfgData_t;

#ifdef __cplusplus
}
#endif

#endif /* COMMONTYPES_H */
