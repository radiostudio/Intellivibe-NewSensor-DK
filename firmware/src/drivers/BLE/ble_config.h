/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for BLE Configuration
service handling. Includes GATT config service UUIDs, config data types,
and read/write handlers.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include "CommonTypes.h"

//******************************************************************************
// DEFINES
//******************************************************************************
/** @brief UUID of the Config Service. */
#define BT_UUID_CONFIG_SERV_VAL \
    BT_UUID_128_ENCODE(0x76f68da3, 0x329d, 0x4e1b, 0xa190, 0x3c88d8fae911)

#define BT_UUID_CONFIG_SERVICE  BT_UUID_DECLARE_128(BT_UUID_CONFIG_SERV_VAL)

/** @brief UUID of the Config Characteristic. */
#define BT_UUID_CONFIG_CHRC_VAL \
    BT_UUID_128_ENCODE(0x76f68db8, 0x329d, 0x4e1b, 0xa190, 0x3c88d8fae911)

#define BT_UUID_CONFIG_CHRC BT_UUID_DECLARE_128(BT_UUID_CONFIG_CHRC_VAL)

//******************************************************************************
// DATA TYPES
//******************************************************************************
typedef struct
{
    uint16_t Cmd;
    uint16_t DInterval;
    uint16_t HighSamplingRate;
    uint16_t HighSamplingDuration;
    uint16_t HSInterval;
    uint8_t  LowerCuttoff;
    uint8_t  AccelRange;
    int8_t   TempOffset;
} BLEConfig_t;

typedef struct
{
    uint16_t   Cmd;
    uint8_t    PolePair;
    uint8_t    MagAxis;
    RpmRange_t RpmRange;
} BLEConfigRpm_t;

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
void    BLEConfig_GetSettings(SystemSettings_t *Settings);
void    BLEConfig_GetFirmwareVersion(const char *Version);
void    BLEConfig_GetSensorState(int8_t *SensorState, uint8_t Count);
ssize_t BLEConfig_Write(struct bt_conn *Conn, const struct bt_gatt_attr *Attr,
                        const void *Buf, uint16_t Len, uint16_t Offset,
                        uint8_t Flags);
ssize_t BLEConfig_Read(struct bt_conn *Conn, const struct bt_gatt_attr *Attr,
                       void *Buf, uint16_t Len, uint16_t Offset);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONFIG_H */
