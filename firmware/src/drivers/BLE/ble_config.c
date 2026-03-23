/*******************************************************************************
********************************************************************************
Description:
This file handles functions for BLE Configuration service management.
Includes GATT config service definition, read/write handlers for system
settings, firmware version, and streaming command dispatch.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BLE_CONFIG);

#include <zephyr/kernel.h>
#include <string.h>
#include "ble_config.h"
#include "ble.h"
#include "UserConfig.h"

extern void Main_StartStreaming(uint16_t Cmd);

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static SystemSettings_t *m_Settings;
static const char       *m_FirmwareVersion;
static int8_t           *m_SensorState;
static uint8_t           m_SensorStateCount;
static uint8_t           m_ReadBuf[100] = {0};
static uint16_t          m_ReadBufLen = 0;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static void BLEConfig_UpdateSettings(const void *Buf, uint16_t Len);
static void BLEConfig_UpdateRpmSettings(const void *Buf, uint16_t Len);
static void BLEConfig_ReadSettings(const void *Buf, uint16_t Len);

/*---------------------------------------------------------------------------*/
/* GATT Config Service Definition                                            */
/*---------------------------------------------------------------------------*/
BT_GATT_SERVICE_DEFINE(m_ConfigSrv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CONFIG_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CONFIG_CHRC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           BLEConfig_Read,
                           BLEConfig_Write,
                           NULL));

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Set the pointer to the system settings struct. Must be called before any
config read/write operations so the config service knows where to store
received settings.

Argument(s):
Settings - Pointer to the system settings structure.

Return:
None
*******************************************************************************/
void BLEConfig_GetSettings(SystemSettings_t *Settings)
{
    m_Settings = Settings;
}

/*******************************************************************************
Description:
Set the firmware version string pointer. Used when a BLE client reads the
firmware version via the config service.

Argument(s):
Version - Pointer to a null-terminated firmware version string.

Return:
None
*******************************************************************************/
void BLEConfig_GetFirmwareVersion(const char *Version)
{
    m_FirmwareVersion = Version;
}

/*******************************************************************************
Description:
Set the pointer to the sensor state array so the config service can return
actual sensor init status when a BLE client requests it (CMD 0x03 sub 0x01).

Argument(s):
SensorState - Pointer to the sensor state array (0 = OK, -1 = failed).
Count       - Number of entries in the array.

Return:
None
*******************************************************************************/
void BLEConfig_GetSensorState(int8_t *SensorState, uint8_t Count)
{
    m_SensorState = SensorState;
    m_SensorStateCount = Count;
}

/*******************************************************************************
Description:
GATT write handler for the config characteristic. Dispatches incoming commands:
  0x01 - Update system settings
  0x02 - Update RPM settings
  0x03 - Read settings/status/version
  0x0A - Request vibration streaming
  0x0B - Request magnetometer streaming
  0x0C - Request dual (vibration + magnetometer) streaming

Argument(s):
Conn   - Pointer to the connection.
Attr   - Pointer to the GATT attribute being written.
Buf    - Pointer to the incoming data.
Len    - Length of the incoming data.
Offset - Write offset.
Flags  - Write flags.

Return:
ssize_t - Number of bytes consumed (Len on success).
*******************************************************************************/
ssize_t BLEConfig_Write(struct bt_conn *Conn, const struct bt_gatt_attr *Attr,
                        const void *Buf, uint16_t Len, uint16_t Offset,
                        uint8_t Flags)
{
    uint16_t Cmd = 0;

    if (bt_uuid_cmp(Attr->uuid, BT_UUID_CONFIG_CHRC) != 0)
    {
        return Len;
    }

    LOG_INF("Config received");
    memcpy(&Cmd, Buf, sizeof(Cmd));
    LOG_INF("Write CMD: %d", Cmd);

    switch (Cmd)
    {
    case 0x01:
        LOG_WRN("New settings received");
        BLEConfig_UpdateSettings(Buf, Len);
        break;

    case 0x02:
        LOG_WRN("New RPM settings received");
        BLEConfig_UpdateRpmSettings(Buf, Len);
        break;

    case 0x03:
        LOG_WRN("Read setting CMD received");
        BLEConfig_ReadSettings(Buf, Len);
        break;

    case 0x0A:
        /* fall through */
    case 0x0B:
        /* fall through */
    case 0x0C:
        LOG_WRN("Streaming CMD 0x%02X received", Cmd);
        Main_StartStreaming(Cmd);
        break;

    default:
        LOG_WRN("Unknown CMD: 0x%04X", Cmd);
        break;
    }

    return Len;
}

/*******************************************************************************
Description:
GATT read handler for the config characteristic. Returns the contents of
m_ReadBuf which was populated by a prior write command (CMD 0x03).

Argument(s):
Conn   - Pointer to the connection.
Attr   - Pointer to the GATT attribute being read.
Buf    - Output buffer for the read data.
Len    - Maximum bytes the caller can accept.
Offset - Read offset within the attribute value.

Return:
ssize_t - Number of bytes written to Buf, or negative error code.
*******************************************************************************/
ssize_t BLEConfig_Read(struct bt_conn *Conn, const struct bt_gatt_attr *Attr,
                       void *Buf, uint16_t Len, uint16_t Offset)
{
    LOG_WRN("Config read requested");

    if (bt_uuid_cmp(Attr->uuid, BT_UUID_CONFIG_CHRC) == 0)
    {
        return bt_gatt_attr_read(Conn, Attr, Buf, Len, Offset,
                                 m_ReadBuf, m_ReadBufLen);
    }

    return Len;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Parse a BLEConfig_t struct from the received BLE write buffer and update the
system settings accordingly.

Argument(s):
Buf - Pointer to the raw received data.
Len - Length of the received data.

Return:
None

Note(s):
Validates accel_range (must be 3-6, defaults to RANGE_8G).
*******************************************************************************/
static void BLEConfig_UpdateSettings(const void *Buf, uint16_t Len)
{
    BLEConfig_t NewSetting;

    if (m_Settings == NULL)
    {
        LOG_ERR("Settings pointer not set");
        return;
    }

    memcpy(&NewSetting, Buf, sizeof(NewSetting));

    m_Settings->DInterval          = NewSetting.DInterval;
    m_Settings->HighSampleRate     = NewSetting.HighSamplingRate;
    m_Settings->HighSampleDuration = NewSetting.HighSamplingDuration;
    m_Settings->Interval           = NewSetting.HSInterval;
    m_Settings->LowerCutoff        = NewSetting.LowerCuttoff;
    m_Settings->TempOffset         = NewSetting.TempOffset;

    if (NewSetting.AccelRange < 3 || NewSetting.AccelRange > 6)
    {
        m_Settings->AccelRange = RANGE_8G;
    }
    else
    {
        m_Settings->AccelRange = NewSetting.AccelRange;
    }

    LOG_INF("Settings updated: dInterval=%d, rate=%d, duration=%d",
            m_Settings->DInterval, m_Settings->HighSampleRate,
            m_Settings->HighSampleDuration);

    UserConfig_SaveSettings();
}

/*******************************************************************************
Description:
Parse a BLEConfigRpm_t struct from the received BLE write buffer and update
the RPM-related fields of system settings.

Argument(s):
Buf - Pointer to the raw received data.
Len - Length of the received data.

Return:
None
*******************************************************************************/
static void BLEConfig_UpdateRpmSettings(const void *Buf, uint16_t Len)
{
    BLEConfigRpm_t NewSetting;

    if (m_Settings == NULL)
    {
        LOG_ERR("Settings pointer not set");
        return;
    }

    memcpy(&NewSetting, Buf, sizeof(NewSetting));

    m_Settings->NoPolePair    = NewSetting.PolePair;
    m_Settings->MagAxis       = NewSetting.MagAxis;
    m_Settings->RpmRange.Min  = NewSetting.RpmRange.Min;
    m_Settings->RpmRange.Max  = NewSetting.RpmRange.Max;

    LOG_INF("RPM settings updated: polePair=%d, magAxis=%d, range=%d-%d",
            m_Settings->NoPolePair, m_Settings->MagAxis,
            m_Settings->RpmRange.Min, m_Settings->RpmRange.Max);

    UserConfig_SaveSettings();
}

/*******************************************************************************
Description:
Handle a read-settings command (CMD 0x03). Extracts the sub-command from the
buffer and populates m_ReadBuf with the requested data:
  0x01 - Sensor status (placeholder, 6 bytes of zeros)
  0x02 - Current system settings
  0x03 - Firmware version string

Argument(s):
Buf - Pointer to the raw received data (contains sub-command at offset 2).
Len - Length of the received data.

Return:
None
*******************************************************************************/
static void BLEConfig_ReadSettings(const void *Buf, uint16_t Len)
{
    uint16_t ReadCmd = 0;

    memcpy(&ReadCmd, (const uint8_t *)Buf + sizeof(uint16_t), sizeof(ReadCmd));
    LOG_INF("Read CMD: %d", ReadCmd);

    switch (ReadCmd)
    {
    case 0x01:
        if (m_SensorState != NULL)
        {
            m_ReadBufLen = m_SensorStateCount;
            memcpy(m_ReadBuf, m_SensorState, m_ReadBufLen);
        }
        break;

    case 0x02:
        if (m_Settings != NULL)
        {
            m_ReadBufLen = sizeof(BLEConfig_t);
            memcpy(m_ReadBuf, m_Settings, m_ReadBufLen);
        }
        break;

    case 0x03:
        if (m_FirmwareVersion != NULL)
        {
            m_ReadBufLen = strlen(m_FirmwareVersion);
            memcpy(m_ReadBuf, m_FirmwareVersion, m_ReadBufLen);
        }
        break;

    default:
        LOG_WRN("Unknown read CMD: 0x%04X", ReadCmd);
        break;
    }
}
