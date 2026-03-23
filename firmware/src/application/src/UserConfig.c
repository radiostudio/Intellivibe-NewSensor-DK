/*******************************************************************************
********************************************************************************
Description:
This file handles functions for system configuration management.
Includes factory default settings, NVS load/save with magic number and
CRC validation, and settings access via getter functions.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(USERCONFIG);

#include <zephyr/kernel.h>
#include <string.h>
#include "UserConfig.h"
#include "NVS/NVS.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define FIRMWARE_VERSION    "V1.0.0"
#define CRC16_POLY          0xA001

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static SystemSettings_t m_Settings = {0};

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static bool UserConfig_ValidateSettings(SystemSettings_t *Settings);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the UserConfig module. Loads factory defaults first, then attempts
to read saved settings from NVS. Validates the loaded settings by checking
the magic number (0xDEAD) and CRC. If validation fails, factory defaults
are used and saved to NVS.

Argument(s):
None

Return:
int - 0 on success, -1 if NVS init fails (settings will still be usable
      with factory defaults).
*******************************************************************************/
int UserConfig_Init(void)
{
    int Ret;
    SystemSettings_t LoadedSettings;

    /* Always start with factory defaults */
    UserConfig_LoadDefaults();

    /* Initialise NVS */
    Ret = NVS_Init();
    if (Ret)
    {
        LOG_ERR("NVS init failed: %d, using factory defaults", Ret);
        return -1;
    }

    /* Attempt to load settings from NVS */
    Ret = NVS_Read(NVS_SETTINGS_ID, &LoadedSettings, sizeof(SystemSettings_t));
    if (Ret < 0)
    {
        LOG_WRN("No saved settings found, using factory defaults");
        UserConfig_SaveSettings();
        return 0;
    }

    /* Validate magic number and CRC */
    if (UserConfig_ValidateSettings(&LoadedSettings))
    {
        memcpy(&m_Settings, &LoadedSettings, sizeof(SystemSettings_t));
        LOG_INF("Settings loaded from NVS (valid)");
    }
    else
    {
        LOG_WRN("Settings validation failed, loading factory defaults");
        UserConfig_LoadDefaults();
        UserConfig_SaveSettings();
    }

    UserConfig_PrintSettings();
    return 0;
}

/*******************************************************************************
Description:
Return a pointer to the internal system settings structure. Other modules
use this to read or modify settings in place. Call UserConfig_SaveSettings()
after modification to persist changes.

Argument(s):
None

Return:
SystemSettings_t* - Pointer to the current settings. Valid for the lifetime
                     of the module.
*******************************************************************************/
SystemSettings_t *UserConfig_GetSettings(void)
{
    return &m_Settings;
}

/*******************************************************************************
Description:
Save the current settings to NVS. Recalculates the CRC and sets the magic
number before writing.

Argument(s):
None

Return:
int - 0 on success, -1 on NVS write failure.
*******************************************************************************/
int UserConfig_SaveSettings(void)
{
    int Rc;

    /* Set magic number */
    m_Settings.MagicNumber = SETTINGS_MAGIC_NUMBER;

    /* Calculate CRC over all fields except the CRC field itself */
    m_Settings.Crc = UserConfig_CalculateCrc((const uint8_t *)&m_Settings,
                     sizeof(SystemSettings_t) - sizeof(m_Settings.Crc));

    Rc = NVS_Write(NVS_SETTINGS_ID, &m_Settings, sizeof(SystemSettings_t));
    if (Rc < 0)
    {
        LOG_ERR("Failed to save settings to NVS");
        return -1;
    }

    LOG_INF("Settings saved to NVS");
    return 0;
}

/*******************************************************************************
Description:
Populate m_Settings with factory default values. Sets magic number and
calculates CRC. Does not write to NVS — call UserConfig_SaveSettings()
separately if persistence is needed.

Argument(s):
None

Return:
None
*******************************************************************************/
void UserConfig_LoadDefaults(void)
{
    memset(&m_Settings, 0, sizeof(SystemSettings_t));

    m_Settings.MagicNumber        = SETTINGS_MAGIC_NUMBER;
    m_Settings.DInterval          = 120;
    m_Settings.Interval           = 900;
    m_Settings.HighSampleDuration = 3;
    m_Settings.HighSampleRate     = 1600;
    m_Settings.LowerCutoff        = 3;
    m_Settings.AccelRange         = RANGE_8G;
    m_Settings.TempOffset         = 0;
    m_Settings.MagAxis            = 2;
    m_Settings.NoPolePair         = 2;
    m_Settings.EnRPM              = false;
    m_Settings.RpmRange.Min       = 1;
    m_Settings.RpmRange.Max       = 100;

    /* Calculate CRC over all fields except CRC itself */
    m_Settings.Crc = UserConfig_CalculateCrc((const uint8_t *)&m_Settings,
                     sizeof(SystemSettings_t) - sizeof(m_Settings.Crc));
}

/*******************************************************************************
Description:
Log the current system settings to the console for debug purposes.

Argument(s):
None

Return:
None
*******************************************************************************/
void UserConfig_PrintSettings(void)
{
    LOG_INF("Settings: Magic=0x%04X, CRC=0x%04X",
            m_Settings.MagicNumber, m_Settings.Crc);
    LOG_INF("Settings: DInterval=%d, Interval=%d",
            m_Settings.DInterval, m_Settings.Interval);
    LOG_INF("Settings: HighSampleRate=%d, Duration=%d",
            m_Settings.HighSampleRate, m_Settings.HighSampleDuration);
    LOG_INF("Settings: LowerCutoff=%d, AccelRange=%d, TempOffset=%d",
            m_Settings.LowerCutoff, m_Settings.AccelRange, m_Settings.TempOffset);
    LOG_INF("Settings: PolePair=%d, MagAxis=%d, EnRPM=%d, RPM=%d-%d",
            m_Settings.NoPolePair, m_Settings.MagAxis, m_Settings.EnRPM,
            m_Settings.RpmRange.Min, m_Settings.RpmRange.Max);
}

/*******************************************************************************
Description:
Calculate a CRC-16 (MODBUS variant, polynomial 0xA001) over a byte buffer.
Used to verify integrity of settings stored in NVS.

Argument(s):
Data - Pointer to the data buffer.
Len  - Number of bytes to process.

Return:
uint16_t - Calculated CRC-16 value.
*******************************************************************************/
uint16_t UserConfig_CalculateCrc(const uint8_t *Data, size_t Len)
{
    uint16_t Crc = 0xFFFF;

    for (size_t i = 0; i < Len; i++)
    {
        Crc ^= Data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (Crc & 0x0001)
            {
                Crc = (Crc >> 1) ^ CRC16_POLY;
            }
            else
            {
                Crc = Crc >> 1;
            }
        }
    }

    return Crc;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Validate a settings structure by checking the magic number and CRC.
Magic must equal SETTINGS_MAGIC_NUMBER (0xDEAD). CRC is recalculated over
all fields except the CRC field and compared to the stored CRC.

Argument(s):
Settings - Pointer to the settings structure to validate.

Return:
bool - true if both magic number and CRC are valid, false otherwise.
*******************************************************************************/
static bool UserConfig_ValidateSettings(SystemSettings_t *Settings)
{
    uint16_t CalculatedCrc;

    /* Check magic number */
    if (Settings->MagicNumber != SETTINGS_MAGIC_NUMBER)
    {
        LOG_ERR("Magic number mismatch: expected 0x%04X, got 0x%04X",
                SETTINGS_MAGIC_NUMBER, Settings->MagicNumber);
        return false;
    }

    /* Calculate and compare CRC */
    CalculatedCrc = UserConfig_CalculateCrc((const uint8_t *)Settings,
                    sizeof(SystemSettings_t) - sizeof(Settings->Crc));

    if (CalculatedCrc != Settings->Crc)
    {
        LOG_ERR("CRC mismatch: expected 0x%04X, got 0x%04X",
                CalculatedCrc, Settings->Crc);
        return false;
    }

    return true;
}
