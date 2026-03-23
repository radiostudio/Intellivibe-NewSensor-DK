/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the TMP112 temperature sensor driver.
Communicates via I2C in one-shot conversion mode. The sensor is kept in
shutdown mode between reads to minimise power consumption.

Driver API:
  Temp_Init()          - Init I2C bus, put sensor in shutdown mode
  Temp_Read()          - Trigger one-shot, wait, read temperature (deg C)
  Temp_SetOffset()     - Set calibration offset (applied on Temp_GetScaledValue)
  Temp_GetScaledValue()- Return (temp + offset) * 100 for BLE advertising

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(TEMP, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "Temp.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define TEMP_I2C_NODE   DT_NODELABEL(temp_i2c)

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device *m_I2cDev;
static float  m_LastTemperature;
static int8_t m_Offset;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int Temp_ReadRegister(uint8_t Reg, uint8_t *Buf, uint8_t Len);
static int Temp_WriteRegister(uint8_t Reg, uint8_t *Buf, uint8_t Len);
static int Temp_EnterShutdown(void);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the TMP112 temperature sensor. Gets the I2C device from
devicetree, configures it for fast mode (400 kHz), and puts the sensor
into shutdown/low-power mode.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Temp_Init(void)
{
    int Ret;

    m_I2cDev = DEVICE_DT_GET(TEMP_I2C_NODE);
    if (!device_is_ready(m_I2cDev))
    {
        LOG_ERR("I2C device not ready");
        return -1;
    }

    Ret = i2c_configure(m_I2cDev,
                         I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER);
    if (Ret)
    {
        LOG_ERR("I2C configure failed: %d", Ret);
        return -1;
    }

    /* Start in shutdown mode to save power */
    Ret = Temp_EnterShutdown();
    if (Ret)
    {
        LOG_ERR("TMP112 shutdown mode failed: %d", Ret);
        return -1;
    }

    m_LastTemperature = 0.0f;
    m_Offset = 0;

    LOG_INF("TMP112 initialised (shutdown mode)");
    return 0;
}

/*******************************************************************************
Description:
Trigger a one-shot temperature conversion, wait for completion, and read
the result. The sensor returns to shutdown mode automatically after the
one-shot conversion.

Argument(s):
Temperature - Pointer to store the temperature result in degrees Celsius.

Return:
int - 0 on success, negative error code on failure.

Note(s):
Conversion time is approximately 30 ms per TMP112 datasheet.
*******************************************************************************/
int Temp_Read(float *Temperature)
{
    int Ret;
    int16_t RawReading;
    uint8_t RegBuf[2];

    /* Read current config */
    Ret = Temp_ReadRegister(TEMP_REG_CONFIG, RegBuf, 1);
    if (Ret)
    {
        LOG_ERR("Config read failed: %d", Ret);
        return Ret;
    }

    /* Trigger one-shot conversion */
    RegBuf[0] |= TEMP_CFG_ONESHOT;
    Ret = Temp_WriteRegister(TEMP_REG_CONFIG, RegBuf, 1);
    if (Ret)
    {
        LOG_ERR("One-shot trigger failed: %d", Ret);
        return Ret;
    }

    /* Wait for conversion to complete */
    k_msleep(TEMP_CONV_WAIT_MS);

    /* Read temperature register (2 bytes, MSB first) */
    Ret = Temp_ReadRegister(TEMP_REG_TEMPERATURE, RegBuf, 2);
    if (Ret)
    {
        LOG_ERR("Temperature read failed: %d", Ret);
        return Ret;
    }

    /* Convert raw reading: 12-bit, left-justified in 16-bit register */
    RawReading = (int16_t)(RegBuf[0] << 8 | RegBuf[1]);
    RawReading >>= 4;

    /* Handle two's complement for negative temperatures */
    if (RawReading >= (1 << 11))
    {
        RawReading -= (1 << 12);
    }

    *Temperature = (float)(RawReading * TEMP_CONV_FACTOR);
    m_LastTemperature = *Temperature;

    LOG_DBG("Temperature: %.2f C", *Temperature);
    return 0;
}

/*******************************************************************************
Description:
Set the temperature calibration offset. This offset is applied when
Temp_GetScaledValue() is called for BLE advertising.

Argument(s):
Offset - Temperature offset in degrees Celsius (signed).

Return:
None
*******************************************************************************/
void Temp_SetOffset(int8_t Offset)
{
    m_Offset = Offset;
}

/*******************************************************************************
Description:
Return the last temperature reading with offset applied, scaled for BLE
advertising. Formula: (temperature + offset) * 100, giving 2 decimal
places of precision in a uint16-compatible integer.

Argument(s):
None

Return:
int - Scaled temperature value. Example: 25.50 C → 2550.
*******************************************************************************/
int Temp_GetScaledValue(void)
{
    return (int)((m_LastTemperature + m_Offset) * 100);
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Read one or more bytes from a TMP112 register via I2C.

Argument(s):
Reg - Register address to read from.
Buf - Buffer to store the read data.
Len - Number of bytes to read.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Temp_ReadRegister(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    struct i2c_msg Msg[2];

    Msg[0].buf   = &Reg;
    Msg[0].len   = 1;
    Msg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    Msg[1].buf   = Buf;
    Msg[1].len   = Len;
    Msg[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(m_I2cDev, Msg, 2, TEMP_I2C_ADDR);
}

/*******************************************************************************
Description:
Write one or more bytes to a TMP112 register via I2C.

Argument(s):
Reg - Register address to write to.
Buf - Buffer containing the data to write.
Len - Number of bytes to write.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Temp_WriteRegister(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    struct i2c_msg Msg[2];

    Msg[0].buf   = &Reg;
    Msg[0].len   = 1;
    Msg[0].flags = I2C_MSG_WRITE;

    Msg[1].buf   = Buf;
    Msg[1].len   = Len;
    Msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer(m_I2cDev, Msg, 2, TEMP_I2C_ADDR);
}

/*******************************************************************************
Description:
Put the TMP112 into shutdown mode. In this mode the sensor draws minimal
current and only performs conversions when triggered via one-shot.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Temp_EnterShutdown(void)
{
    int Ret;
    uint8_t RegBuf[2];

    Ret = Temp_ReadRegister(TEMP_REG_CONFIG, RegBuf, 2);
    if (Ret)
    {
        return Ret;
    }

    RegBuf[0] |= TEMP_CFG_SHUTDOWN;
    return Temp_WriteRegister(TEMP_REG_CONFIG, RegBuf, 2);
}
