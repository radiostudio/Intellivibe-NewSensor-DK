/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the LTC3335 coulomb counter driver.
Communicates via I2C at 400 kHz (fast mode). Reads accumulated charge
register on each acquisition cycle and calculates remaining battery
percentage.

Part: Analog Devices LTC3335 (retained from old firmware)

Features:
  - I2C init and prescaler configuration
  - Accumulated charge read
  - Battery percentage calculation:
    ((BATT_FULL - accumulated_charge_Ah) / BATT_FULL) * 100
  - Counter reset API for battery replacement

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BATTERY, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "Battery.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define BATTERY_I2C_NODE    DT_NODELABEL(temp_i2c)

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device *m_I2cDev;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int Battery_ReadRegister(uint8_t Reg, uint8_t *Data, uint8_t Len);
static int Battery_WriteRegister(uint8_t Reg, uint8_t Value);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the LTC3335 coulomb counter. Gets the I2C device from
devicetree, configures it for fast mode, and writes the prescaler
register for the target battery capacity and peak current.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Battery_Init(void)
{
    int Ret;
    uint8_t ReadBack;

    m_I2cDev = DEVICE_DT_GET(BATTERY_I2C_NODE);
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

    /* Set prescaler for 1.2 Ah battery, 250 mA peak */
    Ret = Battery_WriteRegister(BATTERY_REG_PRESCALER, BATTERY_PRESCALER_VALUE);
    if (Ret)
    {
        LOG_ERR("Prescaler write failed: %d", Ret);
        return -1;
    }

    /* Read back to verify */
    Ret = Battery_ReadRegister(BATTERY_REG_PRESCALER, &ReadBack, 1);
    if (Ret)
    {
        LOG_ERR("Prescaler read-back failed: %d", Ret);
        return -1;
    }

    LOG_INF("LTC3335 initialised (prescaler: 0x%02X)", ReadBack);
    return 0;
}

/*******************************************************************************
Description:
Read the accumulated charge register and calculate remaining battery
percentage. Formula:
  charge_Ah = accumulated_count * QLSB / (2^prescaler)
  percent = ((FULL - charge_Ah) / FULL) * 100

Argument(s):
Percent - Pointer to store the battery percentage (0-100).

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Battery_Read(uint8_t *Percent)
{
    int Ret;
    uint8_t ChargeCount;
    float ChargeAh;
    float Result;

    Ret = Battery_ReadRegister(BATTERY_REG_ACC_CHARGE, &ChargeCount, 1);
    if (Ret)
    {
        LOG_ERR("Charge read failed: %d", Ret);
        return Ret;
    }

    /* Calculate consumed charge in Ah */
    ChargeAh = ((float)ChargeCount * BATTERY_QLSB_UAH) /
               (float)(1 << BATTERY_PRESCALER_VALUE) / 1000000.0f;

    /* Calculate remaining percentage */
    Result = ((BATTERY_FULL_CAPACITY_AH - ChargeAh) / BATTERY_FULL_CAPACITY_AH) * 100.0f;

    /* Clamp to 0-100 */
    if (Result < 0.0f)
    {
        Result = 0.0f;
    }
    if (Result > 100.0f)
    {
        Result = 100.0f;
    }

    *Percent = (uint8_t)Result;

    LOG_DBG("Battery: count=%d, charge=%.4f Ah, percent=%d%%",
            ChargeCount, (double)ChargeAh, *Percent);
    return 0;
}

/*******************************************************************************
Description:
Reset the LTC3335 accumulated charge counter. Should be called when the
battery is replaced to start tracking from full capacity.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Battery_ResetCounter(void)
{
    int Ret;

    Ret = Battery_WriteRegister(BATTERY_REG_INTERRUPT, BATTERY_CLEAR_INTERRUPT);
    if (Ret)
    {
        LOG_ERR("Counter reset failed: %d", Ret);
        return Ret;
    }

    LOG_INF("Coulomb counter reset");
    return 0;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Read one or more bytes from an LTC3335 register via I2C.

Argument(s):
Reg  - Register address to read from.
Data - Buffer to store the read data.
Len  - Number of bytes to read.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Battery_ReadRegister(uint8_t Reg, uint8_t *Data, uint8_t Len)
{
    return i2c_write_read(m_I2cDev, BATTERY_I2C_ADDR, &Reg, 1, Data, Len);
}

/*******************************************************************************
Description:
Write a single byte to an LTC3335 register via I2C.

Argument(s):
Reg   - Register address to write to.
Value - Byte value to write.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Battery_WriteRegister(uint8_t Reg, uint8_t Value)
{
    uint8_t Buf[2];

    Buf[0] = Reg;
    Buf[1] = Value;

    return i2c_write(m_I2cDev, Buf, sizeof(Buf), BATTERY_I2C_ADDR);
}
