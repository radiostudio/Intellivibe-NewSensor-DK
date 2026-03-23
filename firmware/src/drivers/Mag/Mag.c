/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the BMM350 3-axis magnetometer driver.
Communicates via I2C at 400 kHz (fast mode). Supports continuous measurement
with data-ready interrupt for batch sample collection.

Replaces: BM1422AGMV (old firmware)
Part:      Bosch BMM350

Features:
  - I2C init, read/write registers
  - Chip ID validation (0x33)
  - Configurable ODR (1.5625 Hz - 400 Hz)
  - Continuous mode with DRDY interrupt on INT pin
  - Suspend/normal/forced mode control
  - GPIO interrupt configuration
  - 21-bit resolution (3 bytes per axis)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAG, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <math.h>
#include "Mag.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define MAG_I2C_NODE        DT_NODELABEL(temp_i2c)
#define MAG_INT_NODE        DT_ALIAS(sw1)

#define MAG_RESET_WAIT_MS   10
#define MAG_STARTUP_MS      5
#define MAG_PMU_CMD_WAIT_MS 2

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device  *m_I2cDev;
static const struct gpio_dt_spec m_IntPin = GPIO_DT_SPEC_GET_OR(MAG_INT_NODE, gpios, {0});
static struct gpio_callback  m_IntCbData;

static K_SEM_DEFINE(m_DataReadySem, 0, 1);

static uint8_t m_CurrentOdr = MAG_ODR_400HZ;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int  Mag_ReadRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Mag_WriteRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Mag_ReadChipId(void);
static int  Mag_SoftReset(void);
static int  Mag_SetPmuCmd(uint8_t Cmd);
static int  Mag_ConfigureInterrupt(void);
static int32_t Mag_ConvertRaw21Bit(uint8_t Xlsb, uint8_t Lsb, uint8_t Msb);
static void Mag_IntHandler(const struct device *Dev, struct gpio_callback *Cb,
                           uint32_t Pins);
static uint8_t Mag_ConvertOdr(uint16_t RateHz);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the BMM350 magnetometer. Gets the I2C device from devicetree,
validates chip ID, performs soft reset, configures ODR, enables all axes,
configures DRDY interrupt, and leaves the sensor in suspend mode.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Mag_Init(void)
{
    int Ret;
    uint8_t RegVal;

    /* Get I2C device */
    m_I2cDev = DEVICE_DT_GET(MAG_I2C_NODE);
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

    /* Configure interrupt GPIO */
    Ret = Mag_GpioInit();
    if (Ret)
    {
        LOG_ERR("GPIO init failed: %d", Ret);
        return -1;
    }

    /* Validate chip ID */
    Ret = Mag_ReadChipId();
    if (Ret)
    {
        LOG_ERR("Chip ID check failed");
        return -1;
    }

    /* Soft reset */
    Ret = Mag_SoftReset();
    if (Ret)
    {
        LOG_ERR("Soft reset failed: %d", Ret);
        return -1;
    }

    /* Set ODR and averaging: 400 Hz, no averaging */
    RegVal = MAG_AVG_NO_AVG | MAG_ODR_400HZ;
    m_CurrentOdr = MAG_ODR_400HZ;
    Ret = Mag_WriteRegister(MAG_REG_PMU_CMD_AGGR_SET, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("ODR config failed: %d", Ret);
        return -1;
    }

    /* Enable all three axes */
    RegVal = MAG_AXIS_EN_ALL;
    Ret = Mag_WriteRegister(MAG_REG_PMU_CMD_AXIS_EN, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("Axis enable failed: %d", Ret);
        return -1;
    }

    /* Configure DRDY interrupt */
    Ret = Mag_ConfigureInterrupt();
    if (Ret)
    {
        LOG_ERR("Interrupt config failed: %d", Ret);
        return -1;
    }

    /* Leave in suspend mode */
    Ret = Mag_Standby();
    if (Ret)
    {
        LOG_ERR("Suspend mode failed: %d", Ret);
        return -1;
    }

    LOG_INF("BMM350 initialised (suspend mode)");
    return 0;
}

/*******************************************************************************
Description:
Read magnetometer data via DRDY interrupt for a given duration and sampling
rate. Activates the sensor in normal (continuous) mode, collects samples one
per DRDY interrupt, then returns to suspend mode.

Argument(s):
Data   - Buffer to store raw magnetometer samples. Must be large enough
         for Config->Duration * Config->SamplingRate samples.
Config - Pointer to acquisition config (sampling rate and duration).

Return:
int - 0 on success, negative error code on failure.

Note(s):
BMM350 max ODR is 400 Hz. If Config->SamplingRate exceeds this, actual
rate will be capped at 400 Hz.
*******************************************************************************/
int Mag_ReadData(MagRawData_t *Data, MagConfig_t *Config)
{
    int Ret;
    uint16_t SampleCount = 0;
    uint16_t TotalSamples = Config->Duration * Config->SamplingRate;
    uint8_t DataBuf[MAG_DATA_READ_SIZE];

    /* Cap at max supported ODR */
    if (Config->SamplingRate > 400)
    {
        TotalSamples = Config->Duration * 400;
        LOG_WRN("ODR capped to 400 Hz (requested %d Hz)", Config->SamplingRate);
    }

    /* Activate sensor in normal (continuous) mode */
    Ret = Mag_Active();
    if (Ret)
    {
        return Ret;
    }

    while (SampleCount < TotalSamples)
    {
        /* Wait for DRDY interrupt */
        k_sem_take(&m_DataReadySem, K_FOREVER);

        /* Read all 3 axes (9 bytes: 3 bytes per axis) */
        Ret = Mag_ReadRegister(MAG_REG_MAG_X_XLSB, DataBuf, MAG_DATA_READ_SIZE);
        if (Ret)
        {
            continue;
        }

        /* Convert 21-bit raw values */
        Data[SampleCount].XValue = Mag_ConvertRaw21Bit(DataBuf[0], DataBuf[1], DataBuf[2]);
        Data[SampleCount].YValue = Mag_ConvertRaw21Bit(DataBuf[3], DataBuf[4], DataBuf[5]);
        Data[SampleCount].ZValue = Mag_ConvertRaw21Bit(DataBuf[6], DataBuf[7], DataBuf[8]);

        SampleCount++;
    }

    /* Return to suspend mode */
    Mag_Standby();

    LOG_DBG("Read %d mag samples", SampleCount);
    return 0;
}

/*******************************************************************************
Description:
Read a single raw magnetometer sample from the data registers.

Argument(s):
Data - Pointer to store the raw XYZ values.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Mag_ReadRaw(MagRawData_t *Data)
{
    int Ret;
    uint8_t DataBuf[MAG_DATA_READ_SIZE];

    Ret = Mag_ReadRegister(MAG_REG_MAG_X_XLSB, DataBuf, MAG_DATA_READ_SIZE);
    if (Ret)
    {
        return Ret;
    }

    Data->XValue = Mag_ConvertRaw21Bit(DataBuf[0], DataBuf[1], DataBuf[2]);
    Data->YValue = Mag_ConvertRaw21Bit(DataBuf[3], DataBuf[4], DataBuf[5]);
    Data->ZValue = Mag_ConvertRaw21Bit(DataBuf[6], DataBuf[7], DataBuf[8]);

    return 0;
}

/*******************************************************************************
Description:
Set the magnetometer output data rate. Writes to PMU_CMD_AGGR_SET register.
Sensor should be in suspend mode when calling this.

Argument(s):
RateHz - Desired ODR in Hz (e.g., 400, 200, 100, 50, 25).

Return:
None
*******************************************************************************/
void Mag_SetOdr(uint16_t RateHz)
{
    uint8_t RegVal;

    m_CurrentOdr = Mag_ConvertOdr(RateHz);
    RegVal = MAG_AVG_NO_AVG | m_CurrentOdr;
    Mag_WriteRegister(MAG_REG_PMU_CMD_AGGR_SET, &RegVal, 1);

    LOG_INF("ODR set to %d Hz", RateHz);
}

/*******************************************************************************
Description:
Put the BMM350 into normal (continuous measurement) mode.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Mag_Active(void)
{
    return Mag_SetPmuCmd(MAG_PMU_CMD_NORMAL);
}

/*******************************************************************************
Description:
Put the BMM350 into suspend mode (lowest power).

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Mag_Standby(void)
{
    return Mag_SetPmuCmd(MAG_PMU_CMD_SUSPEND);
}

/*******************************************************************************
Description:
Enable the data-ready interrupt on the magnetometer INT GPIO pin.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Mag_EnableInterrupt(void)
{
    return gpio_pin_interrupt_configure_dt(&m_IntPin, GPIO_INT_EDGE_RISING);
}

/*******************************************************************************
Description:
Disable the magnetometer interrupt on the INT GPIO pin.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Mag_DisableInterrupt(void)
{
    return gpio_pin_interrupt_configure_dt(&m_IntPin, GPIO_INT_DISABLE);
}

/*******************************************************************************
Description:
Configure the magnetometer interrupt GPIO pin. Sets it as input with
rising-edge interrupt and registers the callback.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
/*******************************************************************************
Description:
Calculate the RMS of magnetometer data per axis. Converts raw 21-bit values
to microtesla using MAG_UT_PER_LSB, then computes:
  RMS = sqrt(sum(val^2) / N)

Argument(s):
Data        - Pointer to array of MagRawData_t samples.
SampleCount - Number of samples in the buffer.
MagRms      - Output: per-axis RMS in microtesla.

Return:
None
*******************************************************************************/
void Mag_CalculateRms(MagRawData_t *Data, uint16_t SampleCount, OutputData_t *MagRms)
{
    float SumX = 0.0f;
    float SumY = 0.0f;
    float SumZ = 0.0f;
    float ValX;
    float ValY;
    float ValZ;
    float InvN;
    uint16_t Idx;

    for (Idx = 0; Idx < SampleCount; Idx++)
    {
        ValX = (float)Data[Idx].XValue * MAG_UT_PER_LSB;
        ValY = (float)Data[Idx].YValue * MAG_UT_PER_LSB;
        ValZ = (float)Data[Idx].ZValue * MAG_UT_PER_LSB;

        SumX += ValX * ValX;
        SumY += ValY * ValY;
        SumZ += ValZ * ValZ;
    }

    InvN = 1.0f / (float)SampleCount;

    MagRms->XData = sqrtf(SumX * InvN);
    MagRms->YData = sqrtf(SumY * InvN);
    MagRms->ZData = sqrtf(SumZ * InvN);
}

/*******************************************************************************
Description:
Configure the magnetometer interrupt GPIO pin. Sets it as input with
rising-edge interrupt and registers the callback.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Mag_GpioInit(void)
{
    int Ret;

    if (!gpio_is_ready_dt(&m_IntPin))
    {
        LOG_ERR("Interrupt GPIO not ready");
        return -1;
    }

    Ret = gpio_pin_configure_dt(&m_IntPin, GPIO_INPUT);
    if (Ret)
    {
        LOG_ERR("GPIO configure failed: %d", Ret);
        return -1;
    }

    Ret = gpio_pin_interrupt_configure_dt(&m_IntPin, GPIO_INT_EDGE_RISING);
    if (Ret)
    {
        LOG_ERR("Interrupt configure failed: %d", Ret);
        return -1;
    }

    gpio_init_callback(&m_IntCbData, Mag_IntHandler, BIT(m_IntPin.pin));
    gpio_add_callback(m_IntPin.port, &m_IntCbData);

    return 0;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Read one or more bytes from a BMM350 register via I2C.

Argument(s):
Reg  - Register address to read from.
Data - Buffer to store the read data.
Len  - Number of bytes to read.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_ReadRegister(uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    struct i2c_msg Msg[2];

    Msg[0].buf   = &Reg;
    Msg[0].len   = 1;
    Msg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    Msg[1].buf   = Data;
    Msg[1].len   = Len;
    Msg[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(m_I2cDev, Msg, 2, MAG_I2C_ADDR);
}

/*******************************************************************************
Description:
Write one or more bytes to a BMM350 register via I2C.

Argument(s):
Reg  - Register address to write to.
Data - Buffer containing the data to write.
Len  - Number of bytes to write.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_WriteRegister(uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    struct i2c_msg Msg[2];

    Msg[0].buf   = &Reg;
    Msg[0].len   = 1;
    Msg[0].flags = I2C_MSG_WRITE;

    Msg[1].buf   = Data;
    Msg[1].len   = Len;
    Msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer(m_I2cDev, Msg, 2, MAG_I2C_ADDR);
}

/*******************************************************************************
Description:
Read and validate the chip ID register. Expected value for BMM350 is 0x33.

Argument(s):
None

Return:
int - 0 on success, -1 if chip ID mismatch.
*******************************************************************************/
static int Mag_ReadChipId(void)
{
    uint8_t ChipId = 0;
    int Ret;

    Ret = Mag_ReadRegister(MAG_REG_CHIP_ID, &ChipId, 1);
    if (Ret)
    {
        LOG_ERR("Chip ID read failed: %d", Ret);
        return -1;
    }

    LOG_INF("Chip ID: 0x%02X", ChipId);

    if (ChipId != MAG_CHIP_ID_VAL)
    {
        LOG_ERR("Chip ID mismatch: expected 0x%02X, got 0x%02X",
                MAG_CHIP_ID_VAL, ChipId);
        return -1;
    }

    return 0;
}

/*******************************************************************************
Description:
Perform a soft reset of the BMM350. Writes the reset command and waits
for the sensor to restart.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_SoftReset(void)
{
    uint8_t RegVal = MAG_CMD_SOFT_RESET;
    int Ret;

    Ret = Mag_WriteRegister(MAG_REG_CMD, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    k_msleep(MAG_RESET_WAIT_MS);

    LOG_INF("Soft reset complete");
    return 0;
}

/*******************************************************************************
Description:
Send a power mode command to the BMM350 via the PMU_CMD register and
wait for it to take effect.

Argument(s):
Cmd - Power mode command (MAG_PMU_CMD_SUSPEND, MAG_PMU_CMD_NORMAL, etc.).

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_SetPmuCmd(uint8_t Cmd)
{
    int Ret;

    Ret = Mag_WriteRegister(MAG_REG_PMU_CMD, &Cmd, 1);
    if (Ret)
    {
        return Ret;
    }

    k_msleep(MAG_PMU_CMD_WAIT_MS);
    return 0;
}

/*******************************************************************************
Description:
Configure the BMM350 DRDY interrupt. Sets INT pin as active-high,
push-pull, pulsed, with data-ready interrupt enabled.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_ConfigureInterrupt(void)
{
    uint8_t RegVal;

    RegVal = MAG_INT_OUTPUT_EN | MAG_INT_POL_ACTIVE_HIGH |
             MAG_INT_OD_PUSHPULL | MAG_INT_DRDY_EN;

    return Mag_WriteRegister(MAG_REG_INT_CTRL, &RegVal, 1);
}

/*******************************************************************************
Description:
Convert 3 raw bytes from the BMM350 data registers into a signed 21-bit
integer value.

Argument(s):
Xlsb - Extra LSB byte (bits [4:0] are data).
Lsb  - LSB byte.
Msb  - MSB byte.

Return:
int32_t - Signed 21-bit value sign-extended to 32-bit.
*******************************************************************************/
static int32_t Mag_ConvertRaw21Bit(uint8_t Xlsb, uint8_t Lsb, uint8_t Msb)
{
    int32_t RawVal;

    RawVal = ((int32_t)Msb << 16) | ((int32_t)Lsb << 8) | (int32_t)Xlsb;

    /* Right-shift to get 21-bit value (data is left-justified in 24 bits) */
    RawVal >>= 3;

    /* Sign-extend from 21-bit */
    if (RawVal & (1 << 20))
    {
        RawVal |= ~((1 << 21) - 1);
    }

    return RawVal;
}

/*******************************************************************************
Description:
GPIO interrupt handler for magnetometer data-ready. Gives the
m_DataReadySem semaphore to wake the acquisition thread.

Argument(s):
Dev  - Pointer to the GPIO device.
Cb   - Pointer to the callback structure.
Pins - Bitmask of pins that triggered the interrupt.

Return:
None
*******************************************************************************/
static void Mag_IntHandler(const struct device *Dev, struct gpio_callback *Cb,
                           uint32_t Pins)
{
    ARG_UNUSED(Dev);
    ARG_UNUSED(Cb);
    ARG_UNUSED(Pins);

    k_sem_give(&m_DataReadySem);
}

/*******************************************************************************
Description:
Convert an ODR value in Hz to the BMM350 PMU_CMD_AGGR_SET register value.

Argument(s):
RateHz - Desired output data rate in Hz.

Return:
uint8_t - Register value for the ODR field.
*******************************************************************************/
static uint8_t Mag_ConvertOdr(uint16_t RateHz)
{
    if (RateHz >= 400)      return MAG_ODR_400HZ;
    if (RateHz >= 200)      return MAG_ODR_200HZ;
    if (RateHz >= 100)      return MAG_ODR_100HZ;
    if (RateHz >= 50)       return MAG_ODR_50HZ;
    if (RateHz >= 25)       return MAG_ODR_25HZ;
    if (RateHz >= 12)       return MAG_ODR_12_5HZ;
    if (RateHz >= 6)        return MAG_ODR_6_25HZ;
    if (RateHz >= 3)        return MAG_ODR_3_125HZ;

    return MAG_ODR_1_5625HZ;
}
