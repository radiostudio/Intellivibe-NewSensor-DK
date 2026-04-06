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
  - 24-bit resolution (3 bytes per axis, full signed range)

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
#include <errno.h>
#include <math.h>
#include "Mag.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define MAG_I2C_NODE        DT_NODELABEL(temp_i2c)
#define MAG_INT_NODE        DT_ALIAS(sw1)

#define MAG_RESET_WAIT_MS   30
#define MAG_PMU_CMD_WAIT_MS 38                      /* BMM350_SUSPEND_TO_NORMAL_DELAY = 38000 µs */
#define MAG_UPD_OAE_WAIT_MS 1                       /* BMM350_UPD_OAE_DELAY = 1000 µs */
#define MAG_MAX_RATE_HZ     400                     /* BMM350 maximum supported ODR */

/* OTP calibration read */
#define MAG_OTP_NUM_WORDS           32
#define MAG_OTP_CMD_READ            0x20            /* Direct read command prefix */
#define MAG_OTP_ADDR_MASK           0x1F
#define MAG_OTP_STATUS_DONE         0x01
#define MAG_OTP_STATUS_ERR_MASK     0xE0
#define MAG_OTP_POLL_MS             1
/* OTP data word indices */
#define MAG_OTP_IDX_TEMP_OFF_SENS   0x0D
#define MAG_OTP_IDX_OFF_X           0x0E
#define MAG_OTP_IDX_OFF_Y           0x0F
#define MAG_OTP_IDX_OFF_Z_SENS_X    0x10
#define MAG_OTP_IDX_SENS_YZ         0x11
#define MAG_OTP_IDX_TCO_X_TCS_X     0x12
#define MAG_OTP_IDX_TCO_Y_TCS_Y     0x13
#define MAG_OTP_IDX_TCO_Z_TCS_Z     0x14
#define MAG_OTP_IDX_CROSS_XY_YX     0x15
#define MAG_OTP_IDX_CROSS_ZX_ZY     0x16
#define MAG_OTP_IDX_DUT_T0          0x18
/* Temperature default conversion: 1/(0.00204*(1/1.5)*0.714607*1048576) degC/LSB */
#define MAG_TEMP_DEGC_PER_LSB       0.000981f
/* Fixed temperature offset applied after lsb_to_degC, before compensation (Bosch line 1771) */
#define MAG_TEMP_RAW_OFFSET_DEGC    25.49f
#define MAG_DUMMY_BYTES     2                           /* BMM350 always prepends 2 dummy bytes on I2C reads */
#define MAG_I2C_RX_BUF_SIZE (MAG_DATA_READ_SIZE + MAG_DUMMY_BYTES)

/******************************************************************************
// Data types
******************************************************************************/
typedef struct
{
    float OffX,  OffY,  OffZ;          /* Magnetic offset (uT, 12-bit signed from OTP) */
    float SensX, SensY, SensZ;         /* Sensitivity correction (unitless, /256) */
    float TcoX,  TcoY,  TcoZ;          /* Temp coeff of offset (uT/degC, /32) */
    float TcsX,  TcsY,  TcsZ;          /* Temp coeff of sensitivity (1/degC, /16384) */
    float DutT0;                        /* Reference temperature (degC, /512 + 23) */
    float CrossXY, CrossYX;            /* Cross-axis XY/YX (unitless, /800) */
    float CrossZX, CrossZY;            /* Cross-axis ZX/ZY (unitless, /800) */
    float TOff;                         /* Temperature offset correction (/5) */
    float TSens;                        /* Temperature sensitivity correction (/512) */
} MagCalComp_t;

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device  *m_I2cDev;
static MagCalComp_t          m_MagCal;
static const struct gpio_dt_spec m_IntPin = GPIO_DT_SPEC_GET_OR(MAG_INT_NODE, gpios, {0});
static struct gpio_callback  m_IntCbData;

static K_SEM_DEFINE(m_DataReadySem, 0, 1);

/* Private temperature buffer — raw TValue per sample, used only for OTP compensation
   inside Mag_CalculateRms. Kept here so MagRawData_t stays XYZ-only (public/streaming safe). */
static int32_t m_TempBuf[MAG_RAW_BUFFER_LEN];

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int  Mag_ReadRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Mag_WriteRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Mag_ReadChipId(void);
static int  Mag_SoftReset(void);
static int  Mag_SetPmuCmd(uint8_t Cmd);
static int  Mag_ConfigureInterrupt(void);
static int32_t Mag_ConvertRaw24Bit(uint8_t Xlsb, uint8_t Lsb, uint8_t Msb);
static int32_t Mag_FixSign(uint32_t InVal, uint8_t NBits);
static int  Mag_ReadOtpWord(uint8_t Addr, uint16_t *Word);
static int  Mag_LoadCalibration(void);
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

    /* Read OTP calibration coefficients (must happen before OTP power-off) */
    Ret = Mag_LoadCalibration();
    if (Ret)
    {
        LOG_WRN("OTP calibration load failed: %d — using uncalibrated values", Ret);
    }

    /* Power off OTP — completes boot sequence and enables measurements */
    RegVal = MAG_OTP_CMD_PWR_OFF;
    Ret = Mag_WriteRegister(MAG_REG_OTP_CMD, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("OTP power off failed: %d", Ret);
        return -1;
    }

    /* Set ODR and averaging: 400 Hz, no averaging */
    RegVal = MAG_AVG_NO_AVG | MAG_ODR_400HZ;
    Ret = Mag_WriteRegister(MAG_REG_PMU_CMD_AGGR_SET, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("ODR config failed: %d", Ret);
        return -1;
    }

    /* Commit ODR/AVG settings — BMM350 requires UPD_OAE after writing AGGR_SET */
    RegVal = MAG_PMU_CMD_UPD_OAE;
    Ret = Mag_WriteRegister(MAG_REG_PMU_CMD, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("UPD_OAE failed: %d", Ret);
        return -1;
    }
    k_msleep(MAG_UPD_OAE_WAIT_MS);

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

    /* Configure interrupt GPIO — called after sensor is fully initialised and set */
    Ret = Mag_GpioInit();
    if (Ret)
    {
        LOG_ERR("GPIO init failed: %d", Ret);
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
    if (Config->SamplingRate > MAG_MAX_RATE_HZ)
    {
        TotalSamples = Config->Duration * MAG_MAX_RATE_HZ;
        LOG_WRN("ODR capped to %d Hz (requested %d Hz)", MAG_MAX_RATE_HZ, Config->SamplingRate);
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

        /* Read all 3 axes + temperature (12 bytes: 3 bytes per channel) */
        Ret = Mag_ReadRegister(MAG_REG_MAG_X_XLSB, DataBuf, MAG_DATA_READ_SIZE);
        if (Ret)
        {
            continue;
        }

        /* Convert 24-bit raw values — temperature stored in private m_TempBuf, not in public struct */
        Data[SampleCount].XValue  = Mag_ConvertRaw24Bit(DataBuf[0], DataBuf[1], DataBuf[2]);
        Data[SampleCount].YValue  = Mag_ConvertRaw24Bit(DataBuf[3], DataBuf[4], DataBuf[5]);
        Data[SampleCount].ZValue  = Mag_ConvertRaw24Bit(DataBuf[6], DataBuf[7], DataBuf[8]);
        m_TempBuf[SampleCount]    = Mag_ConvertRaw24Bit(DataBuf[9], DataBuf[10], DataBuf[11]);

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

    Data->XValue   = Mag_ConvertRaw24Bit(DataBuf[0], DataBuf[1], DataBuf[2]);
    Data->YValue   = Mag_ConvertRaw24Bit(DataBuf[3], DataBuf[4], DataBuf[5]);
    Data->ZValue   = Mag_ConvertRaw24Bit(DataBuf[6], DataBuf[7], DataBuf[8]);
    m_TempBuf[0]   = Mag_ConvertRaw24Bit(DataBuf[9], DataBuf[10], DataBuf[11]);

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
    uint8_t OdrReg;

    OdrReg = Mag_ConvertOdr(RateHz);
    RegVal = MAG_AVG_NO_AVG | OdrReg;
    Mag_WriteRegister(MAG_REG_PMU_CMD_AGGR_SET, &RegVal, 1);

    /* Commit new ODR — BMM350 requires UPD_OAE after every AGGR_SET write */
    RegVal = MAG_PMU_CMD_UPD_OAE;
    Mag_WriteRegister(MAG_REG_PMU_CMD, &RegVal, 1);
    k_msleep(MAG_UPD_OAE_WAIT_MS);

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
Calculate the RMS of magnetometer data per axis. Applies the full OTP-based
Bosch calibration pipeline per sample before accumulating into RMS:
  1. lsb_to_uT / lsb_to_degC conversion
  2. Fixed temperature raw offset (-25.49 degC)
  3. Temperature compensation (TSens, TOff)
  4. Per-axis sensitivity and offset correction (SensX/Y/Z, OffX/Y/Z)
  5. TCO / TCS temperature drift correction (Tdelta)
  6. Cross-axis compensation
  7. RMS = sqrt(sum(val^2) / N)

Argument(s):
Data        - Pointer to array of MagRawData_t samples (XYZ + TValue).
SampleCount - Number of samples in the buffer.
MagRms      - Output: per-axis RMS in compensated microtesla.

Return:
None
*******************************************************************************/
void Mag_CalculateRms(MagRawData_t *Data, uint16_t SampleCount, OutputData_t *MagRms)
{
    float    SumX = 0.0f;
    float    SumY = 0.0f;
    float    SumZ = 0.0f;
    float    X;
    float    Y;
    float    Z;
    float    T;
    float    Tdelta;
    float    CrossDenom;
    float    CompX;
    float    CompY;
    float    CompZ;
    float    InvN;
    uint16_t Idx;

    CrossDenom = 1.0f - m_MagCal.CrossYX * m_MagCal.CrossXY;

    for (Idx = 0; Idx < SampleCount; Idx++)
    {
        /* Step 1: lsb_to_uT / lsb_to_degC */
        X = (float)Data[Idx].XValue * MAG_UT_PER_LSB_XY;
        Y = (float)Data[Idx].YValue * MAG_UT_PER_LSB_XY;
        Z = (float)Data[Idx].ZValue * MAG_UT_PER_LSB_Z;
        T = (float)m_TempBuf[Idx] * MAG_TEMP_DEGC_PER_LSB;

        /* Step 2: Fixed raw temperature offset */
        T -= MAG_TEMP_RAW_OFFSET_DEGC;

        /* Step 3: Temperature compensation */
        T = (1.0f + m_MagCal.TSens) * T + m_MagCal.TOff;

        /* Step 4: Per-axis sensitivity and offset */
        X = X * (1.0f + m_MagCal.SensX) + m_MagCal.OffX;
        Y = Y * (1.0f + m_MagCal.SensY) + m_MagCal.OffY;
        Z = Z * (1.0f + m_MagCal.SensZ) + m_MagCal.OffZ;

        /* Step 5: TCO / TCS temperature drift */
        Tdelta = T - m_MagCal.DutT0;
        X = (X + m_MagCal.TcoX * Tdelta) / (1.0f + m_MagCal.TcsX * Tdelta);
        Y = (Y + m_MagCal.TcoY * Tdelta) / (1.0f + m_MagCal.TcsY * Tdelta);
        Z = (Z + m_MagCal.TcoZ * Tdelta) / (1.0f + m_MagCal.TcsZ * Tdelta);

        /* Step 6: Cross-axis compensation */
        CompX = (X - m_MagCal.CrossXY * Y) / CrossDenom;
        CompY = (Y - m_MagCal.CrossYX * X) / CrossDenom;
        CompZ = Z + (X * (m_MagCal.CrossYX * m_MagCal.CrossZY - m_MagCal.CrossZX) -
                     Y * (m_MagCal.CrossZY  - m_MagCal.CrossXY * m_MagCal.CrossZX)) / CrossDenom;

        SumX += CompX * CompX;
        SumY += CompY * CompY;
        SumZ += CompZ * CompZ;
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
    /* BMM350 always prepends 2 dummy bytes on I2C reads — read Len+2, skip first 2 */
    uint8_t RxBuf[MAG_I2C_RX_BUF_SIZE];
    int Ret;

    Ret = i2c_write_read(m_I2cDev, MAG_I2C_ADDR, &Reg, 1, RxBuf, Len + MAG_DUMMY_BYTES);
    if (Ret == 0)
    {
        memcpy(Data, &RxBuf[MAG_DUMMY_BYTES], Len);
    }

    return Ret;
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
    return i2c_burst_write(m_I2cDev, MAG_I2C_ADDR, Reg, Data, Len);
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
Convert 3 raw bytes from the BMM350 data registers into a signed 24-bit
integer value, sign-extended to 32 bits.

Argument(s):
Xlsb - Extra LSB byte (bits [7:0]).
Lsb  - Middle byte.
Msb  - MSB byte (bit 7 is the sign bit).

Return:
int32_t - Signed 24-bit value sign-extended to 32-bit.
*******************************************************************************/
static int32_t Mag_ConvertRaw24Bit(uint8_t Xlsb, uint8_t Lsb, uint8_t Msb)
{
    int32_t RawVal;

    RawVal = ((int32_t)Msb << 16) | ((int32_t)Lsb << 8) | (int32_t)Xlsb;

    /* Sign-extend from bit 23 — BMM350 uses full 24-bit signed value (no right-shift) */
    if (RawVal & (1U << 23))
    {
        RawVal |= (int32_t)0xFF000000;
    }

    return RawVal;
}

/*******************************************************************************
Description:
Sign-extend an unsigned value from NBits to 32-bit signed.
Equivalent to Bosch SensorAPI fix_sign().

Argument(s):
InVal - Unsigned value containing the signed data in its lower NBits.
NBits - Number of bits used by the signed value (e.g. 8, 12, 16, 24).

Return:
int32_t - Sign-extended 32-bit signed integer.
*******************************************************************************/
static int32_t Mag_FixSign(uint32_t InVal, uint8_t NBits)
{
    if (InVal & (1u << (NBits - 1u)))
    {
        InVal |= ~((1u << NBits) - 1u);
    }

    return (int32_t)InVal;
}

/*******************************************************************************
Description:
Read one 16-bit word from the BMM350 OTP memory at the given address.
Sends the direct-read command, polls until done, then reads MSB+LSB.

Argument(s):
Addr - OTP word address (0x00..0x1F).
Word - Output pointer for the 16-bit OTP word (MSB<<8 | LSB).

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_ReadOtpWord(uint8_t Addr, uint16_t *Word)
{
    uint8_t Cmd = MAG_OTP_CMD_READ | (Addr & MAG_OTP_ADDR_MASK);
    uint8_t Status;
    uint8_t OtpBuf[2];
    int     Ret;
    int     Timeout = 20;

    Ret = Mag_WriteRegister(MAG_REG_OTP_CMD, &Cmd, 1);
    if (Ret)
    {
        return Ret;
    }

    do
    {
        k_msleep(MAG_OTP_POLL_MS);
        Ret = Mag_ReadRegister(MAG_REG_OTP_STATUS, &Status, 1);
        if (Ret)
        {
            return Ret;
        }
        if (Status & MAG_OTP_STATUS_ERR_MASK)
        {
            return -EIO;
        }
    } while (!(Status & MAG_OTP_STATUS_DONE) && --Timeout > 0);

    if (Timeout == 0)
    {
        return -ETIMEDOUT;
    }

    /* Read MSB and LSB in one burst */
    Ret = Mag_ReadRegister(MAG_REG_OTP_DATA_MSB, OtpBuf, 2);
    if (Ret)
    {
        return Ret;
    }

    *Word = ((uint16_t)OtpBuf[0] << 8) | OtpBuf[1];
    return 0;
}

/*******************************************************************************
Description:
Read all 32 OTP words and extract the per-chip calibration coefficients
into m_MagCal. Mirrors Bosch SensorAPI otp_dump_after_boot() +
update_mag_off_sens(). Must be called before OTP power-off.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Mag_LoadCalibration(void)
{
    uint16_t OtpData[MAG_OTP_NUM_WORDS];
    uint16_t OffXWord, OffYWord, OffZWord;
    uint8_t  Idx;
    int      Ret;

    for (Idx = 0; Idx < MAG_OTP_NUM_WORDS; Idx++)
    {
        Ret = Mag_ReadOtpWord(Idx, &OtpData[Idx]);
        if (Ret)
        {
            LOG_ERR("OTP read failed at word %d: %d", Idx, Ret);
            return Ret;
        }
    }

    /* Offsets — 12-bit signed (packed across two OTP words each) */
    OffXWord = OtpData[MAG_OTP_IDX_OFF_X] & 0x0FFF;
    OffYWord = ((OtpData[MAG_OTP_IDX_OFF_X] & 0xF000) >> 4) +
               (OtpData[MAG_OTP_IDX_OFF_Y] & 0x00FF);
    OffZWord = (OtpData[MAG_OTP_IDX_OFF_Y] & 0x0F00) +
               (OtpData[MAG_OTP_IDX_OFF_Z_SENS_X] & 0x00FF);
    m_MagCal.OffX = (float)Mag_FixSign(OffXWord, 12) * MAG_UT_PER_LSB_XY;
    m_MagCal.OffY = (float)Mag_FixSign(OffYWord, 12) * MAG_UT_PER_LSB_XY;
    m_MagCal.OffZ = (float)Mag_FixSign(OffZWord, 12) * MAG_UT_PER_LSB_Z;

    /* Sensitivity — 8-bit signed, divide by 256 */
    m_MagCal.SensX = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_OFF_Z_SENS_X] & 0xFF00) >> 8, 8) / 256.0f;
    m_MagCal.SensY = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_SENS_YZ] & 0x00FF, 8)             / 256.0f;
    m_MagCal.SensZ = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_SENS_YZ] & 0xFF00) >> 8, 8) / 256.0f;

    /* TCO (temp coeff of offset) — 8-bit signed, divide by 32 */
    m_MagCal.TcoX = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_TCO_X_TCS_X] & 0x00FF, 8) / 32.0f;
    m_MagCal.TcoY = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_TCO_Y_TCS_Y] & 0x00FF, 8) / 32.0f;
    m_MagCal.TcoZ = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_TCO_Z_TCS_Z] & 0x00FF, 8) / 32.0f;

    /* TCS (temp coeff of sensitivity) — 8-bit signed, divide by 16384 */
    m_MagCal.TcsX = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_TCO_X_TCS_X] & 0xFF00) >> 8, 8) / 16384.0f;
    m_MagCal.TcsY = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_TCO_Y_TCS_Y] & 0xFF00) >> 8, 8) / 16384.0f;
    m_MagCal.TcsZ = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_TCO_Z_TCS_Z] & 0xFF00) >> 8, 8) / 16384.0f;

    /* Reference temperature — 16-bit signed, divide by 512, add 23 degC */
    m_MagCal.DutT0 = ((float)Mag_FixSign(OtpData[MAG_OTP_IDX_DUT_T0], 16) / 512.0f) + 23.0f;

    /* Cross-axis coefficients — 8-bit signed, divide by 800 */
    m_MagCal.CrossXY = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_CROSS_XY_YX] & 0x00FF, 8)       / 800.0f;
    m_MagCal.CrossYX = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_CROSS_XY_YX] & 0xFF00) >> 8, 8) / 800.0f;
    m_MagCal.CrossZX = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_CROSS_ZX_ZY] & 0x00FF, 8)        / 800.0f;
    m_MagCal.CrossZY = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_CROSS_ZX_ZY] & 0xFF00) >> 8, 8) / 800.0f;

    /* Temperature offset/sensitivity — 8-bit signed */
    m_MagCal.TOff  = (float)Mag_FixSign(OtpData[MAG_OTP_IDX_TEMP_OFF_SENS] & 0x00FF, 8)       / 5.0f;
    m_MagCal.TSens = (float)Mag_FixSign((OtpData[MAG_OTP_IDX_TEMP_OFF_SENS] & 0xFF00) >> 8, 8) / 512.0f;

    LOG_DBG("OTP cal: OffX=%.2f OffY=%.2f OffZ=%.2f",
            (double)m_MagCal.OffX, (double)m_MagCal.OffY, (double)m_MagCal.OffZ);
    LOG_DBG("OTP cal: SensX=%.4f SensY=%.4f SensZ=%.4f",
            (double)m_MagCal.SensX, (double)m_MagCal.SensY, (double)m_MagCal.SensZ);
    LOG_DBG("OTP cal: CrossXY=%.4f CrossYX=%.4f CrossZX=%.4f CrossZY=%.4f",
            (double)m_MagCal.CrossXY, (double)m_MagCal.CrossYX,
            (double)m_MagCal.CrossZX, (double)m_MagCal.CrossZY);
    LOG_DBG("OTP cal: TcoX=%.4f TcoY=%.4f TcoZ=%.4f DutT0=%.2f",
            (double)m_MagCal.TcoX, (double)m_MagCal.TcoY,
            (double)m_MagCal.TcoZ, (double)m_MagCal.DutT0);
    LOG_INF("OTP calibration loaded");
    return 0;
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
