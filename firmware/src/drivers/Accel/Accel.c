/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the IIM-42352 3-axis accelerometer driver.
Communicates via SPI at 4 MHz. Supports FIFO batch reads for RMS acquisition
and continuous streaming for high-speed BLE transfer.

Replaces: KX132-1211 (old firmware)
Part:      TDK InvenSense IIM-42352

Features:
  - SPI init, read/write registers
  - WHO_AM_I validation
  - Configurable ODR (50 Hz - 32 kHz) and range (2/4/8/16 G)
  - FIFO batch read via watermark interrupt on INT1
  - Standby/active mode control
  - GPIO interrupt configuration
  - Sensitivity conversion (raw counts to g)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ACCEL, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include "Accel.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define ACCEL_SPI_NODE      DT_NODELABEL(accel_spi)
#define ACCEL_INT_NODE      DT_ALIAS(sw0)

#define ACCEL_SPI_FREQ      4000000U
#define ACCEL_SPI_OP        (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE)

#define ACCEL_RESET_WAIT_MS 1
#define ACCEL_STARTUP_MS    10

/* FIFO watermark: 86 samples * 8 bytes = 688 bytes (fits in 2KB FIFO) */
#define ACCEL_FIFO_WM_SAMPLES  86
#define ACCEL_FIFO_WM_BYTES    (ACCEL_FIFO_WM_SAMPLES * ACCEL_FIFO_PACKET_SIZE)

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device *m_SpiDev;

static const struct spi_cs_control m_SpiCs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_accel_spi)),
    .delay = 0,
};

static const struct spi_config m_SpiCfg = {
    .operation  = ACCEL_SPI_OP,
    .frequency  = ACCEL_SPI_FREQ,
    .slave      = 0,
    .cs         = m_SpiCs,
};

static const struct gpio_dt_spec m_IntPin = GPIO_DT_SPEC_GET_OR(ACCEL_INT_NODE, gpios, {0});
static struct gpio_callback m_IntCbData;

static K_SEM_DEFINE(m_DataReadySem, 0, 1);

static float   m_Sensitivity = ACCEL_SENSITIVITY_8G;
static uint8_t m_CurrentFsSel = ACCEL_FS_SEL_8G;
static uint8_t m_CurrentOdr   = ACCEL_ODR_4KHZ;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int  Accel_ReadRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Accel_WriteRegister(uint8_t Reg, uint8_t *Data, uint16_t Len);
static int  Accel_ReadWhoAmI(void);
static int  Accel_SoftReset(void);
static int  Accel_ConfigureFifo(void);
static int  Accel_ConfigureInterrupt(void);
static void Accel_IntHandler(const struct device *Dev, struct gpio_callback *Cb,
                             uint32_t Pins);
static uint8_t Accel_ConvertOdr(uint16_t RateHz);
static uint8_t Accel_ConvertRange(uint8_t Range);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the IIM-42352 accelerometer. Gets the SPI device from devicetree,
validates WHO_AM_I, performs soft reset, configures ODR, range, FIFO, and
interrupt. Leaves the sensor in standby mode.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Accel_Init(void)
{
    int Ret;

    /* Get SPI device */
    m_SpiDev = DEVICE_DT_GET(ACCEL_SPI_NODE);
    if (!device_is_ready(m_SpiDev))
    {
        LOG_ERR("SPI device not ready");
        return -1;
    }

    if (!device_is_ready(m_SpiCs.gpio.port))
    {
        LOG_ERR("SPI CS GPIO not ready");
        return -1;
    }

    /* Configure interrupt GPIO */
    Ret = Accel_GpioInit();
    if (Ret)
    {
        LOG_ERR("GPIO init failed: %d", Ret);
        return -1;
    }

    /* Validate WHO_AM_I */
    Ret = Accel_ReadWhoAmI();
    if (Ret)
    {
        LOG_ERR("WHO_AM_I check failed");
        return -1;
    }

    /* Soft reset */
    Ret = Accel_SoftReset();
    if (Ret)
    {
        LOG_ERR("Soft reset failed: %d", Ret);
        return -1;
    }

    /* Configure SPI mode: 4-wire SPI */
    uint8_t RegVal = 0x00;
    Ret = Accel_WriteRegister(ACCEL_REG_INTF_CONFIG0, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("INTF_CONFIG0 write failed: %d", Ret);
        return -1;
    }

    /* Set default ODR (4 kHz — closest to old 3200 Hz) and range (8G) */
    RegVal = m_CurrentFsSel | ACCEL_ODR_4KHZ;
    m_CurrentOdr = ACCEL_ODR_4KHZ;
    Ret = Accel_WriteRegister(ACCEL_REG_ACCEL_CONFIG0, &RegVal, 1);
    if (Ret)
    {
        LOG_ERR("ACCEL_CONFIG0 write failed: %d", Ret);
        return -1;
    }

    /* Configure FIFO */
    Ret = Accel_ConfigureFifo();
    if (Ret)
    {
        LOG_ERR("FIFO config failed: %d", Ret);
        return -1;
    }

    /* Configure INT1 for FIFO watermark */
    Ret = Accel_ConfigureInterrupt();
    if (Ret)
    {
        LOG_ERR("Interrupt config failed: %d", Ret);
        return -1;
    }

    m_Sensitivity = ACCEL_SENSITIVITY_8G;

    LOG_INF("IIM-42352 initialised (standby mode)");
    return 0;
}

/*******************************************************************************
Description:
Read accelerometer data via FIFO for a given duration and sampling rate.
Activates the sensor, collects samples via FIFO watermark interrupts,
then returns the sensor to standby.

Argument(s):
Data   - Buffer to store raw accelerometer samples. Must be large enough
         for Config->Duration * Config->SamplingRate samples.
Config - Pointer to acquisition config (sampling rate and duration).

Return:
int - 0 on success, negative error code on failure.

Note(s):
FIFO packets are 8 bytes: [header][Xh][Xl][Yh][Yl][Zh][Zl][temp].
Watermark is set to 86 samples. Interrupt fires when watermark reached.
*******************************************************************************/
int Accel_ReadData(AccelRawData_t *Data, AccelConfig_t *Config)
{
    int Ret;
    uint16_t SampleCount = 0;
    uint16_t TotalSamples = Config->Duration * Config->SamplingRate;
    uint8_t IntStatus;
    uint8_t FifoBuf[ACCEL_FIFO_WM_BYTES];
    uint16_t FifoCount;
    uint8_t FifoCountBuf[2];

    /* Activate sensor */
    Ret = Accel_Active();
    if (Ret)
    {
        return Ret;
    }

    while (SampleCount < TotalSamples)
    {
        /* Wait for FIFO watermark interrupt */
        k_sem_take(&m_DataReadySem, K_FOREVER);

        /* Read interrupt status to clear */
        Ret = Accel_ReadRegister(ACCEL_REG_INT_STATUS, &IntStatus, 1);
        if (Ret)
        {
            continue;
        }

        if (IntStatus & ACCEL_INT_STATUS_FIFO_THS)
        {
            /* Read FIFO count */
            Ret = Accel_ReadRegister(ACCEL_REG_FIFO_COUNTH, FifoCountBuf, 2);
            if (Ret)
            {
                continue;
            }
            /* FIFO count register returns bytes — convert to samples */
            FifoCount = (((uint16_t)FifoCountBuf[0] << 8) | FifoCountBuf[1])
                        / ACCEL_FIFO_PACKET_SIZE;

            /* Limit to watermark chunk */
            if (FifoCount > ACCEL_FIFO_WM_SAMPLES)
            {
                FifoCount = ACCEL_FIFO_WM_SAMPLES;
            }

            /* Read FIFO data */
            uint16_t ReadBytes = FifoCount * ACCEL_FIFO_PACKET_SIZE;
            Ret = Accel_ReadRegister(ACCEL_REG_FIFO_DATA, FifoBuf, ReadBytes);
            if (Ret)
            {
                continue;
            }

            /* Parse FIFO packets: [header][Xh][Xl][Yh][Yl][Zh][Zl][temp] */
            for (uint16_t i = 0; i < FifoCount && SampleCount < TotalSamples; i++)
            {
                uint16_t Offset = i * ACCEL_FIFO_PACKET_SIZE;

                /* IIM-42352 FIFO: LSB first (little-endian) per byte pair */
                Data[SampleCount].XValue = (int16_t)((FifoBuf[Offset + 2] << 8) |
                                                      FifoBuf[Offset + 1]);
                Data[SampleCount].YValue = (int16_t)((FifoBuf[Offset + 4] << 8) |
                                                      FifoBuf[Offset + 3]);
                Data[SampleCount].ZValue = (int16_t)((FifoBuf[Offset + 6] << 8) |
                                                      FifoBuf[Offset + 5]);


                SampleCount++;
            }
        }
    }

    /* Return to standby */
    Accel_Standby();
    return 0;
}

/*******************************************************************************
Description:
Read accelerometer data in streaming mode for high-speed BLE transfer.
Similar to Accel_ReadData but stores data into the streaming buffer.

Argument(s):
Data   - Buffer to store raw streaming samples.
Config - Pointer to acquisition config (sampling rate and duration).

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Accel_ReadStreamData(AccelRawData_t *Data, AccelConfig_t *Config)
{
    /* Streaming uses the same FIFO read mechanism */
    return Accel_ReadData(Data, Config);
}

/*******************************************************************************
Description:
Set the accelerometer output data rate. Puts sensor in standby, updates
the ODR field in ACCEL_CONFIG0, then returns to standby.

Argument(s):
RateHz - Desired ODR in Hz (e.g., 3200, 1600, 800, etc.).

Return:
None
*******************************************************************************/
void Accel_SetOdr(uint16_t RateHz)
{
    uint8_t RegVal;

    Accel_Standby();
    k_msleep(ACCEL_STARTUP_MS);

    m_CurrentOdr = Accel_ConvertOdr(RateHz);
    RegVal = m_CurrentFsSel | m_CurrentOdr;
    Accel_WriteRegister(ACCEL_REG_ACCEL_CONFIG0, &RegVal, 1);

    k_msleep(ACCEL_STARTUP_MS);

    LOG_INF("ODR set to %d Hz", RateHz);
}

/*******************************************************************************
Description:
Set the accelerometer full-scale range. Puts sensor in standby, updates
the FS_SEL field in ACCEL_CONFIG0, and updates the sensitivity factor.

Argument(s):
Range - Range code from CommonTypes.h (RANGE_2G through RANGE_16G).

Return:
None
*******************************************************************************/
void Accel_SetRange(uint8_t Range)
{
    uint8_t RegVal;

    Accel_Standby();
    k_msleep(ACCEL_STARTUP_MS);

    m_CurrentFsSel = Accel_ConvertRange(Range);

    switch (Range)
    {
    case RANGE_2G:
        m_Sensitivity = ACCEL_SENSITIVITY_2G;
        break;
    case RANGE_4G:
        m_Sensitivity = ACCEL_SENSITIVITY_4G;
        break;
    case RANGE_8G:
        m_Sensitivity = ACCEL_SENSITIVITY_8G;
        break;
    case RANGE_16G:
        m_Sensitivity = ACCEL_SENSITIVITY_16G;
        break;
    default:
        m_Sensitivity = ACCEL_SENSITIVITY_8G;
        m_CurrentFsSel = ACCEL_FS_SEL_8G;
        break;
    }

    RegVal = m_CurrentFsSel | m_CurrentOdr;
    Accel_WriteRegister(ACCEL_REG_ACCEL_CONFIG0, &RegVal, 1);

    k_msleep(ACCEL_STARTUP_MS);

    LOG_INF("Range set, sensitivity: %.6f", (double)(1.0f / m_Sensitivity));
}

/*******************************************************************************
Description:
Enable the data-ready / FIFO watermark interrupt on the INT1 GPIO pin.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Accel_EnableInterrupt(void)
{
    return gpio_pin_interrupt_configure_dt(&m_IntPin, GPIO_INT_EDGE_RISING);
}

/*******************************************************************************
Description:
Disable the accelerometer interrupt on the INT1 GPIO pin.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Accel_DisableInterrupt(void)
{
    return gpio_pin_interrupt_configure_dt(&m_IntPin, GPIO_INT_DISABLE);
}

/*******************************************************************************
Description:
Put the IIM-42352 into standby mode. Clears the accel mode bits in
PWR_MGMT0 register.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Accel_Standby(void)
{
    uint8_t RegVal;
    int Ret;

    Ret = Accel_ReadRegister(ACCEL_REG_PWR_MGMT0, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    RegVal &= ~0x03;
    RegVal |= ACCEL_PWR_MODE_OFF;

    return Accel_WriteRegister(ACCEL_REG_PWR_MGMT0, &RegVal, 1);
}

/*******************************************************************************
Description:
Put the IIM-42352 into low-noise active mode. Sets the accel mode bits
in PWR_MGMT0 register to low-noise (LN) mode.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Accel_Active(void)
{
    uint8_t RegVal;
    int Ret;

    Ret = Accel_ReadRegister(ACCEL_REG_PWR_MGMT0, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    RegVal &= ~0x03;
    RegVal |= ACCEL_PWR_MODE_LN;

    Ret = Accel_WriteRegister(ACCEL_REG_PWR_MGMT0, &RegVal, 1);
    k_msleep(ACCEL_STARTUP_MS);

    return Ret;
}

/*******************************************************************************
Description:
Configure the accelerometer interrupt GPIO pin. Sets it as input with
rising-edge interrupt and registers the callback.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Accel_GpioInit(void)
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

    gpio_init_callback(&m_IntCbData, Accel_IntHandler, BIT(m_IntPin.pin));
    gpio_add_callback(m_IntPin.port, &m_IntCbData);

    return 0;
}

/*******************************************************************************
Description:
Return the current sensitivity factor (LSB/g) for the active range.

Argument(s):
None

Return:
float - Sensitivity in LSB per g.
*******************************************************************************/
float Accel_GetSensitivity(void)
{
    return m_Sensitivity;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Read one or more bytes from an IIM-42352 register via SPI.
Bit 7 of the register address is set to indicate a read operation.

Argument(s):
Reg  - Register address to read from.
Data - Buffer to store the read data.
Len  - Number of bytes to read.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Accel_ReadRegister(uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    uint8_t TxCmd = Reg | ACCEL_SPI_READ;

    const struct spi_buf TxBuf = {
        .buf = &TxCmd,
        .len = 1,
    };

    const struct spi_buf RxBufs[2] = {
        {
            .buf = NULL,
            .len = 1,
        },
        {
            .buf = Data,
            .len = Len,
        },
    };

    const struct spi_buf_set Tx = {
        .buffers = &TxBuf,
        .count   = 1,
    };

    const struct spi_buf_set Rx = {
        .buffers = RxBufs,
        .count   = ARRAY_SIZE(RxBufs),
    };

    return spi_transceive(m_SpiDev, &m_SpiCfg, &Tx, &Rx);
}

/*******************************************************************************
Description:
Write one or more bytes to an IIM-42352 register via SPI.
Bit 7 of the register address is cleared to indicate a write operation.

Argument(s):
Reg  - Register address to write to.
Data - Buffer containing the data to write.
Len  - Number of bytes to write.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Accel_WriteRegister(uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    uint8_t TxCmd = Reg | ACCEL_SPI_WRITE;

    const struct spi_buf TxBufs[2] = {
        {
            .buf = &TxCmd,
            .len = 1,
        },
        {
            .buf = Data,
            .len = Len,
        },
    };

    const struct spi_buf_set Tx = {
        .buffers = TxBufs,
        .count   = 2,
    };

    return spi_transceive(m_SpiDev, &m_SpiCfg, &Tx, NULL);
}

/*******************************************************************************
Description:
Read and validate the WHO_AM_I register. Expected value for IIM-42352
is 0x67.

Argument(s):
None

Return:
int - 0 on success, -1 if WHO_AM_I mismatch.
*******************************************************************************/
static int Accel_ReadWhoAmI(void)
{
    uint8_t WhoAmI = 0;
    int Ret;

    Ret = Accel_ReadRegister(ACCEL_REG_WHO_AM_I, &WhoAmI, 1);
    if (Ret)
    {
        LOG_ERR("WHO_AM_I read failed: %d", Ret);
        return -1;
    }

    LOG_INF("WHO_AM_I: 0x%02X", WhoAmI);

    if (WhoAmI != ACCEL_WHO_AM_I_VAL)
    {
        LOG_ERR("WHO_AM_I mismatch: expected 0x%02X, got 0x%02X",
                ACCEL_WHO_AM_I_VAL, WhoAmI);
        return -1;
    }

    return 0;
}

/*******************************************************************************
Description:
Perform a soft reset of the IIM-42352. Writes to DEVICE_CONFIG register
and waits for the reset to complete.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Accel_SoftReset(void)
{
    uint8_t RegVal = ACCEL_SOFT_RESET;
    int Ret;

    Ret = Accel_WriteRegister(ACCEL_REG_DEVICE_CONFIG, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    k_msleep(ACCEL_RESET_WAIT_MS);

    /* Wait for reset done in INT_STATUS */
    uint8_t Status = 0;
    int Retries = 10;

    while (Retries--)
    {
        Ret = Accel_ReadRegister(ACCEL_REG_INT_STATUS, &Status, 1);
        if (Ret == 0 && (Status & ACCEL_INT_STATUS_RESET_DONE))
        {
            LOG_INF("Soft reset complete");
            return 0;
        }
        k_msleep(1);
    }

    LOG_WRN("Reset done bit not seen, continuing");
    return 0;
}

/*******************************************************************************
Description:
Configure the IIM-42352 FIFO for accelerometer data collection. Sets FIFO
to stream mode with accel data enabled and watermark at 86 samples.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Accel_ConfigureFifo(void)
{
    int Ret;
    uint8_t RegVal;

    /* Flush FIFO first */
    RegVal = ACCEL_FIFO_FLUSH;
    Ret = Accel_WriteRegister(ACCEL_REG_SIGNAL_PATH_RESET, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }
    k_msleep(1);

    /* FIFO_CONFIG: stream mode */
    RegVal = (ACCEL_FIFO_MODE_STREAM << 6);
    Ret = Accel_WriteRegister(ACCEL_REG_FIFO_CONFIG, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    /* FIFO_CONFIG1: enable accel + temp (8-byte packets: header+accel+temp) */
    RegVal = ACCEL_FIFO_ACCEL_EN | ACCEL_FIFO_TEMP_EN;
    Ret = Accel_WriteRegister(ACCEL_REG_FIFO_CONFIG1, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    /* FIFO watermark: 86 samples x 8 bytes = 688 bytes (register is in bytes) */
    RegVal = ACCEL_FIFO_WM_BYTES & 0xFF;
    Ret = Accel_WriteRegister(ACCEL_REG_FIFO_CONFIG2, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    /* FIFO watermark high nibble */
    RegVal = (ACCEL_FIFO_WM_BYTES >> 8) & 0x0F;
    Ret = Accel_WriteRegister(ACCEL_REG_FIFO_CONFIG3, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    LOG_INF("FIFO configured: stream mode, WM=%d samples", ACCEL_FIFO_WM_SAMPLES);
    return 0;
}

/*******************************************************************************
Description:
Configure INT1 pin on the IIM-42352 for FIFO watermark threshold interrupt.
Sets INT1 as active-high, push-pull, pulsed mode.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
static int Accel_ConfigureInterrupt(void)
{
    int Ret;
    uint8_t RegVal;

    /* INT_CONFIG: INT1 active high, push-pull, pulsed */
    RegVal = ACCEL_INT1_POLARITY_HIGH | ACCEL_INT1_DRIVE_PP;
    Ret = Accel_WriteRegister(ACCEL_REG_INT_CONFIG, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    /* INT_SOURCE0: FIFO watermark threshold on INT1 */
    RegVal = ACCEL_INT_SRC_FIFO_THS;
    Ret = Accel_WriteRegister(ACCEL_REG_INT_SOURCE0, &RegVal, 1);
    if (Ret)
    {
        return Ret;
    }

    LOG_INF("INT1 configured: FIFO watermark, active-high");
    return 0;
}

/*******************************************************************************
Description:
GPIO interrupt handler for accelerometer data-ready / FIFO watermark.
Gives the m_DataReadySem semaphore to wake the acquisition thread.

Argument(s):
Dev  - Pointer to the GPIO device.
Cb   - Pointer to the callback structure.
Pins - Bitmask of pins that triggered the interrupt.

Return:
None
*******************************************************************************/
static void Accel_IntHandler(const struct device *Dev, struct gpio_callback *Cb,
                             uint32_t Pins)
{
    ARG_UNUSED(Dev);
    ARG_UNUSED(Cb);
    ARG_UNUSED(Pins);

    k_sem_give(&m_DataReadySem);
}

/*******************************************************************************
Description:
Convert an ODR value in Hz to the IIM-42352 ACCEL_CONFIG0 register value.

Argument(s):
RateHz - Desired output data rate in Hz.

Return:
uint8_t - Register value for the ODR field.
*******************************************************************************/
static uint8_t Accel_ConvertOdr(uint16_t RateHz)
{
    /* IIM-42352 supported ODRs: 1.5625 Hz to 32 kHz (no 3200 Hz) */
    switch (RateHz)
    {
    case 50:    return ACCEL_ODR_50HZ;
    case 100:   return ACCEL_ODR_100HZ;
    case 200:   return ACCEL_ODR_200HZ;
    case 500:   return ACCEL_ODR_500HZ;
    case 1000:  return ACCEL_ODR_1KHZ;
    case 2000:  return ACCEL_ODR_2KHZ;
    case 4000:  return ACCEL_ODR_4KHZ;
    case 8000:  return ACCEL_ODR_8KHZ;
    case 16000: return ACCEL_ODR_16KHZ;
    case 32000: return ACCEL_ODR_32KHZ;
    default:    return ACCEL_ODR_4KHZ;
    }
}

/*******************************************************************************
Description:
Convert a range code from CommonTypes.h to the IIM-42352 FS_SEL register
value.

Argument(s):
Range - Range code (RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G).

Return:
uint8_t - Register value for the FS_SEL field.
*******************************************************************************/
static uint8_t Accel_ConvertRange(uint8_t Range)
{
    switch (Range)
    {
    case RANGE_2G:  return ACCEL_FS_SEL_2G;
    case RANGE_4G:  return ACCEL_FS_SEL_4G;
    case RANGE_8G:  return ACCEL_FS_SEL_8G;
    case RANGE_16G: return ACCEL_FS_SEL_16G;
    default:        return ACCEL_FS_SEL_8G;
    }
}
