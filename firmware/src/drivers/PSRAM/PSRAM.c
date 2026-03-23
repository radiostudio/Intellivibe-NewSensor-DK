/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the APS6404L-SQH-ZR 64Mbit SPI PSRAM driver.
Communicates via dedicated SPI bus at 10 MHz. Power-gated via a load switch
GPIO (active-high). Used for buffering high-speed accelerometer streaming
data that exceeds internal RAM capacity.

Part: APMemory APS6404L-SQH-ZR (8 MB SPI PSRAM)

Features:
  - SPI bus and load switch GPIO init
  - Device ID validation
  - Sequential read/write with 24-bit addressing
  - Power gating control (on/off via load switch)
  - Self-test (write/read-back verification)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(PSRAM, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include "PSRAM.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define PSRAM_SPI_NODE      DT_NODELABEL(psram_spi)
#define PSRAM_LS_NODE       DT_ALIAS(psramctrl)

#define PSRAM_SPI_FREQ      10000000U
#define PSRAM_SPI_OP        (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE)

#define PSRAM_POWER_DELAY_MS    1
#define PSRAM_RESET_DELAY_US    1

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device *m_SpiDev;

static const struct spi_cs_control m_SpiCs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_psram_spi)),
    .delay = 10,
};

static const struct spi_config m_SpiCfg = {
    .operation  = PSRAM_SPI_OP,
    .frequency  = PSRAM_SPI_FREQ,
    .slave      = 0,
    .cs         = m_SpiCs,
};

static const struct gpio_dt_spec m_LoadSwitch = GPIO_DT_SPEC_GET(PSRAM_LS_NODE, gpios);
static uint8_t m_VddStatus = PSRAM_VDD_OFF;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int  PSRAM_SpiInit(void);
static void PSRAM_Reset(void);
static int  PSRAM_ReadId(void);
static int  PSRAM_SelfTest(void);
static void PSRAM_SpiWrite(uint8_t *Data, uint16_t Len);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the APS6404L PSRAM. Configures the SPI bus, load switch GPIO,
powers on the device, performs a reset, validates the device ID, runs a
self-test, then powers off to save current until needed.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int PSRAM_Init(void)
{
    int Ret;

    /* Init SPI bus and load switch GPIO */
    Ret = PSRAM_SpiInit();
    if (Ret)
    {
        return -1;
    }

    /* Power on */
    Ret = PSRAM_PowerOn();
    if (Ret)
    {
        return -1;
    }

    /* Reset chip */
    PSRAM_Reset();
    k_msleep(PSRAM_POWER_DELAY_MS);

    /* Validate device ID */
    Ret = PSRAM_ReadId();
    if (Ret)
    {
        LOG_ERR("APS6404L not found");
        PSRAM_PowerOff();
        return -1;
    }

    /* Self-test: write and read back */
    Ret = PSRAM_SelfTest();
    if (Ret)
    {
        LOG_ERR("PSRAM self-test failed");
        PSRAM_PowerOff();
        return -1;
    }

    /* Power off until needed */
    PSRAM_PowerOff();

    LOG_INF("APS6404L-SQH-ZR initialised");
    return 0;
}

/*******************************************************************************
Description:
Write data to PSRAM at the specified 24-bit address. Automatically powers
on the load switch if not already on.

Argument(s):
Address - 24-bit PSRAM address to write to.
Data    - Pointer to the data buffer to write.
Len     - Number of bytes to write.

Return:
None
*******************************************************************************/
void PSRAM_Write(uint32_t Address, uint8_t *Data, uint16_t Len)
{
    int Ret;
    uint8_t CmdBuf[4];

    PSRAM_PowerOn();

    CmdBuf[0] = PSRAM_CMD_WRITE;
    CmdBuf[1] = (Address >> 16) & 0xFF;
    CmdBuf[2] = (Address >> 8) & 0xFF;
    CmdBuf[3] = Address & 0xFF;

    const struct spi_buf TxBufs[2] = {
        { .buf = CmdBuf, .len = 4 },
        { .buf = Data,   .len = Len },
    };

    const struct spi_buf_set Tx = {
        .buffers = TxBufs,
        .count   = 2,
    };

    Ret = spi_transceive(m_SpiDev, &m_SpiCfg, &Tx, NULL);
    if (Ret)
    {
        LOG_ERR("PSRAM write failed: %d", Ret);
    }
}

/*******************************************************************************
Description:
Read data from PSRAM at the specified 24-bit address. Automatically powers
on the load switch if not already on.

Argument(s):
Address - 24-bit PSRAM address to read from.
Buffer  - Pointer to the buffer to store read data.
Len     - Number of bytes to read.

Return:
None
*******************************************************************************/
void PSRAM_Read(uint32_t Address, uint8_t *Buffer, uint16_t Len)
{
    int Ret;
    uint8_t CmdBuf[4];

    PSRAM_PowerOn();

    CmdBuf[0] = PSRAM_CMD_READ;
    CmdBuf[1] = (Address >> 16) & 0xFF;
    CmdBuf[2] = (Address >> 8) & 0xFF;
    CmdBuf[3] = Address & 0xFF;

    const struct spi_buf TxBuf = {
        .buf = CmdBuf,
        .len = 4,
    };

    const struct spi_buf RxBufs[2] = {
        { .buf = NULL,   .len = 4 },
        { .buf = Buffer, .len = Len },
    };

    const struct spi_buf_set Tx = {
        .buffers = &TxBuf,
        .count   = 1,
    };

    const struct spi_buf_set Rx = {
        .buffers = RxBufs,
        .count   = 2,
    };

    Ret = spi_transceive(m_SpiDev, &m_SpiCfg, &Tx, &Rx);
    if (Ret)
    {
        LOG_ERR("PSRAM read failed: %d", Ret);
    }
}

/*******************************************************************************
Description:
Power on the PSRAM via the load switch GPIO (active-high). Waits 1 ms
after assertion for the device to be ready (FRS 8.3).

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int PSRAM_PowerOn(void)
{
    int Ret;

    if (m_VddStatus == PSRAM_VDD_ON)
    {
        return 0;
    }

    Ret = gpio_pin_set_dt(&m_LoadSwitch, PSRAM_VDD_ON);
    if (Ret)
    {
        LOG_ERR("Load switch on failed: %d", Ret);
        return Ret;
    }

    m_VddStatus = PSRAM_VDD_ON;
    k_msleep(PSRAM_POWER_DELAY_MS);

    return 0;
}

/*******************************************************************************
Description:
Power off the PSRAM via the load switch GPIO. Waits 1 ms after deassertion
before next operation (FRS 8.3).

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int PSRAM_PowerOff(void)
{
    int Ret;

    if (m_VddStatus == PSRAM_VDD_OFF)
    {
        return 0;
    }

    Ret = gpio_pin_set_dt(&m_LoadSwitch, PSRAM_VDD_OFF);
    if (Ret)
    {
        LOG_ERR("Load switch off failed: %d", Ret);
        return Ret;
    }

    m_VddStatus = PSRAM_VDD_OFF;
    k_msleep(PSRAM_POWER_DELAY_MS);

    return 0;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the SPI bus and load switch GPIO for PSRAM.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
static int PSRAM_SpiInit(void)
{
    int Ret;

    m_SpiDev = DEVICE_DT_GET(PSRAM_SPI_NODE);
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

    if (!gpio_is_ready_dt(&m_LoadSwitch))
    {
        LOG_ERR("Load switch GPIO not ready");
        return -1;
    }

    Ret = gpio_pin_configure_dt(&m_LoadSwitch, GPIO_OUTPUT_ACTIVE);
    if (Ret)
    {
        LOG_ERR("Load switch configure failed: %d", Ret);
        return -1;
    }

    /* Start with power off */
    gpio_pin_set_dt(&m_LoadSwitch, PSRAM_VDD_OFF);
    m_VddStatus = PSRAM_VDD_OFF;

    return 0;
}

/*******************************************************************************
Description:
Perform a soft reset of the APS6404L. Sends RESET_ENABLE followed by
RESET command per the datasheet sequence.

Argument(s):
None

Return:
None
*******************************************************************************/
static void PSRAM_Reset(void)
{
    uint8_t Cmd;

    /* Reset enable */
    Cmd = PSRAM_CMD_RESET_ENABLE;
    PSRAM_SpiWrite(&Cmd, 1);
    k_usleep(PSRAM_RESET_DELAY_US);

    /* Reset */
    Cmd = PSRAM_CMD_RESET;
    PSRAM_SpiWrite(&Cmd, 1);
    k_usleep(PSRAM_RESET_DELAY_US);
}

/*******************************************************************************
Description:
Read and validate the APS6404L device ID. Sends READ_ID command at
address 0x000000 and checks bytes 3 and 4 against known values.

Argument(s):
None

Return:
int - 0 on success, -1 if device ID mismatch.
*******************************************************************************/
static int PSRAM_ReadId(void)
{
    uint8_t IdBuf[8] = {0};

    /* READ_ID command with 3-byte address (0x000000) */
    uint8_t CmdBuf[4] = { PSRAM_CMD_READ_ID, 0x00, 0x00, 0x00 };

    const struct spi_buf TxBuf = {
        .buf = CmdBuf,
        .len = 4,
    };

    const struct spi_buf RxBufs[2] = {
        { .buf = NULL,  .len = 4 },
        { .buf = IdBuf, .len = 8 },
    };

    const struct spi_buf_set Tx = {
        .buffers = &TxBuf,
        .count   = 1,
    };

    const struct spi_buf_set Rx = {
        .buffers = RxBufs,
        .count   = 2,
    };

    int Ret = spi_transceive(m_SpiDev, &m_SpiCfg, &Tx, &Rx);
    if (Ret)
    {
        LOG_ERR("ID read failed: %d", Ret);
        return -1;
    }

    LOG_INF("PSRAM ID: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
            IdBuf[0], IdBuf[1], IdBuf[2], IdBuf[3], IdBuf[4]);

    if (IdBuf[3] != PSRAM_KNOWN_ID_BYTE3 || IdBuf[4] != PSRAM_KNOWN_ID_BYTE4)
    {
        LOG_ERR("PSRAM ID mismatch: expected 0x%02X/0x%02X, got 0x%02X/0x%02X",
                PSRAM_KNOWN_ID_BYTE3, PSRAM_KNOWN_ID_BYTE4,
                IdBuf[3], IdBuf[4]);
        return -1;
    }

    LOG_INF("APS6404L-SQH-ZR found");
    return 0;
}

/*******************************************************************************
Description:
Self-test: write a known pattern to address 0, read it back, and compare.

Argument(s):
None

Return:
int - 0 on success, -1 on mismatch.
*******************************************************************************/
static int PSRAM_SelfTest(void)
{
    uint8_t WriteBuf[10] = {0x01, 0x02, 0x03, 0x04, 0x05,
                            0x06, 0x07, 0x08, 0x09, 0x0A};
    uint8_t ReadBuf[10] = {0};

    PSRAM_Write(0x000000, WriteBuf, sizeof(WriteBuf));
    k_msleep(1);
    PSRAM_Read(0x000000, ReadBuf, sizeof(ReadBuf));

    if (memcmp(WriteBuf, ReadBuf, sizeof(WriteBuf)) != 0)
    {
        LOG_ERR("Self-test failed: data mismatch");
        return -1;
    }

    LOG_INF("Self-test passed");
    return 0;
}

/*******************************************************************************
Description:
Send a raw SPI write (no address). Used for single-byte commands like
RESET_ENABLE and RESET.

Argument(s):
Data - Pointer to the data to send.
Len  - Number of bytes to send.

Return:
None
*******************************************************************************/
static void PSRAM_SpiWrite(uint8_t *Data, uint16_t Len)
{
    const struct spi_buf TxBuf = {
        .buf = Data,
        .len = Len,
    };

    const struct spi_buf_set Tx = {
        .buffers = &TxBuf,
        .count   = 1,
    };

    spi_write(m_SpiDev, &m_SpiCfg, &Tx);
}
