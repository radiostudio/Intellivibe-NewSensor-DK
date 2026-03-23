/*******************************************************************************
********************************************************************************
Description:
This file handles functions for NVS (Non-Volatile Storage) management.
Provides initialisation, read and write access to the Zephyr NVS subsystem
using the internal flash storage partition.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(NVS_DRV);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <string.h>
#include "NVS.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define NVS_PARTITION           storage_partition
#define NVS_PARTITION_DEVICE    FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET    FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_SECTOR_COUNT        3U

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static struct nvs_fs          m_Fs;
static struct flash_pages_info m_PageInfo;
static bool                    m_Initialised = false;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int NVS_SelfTest(void);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the NVS subsystem. Configures the flash device, reads page info,
sets sector size and count, and mounts the NVS filesystem. Runs a self-test
to verify read/write functionality.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int NVS_Init(void)
{
    int Rc;

    m_Fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(m_Fs.flash_device))
    {
        LOG_ERR("Flash device %s is not ready", m_Fs.flash_device->name);
        return -1;
    }

    m_Fs.offset = NVS_PARTITION_OFFSET;
    Rc = flash_get_page_info_by_offs(m_Fs.flash_device, m_Fs.offset, &m_PageInfo);
    if (Rc)
    {
        LOG_ERR("Unable to get page info: %d", Rc);
        return -1;
    }

    m_Fs.sector_size = m_PageInfo.size;
    m_Fs.sector_count = NVS_SECTOR_COUNT;

    Rc = nvs_mount(&m_Fs);
    if (Rc)
    {
        LOG_ERR("NVS mount failed: %d", Rc);
        return -1;
    }

    LOG_INF("NVS initialised: sector_size=%u, sector_count=%u",
            m_Fs.sector_size, m_Fs.sector_count);

    Rc = NVS_SelfTest();
    if (Rc)
    {
        LOG_ERR("NVS self-test failed");
        return -1;
    }

    m_Initialised = true;
    LOG_INF("NVS ready");
    return 0;
}

/*******************************************************************************
Description:
Read data from NVS at the given ID.

Argument(s):
Id   - NVS item identifier.
Data - Pointer to the output buffer.
Len  - Maximum number of bytes to read.

Return:
int - Number of bytes read on success, negative error code on failure.
*******************************************************************************/
int NVS_Read(uint16_t Id, void *Data, size_t Len)
{
    int Rc;

    if (!m_Initialised)
    {
        LOG_ERR("NVS not initialised");
        return -1;
    }

    Rc = nvs_read(&m_Fs, Id, Data, Len);
    if (Rc < 0)
    {
        LOG_ERR("Failed to read NVS Id %d: %d", Id, Rc);
    }

    return Rc;
}

/*******************************************************************************
Description:
Write data to NVS at the given ID. Only writes if the data has changed
(handled internally by Zephyr NVS to minimise flash wear).

Argument(s):
Id   - NVS item identifier.
Data - Pointer to the data to write.
Len  - Number of bytes to write.

Return:
int - Number of bytes written on success, negative error code on failure.
*******************************************************************************/
int NVS_Write(uint16_t Id, const void *Data, size_t Len)
{
    int Rc;

    if (!m_Initialised)
    {
        LOG_ERR("NVS not initialised");
        return -1;
    }

    Rc = nvs_write(&m_Fs, Id, Data, Len);
    if (Rc < 0)
    {
        LOG_ERR("Failed to write NVS Id %d: %d", Id, Rc);
    }

    return Rc;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Run a self-test to verify NVS read/write. Writes a known string to NVS_TEST_ID,
reads it back, and compares. Logs success or failure.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
static int NVS_SelfTest(void)
{
    const char *TestStr = "NVS_OK";
    char ReadBuf[16] = {0};
    int Rc;

    Rc = nvs_write(&m_Fs, NVS_TEST_ID, TestStr, strlen(TestStr));
    if (Rc < 0)
    {
        LOG_ERR("Self-test write failed: %d", Rc);
        return -1;
    }

    Rc = nvs_read(&m_Fs, NVS_TEST_ID, ReadBuf, strlen(TestStr));
    if (Rc < 0)
    {
        LOG_ERR("Self-test read failed: %d", Rc);
        return -1;
    }

    if (memcmp(TestStr, ReadBuf, strlen(TestStr)) != 0)
    {
        LOG_ERR("Self-test data mismatch");
        return -1;
    }

    LOG_INF("Self-test passed");
    return 0;
}
