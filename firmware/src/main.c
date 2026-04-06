/*******************************************************************************
********************************************************************************
Description:
Main entry point for IntelliVibe-BT firmware.
Handles system startup, BLE initialisation, data acquisition thread,
streaming work queues, and low-power sleep cycle.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAIN, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "BLE/ble.h"
#include "BLE/ble_config.h"
#include "UserConfig.h"

#include "SysLP/SysLP.h"

#include "Accel/Accel.h"
#include "Mag/Mag.h"
#include "Temp/Temp.h"
#include "Battery/Battery.h"
#include "Audio/Audio.h"
#include "PSRAM/PSRAM.h"
#include "DataProcessing/DataProcessing.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define LED0_NODE               DT_ALIAS(led0)
#define HEARTBEAT_DELAY_MS      100
#define FIRMWARE_VERSION        "V1.0.0"

#define DATA_ACQ_STACK_SIZE     8192
#define DATA_ACQ_PRIORITY       7
#define DATA_ACQ_START_DELAY_MS 2000

#define STREAM_STACK_SIZE       4096
#define STREAM_PRIORITY         2

#define TX_BUF_SIZE             258
#define INTER_FRAME_SPACE_US    500

#define ACCEL_AXIS_COUNT        3
#define ACCEL_BYTES_PER_SAMPLE  (ACCEL_AXIS_COUNT * sizeof(int16_t))
#define ACCEL_DEFAULT_ODR_HZ    4000

#define MAG_BYTES_PER_SAMPLE    sizeof(MagRawData_t)
#define MAG_DEFAULT_ODR_HZ      400
#define MAG_DEFAULT_DURATION_S  1

#define BLE_CMD_STREAM_ACCEL    0x0A
#define BLE_CMD_STREAM_MAG      0x0B
#define BLE_CMD_STREAM_DUAL     0x0C

#define BLE_STREAM_SVC_ACCEL    2
#define BLE_STREAM_SVC_MAG      5

#define INIT_SETTLE_TIME_S      1

/* Advertising data scaling factors */
#define ADV_ACCEL_SCALE         100
#define ADV_VEL_SCALE           10
#define ADV_MAG_SCALE           10
#define ADV_TEMP_SCALE          100

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct gpio_dt_spec m_HeartbeatLed = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static volatile bool m_InitDone = false;

/* Sensor init status: 0 = OK, -1 = failed/not present */
static int8_t m_SensorState[SENSOR_COUNT] = {
    [SENSOR_ACCEL]   = -1,
    [SENSOR_MAG]     = -1,
    [SENSOR_TEMP]    = -1,
    [SENSOR_PSRAM]   = -1,
    [SENSOR_AUDIO]   = -1,
    [SENSOR_BATTERY] = -1,
    [SENSOR_NVS]     = -1,
};

static AdvMfgData_t m_AdvMfgData = {
    .CompanyCode    = 0x0059,
    .AccelRms       = {0},
    .VelRms         = {0},
    .MagRms         = {0},
    .Temperature    = 0,
    .Noise          = 0,
    .BatLevel       = 0,
};

/* Sensor data buffers */
static AccelRawData_t m_AccelRawBuf[ACCEL_RAW_BUFFER_LEN];
static MagRawData_t   m_MagRawBuf[MAG_RAW_BUFFER_LEN];

/* DSP output */
static OutputData_t m_AccelRmsOut;
static OutputData_t m_VelRmsOut;
static OutputData_t m_MagRmsOut;

/* Latest sensor values for advertising */
static float    m_Temperature;
static uint8_t  m_BatteryPercent;
static uint16_t m_NoiseSpl;

/* Synchronisation primitives (FRS 5.3) */
static K_SEM_DEFINE(m_DataAcqSem, 0, 1);
static K_SEM_DEFINE(m_DataAcqDisconnectSem, 0, 1);

/* Streaming work queues */
static struct k_work_q m_AccelStreamWorkQ;
static K_THREAD_STACK_DEFINE(m_AccelStreamStack, STREAM_STACK_SIZE);
static struct k_work m_AccelStreamWork;

static struct k_work_q m_MagStreamWorkQ;
static K_THREAD_STACK_DEFINE(m_MagStreamStack, STREAM_STACK_SIZE);
static struct k_work m_MagStreamWork;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static void Main_HeartbeatLed(void);
static void Main_DataAcqThread(void *Arg1, void *Arg2, void *Arg3);
static void Main_SensorInit(void);
static void Main_ReadAndProcess(void);
static void Main_UpdateAdvData(void);
static void Main_LowPowerSection(void);
static void Main_AccelStreamCb(struct k_work *Work);
static void Main_MagStreamCb(struct k_work *Work);

/* Data acquisition thread definition */
K_THREAD_DEFINE(DataAcqId, DATA_ACQ_STACK_SIZE, Main_DataAcqThread,
                NULL, NULL, NULL, DATA_ACQ_PRIORITY, 0, DATA_ACQ_START_DELAY_MS);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Main entry point. Initialises UserConfig (NVS + settings), BLE stack,
streaming work queues, and signals the data acquisition thread to start.

Argument(s):
None

Return:
int - 0 on success (never returns in normal operation).
*******************************************************************************/
int main(void)
{
    int Ret;

    LOG_INF("IntelliVibe-BT Firmware %s starting", FIRMWARE_VERSION);
    LOG_INF("Board: %s", CONFIG_BOARD);

    /* Initialise configuration (NVS + settings with magic/CRC validation) */
    UserConfig_Init();

    /* Configure BLE config service with settings and version */
    BLEConfig_GetSettings(UserConfig_GetSettings());
    BLEConfig_GetFirmwareVersion(FIRMWARE_VERSION);
    BLEConfig_GetSensorState(m_SensorState, SENSOR_COUNT);

    /* Initialise BLE stack and start advertising */
    Ret = BLE_Init();
    if (Ret)
    {
        LOG_ERR("BLE init failed: %d", Ret);
        return Ret;
    }

    /* Initialise streaming work queues */
    k_work_queue_init(&m_AccelStreamWorkQ);
    k_work_queue_start(&m_AccelStreamWorkQ, m_AccelStreamStack,
                        K_THREAD_STACK_SIZEOF(m_AccelStreamStack),
                        STREAM_PRIORITY, NULL);
    k_work_init(&m_AccelStreamWork, Main_AccelStreamCb);

    k_work_queue_init(&m_MagStreamWorkQ);
    k_work_queue_start(&m_MagStreamWorkQ, m_MagStreamStack,
                        K_THREAD_STACK_SIZEOF(m_MagStreamStack),
                        STREAM_PRIORITY, NULL);
    k_work_init(&m_MagStreamWork, Main_MagStreamCb);

    k_sleep(K_SECONDS(INIT_SETTLE_TIME_S));

    /* Signal data acquisition thread that init is complete */
    m_InitDone = true;
    LOG_INF("System initialised");

    return 0;
}

/*******************************************************************************
Description:
Called from BLEConfig_Write when a streaming command is received.
Submits the appropriate work item to start streaming.

Argument(s):
Cmd - BLE command byte (0x0A, 0x0B, or 0x0C).

Return:
None

Note(s):
This function is meant to be called from ble_config.c streaming cases.
*******************************************************************************/
void Main_StartStreaming(uint16_t Cmd)
{
    switch (Cmd)
    {
    case BLE_CMD_STREAM_ACCEL:
        LOG_INF("Starting accel streaming");
        k_sem_give(&m_DataAcqSem);
        k_work_submit_to_queue(&m_AccelStreamWorkQ, &m_AccelStreamWork);
        break;

    case BLE_CMD_STREAM_MAG:
        LOG_INF("Starting mag streaming");
        k_sem_give(&m_DataAcqSem);
        k_work_submit_to_queue(&m_MagStreamWorkQ, &m_MagStreamWork);
        break;

    case BLE_CMD_STREAM_DUAL:
        LOG_INF("Starting dual streaming");
        k_sem_give(&m_DataAcqSem);
        k_work_submit_to_queue(&m_AccelStreamWorkQ, &m_AccelStreamWork);
        k_work_submit_to_queue(&m_MagStreamWorkQ, &m_MagStreamWork);
        break;

    default:
        break;
    }
}

/*******************************************************************************
Description:
Called when BLE disconnects to signal the acquisition thread to resume
normal RMS mode.

Argument(s):
None

Return:
None

Note(s):
This function is meant to be called from BLE disconnect callback.
*******************************************************************************/
void Main_StopStreaming(void)
{
    LOG_INF("Streaming stopped, resuming acquisition");
    k_sem_give(&m_DataAcqDisconnectSem);
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Data acquisition thread entry point. Waits for system init to complete,
initialises all sensors, then enters the main acquisition loop:
  1. Read all sensors and run DSP
  2. Update BLE advertising packet
  3. Enter low-power sleep for dInterval seconds
  4. Wake, restore GPIOs, restart advertising, repeat

Argument(s):
Arg1 - Unused.
Arg2 - Unused.
Arg3 - Unused.

Return:
None
*******************************************************************************/
static void Main_DataAcqThread(void *Arg1, void *Arg2, void *Arg3)
{
    ARG_UNUSED(Arg1);
    ARG_UNUSED(Arg2);
    ARG_UNUSED(Arg3);

    SystemSettings_t *Settings;

    /* Wait for main() init to complete */
    while (!m_InitDone)
    {
        k_sleep(K_SECONDS(1));
    }

    /* Drain semaphores in case of spurious initial state */
    k_sem_take(&m_DataAcqSem, K_NO_WAIT);
    k_sem_take(&m_DataAcqDisconnectSem, K_NO_WAIT);

    /* Configure heartbeat LED */
    if (!gpio_is_ready_dt(&m_HeartbeatLed))
    {
        LOG_ERR("Heartbeat LED not ready");
        return;
    }
    gpio_pin_configure_dt(&m_HeartbeatLed, GPIO_OUTPUT_ACTIVE);

    /* Initialise all sensors */
    Main_SensorInit();
    LOG_INF("Sensor init done, entering acquisition loop");

    Settings = UserConfig_GetSettings();

    while (1)
    {
        /* Only run RMS acquisition when not streaming */
        if (BLE_GetCurrentConn() == NULL)
        {
            /* ── CONTINUOUS RMS MODE ── */
            Main_ReadAndProcess();
            Main_UpdateAdvData();
            Main_HeartbeatLed();

            /* Brief pause for BLE connection opportunity */
            k_sleep(K_SECONDS(1));
        }

        /* ── LOW POWER SLEEP SECTION ── */
        Main_LowPowerSection();
    }
}

/*******************************************************************************
Description:
Initialise all sensor peripherals. Each sensor init is independent — a
failure in one does not block the others.

Argument(s):
None

Return:
None

Note(s):
Each sensor init is independent — a failure in one does not block the others.
m_SensorState[] is updated so the acquisition loop can skip failed sensors.
*******************************************************************************/
static void Main_SensorInit(void)
{
    LOG_INF("Initialising sensors...");

    /* Initialise low-power GPIO management */
    SysLP_Init();

    /* Accelerometer (IIM-42352, SPI) */
    m_SensorState[SENSOR_ACCEL] = (Accel_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_ACCEL] != 0)
    {
        LOG_ERR("Accel init failed");
    }

    /* Magnetometer (BMM350, I2C) */
    m_SensorState[SENSOR_MAG] = (Mag_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_MAG] != 0)
    {
        LOG_ERR("Mag init failed");
    }

    /* Temperature (TMP112, I2C) */
    m_SensorState[SENSOR_TEMP] = (Temp_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_TEMP] != 0)
    {
        LOG_ERR("Temp init failed");
    }

    /* Battery (LTC3335, I2C) */
    m_SensorState[SENSOR_BATTERY] = (Battery_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_BATTERY] != 0)
    {
        LOG_ERR("Battery init failed");
    }

    /* Audio (MMICT5838, PDM) */
    m_SensorState[SENSOR_AUDIO] = (Audio_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_AUDIO] != 0)
    {
        LOG_ERR("Audio init failed");
    }

    /* PSRAM (APS6404L, SPI) */
    m_SensorState[SENSOR_PSRAM] = (PSRAM_Init() == 0) ? 0 : -1;
    if (m_SensorState[SENSOR_PSRAM] != 0)
    {
        LOG_ERR("PSRAM init failed");
    }

    /* Data processing (FFT) */
    if (DataProc_Init() != 0)
    {
        LOG_ERR("DataProcessing init failed");
    }

    LOG_INF("Sensor init complete (Accel:%d Mag:%d Temp:%d Bat:%d Audio:%d PSRAM:%d)",
            m_SensorState[SENSOR_ACCEL], m_SensorState[SENSOR_MAG],
            m_SensorState[SENSOR_TEMP], m_SensorState[SENSOR_BATTERY],
            m_SensorState[SENSOR_AUDIO], m_SensorState[SENSOR_PSRAM]);
}

/*******************************************************************************
Description:
Execute one full acquisition cycle: read all sensors and run signal
processing. This produces accel RMS, velocity RMS, mag RMS, temperature,
battery level, and noise SPL.

Argument(s):
None

Return:
None

Note(s):
Reads raw sensor data, then runs DSP: FFT-based accel/velocity RMS via
DataProcessing module, time-domain mag RMS via Mag_CalculateRms.
*******************************************************************************/
static void Main_ReadAndProcess(void)
{
    int64_t Start = k_uptime_get();
    SystemSettings_t *Settings = UserConfig_GetSettings();
    AccelConfig_t AccelCfg;
    uint16_t AccelSampleCount;
    float Temperature;
    uint8_t BatteryPercent;

    if (m_SensorState[SENSOR_ACCEL] == 0)
    {
        AccelCfg.SamplingRate = Settings->HighSampleRate;
        AccelCfg.Duration    = Settings->HighSampleDuration;
        AccelSampleCount     = AccelCfg.SamplingRate * AccelCfg.Duration;

        Accel_Active();
        if (Accel_ReadData(m_AccelRawBuf, &AccelCfg) == 0)
        {
            DataProc_ComputeAccelRms(m_AccelRawBuf, AccelSampleCount,
                                     Accel_GetSensitivity(),
                                     &m_AccelRmsOut, &m_VelRmsOut);
            LOG_DBG("Accel RMS: X=%.3f Y=%.3f Z=%.3f",
                    (double)m_AccelRmsOut.XData,
                    (double)m_AccelRmsOut.YData,
                    (double)m_AccelRmsOut.ZData);
        }
        Accel_Standby();
    }

    if (m_SensorState[SENSOR_MAG] == 0)
    {
        MagConfig_t MagCfg;

        MagCfg.SamplingRate = MAG_DEFAULT_ODR_HZ;
        MagCfg.Duration     = MAG_DEFAULT_DURATION_S;

        if (Mag_ReadData(m_MagRawBuf, &MagCfg) == 0)
        {
            Mag_CalculateRms(m_MagRawBuf, MagCfg.SamplingRate * MagCfg.Duration, &m_MagRmsOut);
            LOG_INF("Mag RMS: X=%.3f Y=%.3f Z=%.3f uT",
                    (double)m_MagRmsOut.XData,
                    (double)m_MagRmsOut.YData,
                    (double)m_MagRmsOut.ZData);
        }
    }

    if (m_SensorState[SENSOR_TEMP] == 0)
    {
        if (Temp_Read(&Temperature) == 0)
        {
            m_Temperature = Temperature;
            LOG_DBG("Temp: %.2f C", (double)m_Temperature);
        }
    }

    if (m_SensorState[SENSOR_BATTERY] == 0)
    {
        if (Battery_Read(&BatteryPercent) == 0)
        {
            m_BatteryPercent = BatteryPercent;
            LOG_DBG("Battery: %d%%", m_BatteryPercent);
        }
    }

    if (m_SensorState[SENSOR_AUDIO] == 0)
    {
        if (Audio_Read() == 0)
        {
            m_NoiseSpl = Audio_GetNoiseSpl();
            LOG_DBG("Noise SPL: %u", m_NoiseSpl);
        }
    }

    LOG_INF("Acquisition cycle time: %lld ms", k_uptime_delta(&Start));
}

/*******************************************************************************
Description:
Pack the latest sensor results into the advertising manufacturer data
structure and update the BLE advertising packet.

Argument(s):
None

Return:
None

Note(s):
All sensor fields are now populated from DSP outputs and sensor reads.
Scaling matches old firmware: accel *100, velocity *10, mag *10, temp *100.
*******************************************************************************/
static void Main_UpdateAdvData(void)
{
    SystemSettings_t *Settings = UserConfig_GetSettings();

    m_AdvMfgData.AccelRms.XData = (uint16_t)(m_AccelRmsOut.XData * ADV_ACCEL_SCALE);
    m_AdvMfgData.AccelRms.YData = (uint16_t)(m_AccelRmsOut.YData * ADV_ACCEL_SCALE);
    m_AdvMfgData.AccelRms.ZData = (uint16_t)(m_AccelRmsOut.ZData * ADV_ACCEL_SCALE);

    m_AdvMfgData.VelRms.XData   = (uint16_t)(m_VelRmsOut.XData * ADV_VEL_SCALE);
    m_AdvMfgData.VelRms.YData   = (uint16_t)(m_VelRmsOut.YData * ADV_VEL_SCALE);
    m_AdvMfgData.VelRms.ZData   = (uint16_t)(m_VelRmsOut.ZData * ADV_VEL_SCALE);

    m_AdvMfgData.MagRms.XData   = (uint16_t)(m_MagRmsOut.XData * ADV_MAG_SCALE);
    m_AdvMfgData.MagRms.YData   = (uint16_t)(m_MagRmsOut.YData * ADV_MAG_SCALE);
    m_AdvMfgData.MagRms.ZData   = (uint16_t)(m_MagRmsOut.ZData * ADV_MAG_SCALE);

    m_AdvMfgData.Temperature = (uint16_t)((m_Temperature + Settings->TempOffset) * ADV_TEMP_SCALE);
    m_AdvMfgData.Noise       = m_NoiseSpl;
    m_AdvMfgData.BatLevel    = m_BatteryPercent;

    BLE_SetAdvData(&m_AdvMfgData);
    BLE_AdvUpdate();

    LOG_DBG("Advertising data updated");
}

/*******************************************************************************
Description:
Handle the low-power sleep phase between acquisition cycles.
  1. Check if streaming was requested (m_DataAcqSem) — if so, wait for
     disconnect (m_DataAcqDisconnectSem) before sleeping.
  2. Stop advertising.
  3. Set GPIOs to low-power state.
  4. Sleep for dInterval seconds.
  5. Restore GPIOs.
  6. Toggle heartbeat LED.
  7. Restart advertising.

Argument(s):
None

Return:
None
*******************************************************************************/
static void Main_LowPowerSection(void)
{
    SystemSettings_t *Settings = UserConfig_GetSettings();

    /* If streaming was requested, wait for disconnect before sleeping */
    if (k_sem_take(&m_DataAcqSem, K_NO_WAIT) == 0)
    {
        LOG_INF("Streaming active, waiting for disconnect...");
        k_sem_take(&m_DataAcqDisconnectSem, K_FOREVER);
    }

    /* Stop advertising before sleep */
    BLE_AdvStop();

    /* Set GPIOs to low-power state */
    SysLP_EnterSleep();

    LOG_INF("Sleeping for %d seconds", Settings->DInterval);
    k_sleep(K_SECONDS(Settings->DInterval));

    /* Restore GPIOs to active state */
    SysLP_ExitSleep();

    Main_HeartbeatLed();

    /* Re-init PSRAM after power gate cycle */
    if (m_SensorState[SENSOR_PSRAM] == 0)
    {
        if (PSRAM_Init() != 0)
        {
            LOG_WRN("PSRAM re-init failed after wake");
            m_SensorState[SENSOR_PSRAM] = -1;
        }
    }

    /* Restart advertising after wake */
    BLE_AdvStart();
}

/*******************************************************************************
Description:
Toggle the heartbeat LED. Called once per acquisition cycle as a visual
indication that the system is alive.

Argument(s):
None

Return:
None
*******************************************************************************/
static void Main_HeartbeatLed(void)
{
    gpio_pin_set_dt(&m_HeartbeatLed, 1);
    k_msleep(HEARTBEAT_DELAY_MS);
    gpio_pin_set_dt(&m_HeartbeatLed, 0);
}

/*******************************************************************************
Description:
Accelerometer high-speed streaming work callback. Triggered by BLE command
0x0A or 0x0C. Continuously reads accelerometer at the configured high sample
rate and sends raw data via BLE notifications.

Argument(s):
Work - Pointer to the work item (unused).

Return:
None

Note(s):
Reads accelerometer at high sample rate, buffers into PSRAM if available,
then streams raw data via BLE notifications until the buffer is exhausted
or the client unsubscribes.
*******************************************************************************/
static void Main_AccelStreamCb(struct k_work *Work)
{
    ARG_UNUSED(Work);

    BLENotifyEn_t *NotifyStatus = BLE_GetNotifyStatus();
    SystemSettings_t *Settings = UserConfig_GetSettings();
    AccelConfig_t StreamCfg;
    uint32_t BufLen;
    uint32_t Offset;

    LOG_INF("Accel streaming started");

    if (m_SensorState[SENSOR_ACCEL] != 0)
    {
        LOG_ERR("Accel not available for streaming");
        return;
    }

    StreamCfg.SamplingRate = Settings->HighSampleRate;
    StreamCfg.Duration     = Settings->HighSampleDuration;

    Accel_SetOdr(Settings->HighSampleRate);
    Accel_Active();

    if (Accel_ReadStreamData(m_AccelRawBuf, &StreamCfg) != 0)
    {
        LOG_ERR("Accel stream read failed");
        Accel_Standby();
        return;
    }

    BufLen = (uint32_t)StreamCfg.Duration * StreamCfg.SamplingRate * ACCEL_BYTES_PER_SAMPLE;
    Offset = 0;

    while (Offset < BufLen && NotifyStatus->VibStreamingEn)
    {
        uint16_t ChunkLen = (BufLen - Offset > TX_BUF_SIZE)
                          ? TX_BUF_SIZE : (uint16_t)(BufLen - Offset);

        BLE_SendStreamNotification(BLE_STREAM_SVC_ACCEL,
                                   (void *)((uint8_t *)m_AccelRawBuf + Offset),
                                   ChunkLen);
        Offset += ChunkLen;
        k_usleep(INTER_FRAME_SPACE_US);
    }

    /* Restore default ODR for RMS acquisition */
    Accel_SetOdr(ACCEL_DEFAULT_ODR_HZ);
    Accel_Standby();

    LOG_INF("Accel streaming complete (%u bytes sent)", Offset);
}

/*******************************************************************************
Description:
Magnetometer high-speed streaming work callback. Triggered by BLE command
0x0B or 0x0C. Reads magnetometer data and sends raw data via BLE
notifications.

Argument(s):
Work - Pointer to the work item (unused).

Return:
None

Note(s):
Reads magnetometer data at 400 Hz, then streams raw data via BLE
notifications until the buffer is exhausted or the client unsubscribes.
*******************************************************************************/
static void Main_MagStreamCb(struct k_work *Work)
{
    ARG_UNUSED(Work);

    BLENotifyEn_t *NotifyStatus = BLE_GetNotifyStatus();
    MagConfig_t MagCfg;
    uint32_t BufLen;
    uint32_t Offset;

    LOG_INF("Mag streaming started");

    if (m_SensorState[SENSOR_MAG] != 0)
    {
        LOG_ERR("Mag not available for streaming");
        return;
    }

    MagCfg.SamplingRate = MAG_DEFAULT_ODR_HZ;
    MagCfg.Duration     = MAG_DEFAULT_DURATION_S;

    Mag_Active();

    if (Mag_ReadData(m_MagRawBuf, &MagCfg) != 0)
    {
        LOG_ERR("Mag stream read failed");
        Mag_Standby();
        return;
    }

    BufLen = (uint32_t)MagCfg.Duration * MagCfg.SamplingRate * MAG_BYTES_PER_SAMPLE;
    Offset = 0;

    while (Offset < BufLen && NotifyStatus->MagStreamingEn)
    {
        uint16_t ChunkLen = (BufLen - Offset > TX_BUF_SIZE)
                          ? TX_BUF_SIZE : (uint16_t)(BufLen - Offset);

        BLE_SendStreamNotification(BLE_STREAM_SVC_MAG,
                                   (void *)((uint8_t *)m_MagRawBuf + Offset),
                                   ChunkLen);
        Offset += ChunkLen;
        k_usleep(INTER_FRAME_SPACE_US);
    }

    Mag_Standby();

    LOG_INF("Mag streaming complete (%u bytes sent)", Offset);
}
