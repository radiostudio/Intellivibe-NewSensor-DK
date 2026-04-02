/*******************************************************************************
********************************************************************************
Description:
This file handles functions for BLE management.
Includes BLE stack initialisation, advertising control, connection callbacks,
MTU/data length negotiation, and streaming GATT service.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BLE);

#include <zephyr/kernel.h>
#include "ble.h"

extern void Main_StopStreaming(void);

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define COMPANY_ID_CODE     0x0059

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static K_SEM_DEFINE(m_BLEInitOk, 0, 1);

static struct bt_conn              *m_CurrentConn;
static struct bt_gatt_exchange_params m_ExchangeParams;

static BLENotifyEn_t m_NotifyCheck = {
    .VibStreamingEn = false,
    .MagStreamingEn = false,
};

static AdvMfgData_t m_AdvMfgData = {
    .CompanyCode    = COMPANY_ID_CODE,
    .AccelRms       = {0},
    .VelRms         = {0},
    .MagRms         = {0},
    .Temperature    = 0,
    .Noise          = 0,
    .BatLevel       = 0,
};

static const struct bt_data m_AdvData[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&m_AdvMfgData, sizeof(m_AdvMfgData)),
};

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static void     BLE_OnConnected(struct bt_conn *Conn, uint8_t Err);
static void     BLE_OnDisconnected(struct bt_conn *Conn, uint8_t Reason);
static void     BLE_OnLeParamUpdated(struct bt_conn *Conn, uint16_t Interval,
                                     uint16_t Latency, uint16_t Timeout);
static void     BLE_OnLeDataLenUpdated(struct bt_conn *Conn,
                                       struct bt_conn_le_data_len_info *Info);
static void     BLE_OnSent(struct bt_conn *Conn, void *UserData);
static void     BLE_UpdateDataLength(struct bt_conn *Conn);
static void     BLE_UpdateMtu(struct bt_conn *Conn);
static void     BLE_ExchangeFunc(struct bt_conn *Conn, uint8_t AttErr,
                                 struct bt_gatt_exchange_params *Params);
static void     BLE_Ready(int Err);
static ssize_t  BLE_StreamingRead(struct bt_conn *Conn,
                                  const struct bt_gatt_attr *Attr,
                                  void *Buf, uint16_t Len, uint16_t Offset);
static void     BLE_VibStreamingCccChanged(const struct bt_gatt_attr *Attr,
                                           uint16_t Value);
static void     BLE_MagStreamingCccChanged(const struct bt_gatt_attr *Attr,
                                           uint16_t Value);

/*---------------------------------------------------------------------------*/
/* GATT Streaming Service Definition                                         */
/*---------------------------------------------------------------------------*/
BT_GATT_SERVICE_DEFINE(m_StreamSrv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_STREAM_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_VIB_STREAM_CHRC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           BLE_StreamingRead, NULL, NULL),
    BT_GATT_CCC(BLE_VibStreamingCccChanged,
                 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_MAG_STREAM_CHRC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           BLE_StreamingRead, NULL, NULL),
    BT_GATT_CCC(BLE_MagStreamingCccChanged,
                 BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

/*---------------------------------------------------------------------------*/
/* Connection Callbacks                                                      */
/*---------------------------------------------------------------------------*/
static struct bt_conn_cb m_BtCallbacks = {
    .connected          = BLE_OnConnected,
    .disconnected       = BLE_OnDisconnected,
    .le_param_updated   = BLE_OnLeParamUpdated,
    .le_data_len_updated = BLE_OnLeDataLenUpdated,
};

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the BLE stack and start advertising. Registers connection callbacks,
enables the BLE stack via bt_enable(), waits for the ready callback, then starts
connectable advertising with manufacturer data.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int BLE_Init(void)
{
    int Err;

    LOG_INF("Initialising BT");

    bt_conn_cb_register(&m_BtCallbacks);

    Err = bt_enable(BLE_Ready);
    if (Err)
    {
        LOG_ERR("bt_enable failed: %d", Err);
        return Err;
    }

    k_sem_take(&m_BLEInitOk, K_FOREVER);

    Err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, m_AdvData, ARRAY_SIZE(m_AdvData),
                          NULL, 0);
    if (Err)
    {
        LOG_ERR("Advertising failed to start: %d", Err);
        return Err;
    }

    LOG_INF("BLE initialised, advertising started");
    return 0;
}

/*******************************************************************************
Description:
Deinitialise the BLE stack. Stops advertising and disables the BLE stack.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int BLE_DeInit(void)
{
    int Err;

    Err = bt_le_adv_stop();
    if (Err)
    {
        LOG_ERR("Failed to stop advertising: %d", Err);
    }
    else
    {
        LOG_INF("Advertising stopped");
    }

    bt_disable();
    LOG_INF("Bluetooth stack disabled");
    return Err;
}

/*******************************************************************************
Description:
Start BLE connectable advertising with current manufacturer data.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int BLE_AdvStart(void)
{
    int Err;

    Err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, m_AdvData, ARRAY_SIZE(m_AdvData),
                          NULL, 0);
    if (Err)
    {
        LOG_ERR("Advertising failed to start: %d", Err);
    }

    return Err;
}

/*******************************************************************************
Description:
Stop BLE advertising.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int BLE_AdvStop(void)
{
    int Err;

    Err = bt_le_adv_stop();
    if (Err)
    {
        LOG_ERR("Failed to stop advertising: %d", Err);
    }
    else
    {
        LOG_INF("Advertising stopped");
    }

    return Err;
}

/*******************************************************************************
Description:
Update the advertising data with the current contents of m_AdvMfgData.
Call this after modifying the advertising payload via BLE_SetAdvData().

Argument(s):
None

Return:
None
*******************************************************************************/
void BLE_AdvUpdate(void)
{
    int Err;

    Err = bt_le_adv_update_data(m_AdvData, ARRAY_SIZE(m_AdvData), NULL, 0);
    if (Err)
    {
        LOG_ERR("Advertising data update failed: %d", Err);
    }
}

/*******************************************************************************
Description:
Copy new advertising payload into the internal manufacturer data struct.

Argument(s):
AdvData - Pointer to the new advertising data to copy in.

Return:
None
*******************************************************************************/
void BLE_SetAdvData(AdvMfgData_t *AdvData)
{
    if (AdvData == NULL)
    {
        return;
    }

    memcpy(&m_AdvMfgData, AdvData, sizeof(AdvMfgData_t));
}

/*******************************************************************************
Description:
Send a BLE GATT notification on the streaming service.

Argument(s):
ServiceNo - Attribute index within the streaming service.
Value     - Pointer to the data to send.
Len       - Length of the data in bytes.

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int BLE_SendStreamNotification(uint8_t ServiceNo, void *Value, uint16_t Len)
{
    int Err = 0;
    struct bt_gatt_notify_params Params = {0};
    const struct bt_gatt_attr *Attr = &m_StreamSrv.attrs[ServiceNo];

    Params.attr = Attr;
    Params.data = Value;
    Params.len  = Len;
    Params.func = BLE_OnSent;

    Err = bt_gatt_notify_cb(m_CurrentConn, &Params);
    if (Err)
    {
        LOG_ERR("Streaming notify failed: %d", Err);
    }

    return Err;
}

/*******************************************************************************
Description:
Return the current active BLE connection, or NULL if not connected.

Argument(s):
None

Return:
struct bt_conn* - Pointer to current connection, or NULL.
*******************************************************************************/
struct bt_conn *BLE_GetCurrentConn(void)
{
    return m_CurrentConn;
}

/*******************************************************************************
Description:
Return a pointer to the notification enable status struct.

Argument(s):
None

Return:
BLENotifyEn_t* - Pointer to the notify enable flags.
*******************************************************************************/
BLENotifyEn_t *BLE_GetNotifyStatus(void)
{
    return &m_NotifyCheck;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
BLE stack ready callback. Called by bt_enable() when the stack is initialised.
Gives the m_BLEInitOk semaphore to unblock BLE_Init().

Argument(s):
Err - 0 on success, negative error code on failure.

Return:
None
*******************************************************************************/
static void BLE_Ready(int Err)
{
    if (Err)
    {
        LOG_ERR("BT enable failed callback: %d", Err);
        return;
    }

    LOG_INF("BT ready");
    k_sem_give(&m_BLEInitOk);
}

/*******************************************************************************
Description:
Connection established callback. Stores the connection reference, logs
connection parameters, and initiates data length and MTU negotiation.

Argument(s):
Conn - Pointer to the new connection object.
Err  - 0 on success, HCI error code on failure.

Return:
None
*******************************************************************************/
static void BLE_OnConnected(struct bt_conn *Conn, uint8_t Err)
{
    if (Err)
    {
        LOG_ERR("Connection error: %d", Err);
        return;
    }

    LOG_INF("Connected");
    m_CurrentConn = bt_conn_ref(Conn);

    struct bt_conn_info Info;
    Err = bt_conn_get_info(Conn, &Info);
    if (Err)
    {
        LOG_ERR("bt_conn_get_info() returned %d", Err);
        return;
    }

    double ConnInterval = (double)Info.le.interval_us / 1000.0;
    uint16_t SupervisionTimeout = Info.le.timeout * 10;
    LOG_INF("Conn params: interval %.2f ms, latency %d, timeout %d ms",
            ConnInterval, Info.le.latency, SupervisionTimeout);

    BLE_UpdateDataLength(Conn);
    BLE_UpdateMtu(Conn);
}

/*******************************************************************************
Description:
Disconnection callback. Unreferences the stored connection and logs the reason.

Argument(s):
Conn   - Pointer to the disconnected connection object.
Reason - HCI disconnect reason code.

Return:
None
*******************************************************************************/
static void BLE_OnDisconnected(struct bt_conn *Conn, uint8_t Reason)
{
    LOG_INF("Disconnected (reason: %d)", Reason);

    if (m_CurrentConn)
    {
        bt_conn_unref(m_CurrentConn);
        m_CurrentConn = NULL;
    }

    /* Signal acquisition thread to resume after streaming */
    Main_StopStreaming();
}

/*******************************************************************************
Description:
Connection parameter update callback. Logs the new connection parameters.

Argument(s):
Conn     - Pointer to the connection object.
Interval - New connection interval in 1.25 ms units.
Latency  - New slave latency.
Timeout  - New supervision timeout in 10 ms units.

Return:
None
*******************************************************************************/
static void BLE_OnLeParamUpdated(struct bt_conn *Conn, uint16_t Interval,
                                 uint16_t Latency, uint16_t Timeout)
{
    double ConnInterval = Interval * 1.25;
    uint16_t SupervisionTimeout = Timeout * 10;

    LOG_INF("Conn params updated: interval %.2f ms, latency %d, timeout %d ms",
            ConnInterval, Latency, SupervisionTimeout);
}

/*******************************************************************************
Description:
Data length update callback. Logs the new TX/RX data lengths and times.

Argument(s):
Conn - Pointer to the connection object.
Info - Pointer to the data length info structure.

Return:
None
*******************************************************************************/
static void BLE_OnLeDataLenUpdated(struct bt_conn *Conn,
                                   struct bt_conn_le_data_len_info *Info)
{
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us",
            Info->tx_max_len, Info->rx_max_len,
            Info->tx_max_time, Info->rx_max_time);
}

/*******************************************************************************
Description:
Notification sent callback. Logs when a notification has been delivered.

Argument(s):
Conn     - Pointer to the connection object.
UserData - User data (unused).

Return:
None
*******************************************************************************/
static void BLE_OnSent(struct bt_conn *Conn, void *UserData)
{
    ARG_UNUSED(UserData);
    LOG_DBG("Notification sent on connection %p", (void *)Conn);
}

/*******************************************************************************
Description:
Request a data length update to the maximum supported values.

Argument(s):
Conn - Pointer to the connection to update.

Return:
None
*******************************************************************************/
static void BLE_UpdateDataLength(struct bt_conn *Conn)
{
    int Err;
    struct bt_conn_le_data_len_param DataLen = {
        .tx_max_len  = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    Err = bt_conn_le_data_len_update(Conn, &DataLen);
    if (Err)
    {
        LOG_ERR("Data length update failed: %d", Err);
    }
}

/*******************************************************************************
Description:
Initiate an ATT MTU exchange with the connected peer.

Argument(s):
Conn - Pointer to the connection to update.

Return:
None
*******************************************************************************/
static void BLE_UpdateMtu(struct bt_conn *Conn)
{
    int Err;

    m_ExchangeParams.func = BLE_ExchangeFunc;
    Err = bt_gatt_exchange_mtu(Conn, &m_ExchangeParams);
    if (Err)
    {
        LOG_ERR("MTU exchange failed: %d", Err);
    }
}

/*******************************************************************************
Description:
MTU exchange completion callback. Logs the result and new payload MTU.

Argument(s):
Conn   - Pointer to the connection.
AttErr - ATT error code (0 on success).
Params - Pointer to the exchange parameters.

Return:
None
*******************************************************************************/
static void BLE_ExchangeFunc(struct bt_conn *Conn, uint8_t AttErr,
                             struct bt_gatt_exchange_params *Params)
{
    LOG_INF("MTU exchange %s", AttErr == 0 ? "successful" : "failed");

    if (!AttErr)
    {
        uint16_t PayloadMtu = bt_gatt_get_mtu(Conn) - 3;
        LOG_INF("New MTU: %d bytes", PayloadMtu);
    }
}

/*******************************************************************************
Description:
GATT read handler for the streaming characteristics. Returns a placeholder
byte (0x41) to acknowledge the read request.

Argument(s):
Conn   - Pointer to the connection.
Attr   - Pointer to the GATT attribute being read.
Buf    - Output buffer for the read data.
Len    - Maximum bytes the caller can accept.
Offset - Read offset within the attribute value.

Return:
ssize_t - Number of bytes written to Buf, or negative error code.
*******************************************************************************/
static ssize_t BLE_StreamingRead(struct bt_conn *Conn,
                                 const struct bt_gatt_attr *Attr,
                                 void *Buf, uint16_t Len, uint16_t Offset)
{
    uint8_t Sign = 0x41;

    return bt_gatt_attr_read(Conn, Attr, Buf, Len, Offset, &Sign, sizeof(Sign));
}

/*******************************************************************************
Description:
CCC changed callback for vibration streaming characteristic. Enables or
disables vibration streaming notifications.

Argument(s):
Attr  - Pointer to the CCC attribute.
Value - New CCC value (BT_GATT_CCC_NOTIFY or 0).

Return:
None
*******************************************************************************/
static void BLE_VibStreamingCccChanged(const struct bt_gatt_attr *Attr,
                                       uint16_t Value)
{
    bool NotifyEnable = (Value == BT_GATT_CCC_NOTIFY);

    LOG_INF("VIB streaming notification %s", NotifyEnable ? "enabled" : "disabled");
    m_NotifyCheck.VibStreamingEn = NotifyEnable;
}

/*******************************************************************************
Description:
CCC changed callback for magnetometer streaming characteristic. Enables or
disables magnetometer streaming notifications.

Argument(s):
Attr  - Pointer to the CCC attribute.
Value - New CCC value (BT_GATT_CCC_NOTIFY or 0).

Return:
None
*******************************************************************************/
static void BLE_MagStreamingCccChanged(const struct bt_gatt_attr *Attr,
                                       uint16_t Value)
{
    bool NotifyEnable = (Value == BT_GATT_CCC_NOTIFY);

    LOG_INF("MAG streaming notification %s", NotifyEnable ? "enabled" : "disabled");
    m_NotifyCheck.MagStreamingEn = NotifyEnable;
}
