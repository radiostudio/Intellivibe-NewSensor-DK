/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for BLE handling.
Includes BLE initialisation, advertising, connection management,
and streaming service.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef BLE_H
#define BLE_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include "CommonTypes.h"

//******************************************************************************
// DEFINES
//******************************************************************************
#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

/** @brief UUID of the Streaming Service. */
#define BT_UUID_VIB_STREAM_SERV_VAL \
    BT_UUID_128_ENCODE(0xc1604cb4, 0x2064, 0x4bb8, 0x8085, 0x9310e83c5782)

/** @brief UUID of the Vibration Streaming Characteristic. */
#define BT_UUID_VIB_STREAM_CHRC_VAL \
    BT_UUID_128_ENCODE(0xc1604cb5, 0x2064, 0x4bb8, 0x8085, 0x9310e83c5782)

/** @brief UUID of the Magnetometer Streaming Characteristic. */
#define BT_UUID_MAG_STREAM_CHRC_VAL \
    BT_UUID_128_ENCODE(0xc1604cb6, 0x2064, 0x4bb8, 0x8085, 0x9310e83c5782)

#define BT_UUID_STREAM_SERVICE  BT_UUID_DECLARE_128(BT_UUID_VIB_STREAM_SERV_VAL)
#define BT_UUID_VIB_STREAM_CHRC BT_UUID_DECLARE_128(BT_UUID_VIB_STREAM_CHRC_VAL)
#define BT_UUID_MAG_STREAM_CHRC BT_UUID_DECLARE_128(BT_UUID_MAG_STREAM_CHRC_VAL)

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int             BLE_Init(void);
int             BLE_DeInit(void);
int             BLE_AdvStart(void);
int             BLE_AdvStop(void);
void            BLE_AdvUpdate(void);
void            BLE_SetAdvData(AdvMfgData_t *AdvData);
int             BLE_SendStreamNotification(uint8_t ServiceNo, void *Value, uint16_t Len);
struct bt_conn *BLE_GetCurrentConn(void);
BLENotifyEn_t  *BLE_GetNotifyStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */
