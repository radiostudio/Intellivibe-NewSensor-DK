/*******************************************************************************
********************************************************************************
Description:
This file defines the macros, register map, data types, and function
prototypes for the IIM-42352 3-axis accelerometer driver. Communicates
via SPI, supports FIFO batch reads and data-ready interrupt.

Replaces: KX132-1211 (old firmware)
Part:      TDK InvenSense IIM-42352

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef ACCEL_H
#define ACCEL_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "CommonTypes.h"

//******************************************************************************
// DEFINES
//******************************************************************************

/* ── Device identification ── */
#define ACCEL_WHO_AM_I_VAL          0x6D

/* ── SPI protocol ── */
#define ACCEL_SPI_READ              0x80
#define ACCEL_SPI_WRITE             0x00

/* ── Register addresses (IIM-42352) ── */
#define ACCEL_REG_DEVICE_CONFIG     0x11
#define ACCEL_REG_INT_CONFIG        0x14
#define ACCEL_REG_FIFO_CONFIG       0x16
#define ACCEL_REG_TEMP_DATA1        0x1D
#define ACCEL_REG_TEMP_DATA0        0x1E
#define ACCEL_REG_ACCEL_DATA_X1     0x1F
#define ACCEL_REG_ACCEL_DATA_X0     0x20
#define ACCEL_REG_ACCEL_DATA_Y1     0x21
#define ACCEL_REG_ACCEL_DATA_Y0     0x22
#define ACCEL_REG_ACCEL_DATA_Z1     0x23
#define ACCEL_REG_ACCEL_DATA_Z0     0x24
#define ACCEL_REG_INT_STATUS        0x2D
#define ACCEL_REG_FIFO_COUNTH       0x2E
#define ACCEL_REG_FIFO_COUNTL       0x2F
#define ACCEL_REG_FIFO_DATA         0x30
#define ACCEL_REG_SIGNAL_PATH_RESET 0x4B
#define ACCEL_REG_INTF_CONFIG0      0x4C
#define ACCEL_REG_PWR_MGMT0        0x4E
#define ACCEL_REG_ACCEL_CONFIG0     0x50
#define ACCEL_REG_ACCEL_CONFIG1     0x53
#define ACCEL_REG_FIFO_CONFIG1      0x5F
#define ACCEL_REG_FIFO_CONFIG2      0x60
#define ACCEL_REG_FIFO_CONFIG3      0x61
#define ACCEL_REG_INT_SOURCE0       0x65
#define ACCEL_REG_WHO_AM_I          0x75

/* ── PWR_MGMT0 bits ── */
#define ACCEL_PWR_MODE_OFF          0x00
#define ACCEL_PWR_MODE_STANDBY      0x01
#define ACCEL_PWR_MODE_LP           0x02
#define ACCEL_PWR_MODE_LN           0x03

/* ── ACCEL_CONFIG0: Full-scale range [6:5] ── */
#define ACCEL_FS_SEL_16G            (0x00 << 5)
#define ACCEL_FS_SEL_8G             (0x01 << 5)
#define ACCEL_FS_SEL_4G             (0x02 << 5)
#define ACCEL_FS_SEL_2G             (0x03 << 5)

/* ── ACCEL_CONFIG0: Output data rate [3:0] ── */
#define ACCEL_ODR_32KHZ             0x01
#define ACCEL_ODR_16KHZ             0x02
#define ACCEL_ODR_8KHZ              0x03
#define ACCEL_ODR_4KHZ              0x04
#define ACCEL_ODR_2KHZ              0x05
#define ACCEL_ODR_1KHZ              0x06
#define ACCEL_ODR_200HZ             0x07
#define ACCEL_ODR_100HZ             0x08
#define ACCEL_ODR_50HZ              0x09
#define ACCEL_ODR_25HZ              0x0A
#define ACCEL_ODR_12_5HZ            0x0B
#define ACCEL_ODR_6_25HZ            0x0C
#define ACCEL_ODR_3_125HZ           0x0D
#define ACCEL_ODR_1_5625HZ          0x0E
#define ACCEL_ODR_500HZ             0x0F

/* ── FIFO configuration ── */
#define ACCEL_FIFO_MODE_BYPASS      0x00
#define ACCEL_FIFO_MODE_STREAM      0x01
#define ACCEL_FIFO_MODE_STOP_FULL   0x02

#define ACCEL_FIFO_ACCEL_EN         (1 << 0)
#define ACCEL_FIFO_TEMP_EN          (1 << 2)
#define ACCEL_FIFO_TMST_EN          (1 << 3)
#define ACCEL_FIFO_HIRES_EN         (1 << 4)
#define ACCEL_FIFO_WM_GT_TH        (1 << 5)

/* ── INT_CONFIG bits ── */
#define ACCEL_INT1_MODE_PULSED      0x00
#define ACCEL_INT1_MODE_LATCHED     (1 << 2)
#define ACCEL_INT1_POLARITY_HIGH    (1 << 0)
#define ACCEL_INT1_DRIVE_PP         (1 << 1)

/* ── INT_SOURCE0 bits ── */
#define ACCEL_INT_SRC_FIFO_THS      (1 << 2)
#define ACCEL_INT_SRC_FIFO_FULL     (1 << 1)
#define ACCEL_INT_SRC_DRDY          (1 << 3)

/* ── INT_STATUS bits ── */
#define ACCEL_INT_STATUS_FIFO_THS   (1 << 2)
#define ACCEL_INT_STATUS_FIFO_FULL  (1 << 1)
#define ACCEL_INT_STATUS_DRDY       (1 << 3)
#define ACCEL_INT_STATUS_RESET_DONE (1 << 4)

/* ── SIGNAL_PATH_RESET ── */
#define ACCEL_SOFT_RESET            (1 << 0)
#define ACCEL_FIFO_FLUSH            (1 << 1)

/* ── FIFO packet ── */
#define ACCEL_FIFO_PACKET_SIZE      8
#define ACCEL_FIFO_MAX_PACKETS      256

/* ── Sensitivity (LSB/g) ── */
#define ACCEL_SENSITIVITY_2G        16384.0f
#define ACCEL_SENSITIVITY_4G        8192.0f
#define ACCEL_SENSITIVITY_8G        4096.0f
#define ACCEL_SENSITIVITY_16G       2048.0f

/* ── Buffer sizes ── */
#define ACCEL_RAW_BUFFER_LEN        4900
#define ACCEL_STREAM_BUFFER_LEN     200

//******************************************************************************
// DATA TYPES
//******************************************************************************
typedef struct
{
    int16_t XValue;
    int16_t YValue;
    int16_t ZValue;
} AccelRawData_t;

typedef struct
{
    uint16_t SamplingRate;
    uint16_t Duration;
} AccelConfig_t;

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int  Accel_Init(void);
int  Accel_ReadData(AccelRawData_t *Data, AccelConfig_t *Config);
int  Accel_ReadStreamData(AccelRawData_t *Data, AccelConfig_t *Config);
void Accel_SetOdr(uint16_t RateHz);
void Accel_SetRange(uint8_t Range);
int  Accel_EnableInterrupt(void);
int  Accel_DisableInterrupt(void);
int  Accel_Standby(void);
int  Accel_Active(void);
int  Accel_GpioInit(void);
float Accel_GetSensitivity(void);

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_H */
