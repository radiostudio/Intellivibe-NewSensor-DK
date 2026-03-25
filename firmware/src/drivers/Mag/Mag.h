/*******************************************************************************
********************************************************************************
Description:
This file defines the macros, register map, data types, and function
prototypes for the BMM350 3-axis magnetometer driver. Communicates via I2C,
supports continuous measurement mode with data-ready interrupt.

Replaces: BM1422AGMV (old firmware)
Part:      Bosch BMM350

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef MAG_H
#define MAG_H

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
#define MAG_I2C_ADDR                0x14
#define MAG_CHIP_ID_VAL             0x33

/* ── Register addresses (BMM350) ── */
#define MAG_REG_CHIP_ID             0x00
#define MAG_REG_REV_ID              0x01
#define MAG_REG_ERR_REG             0x02
#define MAG_REG_PAD_CTRL            0x03
#define MAG_REG_PMU_CMD_AGGR_SET    0x04
#define MAG_REG_PMU_CMD_AXIS_EN     0x05
#define MAG_REG_PMU_CMD             0x06
#define MAG_REG_PMU_CMD_STATUS_0    0x07
#define MAG_REG_INT_CTRL            0x2E
#define MAG_REG_INT_CTRL_IBI        0x2F
#define MAG_REG_INT_STATUS          0x30
#define MAG_REG_MAG_X_XLSB         0x31
#define MAG_REG_MAG_X_LSB          0x32
#define MAG_REG_MAG_X_MSB          0x33
#define MAG_REG_MAG_Y_XLSB         0x34
#define MAG_REG_MAG_Y_LSB          0x35
#define MAG_REG_MAG_Y_MSB          0x36
#define MAG_REG_MAG_Z_XLSB         0x37
#define MAG_REG_MAG_Z_LSB          0x38
#define MAG_REG_MAG_Z_MSB          0x39
#define MAG_REG_TEMP_XLSB          0x3A
#define MAG_REG_TEMP_LSB           0x3B
#define MAG_REG_TEMP_MSB           0x3C
#define MAG_REG_OTP_CMD            0x50
#define MAG_REG_OTP_DATA_MSB       0x52
#define MAG_REG_OTP_DATA_LSB       0x53
#define MAG_REG_OTP_STATUS         0x55
#define MAG_REG_CMD                0x7E

/* ── PMU_CMD power modes ── */
#define MAG_PMU_CMD_SUSPEND         0x00
#define MAG_PMU_CMD_NORMAL          0x01
#define MAG_PMU_CMD_UPD_OAE         0x02        /* Commit ODR/AVG/axis settings (must follow AGGR_SET write) */
#define MAG_PMU_CMD_FORCED          0x03        /* reserved — single forced measurement, not currently used */
#define MAG_PMU_CMD_FORCED_FAST     0x04        /* reserved — fast forced measurement, not currently used */

/* ── PMU_CMD_AGGR_SET: ODR [3:0] ── */
#define MAG_ODR_400HZ               0x02
#define MAG_ODR_200HZ               0x03
#define MAG_ODR_100HZ               0x04
#define MAG_ODR_50HZ                0x05
#define MAG_ODR_25HZ                0x06
#define MAG_ODR_12_5HZ              0x07
#define MAG_ODR_6_25HZ              0x08
#define MAG_ODR_3_125HZ             0x09
#define MAG_ODR_1_5625HZ            0x0A

/* ── PMU_CMD_AGGR_SET: Averaging [4:7] ── */
#define MAG_AVG_NO_AVG              (0x00 << 4)
#define MAG_AVG_2                   (0x01 << 4)    /* reserved — 2x averaging, not currently used */
#define MAG_AVG_4                   (0x02 << 4)    /* reserved — 4x averaging, not currently used */
#define MAG_AVG_8                   (0x03 << 4)    /* reserved — 8x averaging, not currently used */

/* ── PMU_CMD_AXIS_EN bits ── */
#define MAG_AXIS_EN_X               (1 << 0)
#define MAG_AXIS_EN_Y               (1 << 1)
#define MAG_AXIS_EN_Z               (1 << 2)
#define MAG_AXIS_EN_ALL             (MAG_AXIS_EN_X | MAG_AXIS_EN_Y | MAG_AXIS_EN_Z)

/* ── INT_CTRL bits ── */
#define MAG_INT_MODE_PULSED         0x00           /* reserved — pulsed mode (default 0), not explicitly used */
#define MAG_INT_MODE_LATCHED        (1 << 0)       /* reserved — latched mode, not currently used */
#define MAG_INT_POL_ACTIVE_LOW      0x00           /* reserved — active-low polarity (default 0), not explicitly used */
#define MAG_INT_POL_ACTIVE_HIGH     (1 << 1)
#define MAG_INT_OD_PUSHPULL         0x00
#define MAG_INT_OD_OPENDRAIN        (1 << 2)       /* reserved — open-drain output, not currently used */
#define MAG_INT_OUTPUT_EN           (1 << 3)
#define MAG_INT_DRDY_EN             (1 << 7)

/* ── INT_STATUS bits ── */
#define MAG_INT_STATUS_DRDY         (1 << 2)       /* reserved — DRDY status poll, not currently used */

/* ── CMD register ── */
#define MAG_CMD_SOFT_RESET          0xB6

/* ── OTP commands ── */
#define MAG_OTP_CMD_PWR_OFF         0x80        /* Power off OTP after boot — required to enable measurements */

/* ── Conversion (Bosch SensorAPI factors) ── */
#define MAG_UT_PER_LSB_XY           0.00707f        /* XY axes: ~(1e6/1048576)/(14.55*19.46*(1/1.5)*0.714607) */
#define MAG_UT_PER_LSB_Z            0.00718f        /* Z axis:  ~(1e6/1048576)/(9.0*31.0*(1/1.5)*0.714607)   */

/* ── Buffer sizes ── */
#define MAG_RAW_BUFFER_LEN          5010
#define MAG_DATA_READ_SIZE          12              /* 3 axes + temp, 3 bytes each (BMM350_MAG_TEMP_DATA_LEN) */

//******************************************************************************
// DATA TYPES
//******************************************************************************
typedef struct
{
    int32_t XValue;
    int32_t YValue;
    int32_t ZValue;
    int32_t TValue;     /* Raw temperature (24-bit signed), used for per-sample OTP compensation */
} MagRawData_t;


typedef struct
{
    uint16_t SamplingRate;
    uint16_t Duration;
} MagConfig_t;

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int   Mag_Init(void);
int   Mag_ReadData(MagRawData_t *Data, MagConfig_t *Config);
int   Mag_ReadRaw(MagRawData_t *Data);
void  Mag_SetOdr(uint16_t RateHz);
int   Mag_Active(void);
int   Mag_Standby(void);
int   Mag_EnableInterrupt(void);
int   Mag_DisableInterrupt(void);
int   Mag_GpioInit(void);
void  Mag_CalculateRms(MagRawData_t *Data, uint16_t SampleCount, OutputData_t *MagRms);

#ifdef __cplusplus
}
#endif

#endif /* MAG_H */
