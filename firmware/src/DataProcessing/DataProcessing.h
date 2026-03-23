/*******************************************************************************
********************************************************************************
Description:
This file defines the macros, data types, and function prototypes for the
shared signal processing module. Provides FFT-based accel RMS and velocity
RMS computation using CMSIS-DSP.

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include "CommonTypes.h"
#include "Accel/Accel.h"

//******************************************************************************
// DEFINES
//******************************************************************************

/* FFT configuration */
#define DATAPROC_ACCEL_FFT_LEN          4096
#define DATAPROC_ACCEL_SAMPLE_LEN       3200

/* Frequency window for RMS integration (Hz) */
#define DATAPROC_LOWER_CUTOFF_HZ        2
#define DATAPROC_UPPER_CUTOFF_HZ        1000

/*
 * Frequency resolution: ODR / FFT_LEN = 3200 / 4096 = 0.78125 Hz/bin
 * Used to convert Hz cutoffs to bin indices.
 */
#define DATAPROC_FREQ_RES_HZ            0.78125f

/*
 * FFT magnitude scaling factor: 1 / (FFT_LEN / 2) = 1 / 2048
 * Normalises the single-sided spectrum amplitude.
 */
#define DATAPROC_FFT_SCALE_FACTOR       0.00048828125f

/* Axis count */
#define DATAPROC_AXIS_COUNT             3

/* Velocity conversion: g to mm/s requires * 9806.65 / (2 * PI * f) */
#define DATAPROC_GRAVITY_MM_S2          9806.65f

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int  DataProc_Init(void);
int  DataProc_ComputeAccelRms(AccelRawData_t *RawData, uint16_t SampleCount,
                               float Sensitivity,
                               OutputData_t *AccelRms, OutputData_t *VelRms);

#ifdef __cplusplus
}
#endif

#endif /* DATAPROCESSING_H */
