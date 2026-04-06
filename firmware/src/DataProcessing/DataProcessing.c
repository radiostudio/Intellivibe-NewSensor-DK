/*******************************************************************************
********************************************************************************
Description:
This file handles shared signal processing for the IntelliVibe-BT firmware.
Provides FFT-based accelerometer RMS and velocity RMS computation using
the CMSIS-DSP library (arm_rfft_fast_f32).

Processing pipeline per axis:
  1. Convert raw int16 samples to float, scale by sensitivity (counts → g)
  2. Zero-pad to DATAPROC_ACCEL_FFT_LEN (4096)
  3. Real FFT → complex spectrum
  4. Compute single-sided magnitude spectrum
  5. Accel RMS = sqrt(2 * sum(mag^2)) / 2 over [LOWER..UPPER] Hz bins
  6. Velocity RMS = integrate spectrum (divide by 2*pi*f), same window

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(DATAPROC, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <math.h>
#include <string.h>
#include "DataProcessing.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define DATAPROC_FFT_HALF_LEN       (DATAPROC_ACCEL_FFT_LEN / 2)
#define DATAPROC_LOWER_BIN          ((uint16_t)(DATAPROC_LOWER_CUTOFF_HZ / DATAPROC_FREQ_RES_HZ))
#define DATAPROC_UPPER_BIN          ((uint16_t)(DATAPROC_UPPER_CUTOFF_HZ / DATAPROC_FREQ_RES_HZ))
#define DATAPROC_REAL_IMAG_STRIDE   2
#define DATAPROC_RMS_SQRT_SCALE     2.0f

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static arm_rfft_fast_instance_f32 m_RfftInstance;
static float m_FftInput[DATAPROC_ACCEL_FFT_LEN];
static float m_FftOutput[DATAPROC_ACCEL_FFT_LEN];
static float m_Spectrum[DATAPROC_FFT_HALF_LEN];
static bool  m_Initialised = false;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static int   DataProc_FftInit(void);
static void  DataProc_PrepareInput(AccelRawData_t *RawData, uint16_t SampleCount,
                                    float Sensitivity, uint8_t Axis);
static void  DataProc_ComputeSpectrum(void);
static float DataProc_ComputeAccelRmsFromSpectrum(void);
static float DataProc_ComputeVelRmsFromSpectrum(void);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the CMSIS-DSP FFT instance for real-valued 4096-point FFT.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int DataProc_Init(void)
{
    int Ret;

    Ret = DataProc_FftInit();
    if (Ret != 0)
    {
        return -1;
    }

    m_Initialised = true;
    LOG_INF("DataProcessing initialised (FFT len=%d)", DATAPROC_ACCEL_FFT_LEN);
    return 0;
}

/*******************************************************************************
Description:
Compute accel RMS and velocity RMS for all three axes from raw accelerometer
data. Runs FFT + spectrum analysis independently on X, Y, Z.

Argument(s):
RawData     - Pointer to array of AccelRawData_t samples.
SampleCount - Number of valid samples in RawData.
Sensitivity - Accelerometer sensitivity in LSB/g (e.g. 4096.0 for 8G range).
AccelRms    - Output: per-axis accel RMS in g.
VelRms      - Output: per-axis velocity RMS in mm/s.

Return:
int - 0 on success, -1 if not initialised.
*******************************************************************************/
int DataProc_ComputeAccelRms(AccelRawData_t *RawData, uint16_t SampleCount,
                              float Sensitivity,
                              OutputData_t *AccelRms, OutputData_t *VelRms)
{
    if (!m_Initialised)
    {
        LOG_ERR("DataProcessing not initialised");
        return -1;
    }

    /* X axis */
    DataProc_PrepareInput(RawData, SampleCount, Sensitivity, 0);
    DataProc_ComputeSpectrum();
    AccelRms->XData = DataProc_ComputeAccelRmsFromSpectrum();
    VelRms->XData   = DataProc_ComputeVelRmsFromSpectrum();

    /* Y axis */
    DataProc_PrepareInput(RawData, SampleCount, Sensitivity, 1);
    DataProc_ComputeSpectrum();
    AccelRms->YData = DataProc_ComputeAccelRmsFromSpectrum();
    VelRms->YData   = DataProc_ComputeVelRmsFromSpectrum();

    /* Z axis */
    DataProc_PrepareInput(RawData, SampleCount, Sensitivity, DATAPROC_AXIS_COUNT - 1);
    DataProc_ComputeSpectrum();
    AccelRms->ZData = DataProc_ComputeAccelRmsFromSpectrum();
    VelRms->ZData   = DataProc_ComputeVelRmsFromSpectrum();

    return 0;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the CMSIS-DSP real FFT instance for the configured FFT length.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
static int DataProc_FftInit(void)
{
    arm_status Status;

    Status = arm_rfft_fast_init_f32(&m_RfftInstance, DATAPROC_ACCEL_FFT_LEN);
    if (Status != ARM_MATH_SUCCESS)
    {
        LOG_ERR("arm_rfft_fast_init_f32 failed: %d", Status);
        return -1;
    }

    return 0;
}

/*******************************************************************************
Description:
Prepare FFT input buffer: extract one axis from the raw sample array,
convert from raw counts to g using the sensitivity, subtract the DC mean
to remove offset, then zero-pad the remainder of the buffer to the FFT
length.

Argument(s):
RawData     - Pointer to array of AccelRawData_t samples.
SampleCount - Number of valid samples.
Sensitivity - LSB/g conversion factor.
Axis        - 0 = X, 1 = Y, 2 = Z.

Return:
None
*******************************************************************************/
static void DataProc_PrepareInput(AccelRawData_t *RawData, uint16_t SampleCount,
                                   float Sensitivity, uint8_t Axis)
{
    uint16_t Idx;
    float InvSensitivity = 1.0f / Sensitivity;
    float Mean = 0.0f;

    /* Cap to FFT input length to prevent overflow */
    if (SampleCount > DATAPROC_ACCEL_FFT_LEN)
    {
        SampleCount = DATAPROC_ACCEL_FFT_LEN;
    }

    /* Convert raw counts to g */
    for (Idx = 0; Idx < SampleCount; Idx++)
    {
        switch (Axis)
        {
        case 0:
            m_FftInput[Idx] = (float)RawData[Idx].XValue * InvSensitivity;
            break;
        case 1:
            m_FftInput[Idx] = (float)RawData[Idx].YValue * InvSensitivity;
            break;
        default:
            m_FftInput[Idx] = (float)RawData[Idx].ZValue * InvSensitivity;
            break;
        }
        Mean += m_FftInput[Idx];
    }

    /* Remove DC offset (subtract per-axis mean) */
    Mean /= (float)SampleCount;
    for (Idx = 0; Idx < SampleCount; Idx++)
    {
        m_FftInput[Idx] -= Mean;
    }

    /* Zero-pad from SampleCount to FFT length */
    memset(&m_FftInput[SampleCount], 0,
           (DATAPROC_ACCEL_FFT_LEN - SampleCount) * sizeof(float));
}

/*******************************************************************************
Description:
Run the real FFT and compute the single-sided magnitude spectrum.
The CMSIS FFT output is interleaved [Re0, Im0, Re1, Im1, ...].
Magnitude = scale * sqrt(Re^2 + Im^2).

Argument(s):
None

Return:
None

Note(s):
Operates on m_FftInput, writes m_FftOutput and m_Spectrum.
*******************************************************************************/
static void DataProc_ComputeSpectrum(void)
{
    uint16_t Bin;
    uint16_t RealIdx;
    uint16_t ImagIdx;
    float Real;
    float Imag;

    arm_rfft_fast_f32(&m_RfftInstance, m_FftInput, m_FftOutput, 0);

    for (Bin = 1; Bin < DATAPROC_FFT_HALF_LEN; Bin++)
    {
        RealIdx = Bin * DATAPROC_REAL_IMAG_STRIDE;
        ImagIdx = RealIdx + 1;
        Real = m_FftOutput[RealIdx];
        Imag = m_FftOutput[ImagIdx];
        m_Spectrum[Bin] = DATAPROC_FFT_SCALE_FACTOR * sqrtf(Real * Real + Imag * Imag);
    }

    m_Spectrum[0] = 0.0f;
}

/*******************************************************************************
Description:
Compute acceleration RMS from the magnitude spectrum over the configured
frequency window [LOWER_BIN .. UPPER_BIN].

Formula: RMS = sqrt(2 * sum(mag^2)) / 2

Argument(s):
None

Return:
float - Accel RMS value in g.

Note(s):
Must be called after DataProc_ComputeSpectrum().
*******************************************************************************/
static float DataProc_ComputeAccelRmsFromSpectrum(void)
{
    uint16_t Bin;
    float Sum = 0.0f;

    for (Bin = DATAPROC_LOWER_BIN; Bin < DATAPROC_UPPER_BIN; Bin++)
    {
        Sum += m_Spectrum[Bin] * m_Spectrum[Bin];
    }

    return sqrtf(DATAPROC_RMS_SQRT_SCALE * Sum) / DATAPROC_RMS_SQRT_SCALE;
}

/*******************************************************************************
Description:
Compute velocity RMS from the magnitude spectrum. Each spectral bin is
divided by (2 * PI * f) to integrate from acceleration to velocity, then
RMS is calculated over the frequency window.

The result is in mm/s (spectrum is in g, multiplied by gravity in mm/s^2,
divided by angular frequency).

Argument(s):
None

Return:
float - Velocity RMS value in mm/s.

Note(s):
Must be called after DataProc_ComputeSpectrum(). Modifies m_Spectrum in
place (converts from accel to velocity domain).
*******************************************************************************/
static float DataProc_ComputeVelRmsFromSpectrum(void)
{
    uint16_t Bin;
    float FreqHz;
    float Sum = 0.0f;

    for (Bin = DATAPROC_LOWER_BIN; Bin < DATAPROC_UPPER_BIN; Bin++)
    {
        FreqHz = (float)Bin * DATAPROC_FREQ_RES_HZ;
        m_Spectrum[Bin] = (m_Spectrum[Bin] * DATAPROC_GRAVITY_MM_S2)
                        / (DATAPROC_RMS_SQRT_SCALE * PI * FreqHz);
    }

    for (Bin = DATAPROC_LOWER_BIN; Bin < DATAPROC_UPPER_BIN; Bin++)
    {
        Sum += m_Spectrum[Bin] * m_Spectrum[Bin];
    }

    return sqrtf(DATAPROC_RMS_SQRT_SCALE * Sum) / DATAPROC_RMS_SQRT_SCALE;
}
