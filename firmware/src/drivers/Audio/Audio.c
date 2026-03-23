/*******************************************************************************
********************************************************************************
Description:
This file handles functions for the MMICT5838 MEMS microphone driver.
Communicates via PDM interface using the nRF5340 built-in PDM peripheral.
MCU provides PDM clock, mic outputs single-bit PDM data stream. The PDM
peripheral decimates to 16-bit PCM samples internally.

Replaces: ICS-43434 (old firmware, I2S interface)
Part:      TDK MMICT5838 (PDM interface)

Features:
  - PDM peripheral init and configuration
  - Frame capture (128 samples per frame)
  - RMS computation over captured PCM samples
  - SPL calculation: 20 * log10(RMS)
  - Kconfig gated via CONFIG_AUDIO_ENABLED

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AUDIO, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <nrfx_pdm.h>
#include <math.h>
#include <string.h>
#include "Audio.h"

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
#define AUDIO_PDM_CLK_PIN       NRF_GPIO_PIN_MAP(1, 5)
#define AUDIO_PDM_DIN_PIN       NRF_GPIO_PIN_MAP(1, 4)
#define AUDIO_PDM_GAIN          NRF_PDM_GAIN_DEFAULT

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static nrfx_pdm_t    m_PdmInstance = NRFX_PDM_INSTANCE(0);
static int16_t       m_PdmBuffer[AUDIO_SAMPLES_PER_FRAME];
static uint16_t      m_NoiseSpl;
static volatile bool m_DataReady;

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static void     Audio_PdmHandler(nrfx_pdm_evt_t const *Event);
static uint16_t Audio_ComputeSpl(int16_t *Samples, uint16_t Count);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialise the nRF5340 PDM peripheral for MMICT5838 microphone capture.
Configures PDM clock and data pins, gain, mono channel, and edge setting.

Argument(s):
None

Return:
int - 0 on success, -1 on failure.
*******************************************************************************/
int Audio_Init(void)
{
    nrfx_err_t Err;
    nrfx_pdm_config_t PdmCfg = NRFX_PDM_DEFAULT_CONFIG(AUDIO_PDM_CLK_PIN,
                                                         AUDIO_PDM_DIN_PIN);

    PdmCfg.mode           = NRF_PDM_MODE_MONO;
    PdmCfg.edge           = NRF_PDM_EDGE_LEFTFALLING;
    PdmCfg.gain_l         = AUDIO_PDM_GAIN;
    PdmCfg.gain_r         = AUDIO_PDM_GAIN;

    Err = nrfx_pdm_init(&m_PdmInstance, &PdmCfg, Audio_PdmHandler);
    if (Err != NRFX_SUCCESS)
    {
        LOG_ERR("PDM init failed: 0x%08X", Err);
        return -1;
    }

    m_NoiseSpl = 0;
    m_DataReady = false;

    LOG_INF("MMICT5838 PDM initialised (mono, left edge)");
    return 0;
}

/*******************************************************************************
Description:
Capture one PDM audio frame (128 samples), compute RMS, and calculate
SPL in dB. The result is stored internally and retrieved via
Audio_GetNoiseSpl().

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.

Note(s):
FRS 6.7: Collect 128 samples per frame, compute RMS,
SPL = 20 * log10(RMS).
*******************************************************************************/
int Audio_Read(void)
{
    nrfx_err_t Err;

    m_DataReady = false;

    /* Set buffer and start PDM capture */
    Err = nrfx_pdm_buffer_set(&m_PdmInstance, m_PdmBuffer, AUDIO_SAMPLES_PER_FRAME);
    if (Err != NRFX_SUCCESS)
    {
        LOG_ERR("PDM buffer set failed: 0x%08X", Err);
        return -1;
    }

    Err = nrfx_pdm_start(&m_PdmInstance);
    if (Err != NRFX_SUCCESS)
    {
        LOG_ERR("PDM start failed: 0x%08X", Err);
        return -1;
    }

    /* Wait for capture to complete */
    while (!m_DataReady)
    {
        k_sleep(K_MSEC(1));
    }

    /* Stop PDM */
    nrfx_pdm_stop(&m_PdmInstance);

    /* Compute SPL from captured samples */
    m_NoiseSpl = Audio_ComputeSpl(m_PdmBuffer, AUDIO_SAMPLES_PER_FRAME);

    LOG_DBG("Noise SPL: %d dB", m_NoiseSpl);
    return 0;
}

/*******************************************************************************
Description:
Return the last computed noise SPL value in dB.

Argument(s):
None

Return:
uint16_t - SPL value in dB. 0 if no measurement has been taken.
*******************************************************************************/
uint16_t Audio_GetNoiseSpl(void)
{
    return m_NoiseSpl;
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
PDM event handler callback. Called by the nrfx PDM driver when a buffer
is filled with samples. Sets the m_DataReady flag to unblock Audio_Read().

Argument(s):
Event - Pointer to the PDM event structure.

Return:
None
*******************************************************************************/
static void Audio_PdmHandler(nrfx_pdm_evt_t const *Event)
{
    if (Event->buffer_released != NULL)
    {
        m_DataReady = true;
    }
}

/*******************************************************************************
Description:
Compute the Sound Pressure Level (SPL) from an array of 16-bit PCM samples.
Formula: SPL = 20 * log10(RMS) where RMS = sqrt(sum(sample^2) / N).

Argument(s):
Samples - Array of 16-bit signed PCM samples (PDM decimated output).
Count   - Number of samples in the array.

Return:
uint16_t - SPL value in dB. Returns 0 if RMS is zero or negative.
*******************************************************************************/
static uint16_t Audio_ComputeSpl(int16_t *Samples, uint16_t Count)
{
    double SumSquare = 0.0;
    double Rms;
    double SplDb;

    for (uint16_t i = 0; i < Count; i++)
    {
        double Sample = (double)Samples[i];
        SumSquare += Sample * Sample;
    }

    Rms = sqrt(SumSquare / (double)Count);

    if (Rms <= 0.0)
    {
        return 0;
    }

    SplDb = 20.0 * log10(Rms);

    if (SplDb < 0.0)
    {
        return 0;
    }

    return (uint16_t)SplDb;
}
