/*******************************************************************************
********************************************************************************
Description:
This file implements the Audio module for capturing and processing PDM
microphone data using the T5838 microphone on nRF5340 DK.

Pipeline:
    PDM capture (mono RIGHT channel, continuous DMA via Zephyr DMIC API)
        -> DC offset removal (mean subtraction per block)
        -> Audio_PassthroughFilterApply()  [passthrough; real IIR can replace later]
            -- returns sum of squares as byproduct
        -> RMS = sqrt(sum_sq / N)
        -> SPL_dB = 20 * log10(rms / 32767) + 120  [calibrated to T5838 sensitivity]
        -> stored in m_NoiseSpl, retrieved via Audio_GetNoiseSpl()

Two operating modes selectable at compile time via USE_LOW_POWER_MODE:
    Low Power Mode  : 8 kHz,  PDM clock 1.000-1.536 MHz
    High Quality Mode: 16 kHz, PDM clock 1.000-3.072 MHz

T5838 Absolute Operating Limits (from datasheet):
    PDM Clock : 1.0 MHz minimum, 3.072 MHz maximum
    Sensitivity: -26 dBFS @ 94 dB SPL, 1 kHz
    Duty Cycle : 40% to 60%

Author(s): Magdalene Ratna,  Aarthi

Date created: Mar 2026
*******************************************************************************/

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include "Audio.h"

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(Audio, LOG_LEVEL_INF);

//******************************************************************************
// LOCAL DEFINES
//******************************************************************************
// Comment out this line to switch to HIGH QUALITY MODE
// #define USE_LOW_POWER_MODE

#ifdef USE_LOW_POWER_MODE
    // Low Power Mode: 8 kHz sample rate, PDM clock at lower end of T5838 spec.
    // Valid clock range: 1.000-1.536 MHz (both above T5838 minimum of 1.0 MHz).
    // Decimation examples with 12.288 MHz audio PLL:
    //   1.024 MHz / (2 x 64 decimation) = 8000 Hz
    //   1.536 MHz / (2 x 96 decimation) = 8000 Hz
    #define AUDIO_SAMPLE_RATE        8000U
    #define AUDIO_PDM_CLK_FREQ_MIN   1000000U   /* 1.000 MHz — T5838 minimum */
    #define AUDIO_PDM_CLK_FREQ_MAX   1536000U   /* 1.536 MHz — lower end of valid range */
    #define MODE_NAME                "LOW_PWR"
#else
    // High Quality Mode: 16 kHz sample rate, full T5838 PDM clock range.
    // Valid clock range: 1.000-3.072 MHz (T5838 datasheet absolute limits).
    // Decimation examples with 12.288 MHz audio PLL:
    //   2.048 MHz / (2 x 64 decimation) = 16000 Hz
    //   3.072 MHz / (2 x 96 decimation) = 16000 Hz
    #define AUDIO_SAMPLE_RATE        16000U
    #define AUDIO_PDM_CLK_FREQ_MIN   1000000U   /* 1.000 MHz — T5838 minimum */
    #define AUDIO_PDM_CLK_FREQ_MAX   3072000U   /* 3.072 MHz — T5838 datasheet maximum */
    #define MODE_NAME                "HIGH_QUAL"
#endif

// AUDIO_BLOCK_SIZE must be a #define (not static const) because
// K_MEM_SLAB_DEFINE_STATIC requires a compile-time constant expression.
// Always represents 100 ms of audio regardless of sample rate.
#define AUDIO_BLOCK_SIZE    (sizeof(int16_t) * (AUDIO_SAMPLE_RATE / 10U))
#define AUDIO_BLOCK_COUNT   4U  /* Driver holds 2 for DMA ping-pong, app holds 1 at a time */

// PDM Clock Duty Cycle — T5838 datasheet specifies 40% to 60%
#define AUDIO_PDM_CLK_DC_MIN    40U     /* 40% minimum duty cycle */
#define AUDIO_PDM_CLK_DC_MAX    60U     /* 60% maximum duty cycle */

// Audio Format — fixed regardless of operating mode
#define AUDIO_SAMPLE_BIT_WIDTH  16U     /* 16-bit PCM output     */
#define AUDIO_READ_TIMEOUT_MS   1000U   /* dmic_read timeout, ms */

// T5838 SPL Calibration — hardware specification, does not change between modes
// Sensitivity : -26 dBFS @ 94 dB SPL, 1 kHz (T5838 datasheet)
// Formula     : SPL_dB = 20 * log10(rms / FULL_SCALE) + SENSITIVITY_OFFSET
// Derivation  : SENSITIVITY_OFFSET = 94 (ref SPL) + 26 (offset from -26 dBFS) = 120
#define AUDIO_FULL_SCALE            32767.0f    /* Signed 16-bit full scale     */
#define AUDIO_SENSITIVITY_OFFSET    120.0f      /* T5838 sensitivity offset, dB */
#define AUDIO_RMS_FLOOR             1.0f        /* Guard against log10(0)       */

/******************************************************************************
// Data types
******************************************************************************/
// Passthrough IIR filter structure.
// num_sos = 0 and gain = 1.0 makes this a passthrough today.
// A real IIR filter (A-weighting, C-weighting, etc.) can replace this later
// by setting num_sos > 0 and populating section coefficients, without
// changing any call sites.
typedef struct
{
    int   NumSos;   /* Number of second-order sections; 0 = passthrough */
    float Gain;     /* Overall gain factor; 1.0 = unity gain            */
} NoIirFilter_t;

//******************************************************************************
// FILE SCOPE VARIABLES
//******************************************************************************
static const struct device *m_DmicDev  = NULL;
static uint16_t             m_NoiseSpl = 0U;

K_MEM_SLAB_DEFINE_STATIC(m_MemSlab, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_COUNT, 4);

//******************************************************************************
// LOCAL FUNCTION PROTOTYPES
//******************************************************************************
static float Audio_PassthroughFilterApply(const NoIirFilter_t *p_Filter,
                                          const int16_t       *p_Input,
                                          int16_t             *p_Output,
                                          uint32_t             NumSamples,
                                          float                DcOffset);

static void Audio_ProcessBlock(void *p_Buffer, uint32_t Size);

/*******************************************************************************
********************************************************************************
* GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Initialises the PDM peripheral, configures the T5838 microphone for mono
LEFT channel capture at the mode-selected sample rate, and starts on-demand
audio capture via the Zephyr DMIC API.
SELECT (L/R) pin is tied to GND on the hardware, selecting the LEFT channel
(data valid on falling clock edge).

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Audio_Init(void)
{
    int Ret;

    struct pcm_stream_cfg Stream = {
        .pcm_width = AUDIO_SAMPLE_BIT_WIDTH,
        .mem_slab  = &m_MemSlab,
    };

    struct dmic_cfg Cfg = {
        .io = {
            .min_pdm_clk_freq = AUDIO_PDM_CLK_FREQ_MIN,
            .max_pdm_clk_freq = AUDIO_PDM_CLK_FREQ_MAX,
            .min_pdm_clk_dc   = AUDIO_PDM_CLK_DC_MIN,
            .max_pdm_clk_dc   = AUDIO_PDM_CLK_DC_MAX,
        },
        .streams = &Stream,
        .channel = {
            .req_num_streams = 1,
            .req_num_chan    = 1,
            .req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
        },
    };

    m_DmicDev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

    if (!device_is_ready(m_DmicDev))
    {
        LOG_ERR("PDM device not ready");
        return -ENODEV;
    }

    Cfg.streams[0].pcm_rate   = AUDIO_SAMPLE_RATE;
    Cfg.streams[0].block_size = AUDIO_BLOCK_SIZE;

    Ret = dmic_configure(m_DmicDev, &Cfg);
    if (Ret < 0)
    {
        LOG_ERR("dmic_configure failed: %d", Ret);
        return Ret;
    }

    m_NoiseSpl = 0U;

    LOG_INF("[%s] Audio initialised — %u Hz, %u ms blocks, RIGHT ch/rising edge",
            MODE_NAME, AUDIO_SAMPLE_RATE,
            (uint32_t)((AUDIO_BLOCK_SIZE / sizeof(int16_t)) * 1000U / AUDIO_SAMPLE_RATE));

    return 0;
}

/*******************************************************************************
Description:
Captures one valid block of audio data (~100 ms), computes RMS and calibrated
SPL in dB, and stores the result for retrieval via Audio_GetNoiseSpl().

On each call: starts the PDM, discards the first block (CIC decimation filter
settling transient), processes the second block (valid settled audio), then
stops the PDM. This preserves the power-efficient start/stop model while
ensuring the captured data reflects real audio, not the filter warm-up.

Argument(s):
None

Return:
int - 0 on success, negative error code on failure.
*******************************************************************************/
int Audio_Read(void)
{
    void    *p_Buffer;
    uint32_t Size;
    int      Ret;

    Ret = dmic_trigger(m_DmicDev, DMIC_TRIGGER_START);
    if (Ret < 0)
    {
        LOG_ERR("DMIC_TRIGGER_START failed: %d", Ret);
        return Ret;
    }

    /* Discard first block — CIC decimation filter settling transient */
    Ret = dmic_read(m_DmicDev, 0, &p_Buffer, &Size, AUDIO_READ_TIMEOUT_MS);
    if (Ret < 0)
    {
        LOG_ERR("dmic_read (settle) failed: %d", Ret);
        (void)dmic_trigger(m_DmicDev, DMIC_TRIGGER_STOP);
        return Ret;
    }
    k_mem_slab_free(&m_MemSlab, p_Buffer);

    /* Second block — valid settled audio */
    Ret = dmic_read(m_DmicDev, 0, &p_Buffer, &Size, AUDIO_READ_TIMEOUT_MS);
    if (Ret < 0)
    {
        LOG_ERR("dmic_read failed: %d", Ret);
        (void)dmic_trigger(m_DmicDev, DMIC_TRIGGER_STOP);
        return Ret;
    }

    Audio_ProcessBlock(p_Buffer, Size);

    // Return block to pool — must be done after every read to prevent pool exhaustion
    k_mem_slab_free(&m_MemSlab, p_Buffer);

    Ret = dmic_trigger(m_DmicDev, DMIC_TRIGGER_STOP);
    if (Ret < 0)
    {
        LOG_ERR("DMIC_TRIGGER_STOP failed: %d", Ret);
        return Ret;
    }

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
Description:
Stops continuous audio capture by sending the STOP trigger to the PDM
peripheral.

Argument(s):
None

Return:
None
*******************************************************************************/
void Audio_Stop(void)
{
    int Ret;

    Ret = dmic_trigger(m_DmicDev, DMIC_TRIGGER_STOP);
    if (Ret < 0)
    {
        LOG_ERR("DMIC_TRIGGER_STOP failed: %d", Ret);
    }
}

/*******************************************************************************
********************************************************************************
* LOCAL FUNCTION DEFINITIONS
********************************************************************************
*******************************************************************************/
/*******************************************************************************
Description:
Applies a passthrough filter to the input samples with DC offset removal,
and returns the sum of squares as a byproduct for RMS calculation.

DC offset (the mean value of the block) is subtracted from each sample before
processing. This removes any constant bias in the PDM mic output and ensures
RMS reflects only the AC audio signal.

With Gain = 1.0 and NumSos = 0, output = (input - DcOffset). A real IIR
filter replaces this function body without changing any call sites.

Argument(s):
p_Filter   - Pointer to the filter configuration struct.
p_Input    - Pointer to the input sample buffer (int16_t).
p_Output   - Pointer to the output sample buffer (int16_t, may equal p_Input).
NumSamples - Number of samples to process.
DcOffset   - Mean value of the block, subtracted from each sample.

Return:
float - Sum of squares of the DC-corrected output samples.
*******************************************************************************/
static float Audio_PassthroughFilterApply(const NoIirFilter_t *p_Filter,
                                          const int16_t       *p_Input,
                                          int16_t             *p_Output,
                                          uint32_t             NumSamples,
                                          float                DcOffset)
{
    float    SumSq  = 0.0f;
    float    Sample;
    uint32_t i;

    for (i = 0; i < NumSamples; i++)
    {
        Sample      = ((float)p_Input[i] - DcOffset) * p_Filter->Gain;
        p_Output[i] = (int16_t)Sample;
        SumSq      += Sample * Sample;
    }

    return SumSq;
}

/*******************************************************************************
Description:
Computes and stores SPL for one captured block, and logs audio statistics:
  - Operating mode (LOW_PWR or HIGH_QUAL)
  - Min and max sample values (time-domain extremes, post DC removal)
  - RMS amplitude
  - Calibrated SPL in dB using T5838 sensitivity specification

DC offset is computed as the mean of all samples and removed before RMS
calculation to prevent bias from inflating the SPL reading. Result is stored
in m_NoiseSpl for retrieval via Audio_GetNoiseSpl().

Argument(s):
p_Buffer - Pointer to the captured audio block (int16_t samples).
Size     - Size of the block in bytes.

Return:
None
*******************************************************************************/
static void Audio_ProcessBlock(void *p_Buffer, uint32_t Size)
{
    int16_t       *p_Samples  = (int16_t *)p_Buffer;
    uint32_t       NumSamples = Size / sizeof(int16_t);
    NoIirFilter_t  NoFilter   = { .NumSos = 0, .Gain = 1.0f };
    float          DcSum      = 0.0f;
    float          DcOffset;
    float          SumSq;
    float          Rms;
    float          SplDb;
    int16_t        MinVal;
    int16_t        MaxVal;
    uint32_t       i;

    // Stage 1: Compute DC offset — mean value across the block
    for (i = 0; i < NumSamples; i++)
    {
        DcSum += (float)p_Samples[i];
    }
    DcOffset = DcSum / (float)NumSamples;

    // Stage 2: Passthrough filter with DC removal — returns sum of squares
    SumSq = Audio_PassthroughFilterApply(&NoFilter,
                                         p_Samples,
                                         p_Samples,
                                         NumSamples,
                                         DcOffset);

    // Scan min/max on DC-corrected samples
    MinVal = p_Samples[0];
    MaxVal = p_Samples[0];
    for (i = 0; i < NumSamples; i++)
    {
        if (p_Samples[i] < MinVal) { MinVal = p_Samples[i]; }
        if (p_Samples[i] > MaxVal) { MaxVal = p_Samples[i]; }
    }

    // Stage 3: RMS = sqrt(sum_sq / N)
    Rms = sqrtf(SumSq / (float)NumSamples);

    // Guard: prevent log10(0) = -inf when mic is silent or disconnected
    if (Rms < AUDIO_RMS_FLOOR)
    {
        Rms = AUDIO_RMS_FLOOR;
    }

    // Stage 4: SPL in dB calibrated to T5838 sensitivity (-26 dBFS @ 94 dB SPL)
    // SPL = 20 * log10(rms / full_scale) + sensitivity_offset
    SplDb = 20.0f * log10f(Rms / AUDIO_FULL_SCALE) + AUDIO_SENSITIVITY_OFFSET;

    m_NoiseSpl = (uint16_t)SplDb;

    LOG_INF("[%s] min=%6d  max=%6d  rms=%5d  spl=%.1f dB",
            MODE_NAME, (int)MinVal, (int)MaxVal, (int)Rms, (double)SplDb);
}
