/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for the MMICT5838
MEMS microphone driver. Communicates via PDM interface. MCU provides
the PDM clock, mic outputs single-bit PDM data. nRF5340 built-in PDM
peripheral decimates to 16-bit PCM samples.

Replaces: ICS-43434 (old firmware, I2S interface)
Part:      TDK MMICT5838 (PDM interface)

Author(s): Magdalene Ratna, Claude (Anthropic)

Date created: Mar 2026
*******************************************************************************/

#ifndef AUDIO_H
#define AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************************************
// INCLUDE FILES
//******************************************************************************
#include <zephyr/kernel.h>
#include <zephyr/device.h>

//******************************************************************************
// DEFINES
//******************************************************************************
#define AUDIO_PDM_SAMPLE_FREQ           16000
#define AUDIO_SAMPLES_PER_FRAME         128
#define AUDIO_SAMPLE_BYTES              2
#define AUDIO_FRAME_BUF_BYTES           (AUDIO_SAMPLES_PER_FRAME * AUDIO_SAMPLE_BYTES)
#define AUDIO_FRAME_COUNT               10
#define AUDIO_TOTAL_SAMPLES             (AUDIO_FRAME_COUNT * AUDIO_SAMPLES_PER_FRAME)

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int      Audio_Init(void);
int      Audio_Read(void);
uint16_t Audio_GetNoiseSpl(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_H */
