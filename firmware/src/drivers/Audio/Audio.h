/*******************************************************************************
********************************************************************************
Description:
This file defines the macros and function prototypes for the T5838 MEMS
microphone driver. Communicates via PDM interface using the Zephyr DMIC API.
nRF5340 built-in PDM peripheral decimates to 16-bit PCM samples.

Part: TDK MMICT5838 / T5838 (PDM interface)

Author(s): Aarthi

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
#include <stdint.h>

//******************************************************************************
// DEFINES
//******************************************************************************

//******************************************************************************
// DATA TYPES
//******************************************************************************

//******************************************************************************
// FUNCTION PROTOTYPES
//******************************************************************************
int      Audio_Init(void);
int      Audio_Read(void);
uint16_t Audio_GetNoiseSpl(void);
void     Audio_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_H */
