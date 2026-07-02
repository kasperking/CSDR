/**
  ******************************************************************************
  * @file    nr_spec.h
  * @brief   NR2 — spectral noise reduction (Wiener gain, decision-directed SNR)
  *
  *  Streaming WOLA processor: call NRSpec_Process() once per audio sample at
  *  48 kHz.  Output is delayed by one hop (256 samples ≈ 5.3 ms); the first
  *  hop after Reset is silence while the pipeline primes.
  *
  *  Context: CSDR_Loop only (same as DSP_Process) — not ISR-safe.
  ******************************************************************************
  */
#ifndef __NR_SPEC_H
#define __NR_SPEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Build FFT/window tables and reset state.  Call once at boot. */
void  NRSpec_Init(void);

/** Clear signal state (rings, noise estimate).  Call when NR2 is enabled. */
void  NRSpec_Reset(void);

/** level 0-100 → max suppression depth: 0 = 0 dB (bypass), 100 = −24 dB. */
void  NRSpec_SetLevel(uint8_t level);

/** Process one sample; returns the denoised sample one hop delayed. */
float NRSpec_Process(float in);

#ifdef __cplusplus
}
#endif
#endif /* __NR_SPEC_H */
