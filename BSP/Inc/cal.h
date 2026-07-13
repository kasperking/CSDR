#ifndef __CAL_H
#define __CAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sdr_dsp.h"

/**
 * @file  cal.h
 * @brief Calibration parameter block and entry point.
 *
 *  Cal_Run() enters a two-level overlay menu that lets the user edit all
 *  calibration parameters.  Returns true when the user chooses
 *  "Save Settings"; caller is responsible for persisting to flash and
 *  applying the new values.
 *
 *  dsp: pointer to the live DSP state used by the auto-cal routines
 *  (DC offset measurement, IQ mismatch estimation, noise floor sampling).
 */

typedef struct {
  /* Frequency */
  int32_t  xtal_ppb;           /* XTAL correction, ppb (dương = xtal nhanh).
                                  GPS Cal ghi giá trị ppb chính xác; item
                                  "XTAL PPM" chỉnh tay theo bước 1 ppm.    */

  /* IQ */
  int16_t  iq_gain;            /* IQ gain balance    -50 .. +50         */
  int16_t  iq_phase;           /* IQ phase balance   -50 .. +50         */

  /* DC */
  int32_t  dc_i_offset;        /* ADC I DC offset  -2048 .. +2048       */
  int32_t  dc_q_offset;        /* ADC Q DC offset  -2048 .. +2048       */

  /* Audio */
  int16_t  audio_gain_db;      /* RX audio gain      -20 .. +20 dB      */
  int16_t  mic_gain;           /* TX mic gain          0 .. 100         */

  /* RF / Display */
  int16_t  smeter_offset_db;   /* S-meter offset     -20 .. +20 dB      */
  uint32_t lo_offset_hz;       /* LO offset       10000 .. 25000 Hz     */

  /* PA hardware */
  uint8_t  pa_watts;           /* PA power rating: 0=None, 20, 45, 100 W */
  uint8_t  pa_oc_limit_idx;    /* OC limit ×10 A — range 10..200 = 1.0A..20.0A, step 0.1A */
  uint8_t  pwr_scale;          /* Tandem-match FWD power cal 50..200 % (100 = ×1.0) */
} Cal_Params_t;

#define CAL_PARAMS_DEFAULT \
  { 0, 0, 0, 0, 0, 0, 50, 0, 0U, 0, 100, 100 }

/* Run the calibration overlay.  Blocks until the user exits.
 * Returns true  → user chose Save; caller should apply + persist params.
 * Returns false → user cancelled; params are unchanged.
 * dsp: live DSP state; auto-cal routines read signal_power_db and arm
 *      DSP_CalStart/DSP_CalPoll via this pointer. */
bool Cal_Run(Cal_Params_t *params, DSP_State_t *dsp);

#ifdef __cplusplus
}
#endif
#endif /* __CAL_H */
