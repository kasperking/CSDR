#ifndef __CAL_H
#define __CAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @file  cal.h
 * @brief Calibration parameter block and entry point.
 *
 *  Cal_Run() enters a two-level overlay menu that lets the user edit all
 *  calibration parameters.  Returns true when the user chooses
 *  "Save Settings"; caller is responsible for persisting to flash and
 *  applying the new values.
 */

typedef struct {
  /* Frequency */
  int32_t  xtal_ppm;           /* XTAL correction   -200 .. +200 ppm   */

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
} Cal_Params_t;

#define CAL_PARAMS_DEFAULT \
  { 0, 0, 0, 0, 0, 0, 50, 0, 18000U }

/* Run the calibration overlay.  Blocks until the user exits.
 * Returns true  → user chose Save; caller should apply + persist params.
 * Returns false → user cancelled; params may have been modified locally
 *                 by Reset Default but should be discarded by caller. */
bool Cal_Run(Cal_Params_t *params);

#ifdef __cplusplus
}
#endif
#endif /* __CAL_H */
