/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ctl_bus.h
  * @brief   Shared slow serial control bus — CTL_SCK (PA5) + CTL_SDO (PA6)
  *
  *  ── Why one shared bus ──────────────────────────────────────────────────
  *  PA5/PA6 sit on the right edge of the LQFP100, facing the RF/PA area.
  *  That edge has too few free pins for a dedicated clock+data pair per
  *  device, so one pair fans out to three devices, each with its own latch:
  *
  *    device              data/clock pins     commit / select
  *    74AHC595  (BPF)     SER   / SRCLK       BPF_RCLK  (PC4) pulse HIGH->LOW
  *    PE4312    (RX ATT)  DATA* / CLK         ATT_LATCH (PB0) pulse HIGH->LOW
  *    MCP4822   (PA bias) SDI   / SCK         DAC_CS    (PB15) active-LOW
  *      * DATA goes through the 10k series resistor on the ATT side.
  *
  *  All three shift on the RISING clock edge with the clock idling LOW
  *  (SPI mode 0,0 for the MCP4822).  A device that is not latched/selected
  *  only shifts junk into its internal shift register — its outputs do not
  *  move: the 595 updates Q only on RCLK, the PE4312 only on LE, and the
  *  MCP4822 ignores SCK while CS is HIGH.  So each device write is a
  *  self-contained frame: shift all of its bits, then latch it.  A frame
  *  never relies on what a previous frame left in a shift register.
  *
  *  ── Rules ───────────────────────────────────────────────────────────────
  *    - Main-loop context only (BPF/LPF band + T/R switching, RF AGC,
  *      PA_Bias_*).  No ISR may touch this bus: an ISR frame landing in the
  *      middle of another device's frame would corrupt both, and there is
  *      no lock.
  *    - Every frame ends with CTL_SCK LOW.
  *    - DAC_CS must be HIGH before any clock edge.  MX_GPIO_Init parks it
  *      HIGH before BPF_LPF_Init/PE4302_Init run; PB15 has no reset pull,
  *      so a 10k pull-up to 3V3 on the PCB is recommended as well.
  *
  *  Pins are GPIO_SPEED_FREQ_LOW on purpose: the bus runs ~80 mm to the
  *  ATT/DAC, and slow edges keep ringing from double-clocking the 595.
  *  CTL_BUS_DELAY() therefore separates every data/clock transition, so
  *  the 595 also gets a clean setup time despite those slow edges.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CTL_BUS_H
#define __CTL_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ≥ 40 ns even when the M7 dual-issues the NOPs (40 NOPs ≥ 20 cycles at
 * 480 MHz), covering the MCP4822 minimums (tCSSR 40 ns, tHI/tLO 25 ns,
 * tSU 15 ns) and the 595 setup time with slow-edge margin.  Plain NOPs,
 * not DWT->CYCCNT, for the reason given in pe4302.c: bounded, never hangs. */
#define CTL_BUS_DELAY() \
  do { \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); \
  } while (0)

/**
  * @brief  Shift the low @p nbits of @p word out MSB-first on CTL_SDO,
  *         one rising CTL_SCK edge per bit.  Leaves CTL_SCK LOW.
  *         Latching/selecting the target device is the caller's job.
  */
static inline void CtlBus_Shift(uint32_t word, uint8_t nbits)
{
  for (int8_t i = (int8_t)nbits - 1; i >= 0; i--) {
    HAL_GPIO_WritePin(CTL_SDO_GPIO_Port, CTL_SDO_Pin,
                      ((word >> i) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    CTL_BUS_DELAY();
    HAL_GPIO_WritePin(CTL_SCK_GPIO_Port, CTL_SCK_Pin, GPIO_PIN_SET);
    CTL_BUS_DELAY();
    HAL_GPIO_WritePin(CTL_SCK_GPIO_Port, CTL_SCK_Pin, GPIO_PIN_RESET);
    CTL_BUS_DELAY();
  }
}

#ifdef __cplusplus
}
#endif

#endif /* __CTL_BUS_H */
