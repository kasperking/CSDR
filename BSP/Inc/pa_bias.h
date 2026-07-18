/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pa_bias.h
  * @brief   PA gate-bias source — fixed trimmer HOẶC MCP4822 DAC (dual-mode)
  *
  *  Hai cách cấp bias cho tầng PA, chọn bằng jumper phần cứng + menu:
  *
  *    FIXED (pa_bias_src=0, mặc định):
  *      Trimmer divider trên PCB cấp thẳng V_bias.  Firmware KHÔNG chạm SPI —
  *      module này hoàn toàn im lặng (MCP4822 có thể DNP).  Đóng/cắt bias vẫn
  *      do wired-AND Q1 (PA_BIAS_EN + INA226 ALERT, xem pa_overcurrent.h).
  *
  *    DAC (pa_bias_src=1):
  *      MCP4822 VOUTA/VOUTB (0-2.048V, GA=×1) → op-amp ×2.5 → 0-5.1V →
  *      qua mạch cắt Q1 → gate PA.  Kênh A = BIAS_1 (final), B = BIAS_2
  *      (driver).  Firmware ramp bias khi vào TX, về 0 khi ra RX, và ghi 0
  *      trước khi xóa INA226 latch sau fault (chống slam-back).
  *
  *  Sơ đồ vai trò an toàn: DAC chỉ ĐẶT MỨC (chậm, main-loop); đường CẮT
  *  khẩn cấp vẫn là INA226 ALERT → Q1 (~140 µs, thuần phần cứng).  Không
  *  bao giờ dùng SPI làm đường cắt sự cố.
  *
  *  Phần cứng:
  *    SPI3 dùng chung với W25Q (mode 3 = CPOL1/CPHA1, MCP4822 hỗ trợ 1,1).
  *    Kernel SPI3 = 120 MHz, W25Q chạy /4 = 30 MHz — QUÁ 20 MHz max của
  *    MCP4822, nên mỗi transaction DAC tự hạ prescaler /16 (7.5 MHz) rồi
  *    trả lại nguyên trạng.  Mọi truy cập SPI3 (W25Q + DAC) đều từ main
  *    loop, không ISR → không cần lock bus.
  *
  *    CS: PD3 (chân trống, không có trong .ioc — module tự init GPIO).
  *    PCB phải nối net DAC_CS về PD3.  LDAC nối GND (update ngay khi CS nhả).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PA_BIAS_H
#define __PA_BIAS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ─── CS GPIO — phải khớp routing PCB ──────────────────────────────────── */
#define PA_BIAS_CS_GPIO_PORT    GPIOD
#define PA_BIAS_CS_GPIO_PIN     GPIO_PIN_3

#define PA_BIAS_SPI_TIMEOUT_MS  10U

/* Mức bias lưu ở đơn vị 0..200 = 0..100% full-scale DAC (bước 0.5% ≈ 10 mV
 * tại chân DAC ≈ 26 mV tại gate sau op-amp ×2.5) — đủ mịn cho chỉnh Idq. */
#define PA_BIAS_LEVEL_MAX       200U

/* Bù nhiệt NTC: Vgs(th) MOSFET trôi ≈ −2..−5 mV/°C → gate bias phải hạ theo
 * để giữ Idq.  −4 mV/°C tại gate = −3.2 code/°C (1 code = 1.25 mV gate).
 * Tham chiếu 25.0 °C; mức lưu (pa_bias1/2) là mức TẠI 25 °C — auto-cal đã
 * trừ comp lúc cal nên cal ở nhiệt nào cũng ra cùng mức lưu. */
#define PA_BIAS_TC_CODE_X100_PER_C10  32   /* code×100 per 0.1 °C (=3.2 code/°C) */
#define PA_BIAS_TC_REF_C10            250  /* 25.0 °C                            */
#define PA_BIAS_TC_CLAMP_CODE         400  /* ±0.5 V gate max correction         */

typedef enum {
  PA_BIAS_SRC_FIXED = 0,   /*!< trimmer phần cứng — firmware không chạm SPI */
  PA_BIAS_SRC_DAC   = 1,   /*!< MCP4822 điều khiển                          */
} PA_Bias_Src_t;

typedef struct {
  uint8_t  src;            /*!< PA_Bias_Src_t                               */
  uint8_t  level[2];       /*!< mức cấu hình 0..200 (A=final, B=driver)     */
  bool     armed;          /*!< TX active — Tick tính target từ level+comp  */
  int16_t  comp_codes;     /*!< bù nhiệt NTC hiện tại, đơn vị code (debug)  */
  uint16_t code[2];        /*!< mã DAC 12-bit đã ghi lần cuối (shadow)      */
  uint16_t target[2];      /*!< mã DAC đích — Tick ramp code→target         */
  uint32_t last_step_ms;   /*!< tick của bước ramp gần nhất                 */
  uint32_t write_count;    /*!< tổng số lần ghi SPI (debug)                 */
} PA_Bias_State_t;

extern PA_Bias_State_t g_pa_bias;

/**
  * @brief  Init CS GPIO + ghi 0 vào cả 2 kênh (MCU soft-reset không reset DAC —
  *         không được tin POR).  Gọi trong CSDR_Init sau khi SPI3 sẵn sàng,
  *         SAU csdr_load_settings rồi tiếp PA_Bias_Configure.
  */
void PA_Bias_Init(SPI_HandleTypeDef *hspi);

/**
  * @brief  Nạp cấu hình từ settings/menu.  Đổi src→FIXED hoặc đổi mức giữa
  *         TX đều an toàn: target cập nhật, Tick ramp tới (mid-TX chỉnh Idq
  *         sống được).  src→FIXED ghi 0 ngay rồi thôi không chạm SPI nữa.
  */
void PA_Bias_Configure(uint8_t src, uint8_t level_a, uint8_t level_b);

/**
  * @brief  RX→TX transition (csdr_apply_tx): đặt target = mức cấu hình,
  *         ramp bắt đầu từ 0.  Toàn ramp ≈ 32 ms — nằm trong blanking 100 ms
  *         của pa_protect.  No-op khi src=FIXED.
  */
void PA_Bias_OnTxStart(void);

/**
  * @brief  Hạ bias về 0 ngay (2 lần ghi SPI, ~µs).  Gọi từ csdr_finish_rx_hw
  *         (sau drain 12 ms) và không cần thiết gọi chỗ khác.  No-op khi FIXED.
  */
void PA_Bias_OnRxFinish(void);

/**
  * @brief  Fault path: ghi 0 cả 2 kênh NGAY, gọi từ PA_OC_HandleFaultInLoop
  *         TRƯỚC khi xóa INA226 latch — khi ALERT nhả, bias vẫn 0, không
  *         slam-back.  Ghi cả khi src=FIXED (vô hại nếu DAC vắng mặt, và an
  *         toàn hơn nếu jumper đang ở DAC mà settings nói FIXED).
  */
void PA_Bias_OnFault(void);

/**
  * @brief  Ramp engine + bù nhiệt NTC — gọi mỗi vòng CSDR_Loop (và
  *         CSDR_PollTxSequencing cho app toàn màn hình).  Khi armed (TX),
  *         target = level + comp(g_analog.temp_c); mỗi ≥2 ms bước 256 code
  *         về target.  No-op khi src=FIXED hoặc auto-cal đang giữ DAC.
  */
void PA_Bias_Tick(void);

/* ─── Auto-cal Idq closed-loop (kênh A / Bias 1 — tầng final) ─────────────
 * Máy trạng thái non-blocking chạy trong CSDR_Loop: key TX với drive = 0
 * (csdr_apply_tx ép RF zero khi PA_BiasCal_Active), đo dòng nền INA226,
 * nâng DAC A coarse (25 mV gate/30 ms) tới khi Idq vượt đích, lùi 3 bước,
 * dò fine (6 mV/25 ms), lưu mức quy về 25 °C.  Bias 2 (driver) giữ nguyên
 * mức cấu hình trong suốt cal — dòng của nó nằm trong baseline.
 * Abort: quá dòng (2×đích hoặc 80% ngưỡng OC), INA fault/offline, mất TX,
 * timeout 10 s, hết thang DAC.  Kết quả đổ về qua PA_BiasCal_Poll. */

/** @brief  Bắt đầu cal.  false nếu thiếu điều kiện: src≠DAC, pa_watts=0,
  *         INA226 offline, đang TX, hoặc fault đang treo. */
bool PA_BiasCal_Start(uint16_t idq_target_ma);

/** @brief  true khi máy cal đang chạy (csdr_apply_tx dùng để ép drive=0). */
bool PA_BiasCal_Active(void);

/** @brief  Bơm máy cal — gọi mỗi vòng CSDR_Loop, no-op khi IDLE. */
void PA_BiasCal_Tick(void);

/** @brief  Lấy kết quả (một lần): 0 = chưa có gì, 1 = OK (*level_out = mức
  *         Bias 1 mới, caller commit vào g_sdr + arm save), 2 = FAIL. */
uint8_t PA_BiasCal_Poll(uint8_t *level_out);

#ifdef __cplusplus
}
#endif
#endif /* __PA_BIAS_H */
