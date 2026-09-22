/* USER CODE BEGIN Header */
/**
 * @file  encoder_config.h
 * @brief Compile-time profile selection for the TIM4 quadrature encoder.
 *
 *  CÁCH CHỌN: chạy `python tools/hw_config.py` → mục "Front-panel Encoder".
 *  Tool ghi HW_ENC_PROFILE vào hw_config_active.h và file này lấy theo.
 *  Sửa tay ENC_PROFILE bên dưới chỉ dùng khi không chạy hw_config.py —
 *  HW_ENC_PROFILE nếu có sẽ THẮNG giá trị mặc định ở đây.
 *
 *  Chỉ có một câu hỏi cần trả lời để chọn đúng:
 *  **một detent sinh ra bao nhiêu count trong TIM4 X4 mode?**
 *
 *    - Encoder cơ EC11 và các encoder optical có 1 quadrature cycle / detent
 *      (ví dụ CTS 291 mã X24 = 24 PPR / 24 detent)  →  4 count / detent.
 *    - Encoder optical có 4 detent / cycle (CTS 291 mã 832 / 624 / 416,
 *      Grayhill 62S)                                →  1 count / detent.
 *
 *  Encoder KHÔNG detent (CTS 291 mã Y00 / X00 / 800 / 600 / 400): không có
 *  nấc để bám, chọn ENC_COUNTS_PER_STEP sao cho ra ~24-32 bước mỗi vòng:
 *
 *      bước/vòng = (4 × PPR) / ENC_COUNTS_PER_STEP
 *
 *  ĐO ĐỂ CHỌN (không đoán): đọc __HAL_TIM_GET_COUNTER(&htim4) trước và sau
 *  đúng một vòng quay.  Hiệu số chính là (4 × PPR):
 *
 *      96 hoặc 128  →  ENC_PROFILE_OPT_4CPD   (24 / 32 bước mỗi vòng)
 *      16 / 24 / 32 →  ENC_PROFILE_OPT_1CPD   (16 / 24 / 32 bước mỗi vòng)
 *      80           →  ENC_PROFILE_EC11       (20 bước mỗi vòng)
 *
 *  Acceleration KHÔNG nằm trong profile: encoder.c tính hệ số nhân theo
 *  khoảng thời gian giữa hai bước (ms), nên tự chuẩn hoá theo mọi PPR.
 */
/* USER CODE END Header */

#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

/* tools/hw_config.py ghi HW_ENC_PROFILE vào đây (giống cách lcd_panel_config.h
 * đọc HW_LCD_PANEL).  Chạy tool là đủ, không cần sửa file này. */
#include "hw_config_active.h"

/* ── Profile identifiers ──────────────────────────────────────────────────── */
#define ENC_PROFILE_EC11      1  /*!< EC11 cơ, 20 detent/vòng, 4 count/detent  */
#define ENC_PROFILE_OPT_4CPD  2  /*!< Optical 4 count/detent (CTS 291 X24,
                                      hoặc bản không detent 24/32 PPR)         */
#define ENC_PROFILE_OPT_1CPD  3  /*!< Optical 1 count/detent (CTS 291 832/624/
                                      416, Grayhill 62S, bản không detent
                                      4/6/8 PPR)                               */

/* ── Active profile ─────────────────────────────────────────────────────────
 * Thứ tự ưu tiên: -D trên command line > HW_ENC_PROFILE (hw_config.py) >
 * mặc định EC11 (dùng khi hw_config_active.h chưa có mục encoder). */
#if !defined(ENC_PROFILE) && defined(HW_ENC_PROFILE)
  #define ENC_PROFILE  HW_ENC_PROFILE
#endif
#ifndef ENC_PROFILE
  #define ENC_PROFILE  ENC_PROFILE_EC11
#endif

/* ── Derived constants ────────────────────────────────────────────────────── */
#if   ENC_PROFILE == ENC_PROFILE_EC11
  /* Encoder cơ: bounce 5-30 ms trên tiếp điểm, cần direction guard rộng. */
  #define ENC_COUNTS_PER_STEP   4
  #define ENC_DIR_GUARD_MS     40U
#elif ENC_PROFILE == ENC_PROFILE_OPT_4CPD
  /* Optical: không có contact bounce, guard chỉ để chặn rung quanh mép mã. */
  #define ENC_COUNTS_PER_STEP   4
  #define ENC_DIR_GUARD_MS     10U
#elif ENC_PROFILE == ENC_PROFILE_OPT_1CPD
  /* 1 count/detent: KHÔNG được để guard lớn — mỗi count đã là một bước, guard
   * rộng sẽ nuốt thao tác đảo chiều nhanh. */
  #define ENC_COUNTS_PER_STEP   1
  #define ENC_DIR_GUARD_MS     10U
#else
  #error "ENC_PROFILE khong hop le -- xem encoder_config.h"
#endif

/* ── Acceleration (PPR-independent, tính theo dt giữa hai bước) ───────────────
 * dt_avg là trung bình trượt (EMA) của khoảng cách giữa hai bước liên tiếp,
 * nên ngưỡng dưới đây có nghĩa là "số bước mỗi giây", không phụ thuộc encoder:
 *
 *      dt_avg ≤  10 ms  ≈ ≥100 bước/s  → ×100
 *      dt_avg ≤  20 ms  ≈  ≥50 bước/s  →  ×20
 *      dt_avg ≤  45 ms  ≈  ≥22 bước/s  →   ×5
 *      còn lại                          →   ×1
 *
 * Ngưng tay ≥ ENC_ACCEL_IDLE_MS thì dt_avg reset → bước kế tiếp luôn là ×1,
 * bảo đảm chỉnh từng Hz một vẫn chính xác sau khi vừa quay nhanh.            */
#define ENC_ACCEL_DT_HI_MS    10U
#define ENC_ACCEL_DT_MED_MS   20U
#define ENC_ACCEL_DT_LO_MS    45U
#define ENC_ACCEL_MULT_HI    100
#define ENC_ACCEL_MULT_MED    20
#define ENC_ACCEL_MULT_LO      5
#define ENC_ACCEL_IDLE_MS    150U  /*!< dt ≥ ngưỡng này → coi như bắt đầu lại */

/* Đặt 0 để TẮT hẳn gia tốc: mỗi nấc luôn là đúng một step, mọi tốc độ quay.
 * (Trước đây gia tốc gần như không bao giờ kích hoạt được vì ngưỡng tính theo
 * raw count; bản dt-based này chạy thật, nên để sẵn đường tắt nếu thấy nhạy.) */
#ifndef ENC_ACCEL_ENABLE
  #define ENC_ACCEL_ENABLE     1
#endif

#if !ENC_ACCEL_ENABLE
  #undef  ENC_ACCEL_MULT_HI
  #undef  ENC_ACCEL_MULT_MED
  #undef  ENC_ACCEL_MULT_LO
  #define ENC_ACCEL_MULT_HI    1
  #define ENC_ACCEL_MULT_MED   1
  #define ENC_ACCEL_MULT_LO    1
#endif

#endif /* ENCODER_CONFIG_H */
