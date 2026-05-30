/* USER CODE BEGIN Header */
/**
  * @file  pa_overcurrent.c
  * @brief PA Overcurrent Protection — policy layer trên INA226 driver.
  *        Xem pa_overcurrent.h để biết kiến trúc phần cứng và ALERT GPIO.
  */
/* USER CODE END Header */

#include "pa_overcurrent.h"
#include "csdr_app.h"   /* g_sdr.tx_mode, g_sdr.cat_tx_dirty, g_sdr.display_dirty */
#include <string.h>

PA_OC_State_t g_pa_oc = { 0 };

/* ═══════════════════════════════════════════════════════════════════════════
 *  Init
 * ═══════════════════════════════════════════════════════════════════════════ */

void PA_OC_Init(I2C_HandleTypeDef *hi2c)
{
    memset(&g_pa_oc, 0, sizeof(g_pa_oc));
    g_pa_oc.limit_a = PA_OC_LIMIT_DEFAULT_A;

    /* 1. Inialise INA226 handle + soft reset */
    HAL_StatusTypeDef st = INA226_Init(&g_pa_oc.ina, hi2c, PA_OC_INA226_ADDR_7BIT, PA_OC_SHUNT_OHM);
    if (st != HAL_OK) {
        /* INA226 không phản hồi — đánh dấu absent, retry sau PA_OC_I2C_RETRY_MS.
         * PA_OC_ReadCurrent() sẽ skip I2C cho đến khi retry thành công. */
        g_pa_oc.ina_ok       = false;
        g_pa_oc.ina_retry_ms = 0U;   /* elapsed sẽ lớn ngay → retry sau ~5 s */
        return;
    }

    /* 2. Tốc độ nhanh nhất (140 µs/sample), SOL latch, active-LOW alert */
    INA226_Configure(&g_pa_oc.ina, INA226_CFG_FASTEST, INA226_MASK_SOL_LATCH);

    /* 3. Nạp ngưỡng mặc định */
    INA226_SetAlertLimitAmps(&g_pa_oc.ina, PA_OC_LIMIT_DEFAULT_A);

    /* 4. Xóa latch còn sót từ lần boot trước (đọc MASK_EN = clear AFF) */
    INA226_ReadAndClearLatch(&g_pa_oc.ina, NULL);

    g_pa_oc.ina_ok = true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Runtime API
 * ═══════════════════════════════════════════════════════════════════════════ */

void PA_OC_SetCurrentLimit(float current_a)
{
    if (current_a > PA_OC_LIMIT_MAX_A) current_a = PA_OC_LIMIT_MAX_A;
    if (current_a < 0.0f)              current_a = 0.0f;
    g_pa_oc.limit_a = current_a;
    INA226_SetAlertLimitAmps(&g_pa_oc.ina, current_a);
}

/* PA_OC_I2C_RETRY_MS: khoảng thời gian chờ giữa các lần thử khôi phục I2C
 * sau khi INA226 không phản hồi.  5 s là đủ để tránh gây tải I2C liên tục
 * trong khi vẫn phát hiện được khi thiết bị được cắm vào hoặc bus phục hồi. */
#define PA_OC_I2C_RETRY_MS  5000U

float PA_OC_ReadCurrent(void)
{
    uint32_t now = HAL_GetTick();

    if (!g_pa_oc.ina_ok) {
        /* INA226 đang offline — thử khôi phục định kỳ để bắt hot-plug.
         * ina_retry_ms = timestamp của lần fail gần nhất. */
        if ((now - g_pa_oc.ina_retry_ms) < PA_OC_I2C_RETRY_MS) return 0.0f;

        /* Thử đọc một thanh ghi bất kỳ để kiểm tra bus */
        uint16_t dummy = 0;
        if (INA226_ReadReg(&g_pa_oc.ina, INA226_REG_CONFIG, &dummy) != HAL_OK) {
            g_pa_oc.ina_retry_ms = now;   /* lưu timestamp fail để tính elapsed */
            return 0.0f;
        }
        /* Khôi phục thành công — tái cấu hình chip */
        INA226_Configure(&g_pa_oc.ina, INA226_CFG_FASTEST, INA226_MASK_SOL_LATCH);
        INA226_SetAlertLimitAmps(&g_pa_oc.ina, g_pa_oc.limit_a);
        g_pa_oc.ina_ok = true;
    }

    float val = INA226_ReadCurrentAmps(&g_pa_oc.ina);

    /* Nếu đọc không thành công (HAL lỗi → trả 0.0f), INA226_ReadCurrentAmps
     * không có cách báo lỗi khác.  Dựa vào lần retry tiếp theo để phát hiện
     * offline kéo dài.  Giá trị 0.0f là an toàn: không trigger foldback/trip. */
    return val;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ISR — gọi từ EXTI9_5_IRQHandler trong stm32h7xx_it.c
 *
 *  KHÔNG gọi I2C, KHÔNG gọi HAL_Delay, KHÔNG gọi bất kỳ hàm blocking nào.
 * ═══════════════════════════════════════════════════════════════════════════ */

void PA_OC_AlertISR(void)
{
    g_pa_oc.fault_latched = true;
    g_pa_oc.fault_pending = true;
    g_pa_oc.fault_count++;
    g_pa_oc.fault_tick_ms = HAL_GetTick();

    /* Yêu cầu CSDR_Loop tắt TX qua cờ deferred (giống cat_tx_dirty pattern) */
    g_sdr.tx_mode      = false;
    g_sdr.cat_tx_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main-loop fault handler
 * ═══════════════════════════════════════════════════════════════════════════ */

bool PA_OC_HandleFaultInLoop(void)
{
    if (!g_pa_oc.fault_pending) return false;

    /* Chờ 200ms để PA nguội trước khi nhả INA226 latch.
     * Không block: trả về true mỗi vòng lặp cho đến khi đủ thời gian. */
    if ((HAL_GetTick() - g_pa_oc.fault_tick_ms) < 200U) return true;

    g_pa_oc.fault_pending = false;

    /* Xóa INA226 alert latch → ALERT pin trở về HIGH → Q1 gate pull-up kích hoạt.
     * PA chỉ phát lại được khi người dùng nhấn PTT sau khi TX đã tắt. */
    bool aff;
    INA226_ReadAndClearLatch(&g_pa_oc.ina, &aff);
    if (aff) {
        /* Tái nạp MASK_EN sau khi latch bị xóa để đảm bảo cấu hình còn nguyên */
        INA226_Configure(&g_pa_oc.ina, INA226_CFG_FASTEST, INA226_MASK_SOL_LATCH);
        g_pa_oc.fault_latched = false;
    }

    /* Cập nhật UI: TX=OFF, màn hình vẽ lại trạng thái RX */
    g_sdr.tx_mode       = false;
    g_sdr.display_dirty = 0xFFU;

    return true;
}
