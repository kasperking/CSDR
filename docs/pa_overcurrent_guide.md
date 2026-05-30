# Hướng dẫn: Bảo vệ quá dòng PA bằng INA226

**Dự án:** CSDR — HF SDR Transceiver (STM32H750VBT6)  
**Ngày:** 2026-05-30  
**Files liên quan:**
- `BSP/Inc/ina226.h` / `BSP/Src/ina226.c` — INA226 low-level driver
- `BSP/Inc/pa_overcurrent.h` / `BSP/Src/pa_overcurrent.c` — PA protection policy
- `Core/Src/stm32h7xx_it.c` — EXTI9_5 IRQ handler
- `BSP/Src/csdr_app.c` — call sites: Init, Loop, TX apply
- `CSDR.ioc` — PC6 EXTI config, NVIC priority 5

---

## 1. Nguyên lý hoạt động

INA226 đo điện áp trên trở shunt **liên tục** với chu kỳ 140 µs. Khi dòng PA vượt ngưỡng, chip tự động kéo chân **ALERT** (open-drain, active-LOW) xuống GND mà **không cần MCU can thiệp**. Chân ALERT này điều khiển trực tiếp gate của một N-MOSFET đặt trong đường cấp bias PA — MOSFET tắt, bias bị cắt trong vòng **~140 µs + 50 ns**.

Song song đó, MCU nhận ngắt EXTI (falling edge trên PC6) để cập nhật trạng thái phần mềm: tắt TX mode, cập nhật UI.

```
Dòng PA vượt ngưỡng
        │
        ▼  (≤ 140 µs — một chu kỳ chuyển đổi ADC)
INA226 kéo ALERT → LOW
        │
        ├──→ [PHẦN CỨNG] Gate Q1 ≈ 0.3V → Q1 tắt → Bias PA bị cắt
        │
        └──→ [MCU EXTI ISR] PA_OC_AlertISR() đặt cờ fault_pending
                    │
                    ▼  (vòng lặp tiếp theo, ~1 ms)
             CSDR_Loop → PA_OC_HandleFaultInLoop()
             - g_sdr.tx_mode = false
             - chờ 200 ms cho PA nguội
             - xóa INA226 latch (đọc reg 06h)
             - display_dirty = 0xFF
```

---

## 2. Sơ đồ phần cứng

### 2.1 Vị trí trở shunt

Shunt **0.02 Ω** (loại 2512, dung sai 1%) đặt trên đường nguồn cấp drain của PA:

```
+12V_SUPPLY ──[R_shunt 0.02Ω]──[RF Choke]── Drain PA

                IN+ ←──── điểm giữa R_shunt và RF Choke
                IN- ←──── điểm sau R_shunt về phía GND
```

INA226 đo hiệu điện thế giữa IN+ và IN−, tính dòng qua: `I = V_shunt / R_shunt`.

### 2.2 Mạch cắt bias (N-MOSFET Wired-AND)

```
+VDD_BIAS (nguồn cấp Gate/Base bias PA)
     │
  Drain ── Q1 (IRLML2502 hoặc 2N7002)
  Source ── GND
  
  Gate của Q1:
     ├── R_gp  10 kΩ ──→ +3.3V          giữ gate HIGH mặc định
     ├── R_ser 100 Ω ←── ALERT (INA226)  override cứng khi lỗi
     └── R_mc  1 kΩ  ←── MCU PA_BIAS_EN  điều khiển PTT
```

**Phân tích Wired-AND:**

| ALERT | MCU PA_BIAS_EN | V_gate (tính) | Q1   | Bias PA       |
|-------|---------------|---------------|------|---------------|
| HIGH  | HIGH (TX)     | 3.3 V         | ON   | ✅ Hoạt động  |
| **LOW** | HIGH (TX)   | **≈ 0.3 V**   | **OFF** | ❌ **Cắt ngay** |
| HIGH  | LOW (RX)      | ≈ 0.3 V       | OFF  | ❌ Không cấp  |

Khi ALERT = LOW (INA226 sink current qua 100 Ω):

```
V_gate = 3.3V × 100Ω / (1kΩ + 100Ω + 10kΩ) ≈ 0.3V  < V_gs(th) → Q1 tắt
```

**Chọn MOSFET:**

| Tình huống | Linh kiện | Lý do |
|-----------|-----------|-------|
| I_bias < 100 mA | 2N7002 SOT-23 | V_gs(th) = 1–2.5 V, 0.3 V chắc chắn tắt |
| I_bias > 100 mA | IRLML2502 SOT-23 | I_d = 4.2 A, V_ds = 20 V |

> ⚠️ IRLML2502 có V_gs(th) = 0.45 V typ — cần xác nhận 0.3 V đủ thấp để tắt hoàn toàn. Nếu cần, tăng R_ser lên 470 Ω để hạ V_gate thêm, hoặc dùng 2N7002 cho bias line.

### 2.3 Kết nối INA226 (chân quan trọng)

| Chân INA226 | Kết nối |
|-------------|---------|
| IN+         | Điểm giữa R_shunt và PA drain |
| IN−         | GND sense (phía shunt về GND) |
| VCC         | +3.3V, 100 nF + 10 µF ngay cạnh chân |
| GND         | GND power plane, via thẳng |
| SDA         | PB11 (I2C2_SDA, 10 kΩ pull-up) |
| SCL         | PB10 (I2C2_SCL, 10 kΩ pull-up) |
| ALERT       | PC6, 10 kΩ pull-up → 3.3V, 100 Ω series → R_ser |
| A0, A1      | GND → địa chỉ 0x40 |

---

## 3. Tính toán thanh ghi INA226

### 3.1 Configuration Register (00h) = `0x0027`

```
Bit 15    : RST        = 0
Bits 14-12: AVG[2:0]   = 000  → 1 sample, không lấy trung bình (phản ứng nhanh nhất)
Bits 11-9 : VBUS_CT    = 000  → 140 µs bus voltage conversion time
Bits  8-6 : VSH_CT     = 000  → 140 µs shunt voltage conversion time
Bits  5-3 : reserved   = 100  (always-1 per datasheet, bit 5 = 1 → 0x0020)
Bits  2-0 : MODE       = 111  → Shunt + Bus, Continuous

= 0b 0_000_000_000_100_111 = 0x0027

Tốc độ phát hiện: ≤ 140 µs (một chu kỳ VSH_CT)
```

### 3.2 Mask/Enable Register (06h) = `0x8001`

```
Bit 15: SOL  = 1  → Shunt Over-Limit là nguồn kích hoạt ALERT
Bit  1: APOL = 0  → ALERT active LOW (open-drain kéo GND khi lỗi)
Bit  0: LEN  = 1  → Latch mode: ALERT giữ LOW cho đến khi MCU đọc reg 06h
                    → ngăn PA bật/tắt liên tục khi dòng dao động quanh ngưỡng

= 0b1000_0000_0000_0001 = 0x8001
```

### 3.3 Alert Limit Register (07h) — theo ngưỡng dòng

```
Công thức: Alert_reg = (I_limit × R_shunt × 1e6) / 2.5

Với R_shunt = 0.02 Ω, LSB shunt voltage = 2.5 µV:

  I = 3.5 A → V_shunt = 70 mV = 70000 µV → reg = 70000/2.5 = 28000 = 0x6D60
  I = 3.2 A → V_shunt = 64 mV = 64000 µV → reg = 64000/2.5 = 25600 = 0x6400
  I = 3.0 A → V_shunt = 60 mV = 60000 µV → reg = 60000/2.5 = 24000 = 0x5DC0
  I = 4.0 A → V_shunt = 80 mV = 80000 µV → reg = 80000/2.5 = 32000 = 0x7D00
  I_max     → reg max = 0x7FFF → I = 0x7FFF × 2.5µV / 0.02Ω ≈ 4.096 A
```

**Ngưỡng theo chế độ phát (đã cài trong `csdr_apply_tx`):**

| Chế độ | Ngưỡng | Lý do |
|--------|--------|-------|
| SSB | 3.5 A | Dòng đỉnh voice |
| DIGI (FT8/WSJT-X) | 3.2 A | Carrier liên tục, bảo vệ PA chặt hơn |
| CW | 3.0 A | Carrier liên tục, duty cycle cao nhất |

---

## 4. Kiến trúc phần mềm

### 4.1 Phân lớp

```
csdr_app.c
    │  gọi PA_OC_Init / PA_OC_SetCurrentLimit / PA_OC_HandleFaultInLoop
    ▼
pa_overcurrent.h/c          ← POLICY LAYER
    │  chứa logic bảo vệ PA, biết về g_sdr, CSDR_Loop
    │  lưu INA226_Handle_t bên trong g_pa_oc.ina
    │  gọi INA226_* functions
    ▼
ina226.h/c                  ← DRIVER LAYER (không biết gì về PA/CSDR)
    │  register access, I2C transport
    │  INA226_Handle_t: hi2c + dev_addr + shunt_ohm
    ▼
HAL_I2C_Master_Transmit/Receive
```

### 4.2 INA226 Driver API (`ina226.h`)

| Hàm | Mô tả |
|-----|-------|
| `INA226_Init(h, hi2c, addr, shunt)` | Điền handle, soft reset chip |
| `INA226_Configure(h, cfg, mask_en)` | Ghi CONFIG + MASK_EN |
| `INA226_SetAlertLimitAmps(h, A)` | Tính + ghi Alert Limit register |
| `INA226_ReadCurrentAmps(h)` | Đọc Shunt Voltage → trả về dòng (A) |
| `INA226_ReadBusVolts(h)` | Đọc Bus Voltage → trả về điện áp (V) |
| `INA226_ReadAndClearLatch(h, &aff)` | Đọc MASK_EN → xóa AFF latch, trả AFF flag |
| `INA226_WriteReg(h, reg, val)` | Ghi thẳng thanh ghi (dùng cho debug) |
| `INA226_ReadReg(h, reg, &val)` | Đọc thẳng thanh ghi (dùng cho debug) |

### 4.3 PA Overcurrent Policy API (`pa_overcurrent.h`)

| Hàm | Gọi từ | Mô tả |
|-----|--------|-------|
| `PA_OC_Init(&hi2c2)` | `CSDR_Init` | Khởi tạo INA226, nạp config bảo vệ |
| `PA_OC_SetCurrentLimit(A)` | `csdr_apply_tx` | Thay đổi ngưỡng khi vào TX |
| `PA_OC_ReadCurrent()` | Main loop (tuỳ chọn) | Đọc dòng tức thời |
| `PA_OC_AlertISR()` | `EXTI9_5_IRQHandler` | Đặt cờ fault — ISR safe |
| `PA_OC_HandleFaultInLoop()` | `CSDR_Loop` | Xử lý fault, clear latch, update UI |

### 4.4 Luồng thời gian thực

```
t = 0          Dòng PA vượt ngưỡng
t ≤ 140 µs     INA226 hoàn thành VSH_CT, so sánh với Alert Limit
t = 140 µs     ALERT → LOW  (INA226 analog comparator, không qua MCU)
t = 140 µs     + 50 ns  Gate Q1 ≈ 0.3V → Q1 tắt → Bias bị cắt  ◄ PHẦN CỨNG
t = 140 µs     + Δ_irq  MCU EXTI ISR → PA_OC_AlertISR() set cờ
t ≈ 1 ms       CSDR_Loop thấy fault_pending → g_sdr.tx_mode = false
t = 200 ms     PA_OC_HandleFaultInLoop() xóa INA226 latch
               ALERT → HIGH → Q1 gate pull-up active trở lại
               display_dirty = 0xFF → UI vẽ lại trạng thái RX
```

---

## 5. Tích hợp vào project

### 5.1 `Core/Src/stm32h7xx_it.c`

```c
#include "pa_overcurrent.h"   /* thêm vào USER CODE BEGIN Includes */

/* EXTI9_5 — PA_OC_ALERT trên PC6, falling edge */
void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(PA_OC_ALERT_GPIO_PIN)) {
        PA_OC_AlertISR();
        __HAL_GPIO_EXTI_CLEAR_IT(PA_OC_ALERT_GPIO_PIN);
    }
    HAL_GPIO_EXTI_IRQHandler(PA_OC_ALERT_GPIO_PIN);
}
```

### 5.2 `BSP/Src/csdr_app.c`

```c
/* Includes */
#include "pa_overcurrent.h"

/* CSDR_Init — sau RFAGC_SetEnabled */
PA_OC_Init(&hi2c2);

/* CSDR_Loop — đầu vòng lặp */
PA_OC_HandleFaultInLoop();

/* csdr_apply_tx — đầu nhánh TX */
if (g_sdr.tx_mode) {
    float lim = (g_sdr.mode == MODE_CW)                               ? 3.0f :
                (g_sdr.mode == MODE_DIGU || g_sdr.mode == MODE_DIGL) ? 3.2f : 3.5f;
    PA_OC_SetCurrentLimit(lim);
    /* ... WM8731_SetMute, BPF_SetMode, T/R relay ... */
}
```

### 5.3 CubeMX / `CSDR.ioc`

Đã cấu hình sẵn trong file `.ioc`:

| Mục | Giá trị |
|-----|---------|
| PC6 Signal | `GPXTI6` (GPIO External Interrupt) |
| PC6 Mode | `GPIO_MODE_IT_FALLING` |
| PC6 Pull | `GPIO_PULLUP` |
| PC6 Label | `PA_OC_ALERT` |
| NVIC `EXTI9_5_IRQn` | Enabled, PreemptPriority = **5** |

> NVIC priority 5 < audio DMA (0) < USB (2) — ISR không tranh chấp với audio/USB.

Sau khi Generate Code, CubeMX tự thêm vào `main.h`:
```c
#define PA_OC_ALERT_Pin        GPIO_PIN_6
#define PA_OC_ALERT_GPIO_Port  GPIOC
#define PA_OC_ALERT_EXTI_IRQn  EXTI9_5_IRQn
```

---

## 6. Lưu ý chống nhiễu RF cho PCB Layout

INA226 đặt cạnh tầng PA là môi trường RF khắc nghiệt. Các biện pháp bắt buộc:

### Cô lập vật lý
- Đặt INA226 cách vùng RF tối thiểu **10 mm**.
- Bao quanh bằng **2 hàng via shield** xuống GND (khoảng cách ≤ λ/20 ở tần số cao nhất).

### Lọc đường SDA/SCL
- **RC low-pass ngay tại chân INA226:** R = 100 Ω, C = 100 pF (f_c ≈ 16 MHz).
- Loại nhiễu HF trong khi giữ I2C hoạt động ở 400 kHz.

### Lọc chân ALERT
- **100 Ω series + 100 pF xuống GND** trên đường ALERT trước khi vào PC6.
- **TVS diode 5.6 V** (ví dụ PESD5V0S1BA) từ ALERT xuống GND để clamp spike RF.

### Lọc nguồn VCC của INA226
- **10 µF + 100 nF ceramic** đặt < 2 mm từ chân VCC.
- **Ferrite bead 600 Ω @ 100 MHz** trên đường cấp VCC.

### Routing Kelvin IN+/IN−
- Trace `IN+` và `IN−` phải đi **song song, độ dài bằng nhau** (matched-length Kelvin).
- Thêm **10 nF** từ IN+ xuống GND **và** từ IN− xuống GND (differential RF filter).
- **Không** đặt via hay connector giữa chân shunt và INA226.

### Ground plane
- **Solid ground plane** xuyên suốt dưới INA226 — không cắt slot, không chia plane.
- Chân GND nối via thẳng xuống plane, không đi trace dài.

---

## 7. Debug & kiểm tra

### Biến debug theo dõi qua debugger

| Biến | Ý nghĩa |
|------|---------|
| `g_pa_oc.fault_count` | Tổng số lần quá dòng kể từ boot |
| `g_pa_oc.fault_latched` | `true` = INA226 latch chưa xóa |
| `g_pa_oc.fault_tick_ms` | Timestamp (ms) của lần quá dòng gần nhất |
| `g_pa_oc.limit_a` | Ngưỡng hiện tại đang nạp vào INA226 |
| `g_pa_oc.ina.dev_addr` | Địa chỉ I2C đang dùng (0x80 = 7-bit 0x40) |

### Đọc dòng tức thời từ debugger

Gọi `PA_OC_ReadCurrent()` từ Watch window hoặc gọi `INA226_ReadCurrentAmps(&g_pa_oc.ina)` trực tiếp.

### Kiểm tra INA226 có phản hồi không

```c
uint16_t id = 0;
INA226_ReadReg(&g_pa_oc.ina, INA226_REG_DIE_ID, &id);
/* Kết quả mong đợi: id = 0x2260 (INA226) hoặc 0x2270 (INA226B) */
```

### Test ngắt ALERT thủ công

Kéo chân ALERT xuống GND tạm thời (< 1 ms) — hệ thống phải:
1. Tắt TX (g_sdr.tx_mode = false)
2. `g_pa_oc.fault_count` tăng 1
3. Sau 200 ms, `g_pa_oc.fault_latched` trở về false
