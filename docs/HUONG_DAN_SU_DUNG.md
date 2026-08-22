# CSDR – Hướng dẫn sử dụng

## Máy thu phát SDR HF 160m–10m · STM32H750 · màn hình FMC 480×320 (ST7796) hoặc 240×320 (ST7789)

Tài liệu này hướng dẫn **vận hành** máy CSDR đã lắp ráp/nạp firmware xong. Nếu bạn cần build firmware từ mã nguồn, xem **README_INSTALL.md**. Nếu cần chi tiết bảo vệ PA quá dòng, xem **docs/pa_overcurrent_guide.md**.

---

## Mục lục

1. [Mặt máy – phím và núm](#1-mặt-máy--phím-và-núm)
2. [Bố cục màn hình chính](#2-bố-cục-màn-hình-chính)
3. [Bật / tắt máy](#3-bật--tắt-máy)
4. [Vận hành cơ bản](#4-vận-hành-cơ-bản)
5. [Các chế độ thu (RX)](#5-các-chế-độ-thu-rx)
6. [Âm thanh (Audio)](#6-âm-thanh-audio)
7. [Phát sóng (TX) – SSB/AM/FM](#7-phát-sóng-tx--ssbamfm)
8. [Chế độ CW](#8-chế-độ-cw)
9. [Chế độ số RTTY](#9-chế-độ-số-rtty)
10. [Chế độ số FT8 (ứng dụng toàn màn hình)](#10-chế-độ-số-ft8-ứng-dụng-toàn-màn-hình)
11. [TUNE và SWR Scan](#11-tune-và-swr-scan)
12. [Sơ đồ đầy đủ hệ thống Menu](#12-sơ-đồ-đầy-đủ-hệ-thống-menu)
13. [Hiệu chuẩn máy (Calibration)](#13-hiệu-chuẩn-máy-calibration)
14. [Điều khiển CAT (flrig / WSJT-X / Hamlib)](#14-điều-khiển-cat-flrig--wsjtx--hamlib)
15. [USB Audio – truyền IQ ra máy tính](#15-usb-audio--truyền-iq-ra-máy-tính)
16. [GPS – hiệu chuẩn LO & đồng bộ giờ (tùy chọn)](#16-gps--hiệu-chuẩn-lo--đồng-bộ-giờ-tùy-chọn)
17. [Bảo vệ PA & cảnh báo phần cứng](#17-bảo-vệ-pa--cảnh-báo-phần-cứng)
18. [Lưu cài đặt & Factory Reset](#18-lưu-cài-đặt--factory-reset)
19. [Khắc phục sự cố](#19-khắc-phục-sự-cố)
20. [Phụ lục](#20-phụ-lục)

---

## 1. Mặt máy – phím và núm

| Phím / núm | Nhấn ngắn | Giữ (~0.6s) |
|---|---|---|
| **MENU** | Mở/đóng menu cài đặt | — |
| **F1** | Menu: di lên · Ngoài menu: giảm Volume (giữ để lặp) | — |
| **F2** | Menu: di xuống · Ngoài menu: tăng Volume (giữ để lặp) | — |
| **F3** | Menu: xác nhận/chạy action · Ngoài menu: đảo VFO A↔B | CW/RTTY: bật/tắt bộ giải mã (CW Decode / RTTY Decode) |
| **F4** | Menu: Back/thoát · Ngoài menu: chạy nhanh SWR Scan | — |
| **BAND** | Nhảy lên băng tần kế tiếp (thả ra mới áp dụng) | Mở bảng chọn băng tần (overlay 2 cột) |
| **MODE** | Nhảy sang mode kế tiếp (AM→FM→USB→LSB→CW→DIGU→DIGL→…) | Mở bảng chọn mode |
| **TUNE** | Giữ để phát sóng mang liên tục công suất thấp (nhả ra để dừng), dùng chỉnh ATU/kiểm tra SWR | tự động dừng sau thời gian an toàn nếu bị kẹt |
| **PTT** | Chuyển sang phát (TX) khi nhấn, về thu (RX) khi thả | — |
| **DIT / DAH** (paddle) | Bàn khóa điện tử CW (Iambic A/B hoặc Straight key) | — |
| **Encoder (núm xoay)** | Xoay: chỉnh tần số theo Step hiện tại · Nhấn: đổi Step (1Hz→10Hz→100Hz→1kHz→10kHz→100kHz) | Giữ: đổi Span màn hình phổ (±24k→±12k→±6k→±3k) |
| **PW (nguồn)** | — | Giữ **> 3 giây** để tắt máy an toàn (lưu cài đặt trước khi cắt nguồn) |

Khi bảng chọn Band/Mode đang mở: **F1**/**F2** = di chuyển con trỏ lên/xuống (hoặc xoay encoder), **F3** (hoặc nhấn encoder) = chọn, **F4** hoặc phím BAND/MODE = hủy.

> Khi menu đang mở, phím **TUNE** bị vô hiệu hóa để tránh phát nhầm.

---

## 2. Bố cục màn hình chính

### Panel ST7796 480×320 (mặc định)

```
┌─────────────────────────────────────────────────────────────┐
│  HEADER 480×24                               13.9V           │  ← điện áp nguồn, cảnh báo phần cứng
├─────────┬────────────────────────────────────┬─────────────┤
│ SBL 80  │  VFO  320×64                       │ SBR 80      │
│ Mode    │  14.200.000                         │BW 2.7k ST1k│
│ VFO A/B │  A  USB  RX                         │MIC 25  AT6d│
│ NR  NB  ├────────────────────────────────────┤DSP  1      │
│ VOL 78  │  METER  320×32 – S-meter dạng thước │            │
│ SQL  0  │  S 1  3  5  7  9  +20  +40         │            │
├─────────┴────────────────────────────────────┴─────────────┤
│  INFO STRIP 480×24 – nhãn phím chức năng / trạng thái       │
├─────────────────────────────────────────────────────────────┤
│  SPECTRUM 480×72                                            │
├─────────────────────────────────────────────────────────────┤
│  WATERFALL 480×72                                           │
├─────────────────────────────────────────────────────────────┤
│  FOOTER 480×32     -24k        0        +24k                │
└─────────────────────────────────────────────────────────────┘
```

### Panel ST7789 240×320 (compact)

```
┌───────────────────────┐
│  HEADER 240×16        │  điện áp
├───────────────────────┤
│  VFO 240×48           │  14.200.000  A USB RX
├───────────────────────┤
│  METER 240×24         │  S 1 3 5 7 9 +20 +40
├───────────────────────┤
│  SPECTRUM 240×76      │
├───────────────────────┤
│  WATERFALL 240×96     │
├───────────────────────┤
│  STATUS 240×28        │  USB VOL:78 SQL:0 / BW:2.7k NR NB
├───────────────────────┤
│  FOOTER 240×32        │  -24k   0   +24k
└───────────────────────┘
```

Ghi chú chung:
- **VFO**: tần số đang nghe, chữ nhỏ hơn phía dưới hiển thị VFO không hoạt động (mờ).
- **S-meter**: dải "thước" progressive-density, sáng dần theo cường độ tín hiệu; khi TX chuyển sang hiển thị công suất/SWR.
- **Spectrum/Waterfall**: cập nhật độc lập (~13–28 fps tùy chế độ), có marker đánh dấu tần số đang nghe. Ở chế độ **Marker Track**, marker di chuyển trong dải hiển thị còn phổ/waterfall đứng yên; ở chế độ **Fix** (mặc định), marker cố định giữa màn hình và toàn bộ phổ cuộn theo khi đổi tần số.
- **INFO STRIP / STATUS**: hiện gợi ý phím đang dùng, chữ chạy CW/RTTY/FT8 giải mã, hoặc cảnh báo lỗi phần cứng.
- Khi phát (TX), khu vực SPECTRUM chuyển sang hiển thị phổ audio micro màu hổ phách (thay cho phổ RX) để theo dõi chất lượng giọng nói/tín hiệu số đang phát.

---

## 3. Bật / tắt máy

- **Bật máy**: nhấn phím PW — nguồn tự giữ (latch), máy chạy tự-kiểm tra phần cứng (self-test) khi khởi động rồi vào màn hình chính.
- **Tắt máy**: **giữ phím PW hơn 3 giây**. Màn hình hiện "POWERING OFF…", máy tự lưu toàn bộ cài đặt rồi cắt nguồn an toàn. **Không rút nguồn đột ngột** khi máy đang ghi cài đặt (xem mục 18).
- Nếu một mục tự-kiểm tra thất bại lúc khởi động, thanh HEADER hiện cảnh báo màu hổ phách — xem mục 17.

---

## 4. Vận hành cơ bản

- **Chỉnh tần số**: xoay encoder — bước nhảy theo **Step** hiện hành (hiển thị ở sidebar phải trên panel lớn, hoặc STATUS bar trên panel nhỏ).
- **Đổi Step**: nhấn encoder (ngoài menu) để xoay vòng 1Hz → 10Hz → 100Hz → 1kHz → 10kHz → 100kHz; hoặc vào **Tuning → Step**.
- **Đổi băng tần**: nhấn nhanh **BAND** để nhảy băng kế tiếp, hoặc giữ **BAND** để mở bảng chọn nhanh 2 cột (10 băng: 160m–10m). Máy tự chuyển bộ lọc BPF/LPF tương ứng.
- **Đổi mode**: nhấn nhanh **MODE** để xoay vòng AM→FM→USB→LSB→CW→DIGU→DIGL, hoặc giữ **MODE** để mở bảng chọn nhanh.
- **Đảo VFO A/B**: nhấn nhanh **F3** khi menu đang đóng.
- **RIT** (Receiver Incremental Tuning): chỉnh trong **RX → RIT(Hz)**, phạm vi ±999 Hz — chỉ lệch tần số thu, không ảnh hưởng tần số phát. RIT **không được lưu** vào bộ nhớ khi tắt máy (luôn về 0 khi khởi động lại), theo thiết kế.
- **RX Shift**: dịch dải nghe (IF shift) ±2000 Hz mà không đổi tần số VFO — hữu ích khi có nhiễu/đài lân cận đè lên tín hiệu cần nghe (**RX → RX Shift**).
- **Zoom phổ**: giữ encoder để xoay vòng span ±24k → ±12k → ±6k → ±3k, hoặc vào **RX → Span**.
- **Marker Track**: **Tuning → Marker** = FIX (mặc định) hoặc TRACK. Ở TRACK, xoay encoder di chuyển điểm nghe trong dải hiển thị mà không làm giật hình phổ — tiện khi rà nhiều đài gần nhau.

---

## 5. Các chế độ thu (RX)

Vào **Menu → RX**:

| Mục | Phạm vi | Mô tả |
|---|---|---|
| BW | 100–24000 Hz | Độ rộng băng thông thu (bộ lọc FIR) |
| AGC | SLOW / FAST / AUTO | Tốc độ tự động điều chỉnh độ lợi |
| ATT(dB) | 0–31 dB | Suy hao đầu vào RX (PE4302), chống quá tải khi tín hiệu mạnh |
| Squelch | 0–100 | Câm tiếng khi tín hiệu dưới ngưỡng |
| Span | ±24k/±12k/±6k/±3k | Độ rộng hiển thị phổ/waterfall |
| NR | OFF/NR1/NR2 | Giảm nhiễu: **NR1** = bộ khử tiếng ồn theo đường tần số (line enhancer, hợp giọng nói/CW ổn định); **NR2** = khử nhiễu phổ kiểu Wiener (hợp nhiễu nền/trắng) |
| NR Level | 0–100 | Mức độ can thiệp của NR |
| NB | OFF/ON | Noise Blanker – chặn xung nhiễu tức thời (đánh lửa, sét) |
| NB Level | 0–100 | Độ nhạy phát hiện xung nhiễu |
| Notch | OFF/ON + Notch Hz (100–4000) | Bộ lọc khấc thủ công — tự chọn tần số cần triệt (tiếng rít cố định) |
| Beat Cxl | OFF/BC1/BC2 | Auto-notch tự động triệt sóng mang/tiếng rít liên tục mà không cần dò tay |
| RIT(Hz) | ±999 | Xem mục 4 |
| RX Shift | ±2000 Hz | Xem mục 4 |

---

## 6. Âm thanh (Audio)

**Menu → Audio**:

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| Volume | 0–100 | Cũng chỉnh nhanh bằng F1(-)/F2(+) ngoài menu |
| Bass | ±10 dB | Chỉnh âm trầm loa/tai nghe |
| Treble | ±10 dB | Chỉnh âm cao |
| Mic Gain | 0–100 | Độ lợi micro khi phát thoại |
| Digi Drive | 0–100 | Độ lợi audio riêng cho chế độ số (tách biệt với Mic Gain), dùng khi TX bằng tone số (RTTY/FT8) |
| Mic In | USB / MIC | Nguồn audio phát: micro vật lý hay audio nhận từ máy tính qua USB (khi chạy phần mềm số như WSJT-X) |

---

## 7. Phát sóng (TX) – SSB/AM/FM

**Menu → TX**:

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| RF Power | 5–100% (hoặc Watt nếu đã hiệu chuẩn PA Power) | Công suất phát ra |
| VOX | OFF/ON | Tự động chuyển TX khi có tiếng nói |
| VOX Gain | 0–100 | Độ nhạy VOX |
| VOX Delay | 100–2000 ms | Thời gian giữ TX sau khi hết tiếng |
| TX Low | 100–500 Hz | Cắt tần số thấp của audio phát (bộ lọc thoại) |
| TX High | 2200–3500 Hz | Cắt tần số cao của audio phát |

Nhấn giữ **PTT** để phát; thả ra để về thu. Khi TX, HEADER/S-meter chuyển sang hiển thị công suất phát và SWR; nếu bật **External ALC** hoặc **Power ALC**, driver tự điều chỉnh để giữ đúng công suất đã đặt (xem mục 17).

---

## 8. Chế độ CW

Chuyển **Mode → CW**. Cấu hình tại **Menu → CW**:

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| CW Decode | OFF/ON | Giải mã Morse hiển thị chữ trên màn hình; bật/tắt nhanh bằng **giữ F3** khi đang ở mode CW |
| Pitch | 300–900 Hz | Tần số tông nghe CW (cũng là tâm bộ lọc thu — passband tự dịch theo Pitch) |
| Speed | 5–40 WPM | Tốc độ chuột bấm/keyer |
| Keyer | STRAIGHT/IAMBIC-A/IAMBIC-B | Kiểu bàn khóa: chuột đơn hoặc bàn khóa điện tử 2 cần (paddle) kiểu A/B |
| Sidetone | 0–100% | Âm lượng tiếng "tự nghe" khi gõ |
| BK-IN | OFF/SEMI/FULL | Break-in: SEMI = tự chuyển TX khi gõ, tự về RX sau BK Delay; FULL = nghe được giữa các dấu chấm/gạch |
| BK Delay | 50–2000 ms | Thời gian giữ TX sau nhịp gõ cuối (chế độ SEMI) |
| CW Rev | OFF/ON | Đảo cạnh thu CW (CW/CW-R) |
| Paddle Rev | OFF/ON | Đảo DIT/DAH nếu bàn khóa lắp ngược |
| Filter | 50–500 Hz | Độ rộng bộ lọc CW quanh Pitch (băng hẹp giúp giảm nhiễu) |

Bàn khóa điện tử nối vào chân **DIT/DAH** (paddle); PTT tay vẫn hoạt động song song nếu cần lên tiếng bằng giọng nói giữa các contact.

---

## 9. Chế độ số RTTY

Dùng mode **USB, LSB, DIGU hoặc DIGL** (RTTY hoạt động trên cả 4 mode thoại/số). Cấu hình tại **Menu → RTTY**:

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| Decode | OFF/ON | Bật giải mã RTTY hiển thị chữ trên màn hình; bật/tắt nhanh bằng **giữ F3** |
| Baud | 45.45 / 50 / 75 | Tốc độ baud |
| Shift(Hz) | 170 / 425 / 850 | Độ lệch tần số Mark/Space |

Bộ giải mã dùng bộ dò biên độ vi sai (diff-ATC slicer) kèm AFC bám tần số ±60 Hz và hiển thị vạch đánh dấu tần số Mark/Space trên phổ để canh đài.

---

## 10. Chế độ số FT8 (ứng dụng toàn màn hình)

Từ menu gốc, chọn **FT8** (mục action cuối danh sách) để mở ứng dụng FT8 toàn màn hình — giải mã và (tùy chọn) tự động gọi CQ/trả lời không cần máy tính.

**Ở màn hình danh sách giải mã:**
- **F1** — mở màn hình khai báo callsign/grid của trạm mình.
- **F2** — bật/tắt (arm/disarm) chế độ tự phát CQ (phát "CQ <call> <grid>" vào các khe 15 giây xen kẽ).
- **Xoay encoder** — chọn dòng đài đã giải mã; **nhấn encoder** — bắt đầu QSO tự động với đài đó (máy tự chạy chuỗi grid → report → R-report → RR73).
- **MENU** hoặc **F4** — thoát ứng dụng, quay lại màn hình chính.

**Ở màn hình khai báo trạm (F1):**
- **Xoay encoder** — đổi ký tự tại vị trí đang chọn; **nhấn encoder** — sang vị trí kế tiếp.
- **F1** — lưu; **F4** — hủy.

> Cần khai báo đúng callsign + grid (F1) trước khi bật CQ beacon (F2), nếu không máy sẽ báo "SET CALL+GRID FIRST".

---

## 11. TUNE và SWR Scan

- **TUNE**: giữ phím TUNE để phát sóng mang liên tục công suất thấp — dùng chỉnh ATU ngoài hoặc kiểm tra SWR nhanh trước khi phát công suất đầy đủ. Nhả phím để dừng; có auto-stop an toàn nếu phím bị kẹt.
- **SWR Scan**: nhấn **F4** ngoài menu (khi menu đóng) để quét nhanh SWR quanh băng hiện tại, hoặc chọn mục **SWR Scan** ở menu gốc để quét đầy đủ. Trong lúc quét có thể giữ **F4** để hủy giữa chừng.

---

## 12. Sơ đồ đầy đủ hệ thống Menu

Nhấn **MENU** để mở/đóng. Điều hướng: **F1/F2** hoặc xoay encoder = di chuyển; **F3** hoặc nhấn encoder = chọn/vào group/chỉnh giá trị; **F4** = lùi lại một cấp hoặc thoát menu.

```
Menu (gốc)
├─ RX      → BW · AGC · ATT(dB) · Squelch · Span · NR · NR Level · NB · NB Level
│            · Notch · Notch Hz · Beat Cxl · RIT(Hz) · RX Shift
├─ Audio   → Volume · Bass · Treble · Mic Gain · Digi Drive · Mic In
├─ Tuning  → Step · Band · Mode · Marker
├─ TX      → RF Power · VOX · VOX Gain · VOX Delay · TX Low · TX High
├─ CW      → CW Decode · Pitch · Speed · Keyer · Sidetone · BK-IN · BK Delay
│            · CW Rev · Paddle Rev · Filter
├─ RTTY    → Decode · Baud · Shift(Hz)
├─ System  → Backlight · USB · USB Stream · PA (nhóm con) · Calibration (action)
│            · Factory Reset (action) · Clock (nhóm con) · About (nhóm con)
│   ├─ PA     → External PA · PA Key Delay · PA Drive Max · External ALC
│   │           · Bias Source · Bias 1 · Bias 2 · Idq Target · Bias Calibration (action)
│   ├─ Clock  → Set Time (HH:MM:SS) · Time Zone (giờ UTC±)
│   └─ About  → Version · Build Date
├─ SWR Scan   (action ở menu gốc)
└─ FT8        (action ở menu gốc — mở ứng dụng toàn màn hình)
```

Chi tiết nhóm **System**:

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| Backlight | 0–100% | Độ sáng đèn nền LCD |
| USB | Off/On | Bật/tắt cổng USB (CDC/Audio) |
| USB Stream | IQ / Demod | Dữ liệu audio gửi ra USB Audio: IQ thô (cho SDR#/HDSDR) hoặc audio đã giải điều chế |
| Calibration | action | Mở wizard hiệu chuẩn — xem mục 13 |
| Factory Reset | action | Đưa cài đặt vận hành về mặc định, **giữ nguyên** hiệu chuẩn phần cứng |
| Clock → Set Time | HH:MM:SS | Xoay encoder chỉnh từng trường giờ/phút/giây, nhấn để chuyển trường, nhấn ở trường giây để lưu |
| Clock → Time Zone | UTC −12…+14h | Múi giờ để quy đổi khi đồng bộ qua GPS/NMEA |

Chi tiết nhóm con **PA** (System → PA):

| Mục | Phạm vi | Ghi chú |
|---|---|---|
| External PA | OFF/ON | Bật giao tiếp với ampli công suất ngoài (relay keying) |
| PA Key Delay | 0–50 ms | Trễ trước khi cấp RF, chờ relay ampli ngoài đóng ổn định |
| PA Drive Max | 5–100% | Giới hạn trần drive khi dùng ampli ngoài |
| External ALC | OFF/ON | Nhận điện áp ALC phản hồi từ ampli ngoài để tự hạ drive tránh quá kích |
| Bias Source | FIXED/DAC | Phân cực PA: biến trở cố định hay DAC điều khiển được (MCP4822) |
| Bias 1 / Bias 2 | 0–200 | Mức DAC phân cực từng sò/tầng (chỉ có tác dụng khi Bias Source = DAC) |
| Idq Target | 50–2000 mA | Dòng tĩnh mục tiêu cho auto-cal vòng kín (cần cảm biến INA226) |
| Bias Calibration | action | Chạy tự động cân dòng tĩnh Idq theo Idq Target |

---

## 13. Hiệu chuẩn máy (Calibration)

Vào **System → Calibration** (hoặc nhấn F3/encoder khi đang chọn mục này) để mở màn hình wizard riêng, gồm các nhóm:

| Nhóm | Nội dung |
|---|---|
| **Frequency Cal** | XTAL PPM (bù sai số thạch anh), GPS Cal (lấy hiệu chuẩn tự động từ GPS 1PPS nếu có lắp), Apply |
| **IQ Calibration** | IQ Gain, IQ Phase, Auto IQ Cal (tự động cân bằng I/Q để giảm ảnh gương) |
| **DC Offset** | DC I Offset, DC Q Offset, Auto DC Cal (triệt DC leakage ở tâm băng thu) |
| **Audio Cal** | Audio Gain dB, Mic Gain |
| **RF / Display Cal** | S-Meter Offset, LO Offset Hz, Auto S-Meter, Meas Noise Floor, Auto AGC Ref |
| **PA Hardware** | PA Power (None/20W/45W/100W), OC Limit (ngưỡng bảo vệ quá dòng), PWR Scale % (hiệu chuẩn cảm biến công suất thuận/phản xạ) |
| **Band Cal** *(theo từng băng đang chọn)* | RX Gain Trim, Noise Floor Offset, TX Drive Trim, SWR Scale %, Auto Noise, Save Band Cal |
| **Save Settings / Load Settings / Reset Default / Exit Calibration** | Thao tác cấp cao nhất của wizard |

> Đây là công cụ hiệu chuẩn kỹ thuật (thường chỉ cần làm một lần lúc lắp máy hoặc khi thay linh kiện RF). Mỗi mục có action "Auto…" tự động đo và tính; chỉ chỉnh tay khi không có điều kiện đo chuẩn.

---

## 14. Điều khiển CAT (flrig / WSJT-X / Hamlib)

Máy giả lập giao thức **Kenwood TS-2000** (ID `019`) qua cổng **USB CDC (Virtual COM Port)**, khung ASCII kết thúc bằng `;`.

**Cấu hình phần mềm CAT** (flrig, Hamlib, WSJT-X…):
- Rig model: **Kenwood TS-2000**
- Port: cổng COM ảo do máy tạo ra khi cắm USB
- Baud rate: theo cấu hình driver CDC (mặc định driver không phụ thuộc baud thật do là USB ảo — chọn baud bất kỳ theo yêu cầu phần mềm, ví dụ 9600 hoặc 57600)
- Tắt polling/Auto Info liên tục quá nhanh nếu phần mềm cho tùy chọn — máy đã tối ưu để trả lời tuần tự, ổn định với flrig/WSJT-X.

**Bảng lệnh CAT hỗ trợ (tóm tắt):**

| Lệnh | Chức năng | Trạng thái |
|---|---|---|
| FA / FB | Tần số VFO A / B | Thật (điều khiển sống) |
| MD | Mode (LSB/USB/CW/FM/AM/DIGU/DIGL) | Thật |
| IF | Khung trạng thái đầy đủ | Thật (các trường chính) |
| TX / RX / TQ | Bật/tắt phát, hỏi trạng thái TX | Thật |
| AC | TUNE (tương thích TS-2000) | Thật |
| AG | Audio gain (volume) | Thật |
| SQ | Squelch | Thật |
| RA | RX attenuator | Thật |
| NR / RL | Noise Reduction + mức | Thật |
| NB | Noise Blanker | Thật |
| BC | Beat Canceller (auto-notch) | Thật |
| FW / SH / SL | Độ rộng lọc / cắt cao / cắt thấp | Thật |
| SM | S-meter | Thật |
| RM | Đồng hồ TX (SWR thật, ALC…) | Thật (SWR đo thật) |
| PC | Công suất phát | Thật |
| XA | RF-AGC (suy hao đầu vào tự động, lệnh riêng ngoài chuẩn TS-2000) | Thật |
| XS | Kiểu USB Audio stream (IQ/Demod) | Thật |
| VS / FR / FT / SP / DC | Chọn VFO, route RX/TX, Split, Dual-VFO | Thật |
| RT / RC / RU / RD | RIT bật/tắt/xóa/tăng/giảm | Thật |
| IS | IF shift | Thật |
| KS | Tốc độ keyer CW (WPM) | Thật |
| ID / PS / GT / PA / RG / TS / XT / MN / MP / LK / MG / EX | Nhận diện máy / trạng thái cố định | Giả lập cố định (để tương thích phần mềm) |
| UP/DN/BU/BD/MW/MR/DS/TC/KY/VV | Các lệnh phụ | Chỉ ACK, không thao tác phần cứng |

Với các lệnh không hỗ trợ, máy trả về `?;` theo đúng chuẩn Kenwood.

**Split/Dual-VFO**: điều khiển qua CAT (VS/FR/FT/DC) từ phần mềm — mặt máy không có phím Split riêng, chỉ có đảo VFO A/B nhanh bằng F3.

---

## 15. USB Audio – truyền IQ ra máy tính

Máy có thiết bị **USB Audio Class 1.0** (48kHz, stereo, 16-bit) song song với CAT:

- **RX → PC**: kênh trái = I, kênh phải = Q (khi **System → USB Stream = IQ**) — dùng với phần mềm SDR trên máy tính như HDSDR, SDR#. Nếu chọn **Demod**, máy gửi audio đã giải điều chế thay vì IQ thô.
- **PC → RX (TX)**: audio từ máy tính (ví dụ WSJT-X phát FT8/RTTY) đi vào máy qua cùng cổng USB Audio, chọn nguồn phát ở **Audio → Mic In = USB**.

---

## 16. GPS – hiệu chuẩn LO & đồng bộ giờ (tùy chọn)

Nếu có lắp module GPS (đầu ra 1PPS + NMEA):

- **1PPS** hiệu chuẩn sai số thạch anh LO tự động — chạy qua **Calibration → Frequency Cal → GPS Cal**.
- **NMEA ($GxRMC)** đồng bộ đồng hồ hệ thống theo giờ UTC vệ tinh, tự cộng với **System → Clock → Time Zone** để hiển thị đúng giờ địa phương. Nếu không có GPS, chỉnh giờ tay ở **Clock → Set Time**.

---

## 17. Bảo vệ PA & cảnh báo phần cứng

### Tự-kiểm tra lúc khởi động (Self-test)

Máy kiểm tra 6 hạng mục phần cứng khi bật nguồn: **FLASH** (bộ nhớ cài đặt), **CODEC** (WM8731 audio), **PLL** (Si5351 LO), **INA** (cảm biến dòng PA), **SAI** (giao tiếp audio DMA), **KEYS** (bàn phím PCA9555). Nếu mục nào lỗi, thanh HEADER hiện cảnh báo màu hổ phách — máy vẫn tiếp tục khởi động (không treo), nhưng chức năng liên quan có thể không hoạt động đúng (ví dụ lỗi INA thì không cân được Bias tự động, lỗi CODEC/SAI thì mất âm thanh).

### Bảo vệ PA khi phát (PA Protect)

Máy giám sát liên tục dòng điện, nhiệt độ, SWR khi TX và tự động giảm công suất theo các cấp:

```
NORMAL → FOLDBACK (còn 75% → 50%) → LIMIT (còn 25%) → TRIP (cắt 0%, ngừng TX) → COOLDOWN → NORMAL
```

- Ngưỡng mặc định: SWR cảnh báo 2.0 / cắt 4.0; nhiệt độ cảnh báo 75°C / cắt 90°C; dòng điện cảnh báo và cắt theo **OC Limit** đã hiệu chuẩn (Calibration → PA Hardware).
- Khi vào **TRIP**, máy tự chuyển **COOLDOWN** và giữ TX khóa đến khi các thông số về dưới ngưỡng an toàn (~90% mức cảnh báo) rồi mới cho phát lại bình thường.
- **External ALC** (nếu bật) giảm dần drive khi điện áp phản hồi từ ampli ngoài vượt 70%, không phụ thuộc state machine trên.
- **Power ALC** (luôn chạy khi đã khai báo PA Power khác None) tự bù để công suất ra ăng-ten đúng bằng giá trị đặt ở RF Power, bù cả khi sụt áp nguồn hay đổi băng tần.

---

## 18. Lưu cài đặt & Factory Reset

- Cài đặt được lưu tự động (không cần thao tác gì thêm) khi: đóng menu, sau ~3 giây không thao tác thêm (debounce) kể từ lần thay đổi cuối, hoặc khi giữ phím PW để tắt máy (lưu ngay trước khi cắt nguồn).
- **RIT không được lưu** — luôn về 0 sau khi khởi động lại (theo thiết kế, tránh quên RIT còn lệch từ phiên trước).
- **Factory Reset** (System → Factory Reset) đưa các cài đặt vận hành (band, mode, volume, RF power, v.v.) về mặc định nhà sản xuất, nhưng **giữ nguyên toàn bộ giá trị hiệu chuẩn phần cứng** (xtal, IQ, DC offset, LO offset, S-meter offset, PA Power/OC Limit…) — không cần chạy lại Calibration sau khi Factory Reset.

---

## 19. Khắc phục sự cố

| Hiện tượng | Nguyên nhân thường gặp | Cách xử lý |
|---|---|---|
| Mất âm thanh RX hoàn toàn | Squelch đặt quá cao, hoặc lỗi CODEC/SAI (xem cảnh báo HEADER) | Kiểm tra **RX → Squelch = 0**, kiểm tra self-test khi khởi động |
| Không nghe thấy gì dù đúng tần số | ATT quá cao, hoặc BW/Notch/Beat Cxl chặn nhầm dải cần nghe | Giảm **ATT**, tắt thử **Notch**/**Beat Cxl** |
| Tự động giảm công suất khi phát lâu | PA Protect vào FOLDBACK/LIMIT do nóng/dòng cao/SWR cao | Kiểm tra ăng-ten (SWR), tản nhiệt PA, chờ COOLDOWN |
| Không phát được (TX bị khóa hẳn) | PA đang ở TRIP/COOLDOWN | Chờ nhiệt độ/dòng điện giảm, hoặc nhả PTT và thử lại sau vài giây |
| flrig/WSJT-X không điều khiển được | Sai rig model hoặc cổng COM | Chọn đúng **Kenwood TS-2000**, đúng cổng COM CDC của máy |
| Không thấy đài nào trong FT8/RTTY | Chưa đúng mode (nên để DIGU/DIGL hoặc USB/LSB), chưa bật Decode | Kiểm tra **Tuning → Mode**, bật **RTTY Decode** hoặc mở app **FT8** |
| Không phát được CQ FT8 | Chưa khai báo Call/Grid | Vào app FT8 → **F1** nhập callsign + grid trước khi bấm **F2** |
| Máy không lưu cài đặt sau khi tắt | Rút nguồn đột ngột thay vì giữ PW ≥3 giây | Luôn tắt máy bằng cách giữ phím PW cho đến khi thấy "POWERING OFF" |
| Cảnh báo hổ phách trên HEADER lúc khởi động | Một hạng mục self-test lỗi (FLASH/CODEC/PLL/INA/SAI/KEYS) | Xem mục 17; nếu lặp lại, kiểm tra kết nối phần cứng tương ứng |

---

## 20. Phụ lục

### Bảng băng tần

| Band | Tần số trung tâm | Band | Tần số trung tâm |
|---|---|---|---|
| 160m | 1.8 MHz | 17m | 18.1 MHz |
| 80m | 3.5 MHz | 15m | 21.0 MHz |
| 60m | 5.3 MHz | 12m | 24.9 MHz |
| 40m | 7.0 MHz | 10m | 28.0 MHz |
| 30m | 10.1 MHz | | |
| 20m | 14.0 MHz | | |

*(6m đã được loại khỏi danh sách băng tần ở firmware hiện hành.)*

### Bảng mode

| Mode | Ý nghĩa |
|---|---|
| AM | Điều biên |
| FM | Điều tần |
| USB | Đơn biên trên (thoại) |
| LSB | Đơn biên dưới (thoại) |
| CW | Morse liên tục sóng mang (xem mục 8) |
| DIGU | Số liệu USB (WSJT-X/FT8/RTTY/DATA-USB) |
| DIGL | Số liệu LSB (DATA-LSB) |

### Xem thêm phím tắt nhanh

- Volume: **F1** (giảm) / **F2** (tăng), giữ để lặp.
- Đổi Step: nhấn encoder.
- Đổi Span phổ: giữ encoder.
- Đảo VFO A/B: **F3** (ngoài menu).
- SWR Scan nhanh: **F4** (ngoài menu).
- Bật/tắt CW hoặc RTTY Decode: giữ **F3** khi đúng mode tương ứng.
