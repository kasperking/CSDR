/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_composite.c
  * @brief   USB Composite Device: CDC VCP + UAC 1.0 Audio (Full-Duplex)
  *
  *  Self-contained composite driver. Handles BOTH classes inline — does NOT
  *  depend on USBD_CDC class library (avoids API breakage across ST USBD
  *  library versions).
  *
  *  Interface layout:
  *    IAD Audio (IF 0,1,2) + IAD CDC (IF 3,4)
  *  Endpoints:
  *    EP1 IN/OUT  bulk 64B   CDC Data
  *    EP2 IN      intr 8B    CDC Notification
  *    EP3 IN      iso 192B   Audio MCU→PC
  *    EP3 OUT     iso 192B   Audio PC→MCU
  *
  *  CDC interface layer (usbd_cdc_if.c) fops are invoked directly for
  *  Init/DeInit/Receive/TransmitCplt.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usb_composite.h"
#include "usbd_ioreq.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_if.h"
#include "usb_audio.h"
#include "csdr_app.h"
#include <string.h>

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* ───────────── Forward declarations ───────────── */
static uint8_t Comp_Init      (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t Comp_DeInit    (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t Comp_Setup     (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t Comp_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t Comp_DataIn    (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t Comp_DataOut   (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t Comp_SOF       (USBD_HandleTypeDef *pdev);
static uint8_t Comp_IsoInInc  (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t Comp_IsoOutInc (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *Comp_GetCfg   (uint16_t *length);
static uint8_t *Comp_GetDev   (uint16_t *length);

/* ───────────── Audio state ───────────── */
static volatile uint8_t s_as_out_alt = 0U;
static volatile uint8_t s_as_in_alt  = 0U;
static volatile uint8_t s_audio_in_tx_pending = 0U;

/* Debug: watch these in Live Expressions to verify interface activation */
volatile uint8_t  dbg_as_out_alt   = 0U;
volatile uint8_t  dbg_as_in_alt    = 0U;
volatile uint32_t dbg_iso_out_inc  = 0U;

/* USB audio watchdog counters – watched from csdr_app.c via extern.
 * SOF should tick at exactly 1 kHz when the host is connected.
 * iso_in_cnt should track ~1:1 with SOF while streaming.
 * stall_cnt > 0 means USBD_LL_Transmit is rejecting packets (endpoint not ready). */
volatile uint32_t dbg_usb_sof_cnt    = 0U;
volatile uint32_t dbg_usb_iso_in_cnt = 0U;
volatile uint32_t dbg_usb_stall_cnt  = 0U;

/* USB lifecycle counters — watch these in Live Expressions.
 *  reset_cnt:    USB bus reset events (host re-enumeration, cable pull, suspend→resume).
 *                Should increment once at attach. Multiple increments indicate bus noise.
 *  suspend_cnt:  Host sent SUSPEND (PC sleep, idle 3ms+ without SOF). SOF stops.
 *  resume_cnt:   Host sent RESUME (SOF restarts). Should pair 1:1 with suspend_cnt.
 *  cdc_rx_pkts:  CDC OUT packets accepted from host (each CAT command burst = 1 packet).
 *  cdc_tx_pkts:  CDC IN packets successfully queued to host.
 *  cdc_tx_drop:  CAT FlushTX found USBD_BUSY — response deferred (retry next call).
 *  cdc_busy_max: Longest BUSY run seen (consecutive CAT_FlushTX calls that returned BUSY).
 *                If this climbs without bound, the CDC IN endpoint has stalled permanently.
 *  cat_rx_bytes: Total bytes received by CAT_Receive from USB ISR.
 *  cat_tx_bytes: Total bytes sent to CDC_Transmit_FS by CAT_FlushTX.
 *  cat_tx_fifo_drop: Characters silently dropped because s_tx_fifo was full (512B limit). */
volatile uint32_t dbg_usb_reset_cnt   = 0U;
volatile uint32_t dbg_usb_suspend_cnt = 0U;
volatile uint32_t dbg_usb_resume_cnt  = 0U;
volatile uint32_t dbg_cdc_rx_pkts     = 0U;
volatile uint32_t dbg_cdc_tx_pkts     = 0U;
volatile uint32_t dbg_cdc_tx_drop     = 0U;
volatile uint32_t dbg_cdc_busy_max    = 0U;
static   uint32_t s_cdc_busy_run      = 0U;   /* current consecutive BUSY run length */

/* dbg_cdc_fail_cnt: USBD_LL_Transmit returned a non-OK, non-BUSY status.
 *   Non-zero at startup means enumeration incomplete when FlushTX fires.
 *   Non-zero after streaming = endpoint closed or USB not configured. */
volatile uint32_t dbg_cdc_fail_cnt   = 0U;
/* dbg_comp_datain_cnt: CDC DataIn callback fire count.
 *   Must grow at the same rate as dbg_cdc_tx_pkts.  If tx_pkts grows but
 *   this stops, the CDC IN endpoint is stuck in hardware (FIFO stall). */
volatile uint32_t dbg_comp_datain_cnt = 0U;
/* dbg_cdc_stuck_recovery: s_cdc_tx_busy stuck with stalled EP, self-cleared.
 *   Non-zero = stall-recovery path fired in Composite_CDC_IsBusy.
 *   Increments on each stall detection. */
volatile uint32_t dbg_cdc_stuck_recovery = 0U;

static __ALIGN_BEGIN uint8_t s_audio_out_buf[COMP_EP_AUDIO_SIZE] __ALIGN_END;
static __ALIGN_BEGIN uint8_t s_audio_in_buf [COMP_EP_AUDIO_SIZE] __ALIGN_END;
static __ALIGN_BEGIN uint8_t s_ac_scratch   [8]                  __ALIGN_END;

/* ───────────── CDC state (internal) ───────────── */
static __ALIGN_BEGIN uint8_t s_cdc_rx_buf [COMP_EP_CDC_SIZE] __ALIGN_END;
static __ALIGN_BEGIN uint8_t s_cdc_cmd_buf[8]                __ALIGN_END;
static volatile uint8_t      s_cdc_tx_busy = 0U;
static uint8_t               s_cdc_line_coding[7] =
  { 0x00, 0xC2, 0x01, 0x00,  /* 115200 bps */
    0x00,                    /* 1 stop bit */
    0x00,                    /* no parity  */
    0x08 };                  /* 8 data bits*/
static uint8_t s_cdc_cmd_opcode = 0xFFU;
static uint8_t s_cdc_cmd_length = 0U;

/* CDC class request codes come from usbd_cdc.h (via usb_composite.h) */

/* ───────────── Class driver export ───────────── */
USBD_ClassTypeDef USBD_Composite =
{
  Comp_Init,
  Comp_DeInit,
  Comp_Setup,
  NULL,
  Comp_EP0_RxReady,
  Comp_DataIn,
  Comp_DataOut,
  Comp_SOF,
  Comp_IsoInInc,
  Comp_IsoOutInc,
  Comp_GetCfg,
  Comp_GetCfg,
  Comp_GetCfg,
  Comp_GetDev,
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration descriptor
 * ═══════════════════════════════════════════════════════════════════════════ */

#define UNIT_IT_OUT   0x01U
#define UNIT_FU_OUT   0x02U
#define UNIT_OT_OUT   0x03U
#define UNIT_IT_IN    0x04U
#define UNIT_FU_IN    0x05U
#define UNIT_OT_IN    0x06U

#define TT_USB_STREAMING   0x0101U
#define TT_SPEAKER         0x0301U
#define TT_MICROPHONE      0x0201U

__ALIGN_BEGIN static const uint8_t s_cfg_desc[] __ALIGN_END =
{
  /* Configuration (9), wTotalLength = 268 */
  0x09, USB_DESC_TYPE_CONFIGURATION,
  (uint8_t)(COMP_CFG_DESC_SIZ & 0xFFU), (uint8_t)(COMP_CFG_DESC_SIZ >> 8),
  COMP_NUM_INTERFACES, 0x01, 0x00, 0xC0, 0x32,

  /* IAD Audio (8) */
  0x08, 0x0B, COMP_IF_AC, 0x03, 0x01, 0x01, 0x00, 0x00,

  /* AC IF (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_AC, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

  //* AC Header (10), class-specific AC wTotalLength = 72 */
  0x0A, 0x24, 0x01, 0x00, 0x01, 0x48, 0x00, 0x02,
  COMP_IF_AS_OUT, COMP_IF_AS_IN,

  /* IT OUT (12) */
  0x0C, 0x24, 0x02, UNIT_IT_OUT,
  (uint8_t)(TT_USB_STREAMING & 0xFF), (uint8_t)(TT_USB_STREAMING >> 8),
  0x00, 0x02, 0x03, 0x00, 0x00, 0x00,

  /* FU OUT (10): master + left + right controls */
  0x0A, 0x24, 0x06, UNIT_FU_OUT, UNIT_IT_OUT,
  0x01, 0x01, 0x00, 0x00, 0x00,

  /* OT OUT (9) */
  0x09, 0x24, 0x03, UNIT_OT_OUT,
  (uint8_t)(TT_SPEAKER & 0xFF), (uint8_t)(TT_SPEAKER >> 8),
  0x00, UNIT_FU_OUT, 0x00,

  /* IT IN (12) */
  0x0C, 0x24, 0x02, UNIT_IT_IN,
  (uint8_t)(TT_MICROPHONE & 0xFF), (uint8_t)(TT_MICROPHONE >> 8),
  0x00, 0x02, 0x03, 0x00, 0x00, 0x00,

   /* FU IN (10): master + left + right controls */
  0x0A, 0x24, 0x06, UNIT_FU_IN, UNIT_IT_IN,
  0x01, 0x01, 0x00, 0x00, 0x00,

  /* OT IN (9) */
  0x09, 0x24, 0x03, UNIT_OT_IN,
  (uint8_t)(TT_USB_STREAMING & 0xFF), (uint8_t)(TT_USB_STREAMING >> 8),
  0x00, UNIT_FU_IN, 0x00,

  /* AS OUT alt 0 (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_AS_OUT, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,

  /* AS OUT alt 1 (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_AS_OUT, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00,

  /* AS General (7) */
  0x07, 0x24, 0x01, UNIT_IT_OUT, 0x01, 0x01, 0x00,

  /* Format Type I PCM 48kHz s16 stereo (11) */
  0x0B, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10,
  0x01, 0x80, 0xBB, 0x00,

  /* AS OUT iso EP std (9) */
  0x09, USB_DESC_TYPE_ENDPOINT,
  COMP_EP_AUDIO_OUT, 0x0D,
  (uint8_t)(COMP_EP_AUDIO_SIZE & 0xFF),
  (uint8_t)(COMP_EP_AUDIO_SIZE >> 8),
  0x01, 0x00, 0x00,

  /* AS OUT iso EP CS (7) */
  0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* AS IN alt 0 (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_AS_IN, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,

  /* AS IN alt 1 (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_AS_IN, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00,

  /* AS General (7) */
  0x07, 0x24, 0x01, UNIT_OT_IN, 0x01, 0x01, 0x00,

  /* Format Type I (11) */
  0x0B, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10,
  0x01, 0x80, 0xBB, 0x00,

  /* AS IN iso EP std (9) */
  0x09, USB_DESC_TYPE_ENDPOINT,
  COMP_EP_AUDIO_IN, 0x0D,
  (uint8_t)(COMP_EP_AUDIO_SIZE & 0xFF),
  (uint8_t)(COMP_EP_AUDIO_SIZE >> 8),
  0x01, 0x00, 0x00,

  /* AS IN iso EP CS (7) */
  0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* IAD CDC (8) */
  0x08, 0x0B, COMP_IF_CDC_CTRL, 0x02, 0x02, 0x02, 0x01, 0x00,

  /* CDC ACM IF (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_CDC_CTRL, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

  /* CDC Header (5) */
  0x05, 0x24, 0x00, 0x10, 0x01,

  /* CDC Call Mgmt (5) */
  0x05, 0x24, 0x01, 0x00, COMP_IF_CDC_DATA,

  /* CDC ACM (4) */
  0x04, 0x24, 0x02, 0x02,

  /* CDC Union (5) */
  0x05, 0x24, 0x06, COMP_IF_CDC_CTRL, COMP_IF_CDC_DATA,

  /* EP CDC Notif (7) */
  0x07, USB_DESC_TYPE_ENDPOINT,
  COMP_EP_CDC_NOTIF, 0x03,
  (uint8_t)(COMP_EP_CDC_NOTIF_SIZE & 0xFF),
  (uint8_t)(COMP_EP_CDC_NOTIF_SIZE >> 8),
  0x10,

  /* CDC Data IF (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  COMP_IF_CDC_DATA, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

  /* EP CDC OUT (7) */
  0x07, USB_DESC_TYPE_ENDPOINT,
  COMP_EP_CDC_OUT, 0x02,
  (uint8_t)(COMP_EP_CDC_SIZE & 0xFF),
  (uint8_t)(COMP_EP_CDC_SIZE >> 8),
  0x00,

  /* EP CDC IN (7) */
  0x07, USB_DESC_TYPE_ENDPOINT,
  COMP_EP_CDC_IN, 0x02,
  (uint8_t)(COMP_EP_CDC_SIZE & 0xFF),
  (uint8_t)(COMP_EP_CDC_SIZE >> 8),
  0x00,
};

enum { COMP_CFG_DESC_SIZE_CHECK = 1 / ((sizeof(s_cfg_desc) == COMP_CFG_DESC_SIZ) ? 1 : 0) };

static uint8_t *Comp_GetCfg(uint16_t *length)
{
  *length = (uint16_t)sizeof(s_cfg_desc);
  return (uint8_t *)s_cfg_desc;
}

static uint8_t *Comp_GetDev(uint16_t *length)
{
  static uint8_t qualifier[] = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00, 0x02, 0xEF, 0x02, 0x01, 0x40, 0x01, 0x00
  };
  *length = sizeof(qualifier);
  return qualifier;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  CDC handling (standalone - does not use USBD_CDC class)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void CDC_Open(USBD_HandleTypeDef *pdev)
{
  USBD_LL_OpenEP(pdev, COMP_EP_CDC_IN,    USBD_EP_TYPE_BULK, COMP_EP_CDC_SIZE);
  pdev->ep_in[COMP_EP_CDC_IN & 0x0FU].is_used = 1U;
  USBD_LL_OpenEP(pdev, COMP_EP_CDC_OUT,   USBD_EP_TYPE_BULK, COMP_EP_CDC_SIZE);
  pdev->ep_out[COMP_EP_CDC_OUT & 0x0FU].is_used = 1U;
  USBD_LL_OpenEP(pdev, COMP_EP_CDC_NOTIF, USBD_EP_TYPE_INTR, COMP_EP_CDC_NOTIF_SIZE);
  pdev->ep_in[COMP_EP_CDC_NOTIF & 0x0FU].is_used = 1U;
  pdev->ep_in[COMP_EP_CDC_NOTIF & 0x0FU].bInterval = 0x10U;

  s_cdc_tx_busy        = 0U;
  dbg_comp_datain_cnt  = 0U;   /* reset per-session so it pairs 1:1 with dbg_cdc_tx_pkts */
  USBD_Interface_fops_FS.Init();

  USBD_LL_PrepareReceive(pdev, COMP_EP_CDC_OUT,
                          s_cdc_rx_buf, COMP_EP_CDC_SIZE);
}

static void CDC_Close(USBD_HandleTypeDef *pdev)
{
  USBD_LL_CloseEP(pdev, COMP_EP_CDC_IN);
  pdev->ep_in[COMP_EP_CDC_IN & 0x0FU].is_used = 0U;
  USBD_LL_CloseEP(pdev, COMP_EP_CDC_OUT);
  pdev->ep_out[COMP_EP_CDC_OUT & 0x0FU].is_used = 0U;
  USBD_LL_CloseEP(pdev, COMP_EP_CDC_NOTIF);
  pdev->ep_in[COMP_EP_CDC_NOTIF & 0x0FU].is_used = 0U;

  s_cdc_tx_busy = 0U;
  /* Flush stale CAT state so a reconnecting host gets a clean slate.
   * Without this, partial commands from the previous session get parsed
   * on reconnect, and queued responses cause infinite CAT_FlushTX retries
   * until the endpoint is fully open. */
  CSDR_CDC_ResetCAT();

  USBD_Interface_fops_FS.DeInit();
}

static uint8_t CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bRequest)
  {
    case CDC_GET_LINE_CODING:
      USBD_CtlSendData(pdev, s_cdc_line_coding, 7U);
      break;

    case CDC_SET_LINE_CODING:
      s_cdc_cmd_opcode = CDC_SET_LINE_CODING;
      s_cdc_cmd_length = (uint8_t)((req->wLength > 7U) ? 7U : req->wLength);
      USBD_CtlPrepareRx(pdev, s_cdc_line_coding, s_cdc_cmd_length);
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* DTR/RTS — just ACK */
      break;

    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SEND_BREAK:
      if ((req->bmRequest & 0x80U) && (req->wLength > 0U)) {
        USBD_CtlSendData(pdev, s_cdc_cmd_buf,
                          (req->wLength > 8U) ? 8U : req->wLength);
      }
      break;

    default:
      break;
  }
  return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Audio class request handler (MUTE only)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define AC_SET_CUR   0x01U
#define AC_GET_CUR   0x81U
#define AC_GET_MIN   0x82U
#define AC_GET_MAX   0x83U
#define AC_GET_RES   0x84U

static uint8_t Comp_AC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bRequest) {
    case AC_GET_CUR:
    case AC_GET_MIN:
    case AC_GET_MAX:
    case AC_GET_RES:
      s_ac_scratch[0] = 0U;
      s_ac_scratch[1] = 0U;
      USBD_CtlSendData(pdev, s_ac_scratch,
                        (req->wLength < 2U) ? req->wLength : 2U);
      break;

    case AC_SET_CUR: {
      uint16_t ln = req->wLength;
      if (ln > sizeof(s_ac_scratch)) ln = sizeof(s_ac_scratch);
      USBD_CtlPrepareRx(pdev, s_ac_scratch, ln);
      break;
    }

    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
  }
  return USBD_OK;
}

static uint8_t Comp_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  (void)pdev;
  if (s_cdc_cmd_opcode == CDC_SET_LINE_CODING) {
    s_cdc_cmd_opcode = 0xFFU;
    s_cdc_cmd_length = 0U;
  }
  return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main class callbacks
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t Comp_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)cfgidx;
  CDC_Open(pdev);
  s_as_out_alt = 0U;
  s_as_in_alt  = 0U;
  return USBD_OK;
}

static uint8_t Comp_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  (void)cfgidx;
  if (s_as_in_alt) {
    USBD_LL_CloseEP(pdev, COMP_EP_AUDIO_IN);
    pdev->ep_in[COMP_EP_AUDIO_IN & 0x0FU].is_used = 0U;
    s_as_in_alt = 0U;
  }
  if (s_as_out_alt) {
    USBD_LL_CloseEP(pdev, COMP_EP_AUDIO_OUT);
    pdev->ep_out[COMP_EP_AUDIO_OUT & 0x0FU].is_used = 0U;
    s_as_out_alt = 0U;
  }
  CDC_Close(pdev);
  USB_Audio_SetStreaming(&g_usb_audio, 0);
  return USBD_OK;
}

static uint8_t Comp_AS_SetInterface(USBD_HandleTypeDef *pdev,
                                     uint8_t ifnum, uint8_t alt)
{
  if (ifnum == COMP_IF_AS_OUT) {
    if (alt == COMP_AS_ALT_ACTIVE && s_as_out_alt == 0U) {
      USBD_LL_OpenEP(pdev, COMP_EP_AUDIO_OUT,
                      USBD_EP_TYPE_ISOC, COMP_EP_AUDIO_SIZE);
      pdev->ep_out[COMP_EP_AUDIO_OUT & 0x0FU].is_used = 1U;
      /* ISO OUT uses hardware double-buffering: arm BOTH banks at open */
      USBD_LL_PrepareReceive(pdev, COMP_EP_AUDIO_OUT,
                              s_audio_out_buf, COMP_EP_AUDIO_SIZE);
      USBD_LL_PrepareReceive(pdev, COMP_EP_AUDIO_OUT,
                              s_audio_out_buf, COMP_EP_AUDIO_SIZE);
      s_as_out_alt     = 1U;
      dbg_as_out_alt   = 1U;
    } else if (alt == COMP_AS_ALT_ZERO && s_as_out_alt == 1U) {
      USBD_LL_CloseEP(pdev, COMP_EP_AUDIO_OUT);
      pdev->ep_out[COMP_EP_AUDIO_OUT & 0x0FU].is_used = 0U;
      s_as_out_alt   = 0U;
      dbg_as_out_alt = 0U;
    }
  } else if (ifnum == COMP_IF_AS_IN) {
    if (alt == COMP_AS_ALT_ACTIVE && s_as_in_alt == 0U) {
      USBD_LL_OpenEP(pdev, COMP_EP_AUDIO_IN,
                      USBD_EP_TYPE_ISOC, COMP_EP_AUDIO_SIZE);
      pdev->ep_in[COMP_EP_AUDIO_IN & 0x0FU].is_used = 1U;
      s_as_in_alt   = 1U;
      dbg_as_in_alt = 1U;
      /* Kick-start: try transmit first packet synchronously. If it fails
       * (EP not ready), SOF callback will retry next frame. */
      (void)USB_Audio_ReadRXPacket(&g_usb_audio, s_audio_in_buf);
      if (USBD_LL_Transmit(pdev, COMP_EP_AUDIO_IN,
                            s_audio_in_buf, COMP_EP_AUDIO_SIZE) == USBD_OK) {
        s_audio_in_tx_pending = 1U;
      } else {
        s_audio_in_tx_pending = 0U;
      }
    } else if (alt == COMP_AS_ALT_ZERO && s_as_in_alt == 1U) {
      USBD_LL_CloseEP(pdev, COMP_EP_AUDIO_IN);
      pdev->ep_in[COMP_EP_AUDIO_IN & 0x0FU].is_used = 0U;
      s_as_in_alt           = 0U;
      dbg_as_in_alt         = 0U;
      s_audio_in_tx_pending = 0U;
    }
  }
  USB_Audio_SetStreaming(&g_usb_audio, (s_as_in_alt || s_as_out_alt) ? 1 : 0);
  return USBD_OK;
}

static uint8_t Comp_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t  recipient = req->bmRequest & USB_REQ_RECIPIENT_MASK;
  uint8_t  type      = req->bmRequest & USB_REQ_TYPE_MASK;
  uint16_t ifnum     = req->wIndex & 0x00FFU;

  if (type == USB_REQ_TYPE_STANDARD) {
    switch (req->bRequest) {
      case USB_REQ_SET_INTERFACE:
        if (ifnum == COMP_IF_AS_OUT || ifnum == COMP_IF_AS_IN) {
          return Comp_AS_SetInterface(pdev, (uint8_t)ifnum, (uint8_t)req->wValue);
        }
        return USBD_OK;

      case USB_REQ_GET_INTERFACE: {
        uint8_t alt = 0U;
        if (ifnum == COMP_IF_AS_OUT) alt = s_as_out_alt;
        else if (ifnum == COMP_IF_AS_IN) alt = s_as_in_alt;
        USBD_CtlSendData(pdev, &alt, 1U);
        return USBD_OK;
      }

      case USB_REQ_GET_STATUS:
        s_ac_scratch[0] = 0U;
        s_ac_scratch[1] = 0U;
        USBD_CtlSendData(pdev, s_ac_scratch, 2U);
        return USBD_OK;

      default:
        USBD_CtlError(pdev, req);
        return USBD_FAIL;
    }
  }

  if (type == USB_REQ_TYPE_CLASS) {
    if (recipient == USB_REQ_RECIPIENT_INTERFACE) {
      if (ifnum == COMP_IF_AC ||
          ifnum == COMP_IF_AS_OUT ||
          ifnum == COMP_IF_AS_IN) {
        return Comp_AC_Setup(pdev, req);
      }
      if (ifnum == COMP_IF_CDC_CTRL ||
          ifnum == COMP_IF_CDC_DATA) {
        return CDC_Setup(pdev, req);
      }
    }
  }

  USBD_CtlError(pdev, req);
  return USBD_FAIL;
}

static uint8_t Comp_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  (void)pdev;

  /* Audio IN: transfer complete, clear pending and queue next */
  if ((epnum | 0x80U) == COMP_EP_AUDIO_IN) {
    s_audio_in_tx_pending = 0U;
    if (s_as_in_alt) {
      (void)USB_Audio_ReadRXPacket(&g_usb_audio, s_audio_in_buf);
      if (USBD_LL_Transmit(pdev, COMP_EP_AUDIO_IN,
                            s_audio_in_buf, COMP_EP_AUDIO_SIZE) == USBD_OK) {
        s_audio_in_tx_pending = 1U;
        dbg_usb_iso_in_cnt++;
      } else {
        /* Endpoint not ready — SOF will retry next frame. */
        dbg_usb_stall_cnt++;
      }
    }
    return USBD_OK;
  }

  /* CDC IN bulk completion */
  if ((epnum | 0x80U) == COMP_EP_CDC_IN) {
    dbg_comp_datain_cnt++;
    s_cdc_tx_busy = 0U;
    if (USBD_Interface_fops_FS.TransmitCplt != NULL) {
      uint32_t zero_len = 0U;
      (void)USBD_Interface_fops_FS.TransmitCplt(NULL, &zero_len, epnum);
    }
    return USBD_OK;
  }

  /* CDC Notification — nothing to do */
  return USBD_OK;
}

static uint8_t Comp_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* Audio OUT */
  if (epnum == (COMP_EP_AUDIO_OUT & 0x0FU)) {
    uint32_t len = USBD_LL_GetRxDataSize(pdev, COMP_EP_AUDIO_OUT);
    if (len > 0U && len <= COMP_EP_AUDIO_SIZE) {
      USB_Audio_WriteTX(&g_usb_audio, s_audio_out_buf, (uint16_t)len);
    }
    if (s_as_out_alt) {
      USBD_LL_PrepareReceive(pdev, COMP_EP_AUDIO_OUT,
                              s_audio_out_buf, COMP_EP_AUDIO_SIZE);
    }
    return USBD_OK;
  }

  /* CDC Data OUT */
  if (epnum == (COMP_EP_CDC_OUT & 0x0FU)) {
    uint32_t len = USBD_LL_GetRxDataSize(pdev, COMP_EP_CDC_OUT);
    dbg_cdc_rx_pkts++;
    (void)USBD_Interface_fops_FS.Receive(s_cdc_rx_buf, &len);
    USBD_LL_PrepareReceive(pdev, COMP_EP_CDC_OUT,
                            s_cdc_rx_buf, COMP_EP_CDC_SIZE);
    return USBD_OK;
  }

  return USBD_OK;
}

static uint8_t Comp_SOF(USBD_HandleTypeDef *pdev)
{
  dbg_usb_sof_cnt++;
  /* Pump Audio IN: if streaming and no transfer in flight, send a packet.
   * This kick-starts the pipeline after SET_INTERFACE alt 1. */
  if (s_as_in_alt && !s_audio_in_tx_pending) {
    (void)USB_Audio_ReadRXPacket(&g_usb_audio, s_audio_in_buf);
    if (USBD_LL_Transmit(pdev, COMP_EP_AUDIO_IN,
                          s_audio_in_buf, COMP_EP_AUDIO_SIZE) == USBD_OK) {
      s_audio_in_tx_pending = 1U;
      dbg_usb_iso_in_cnt++;
    } else {
      dbg_usb_stall_cnt++;
    }
  }
  return USBD_OK;
}

static uint8_t Comp_IsoInInc (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* ISO IN transfer incomplete (host didn't poll in time).
   * Clear pending so SOF can retry. */
  (void)pdev; (void)epnum;
  s_audio_in_tx_pending = 0U;
  return USBD_OK;
}

static uint8_t Comp_IsoOutInc(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* ISO OUT incomplete (host sent nothing this frame). Re-arm so the
   * double-buffer stays primed and DataOut fires on the next SOF. */
  (void)epnum;
  if (s_as_out_alt) {
    USBD_LL_PrepareReceive(pdev, COMP_EP_AUDIO_OUT,
                            s_audio_out_buf, COMP_EP_AUDIO_SIZE);
  }
  dbg_iso_out_inc++;
  return USBD_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

uint8_t Composite_CDC_Transmit(USBD_HandleTypeDef *pdev,
                                uint8_t *buf, uint16_t len)
{
  if (pdev == NULL || buf == NULL || len == 0U) return USBD_FAIL;

  /* Stall-recovery: if the host stalled then un-stalled CDC IN (e.g. Windows
   * COM port reset), DataIn never fires so s_cdc_tx_busy stays 1 permanently.
   * Detect this and self-recover so CAT TX is not silently frozen. */
  if (s_cdc_tx_busy && USBD_LL_IsStallEP(pdev, COMP_EP_CDC_IN)) {
    s_cdc_tx_busy = 0U;
    s_cdc_busy_run = 0U;
  }

  if (s_cdc_tx_busy) {
    dbg_cdc_tx_drop++;
    s_cdc_busy_run++;
    if (s_cdc_busy_run > dbg_cdc_busy_max) dbg_cdc_busy_max = s_cdc_busy_run;
    return USBD_BUSY;
  }
  s_cdc_busy_run = 0U;

  s_cdc_tx_busy = 1U;
  USBD_StatusTypeDef st = USBD_LL_Transmit(pdev, COMP_EP_CDC_IN, buf, len);
  if (st != USBD_OK) {
    s_cdc_tx_busy = 0U;
    if (st != USBD_BUSY) dbg_cdc_fail_cnt++;
    return (uint8_t)st;
  }
  dbg_cdc_tx_pkts++;
  return USBD_OK;
}

uint8_t Composite_AudioIN_IsStreaming (void) { return s_as_in_alt; }
uint8_t Composite_AudioOUT_IsStreaming(void) { return s_as_out_alt; }

uint8_t Composite_CDC_IsBusy(void)
{
    /* Stall-recovery: Composite_CDC_Transmit has its own stall check, but the
     * IsBusy guard in CAT_FlushTX prevents Transmit from being called while
     * s_cdc_tx_busy=1.  If the endpoint stalled (DataIn will never fire),
     * that creates a deadlock: IsBusy always returns 1, Transmit never runs,
     * stall recovery never executes.  Mirror the check here so the guard itself
     * can self-recover without relying on a Transmit call going through. */
    if (s_cdc_tx_busy) {
        extern USBD_HandleTypeDef hUsbDeviceFS;
        if (USBD_LL_IsStallEP(&hUsbDeviceFS, COMP_EP_CDC_IN)) {
            s_cdc_tx_busy  = 0U;
            s_cdc_busy_run = 0U;
            dbg_cdc_stuck_recovery++;
        }
    }
    return s_cdc_tx_busy;
}
