/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7789.c
  * @brief   ST7789VW mcHF-style UI – Scanline Renderer
  *
  *  Tất cả UI tính toán vào line buffer trước, đẩy DMA theo block/scanline.
  *  Không có HAL_SPI_Transmit per-pixel.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "st7789.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TO          20U
#define SWAP16(x)       (uint16_t)((((x)&0xFFU)<<8U)|(((x)>>8U)&0xFFU))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define _CSL(h)  HAL_GPIO_WritePin((h)->cs_port,(h)->cs_pin,GPIO_PIN_RESET)
#define _CSH(h)  HAL_GPIO_WritePin((h)->cs_port,(h)->cs_pin,GPIO_PIN_SET)
#define _DCcmd(h)HAL_GPIO_WritePin((h)->dc_port,(h)->dc_pin,GPIO_PIN_RESET)
#define _DCdat(h)HAL_GPIO_WritePin((h)->dc_port,(h)->dc_pin,GPIO_PIN_SET)
#define _RSTL(h) HAL_GPIO_WritePin((h)->rst_port,(h)->rst_pin,GPIO_PIN_RESET)
#define _RSTH(h) HAL_GPIO_WritePin((h)->rst_port,(h)->rst_pin,GPIO_PIN_SET)
/* USER CODE END PM */

/* ── DMA Buffers (SRAM1) ──────────────────────────────────────────────────── */

/* Working line: 320×2 = 640B */
static uint16_t s_line[LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Top bar frame: 38×320 = 12160B */
static uint16_t s_topbar[ZONE_TOPBAR_H * LCD_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Status panel: 164×78 = 12792B */
#define PANEL_H  (ZONE_WF_Y2 - ZONE_TOPBAR_Y2)   /* 178 */
static uint16_t s_panel[PANEL_H * PANEL_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Function bar: 38×320 = 12160B */

/* Waterfall line: DISP_W×2 = 482B */
static uint16_t s_wfline[DISP_W]
    __attribute__((aligned(32), section(".DMA_SRAM")));

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Font 6×8 ASCII 32-90, column-major, LSB=top */
static const uint8_t s_f6x8[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x00,0x00,0x00,
  0x00,0x07,0x00,0x07,0x00,0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,
  0x24,0x2A,0x7F,0x2A,0x12,0x00,0x23,0x13,0x08,0x64,0x62,0x00,
  0x36,0x49,0x55,0x22,0x50,0x00,0x00,0x05,0x03,0x00,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x41,0x22,0x1C,0x00,0x00,
  0x14,0x08,0x3E,0x08,0x14,0x00,0x08,0x08,0x3E,0x08,0x08,0x00,
  0x00,0x50,0x30,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x00,
  0x00,0x60,0x60,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x00,
  0x3E,0x51,0x49,0x45,0x3E,0x00,0x00,0x42,0x7F,0x40,0x00,0x00,
  0x42,0x61,0x51,0x49,0x46,0x00,0x21,0x41,0x45,0x4B,0x31,0x00,
  0x18,0x14,0x12,0x7F,0x10,0x00,0x27,0x45,0x45,0x45,0x39,0x00,
  0x3C,0x4A,0x49,0x49,0x30,0x00,0x01,0x71,0x09,0x05,0x03,0x00,
  0x36,0x49,0x49,0x49,0x36,0x00,0x06,0x49,0x49,0x29,0x1E,0x00,
  0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x56,0x36,0x00,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00,0x00,0x14,0x14,0x14,0x14,0x14,0x00,
  0x00,0x41,0x22,0x14,0x08,0x00,0x02,0x01,0x51,0x09,0x06,0x00,
  0x3E,0x41,0x5D,0x55,0x1E,0x00,0x7E,0x11,0x11,0x11,0x7E,0x00,
  0x7F,0x49,0x49,0x49,0x36,0x00,0x3E,0x41,0x41,0x41,0x22,0x00,
  0x7F,0x41,0x41,0x22,0x1C,0x00,0x7F,0x49,0x49,0x49,0x41,0x00,
  0x7F,0x09,0x09,0x09,0x01,0x00,0x3E,0x41,0x49,0x49,0x7A,0x00,
  0x7F,0x08,0x08,0x08,0x7F,0x00,0x00,0x41,0x7F,0x41,0x00,0x00,
  0x20,0x40,0x41,0x3F,0x01,0x00,0x7F,0x08,0x14,0x22,0x41,0x00,
  0x7F,0x40,0x40,0x40,0x40,0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,
  0x7F,0x04,0x08,0x10,0x7F,0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,
  0x7F,0x09,0x09,0x09,0x06,0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,
  0x7F,0x09,0x19,0x29,0x46,0x00,0x46,0x49,0x49,0x49,0x31,0x00,
  0x01,0x01,0x7F,0x01,0x01,0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,
  0x1F,0x20,0x40,0x20,0x1F,0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,
  0x63,0x14,0x08,0x14,0x63,0x00,0x07,0x08,0x70,0x08,0x07,0x00,
  0x61,0x51,0x49,0x45,0x43,0x00,
};

/* USER CODE END PV */

/* Exported variables --------------------------------------------------------*/
const Font_t Font6x8 = { s_f6x8, 6U, 8U };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void hw_cmd(ST7789_Handle_t *h, uint8_t c);
static void hw_d8(ST7789_Handle_t *h, uint8_t d);
static void hw_d16(ST7789_Handle_t *h, uint16_t d);
static void dma_wait(ST7789_Handle_t *h);
static void dma_push(ST7789_Handle_t *h, const uint8_t *buf, uint32_t bytes);

/* Helpers cho mcHF "large" numerics (2× scale via scanline) */
static void ln_bigchar(uint16_t *ln, uint16_t x, uint16_t frow,
                        char c, uint16_t fg, uint16_t bg);
static void ln_bigstr(uint16_t *ln, uint16_t x, uint16_t frow,
                       const char *s, uint16_t fg, uint16_t bg);

static uint16_t wf_color_mchf(float norm);
static uint16_t spec_color_mchf(float norm);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ── Low-level ── */
static void hw_cmd(ST7789_Handle_t *h, uint8_t c)
{ _CSL(h); _DCcmd(h); HAL_SPI_Transmit(h->hspi,&c,1,SPI_TO); _CSH(h); }
static void hw_d8(ST7789_Handle_t *h, uint8_t d)
{ _CSL(h); _DCdat(h); HAL_SPI_Transmit(h->hspi,&d,1,SPI_TO); _CSH(h); }
static void hw_d16(ST7789_Handle_t *h, uint16_t d)
{ uint8_t b[2]={(uint8_t)(d>>8),(uint8_t)d};
  _CSL(h); _DCdat(h); HAL_SPI_Transmit(h->hspi,b,2,SPI_TO); _CSH(h); }

static void dma_wait(ST7789_Handle_t *h)
{ uint32_t t=HAL_GetTick();
  while(h->dma_busy){ if((HAL_GetTick()-t)>500U){ h->dma_busy=false; break; } } }

static void dma_push(ST7789_Handle_t *h, const uint8_t *buf, uint32_t bytes)
{ SCB_CleanDCache_by_Addr((uint32_t*)(uintptr_t)buf,(int32_t)((bytes+31U)&~31U));
  h->dma_busy=true;
  HAL_SPI_Transmit_DMA(h->hspi,(uint8_t*)(uintptr_t)buf,(uint16_t)(bytes&0xFFFF)); }

/* ── LCD_LineFill / Char / Str ── */
void LCD_LineFill(uint16_t *ln, uint16_t x0, uint16_t w, uint16_t color)
{ uint16_t sw=SWAP16(color);
  uint16_t x1=(uint16_t)(x0+w); if(x1>LCD_W)x1=LCD_W;
  for(uint16_t x=x0;x<x1;x++) ln[x]=sw; }

void LCD_LineChar(uint16_t *ln, uint16_t x, uint16_t frow,
                  char c, const Font_t *f, uint16_t fg, uint16_t bg)
{ /* Map lowercase a-z → A-Z (font only has ASCII 32-90) */
  if(c>='a'&&c<='z') c=(char)(c-32);
  /* Map common chars not in font */
  if(c==':') c=';'; /* nearest available */
  if((uint8_t)c<32U||(uint8_t)c>90U) c=' '; /* unknown → space */
  const uint8_t *bmp=f->data+((uint8_t)c-32U)*f->width;
  for(uint8_t col=0;col<f->width;col++){
    uint8_t bit=(bmp[col]>>frow)&1U;
    uint16_t cx=x+col;
    if(cx<LCD_W) ln[cx]=SWAP16(bit?fg:bg); } }

void LCD_LineStr(uint16_t *ln, uint16_t x, uint16_t frow,
                 const char *s, const Font_t *f, uint16_t fg, uint16_t bg)
{ while(*s&&x<LCD_W){ LCD_LineChar(ln,x,frow,*s++,f,fg,bg); x+=(uint16_t)f->width; } }

void LCD_LineRect(uint16_t *ln, uint16_t x0, uint16_t w,
                  uint16_t row, uint16_t total_h,
                  uint16_t border, uint16_t fill, uint16_t border_color)
{ uint16_t x1=(uint16_t)(x0+w); if(x1>LCD_W)x1=LCD_W;
  bool top=(row==0), bot=(row==total_h-1U);
  uint16_t sb=SWAP16(border_color), sf=SWAP16(fill);
  for(uint16_t x=x0;x<x1;x++){
    bool edge=(x==x0)||(x==x1-1U)||top||bot;
    ln[x]= edge ? sb : sf; } }

/* ── Big numerics: 2× scale via pixel-doubling in scanline ──
 *    Font6x8 → rendered as 10×16 (width=ceil(6×1.6), height=16 rows)
 *    frow: 0..15 (font row trong không gian 16px)
 *    Maps frow→original font row (frow/2), and scales width.
 */
#define BIG_W   10U   /* pixel width per big digit (6 × 1.67 ≈ 10) */
#define BIG_H   16U   /* pixel height (8 × 2) */

static void ln_bigchar(uint16_t *ln, uint16_t x, uint16_t frow,
                        char c, uint16_t fg, uint16_t bg)
{
  if((uint8_t)c<32U||(uint8_t)c>90U||frow>=BIG_H) return;
  const uint8_t *bmp = s_f6x8 + ((uint8_t)c-32U)*6U;
  uint16_t orig_row = frow>>1U;   /* pixel doubling vertically */
  for(uint16_t col=0;col<6U;col++){
    uint8_t bit=(bmp[col]>>orig_row)&1U;
    uint16_t pix=SWAP16(bit?fg:bg);
    /* Horizontal: spread 6 source cols over 10 px (×1.67) */
    uint16_t px0=(uint16_t)(x + col*10U/6U);
    uint16_t px1=(uint16_t)(x + (col+1U)*10U/6U);
    if(px1>x+BIG_W) px1=x+BIG_W;
    for(uint16_t px=px0;px<px1&&px<LCD_W;px++) ln[px]=pix;
  }
  /* fill any gap after last column */
  uint16_t last=(uint16_t)(x+BIG_W); if(last>LCD_W)last=LCD_W;
  uint16_t filled=(uint16_t)(x+6U*10U/6U);
  uint16_t sbg=SWAP16(bg);
  for(uint16_t px=filled;px<last;px++) ln[px]=sbg;
}

static void ln_bigstr(uint16_t *ln, uint16_t x, uint16_t frow,
                       const char *s, uint16_t fg, uint16_t bg)
{ while(*s){
    if(*s=='.'){
      /* Dot indicator: 2×2 block centred at bottom */
      if(frow>=BIG_H-4U&&frow<BIG_H-2U){
        uint16_t dx=(uint16_t)(x+4U);
        if(dx<LCD_W) ln[dx]=SWAP16(fg);
        if(dx+1U<LCD_W) ln[dx+1]=SWAP16(fg);
      }
      x+=4U;
    } else {
      ln_bigchar(ln,x,frow,*s,fg,bg);
      x+=BIG_W;
    }
    s++;
    if(x>=LCD_W) break; }
}

/* ── Waterfall color (mcHF style) ── */
static uint16_t wf_color_mchf(float norm)
{
  uint8_t r,g,b;
  if(norm<0.20f)      { r=0;g=0;b=(uint8_t)(norm*5.0f*15.0f); }
  else if(norm<0.45f) { float t=(norm-0.20f)*4.0f;
                         r=0;g=(uint8_t)(t*40.0f);b=(uint8_t)(15.0f+t*16.0f); }
  else if(norm<0.70f) { float t=(norm-0.45f)*4.0f;
                         r=(uint8_t)(t*20.0f);g=(uint8_t)(40.0f+t*23.0f);b=(uint8_t)(31.0f-t*31.0f); }
  else                { float t=(norm-0.70f)/0.30f;
                         r=(uint8_t)(20.0f+t*11.0f);g=(uint8_t)((1.0f-t)*63.0f);b=0; }
  return SWAP16((uint16_t)((r<<11U)|(g<<5U)|b));
}

/* ── Spectrum color (mcHF: blue→green→orange) ── */
static uint16_t spec_color_mchf(float norm)
{
  uint8_t r,g,b;
  if(norm<0.40f)      { float t=norm/0.40f;
                         r=0;g=(uint8_t)(t*30.0f);b=(uint8_t)((1.0f-t*0.5f)*31.0f); }
  else if(norm<0.75f) { float t=(norm-0.40f)/0.35f;
                         r=(uint8_t)(t*24.0f);g=(uint8_t)(30.0f+t*33.0f);b=(uint8_t)((0.5f-t*0.5f)*31.0f); }
  else                { float t=(norm-0.75f)/0.25f;
                         r=31U;g=(uint8_t)((1.0f-t)*63.0f);b=0; }
  return (uint16_t)((r<<11U)|(g<<5U)|b);
}

/* USER CODE END 0 */

/* ════════════════════════════════════════════════════════════════
 *  Core LCD
 * ════════════════════════════════════════════════════════════════ */

void ST7789_DMA_TxCpltCallback(ST7789_Handle_t *lcd){ lcd->dma_busy=false; }

void ST7789_Init(ST7789_Handle_t *lcd)
{
  lcd->dma_busy=false; lcd->wf_row=0;

  _RSTL(lcd); HAL_Delay(10); _RSTH(lcd); HAL_Delay(120);
  hw_cmd(lcd,ST7789_SWRESET); HAL_Delay(150);
  hw_cmd(lcd,ST7789_SLPOUT);  HAL_Delay(10);
  hw_cmd(lcd,ST7789_COLMOD); hw_d8(lcd,0x55);
  hw_cmd(lcd,ST7789_MADCTL); hw_d8(lcd,ST7789_MADCTL_LANDSCAPE);
  hw_cmd(lcd,ST7789_PORCTRL);
  hw_d8(lcd,0x0C);hw_d8(lcd,0x0C);hw_d8(lcd,0x00);hw_d8(lcd,0x33);hw_d8(lcd,0x33);
  hw_cmd(lcd,ST7789_GCTRL);   hw_d8(lcd,0x35);
  hw_cmd(lcd,ST7789_VCOMS);   hw_d8(lcd,0x19);
  hw_cmd(lcd,ST7789_LCMCTRL); hw_d8(lcd,0x2C);
  hw_cmd(lcd,ST7789_VDVVRHEN);hw_d8(lcd,0x01);
  hw_cmd(lcd,ST7789_VRHS);    hw_d8(lcd,0x12);
  hw_cmd(lcd,ST7789_VDVS);    hw_d8(lcd,0x20);
  hw_cmd(lcd,ST7789_FRCTRL2); hw_d8(lcd,0x0F);
  hw_cmd(lcd,ST7789_PWCTRL1); hw_d8(lcd,0xA4);hw_d8(lcd,0xA1);
  hw_cmd(lcd,ST7789_PVGAMCTRL);
  hw_d8(lcd,0xD0);hw_d8(lcd,0x04);hw_d8(lcd,0x0D);hw_d8(lcd,0x11);
  hw_d8(lcd,0x13);hw_d8(lcd,0x2B);hw_d8(lcd,0x3F);hw_d8(lcd,0x54);
  hw_d8(lcd,0x4C);hw_d8(lcd,0x18);hw_d8(lcd,0x0D);hw_d8(lcd,0x0B);
  hw_d8(lcd,0x1F);hw_d8(lcd,0x23);
  hw_cmd(lcd,ST7789_NVGAMCTRL);
  hw_d8(lcd,0xD0);hw_d8(lcd,0x04);hw_d8(lcd,0x0C);hw_d8(lcd,0x11);
  hw_d8(lcd,0x13);hw_d8(lcd,0x2C);hw_d8(lcd,0x3F);hw_d8(lcd,0x44);
  hw_d8(lcd,0x51);hw_d8(lcd,0x2F);hw_d8(lcd,0x1F);hw_d8(lcd,0x1F);
  hw_d8(lcd,0x20);hw_d8(lcd,0x23);
  hw_cmd(lcd,ST7789_INVON); HAL_Delay(10);
  hw_cmd(lcd,ST7789_NORON); HAL_Delay(10);
  hw_cmd(lcd,ST7789_DISPON);HAL_Delay(10);
  ST7789_FillScreen(lcd,MCH_BG);
}

void ST7789_SetBacklight(ST7789_Handle_t *lcd, bool on)
{ if(lcd->bl_port) HAL_GPIO_WritePin(lcd->bl_port,lcd->bl_pin,on?GPIO_PIN_SET:GPIO_PIN_RESET); }

void ST7789_SetWindow(ST7789_Handle_t *lcd, uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{ hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,x0); hw_d16(lcd,x1);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y0); hw_d16(lcd,y1);
  hw_cmd(lcd,ST7789_RAMWR); }

void ST7789_FillScreen(ST7789_Handle_t *lcd, uint16_t color)
{ uint16_t sw=SWAP16(color);
  for(uint16_t i=0;i<LCD_W;i++) s_line[i]=sw;
  for(uint16_t y=0;y<LCD_H;y++){ ST7789_PushScanline(lcd,y,s_line); dma_wait(lcd); _CSH(lcd); } }

/* ════════════════════════════════════════════════════════════════
 *  Scanline primitives
 * ════════════════════════════════════════════════════════════════ */

void ST7789_PushScanline(ST7789_Handle_t *lcd, uint16_t y, const uint16_t *line)
{ dma_wait(lcd);
  hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,0); hw_d16(lcd,LCD_W-1U);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y); hw_d16(lcd,y);
  hw_cmd(lcd,ST7789_RAMWR);
  _CSL(lcd); _DCdat(lcd);
  dma_push(lcd,(const uint8_t*)line,(uint32_t)LCD_W*2U); }

void ST7789_PushBlock(ST7789_Handle_t *lcd, uint16_t y0, uint16_t y1, const uint16_t *buf)
{ dma_wait(lcd);
  hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,0); hw_d16(lcd,LCD_W-1U);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y0); hw_d16(lcd,y1);
  hw_cmd(lcd,ST7789_RAMWR);
  _CSL(lcd); _DCdat(lcd);
  uint32_t bytes=(uint32_t)(y1-y0+1U)*LCD_W*2U;
  dma_push(lcd,(const uint8_t*)buf,bytes);
  dma_wait(lcd); _CSH(lcd); }

/* ── Push partial block (right display area only: x=DISP_X..319) ──
 *    Không push toàn dòng, chỉ push cột DISP_X..319 để tiết kiệm.
 *    Dùng cho spectrum và waterfall để tránh đè panel trái.
 */
static void push_disp_scanline(ST7789_Handle_t *lcd, uint16_t y,
                                const uint16_t *full_line)
{ dma_wait(lcd);
  hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,DISP_X); hw_d16(lcd,LCD_W-1U);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y); hw_d16(lcd,y);
  hw_cmd(lcd,ST7789_RAMWR);
  _CSL(lcd); _DCdat(lcd);
  dma_push(lcd,(const uint8_t*)(full_line+DISP_X),(uint32_t)DISP_W*2U);
  dma_wait(lcd); _CSH(lcd); }

/* ── Push panel block (x=0..PANEL_W-1) ── */
static void push_panel_block(ST7789_Handle_t *lcd, uint16_t y0, uint16_t y1,
                              const uint16_t *pbuf)
{ /* pbuf layout: row × PANEL_W */
  dma_wait(lcd);
  uint16_t rows=y1-y0+1U;
  for(uint16_t r=0;r<rows;r++){
    hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,PANEL_X); hw_d16(lcd,PANEL_W-1U);
    hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y0+r); hw_d16(lcd,y0+r);
    hw_cmd(lcd,ST7789_RAMWR);
    _CSL(lcd); _DCdat(lcd);
    dma_push(lcd,(const uint8_t*)(pbuf+(uint32_t)r*PANEL_W),(uint32_t)PANEL_W*2U);
    dma_wait(lcd); _CSH(lcd);
  } }

/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawFrame  –  Static frame (1× at boot)
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawFrame(ST7789_Handle_t *lcd)
{
  ST7789_FillScreen(lcd,MCH_BG);

  /* Top bar bottom border */
  uint16_t sw=SWAP16(MCH_DIVIDER);
  for(uint16_t i=0;i<LCD_W;i++) s_line[i]=sw;
  ST7789_PushScanline(lcd,ZONE_TOPBAR_Y2-1U,s_line);
  dma_wait(lcd); _CSH(lcd);

  /* Vertical divider between panel and display area */
  sw=SWAP16(MCH_BORDER);
  for(uint16_t y=ZONE_TOPBAR_Y2;y<ZONE_WF_Y2;y++){
    for(uint16_t x=0;x<LCD_W;x++) s_line[x]=SWAP16(MCH_BG);
    s_line[PANEL_DIV_X]=sw;
    ST7789_PushScanline(lcd,y,s_line);
    dma_wait(lcd); _CSH(lcd);
  }

  /* Spectrum/Waterfall separator */
  sw=SWAP16(MCH_DIVIDER);
  for(uint16_t i=0;i<LCD_W;i++) s_line[i]=sw;
  ST7789_PushScanline(lcd,ZONE_SPEC_Y2-1U,s_line);
  dma_wait(lcd); _CSH(lcd);

  /* Function bar top border */
  }
/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawTopBar  –  62 dòng, 2 rows
 *
 *  Row 1 (y=0..27, 28px):
 *   x=0..31  : [MODE] tag
 *   x=33..74 : [BAND] tag
 *   x=76..319: Frequency big font (cyan MHz / green kHz / dark Hz)
 *
 *  Row 2 (y=28..61, 34px):
 *   x=0..7   : "S " label
 *   x=10..249: 16 bars × (14px + 1px gap) = 240px  ← lớn, rõ
 *   x=2..30:    S-value text below bars  e.g. "S7" / "S9+30"
 *   x=264..319: [RX]/[TX] 56px × 34px  ← to và nổi bật
 *
 *  Compute 62 dòng → PushBlock (62×320×2 = 39680B, 1 DMA call)
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawTopBar(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui)
{
  static const char *const mode_s[]={"AM ","FM ","USB","LSB","CW "};
  static const char *const band_s[]={"160","80m","60m","40m","30m",
                                      "20m","17m","15m","12m","10m","6m"};
  const char *mode_str=(ui->mode<5U)?mode_s[ui->mode]:"---";
  const char *band_str=(ui->band_idx<11U)?band_s[ui->band_idx]:"??m";

  /* Frequency parts */
  uint32_t mhz=ui->freq_hz/1000000UL;
  uint32_t khz=(ui->freq_hz%1000000UL)/1000UL;
  uint32_t hz_r=ui->freq_hz%1000UL;
  char mhz_s[6],khz_s[8],hz_s[8];
  snprintf(mhz_s,sizeof(mhz_s),"%lu",mhz);
  snprintf(khz_s,sizeof(khz_s),"%03lu",khz);
  snprintf(hz_s, sizeof(hz_s), "%03lu",hz_r);

  /* S-meter bars */
  int32_t bars=(int32_t)((ui->signal_db+73.0f)/3.0f);
  if(bars<0){bars=0;} if(bars>SM_BARS){bars=SM_BARS;}
  char s_str[8];
  if(bars<=9) snprintf(s_str,sizeof(s_str),"S%ld",(long)bars);
  else        snprintf(s_str,sizeof(s_str),"S9+%ld",(long)((bars-9)*3));

  /* TX indicator */
  bool tx=ui->tx_mode;
  uint16_t ind_bg=tx?MCH_TX_BG:MCH_RX_BG;
  uint16_t ind_fg=tx?MCH_TX_FG:MCH_RX_FG;
  const char *ind_str=tx?"TX":"RX";

  /* ─── Build 62 dòng ─── */
  for(uint16_t row=0;row<ZONE_TOPBAR_H;row++)
  {
    uint16_t *ln=s_topbar+(uint32_t)row*LCD_W;

    /* Bottom separator */
    if(row==ZONE_TOPBAR_H-1U){ LCD_LineFill(ln,0,LCD_W,MCH_DIVIDER); continue; }

    /* ══ ROW 1: y=0..27 ══ */
    if(row<ZONE_TOPBAR_R1H)
    {
      LCD_LineFill(ln,0,LCD_W,MCH_TOPBAR_BG);

      bool top=(row==0), bot=(row==ZONE_TOPBAR_R1H-1U);
      /* Separator between row1 and row2 */
      if(bot){ LCD_LineFill(ln,0,LCD_W,MCH_BORDER); continue; }

      /* [MODE] x=0..31 */
      LCD_LineFill(ln,0,32U,top?MCH_BORDER:MCH_MODE_BG);
      if(!top){ ln[0]=SWAP16(MCH_BORDER); ln[31]=SWAP16(MCH_BORDER);
        if(row>=6U&&(row-6U)<BIG_H)
          ln_bigstr(ln,2U,(uint16_t)(row-6U),mode_str,MCH_MODE_FG,MCH_MODE_BG);
      }

      /* [BAND] x=33..73 */
      LCD_LineFill(ln,33U,41U,top?MCH_BORDER:MCH_BAND_BG);
      if(!top){ ln[33]=SWAP16(MCH_BORDER); ln[73]=SWAP16(MCH_BORDER);
        if(row>=6U&&(row-6U)<BIG_H)
          ln_bigstr(ln,35U,(uint16_t)(row-6U),band_str,MCH_BAND_FG,MCH_BAND_BG);
      }

      /* Frequency big font x=76..319
         BIG_H=16, center in 28px: top offset = (28-16)/2 = 6 → rows 6..21 */
      if(row>=6U&&(row-6U)<BIG_H){
        uint16_t fr=(uint16_t)(row-6U);
        uint16_t fx=76U;
        /* MHz: cyan */
        ln_bigstr(ln,fx,fr,mhz_s,MCH_FREQ_MHZ,MCH_TOPBAR_BG);
        fx=(uint16_t)(fx+(uint16_t)strlen(mhz_s)*BIG_W);
        /* dot */
        ln_bigstr(ln,fx,fr,".",MCH_FREQ_DOT,MCH_TOPBAR_BG); fx+=4U;
        /* kHz: green */
        ln_bigstr(ln,fx,fr,khz_s,MCH_FREQ_KHZ,MCH_TOPBAR_BG);
        fx=(uint16_t)(fx+3U*BIG_W);
        /* dot */
        ln_bigstr(ln,fx,fr,".",MCH_FREQ_DOT,MCH_TOPBAR_BG); fx+=4U;
        /* Hz: dark green */
        ln_bigstr(ln,fx,fr,hz_s,MCH_FREQ_HZ,MCH_TOPBAR_BG);
      }
    }

    /* ══ ROW 2: y=28..61, 34px ══ */
    else
    {
      uint16_t r2=(uint16_t)(row-ZONE_TOPBAR_R1H);   /* 0..33 */

      LCD_LineFill(ln,0,LCD_W,MCH_TOPBAR_BG);

      /* ══ S-METER row2 layout (34px total) ══════════════
       *  r2= 0.. 7 (8px) : scale labels S1 S3 S5 S7 S9 +30
       *  r2= 8     (1px) : gap
       *  r2= 9..30 (22px): bars  [SM_BAR_YOFF=9, SM_BAR_H=22]
       *  r2=31..33  (3px): margin
       * ════════════════════════════════════════════════ */

      /* Scale labels r2=0..7 (Font8px, nhưng chỉ cần 4px → r2=0..3) */
      if(r2<Font6x8.height){
        static const char *const slbls[]={"S1","S3","S5","S7","S9","+30"};
        static const uint8_t spos[]={0,2,4,6,8,10};
        for(uint8_t t=0;t<6U;t++){
          uint16_t tx=(uint16_t)(SM_START_X+(uint16_t)spos[t]*(SM_BAR_W+SM_BAR_GAP));
          LCD_LineStr(ln,tx,r2,slbls[t],&Font6x8,MCH_STATUS_LBL,MCH_TOPBAR_BG);
        }
      }

      /* Tick marks r2=8 (gap row between labels and bars) */
      if(r2==8U){
        static const uint8_t tick_pos[]={0,2,4,6,8,10};
        for(uint8_t t=0;t<6U;t++){
          uint16_t tx=(uint16_t)(SM_START_X+(uint16_t)tick_pos[t]*(SM_BAR_W+SM_BAR_GAP)+SM_BAR_W/2U);
          if(tx<RXTX_X) ln[tx]=SWAP16(MCH_SMETER_TICK);
        }
      }

      /* Bars r2=6..27 */
      if(r2>=SM_BAR_YOFF&&r2<(uint16_t)(SM_BAR_YOFF+SM_BAR_H))
      {
        bool bar_top=(r2==SM_BAR_YOFF);
        bool bar_bot=(r2==(uint16_t)(SM_BAR_YOFF+SM_BAR_H-1U));
        for(uint8_t b=0;b<SM_BARS;b++)
        {
          uint16_t bx=(uint16_t)(SM_START_X+(uint16_t)b*(SM_BAR_W+SM_BAR_GAP));
          uint16_t bc_on,bc_off=MCH_SMETER_BG;
          if(b<6U)       bc_on=MCH_S1_6;
          else if(b<9U)  bc_on=MCH_S7_9;
          else           bc_on=MCH_S9P;
          uint16_t fill=(b<(uint8_t)bars)?bc_on:bc_off;
          for(uint16_t px=bx;px<bx+SM_BAR_W&&px<RXTX_X;px++){
            bool edge=(px==bx||px==bx+SM_BAR_W-1U||bar_top||bar_bot);
            ln[px]=SWAP16(edge?MCH_BORDER:fill);
          }
          if(bx+SM_BAR_W<RXTX_X) ln[bx+SM_BAR_W]=SWAP16(MCH_TOPBAR_BG);
        }
      }

      /* S-value overlay: ĐÈ LÊN bars tại vùng giữa (r2=11..18)
       * Hiển thị "S7" hoặc "S9+30" sau bars → luôn visible */
      if(r2>=11U&&r2<11U+Font6x8.height){
        uint16_t frow=(uint16_t)(r2-11U);
        uint16_t vcol=(bars>9)?MCH_S9P:(bars>5)?MCH_S7_9:MCH_S1_6;
        /* Background pixel behind text để dễ đọc */
        LCD_LineStr(ln,SM_SVAL_X,frow,s_str,&Font6x8,vcol,MCH_TOPBAR_BG);
      }

      /* ── RX/TX indicator x=264..319 (56px × 34px) ── */
      LCD_LineFill(ln,RXTX_X,RXTX_W,ind_bg);
      /* Border */
      ln[RXTX_X]=SWAP16(MCH_BORDER);
      ln[LCD_W-1U]=SWAP16(MCH_BORDER);
      if(r2==0||r2==ZONE_TOPBAR_R2H-2U)
        LCD_LineFill(ln,RXTX_X,RXTX_W,MCH_BORDER);

      /* "RX"/"TX" text centred: font 6×8, "RX"=12px wide
         Centre in 56px: x_off=(56-12)/2=22, x_abs=264+22=286
         Centre in 34px: y_off=(34-8)/2=13 → rows 13..20 */
      if(r2>=13U&&r2<13U+Font6x8.height)
        LCD_LineStr(ln,(uint16_t)(RXTX_X+22U),(uint16_t)(r2-13U),
                    ind_str,&Font6x8,ind_fg,ind_bg);

      /* SI dot: small 3×3 at x=266, r2=10..12 */
      if(r2>=10U&&r2<=12U){
        uint16_t si_c=ui->si5351_ok?SWAP16(MCH_STATUS_ON):SWAP16(MCH_STATUS_OFF);
        ln[RXTX_X+4U]=si_c; ln[RXTX_X+5U]=si_c; ln[RXTX_X+6U]=si_c;
      }
    }
  }

  ST7789_PushBlock(lcd,ZONE_TOPBAR_Y,(uint16_t)(ZONE_TOPBAR_Y2-1U),s_topbar);
}

/* ════════════════════════════════════════════════════════════════
 *  McHF_UpdateSMeter  –  Partial update chỉ Row 2 bars (r2=6..27)
 *  Push 22 dòng × 241px partial window → ~10KB, không đè Row 1
 * ════════════════════════════════════════════════════════════════ */

/* TX/RX state for McHF_UpdateSMeter partial updates */
static uint16_t s_smeter_ind_bg  = MCH_RX_BG;
static uint16_t s_smeter_ind_fg  = MCH_RX_FG;
static const char *s_smeter_ind_str = "RX";

void McHF_UpdateSMeter_SetTX(bool tx)
{
  s_smeter_ind_bg  = tx ? MCH_TX_BG : MCH_RX_BG;
  s_smeter_ind_fg  = tx ? MCH_TX_FG : MCH_RX_FG;
  s_smeter_ind_str = tx ? "TX" : "RX";
}

void McHF_UpdateSMeter(ST7789_Handle_t *lcd, float signal_db)
{
  int32_t bars=(int32_t)((signal_db+73.0f)/3.0f);
  if(bars<0){bars=0;} if(bars>SM_BARS){bars=SM_BARS;}

  char s_str[8];
  if(bars<=9) snprintf(s_str,sizeof(s_str),"S%ld",(long)bars);
  else        snprintf(s_str,sizeof(s_str),"S9+%ld",(long)((bars-9)*3));

  uint16_t abs_y0=(uint16_t)(ZONE_TOPBAR_R1H+SM_BAR_YOFF);  /* absolute y of bar top */

  for(uint16_t r2=(uint16_t)SM_BAR_YOFF;r2<(uint16_t)(SM_BAR_YOFF+SM_BAR_H);r2++)
  {
    bool bar_top=(r2==SM_BAR_YOFF);
    bool bar_bot=(r2==(uint16_t)(SM_BAR_YOFF+SM_BAR_H-1U));

    /* Build full-width line (easier than partial, same DMA width) */
    LCD_LineFill(s_line,0,LCD_W,MCH_TOPBAR_BG);

    for(uint8_t b=0;b<SM_BARS;b++){
      uint16_t bx=(uint16_t)(SM_START_X+(uint16_t)b*(SM_BAR_W+SM_BAR_GAP));
      uint16_t bc_on;
      if(b<6U)bc_on=MCH_S1_6; else if(b<9U)bc_on=MCH_S7_9; else bc_on=MCH_S9P;
      uint16_t fill=(b<(uint8_t)bars)?bc_on:MCH_SMETER_BG;
      for(uint16_t px=bx;px<bx+SM_BAR_W&&px<RXTX_X;px++){
        bool edge=(px==bx||px==bx+SM_BAR_W-1U||bar_top||bar_bot);
        s_line[px]=SWAP16(edge?MCH_BORDER:fill);
      }
    }
    /* S-value */
    if(!bar_top&&!bar_bot){
      uint16_t fr=(uint16_t)(r2-SM_BAR_YOFF-1U);
      if(fr<Font6x8.height)
        LCD_LineStr(s_line,SM_SVAL_X,fr,s_str,&Font6x8,
                    (bars>9)?MCH_S9P:(bars>5)?MCH_S7_9:MCH_S1_6,MCH_TOPBAR_BG);
    }
    /* Keep RX/TX indicator */
    LCD_LineFill(s_line,RXTX_X,RXTX_W,s_smeter_ind_bg);
    if(!bar_top&&!bar_bot){
      uint16_t fr2=(uint16_t)(r2-SM_BAR_YOFF);
      if(fr2>=7U&&fr2<7U+Font6x8.height)
        LCD_LineStr(s_line,(uint16_t)(RXTX_X+22U),(uint16_t)(fr2-7U),
                    s_smeter_ind_str,&Font6x8,s_smeter_ind_fg,s_smeter_ind_bg);
    }
    s_line[RXTX_X]=SWAP16(MCH_BORDER);
    s_line[LCD_W-1U]=SWAP16(MCH_BORDER);

    uint16_t abs_y=(uint16_t)(abs_y0+(r2-SM_BAR_YOFF));
    dma_wait(lcd);
    hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,0); hw_d16(lcd,LCD_W-1U);
    hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,abs_y); hw_d16(lcd,abs_y);
    hw_cmd(lcd,ST7789_RAMWR);
    _CSL(lcd); _DCdat(lcd);
    dma_push(lcd,(const uint8_t*)s_line,(uint32_t)LCD_W*2U);
    dma_wait(lcd); _CSH(lcd);
  }
}


/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawStatusPanel  –  Left panel 78px × 138 rows
 *  Rows relative to ZONE_TOPBAR_Y2 (=62):
 *   0-17:  AGC  [FAST/SLOW]
 *  17-34:  NB   [ON /OFF ]
 *  34-51:  NR   [ON /OFF ]
 *  51-68:  RIT  [+000Hz  ]
 *  68-85:  VOL  [70%     ]
 *  85-102: SQ   [000     ]
 * 102-119: STEP [100Hz   ]
 * 119-137: CLK  [OK/FAIL ]
 *
 *  Build vào s_panel → push_panel_block
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawStatusPanel(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui)
{
  char buf[10];
  const uint16_t H=(uint16_t)PANEL_H;

  for(uint32_t i=0;i<(uint32_t)H*PANEL_W;i++) s_panel[i]=SWAP16(MCH_PANEL_BG);
  for(uint16_t r=0;r<H;r++) s_panel[(uint32_t)r*PANEL_W+(PANEL_W-1U)]=SWAP16(MCH_BORDER);
  uint16_t sep_step=(uint16_t)(H/8U);
  for(uint8_t s=1;s<8U;s++){
    uint16_t ry=(uint16_t)(s*sep_step);
    if(ry<H) for(uint16_t x=0;x<PANEL_W-1U;x++) s_panel[(uint32_t)ry*PANEL_W+x]=SWAP16(MCH_BORDER);
  }

#define PI(r0,lbl,val_str,vc) do{ \
    for(uint16_t _fr=0;_fr<Font6x8.height;_fr++){ \
      uint16_t _row=(uint16_t)((r0)+(_fr)+5U); if(_row>=H)break; \
      uint16_t *_ln=s_panel+(uint32_t)_row*PANEL_W; \
      LCD_LineStr(_ln,2U,_fr,(lbl),&Font6x8,MCH_STATUS_LBL,MCH_PANEL_BG); \
      LCD_LineStr(_ln,38U,_fr,(val_str),&Font6x8,(vc),MCH_PANEL_BG); \
    } }while(0)

  PI(0U,  "AGC",ui->agc_fast?"FAST":"SLOW",
     ui->agc_fast?MCH_STATUS_VAL:MCH_STATUS_LBL);
  PI(sep_step*1U,"NB ",ui->nb_on?"ON ":"OFF",
     ui->nb_on?MCH_STATUS_ON:MCH_STATUS_OFF);
  PI(sep_step*2U,"NR ",ui->nr_on?"ON ":"OFF",
     ui->nr_on?MCH_STATUS_ON:MCH_STATUS_OFF);
  snprintf(buf,sizeof(buf),"%+05d",(int)ui->rit_hz);
  PI(sep_step*3U,"RIT",buf,ui->rit_hz!=0?MCH_FREQ_KHZ:MCH_STATUS_LBL);
  snprintf(buf,sizeof(buf),"%3d%%",(int)ui->volume);
  PI(sep_step*4U,"VOL",buf,MCH_STATUS_VAL);
  snprintf(buf,sizeof(buf),"%3d",(int)ui->squelch);
  PI(sep_step*5U,"SQ ",buf,MCH_STATUS_VAL);
  uint32_t st=(uint32_t)ui->step;
  if(st>=100000U)      snprintf(buf,sizeof(buf),"100KHz");
  else if(st>=10000U)  snprintf(buf,sizeof(buf)," 10KHz");
  else if(st>=1000U)   snprintf(buf,sizeof(buf),"  1KHz");
  else if(st>=100U)    snprintf(buf,sizeof(buf),"100Hz ");
  else if(st>=10U)     snprintf(buf,sizeof(buf)," 10Hz ");
  else                 snprintf(buf,sizeof(buf),"  1Hz ");
  PI(sep_step*6U,"STP",buf,MCH_FREQ_KHZ);
  if (ui->bw_hz >= 1000U)
    snprintf(buf, sizeof(buf), "%2ukHz", (unsigned)(ui->bw_hz / 1000U));
  else
    snprintf(buf, sizeof(buf), "%3uHz", (unsigned)ui->bw_hz);
  PI(sep_step*7U,"BW ",buf,MCH_FREQ_KHZ);
#undef PI

  push_panel_block(lcd,ZONE_TOPBAR_Y2,(uint16_t)(ZONE_WF_Y2-1U),s_panel);
}

/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawSpectrum  –  Scanline renderer, DISP_W × ZONE_SPEC_H
 *  Precompute bar_h[bins] → 88 DMA calls (1 per scanline)
 *  Peak hold 2.5s decay, center marker magenta
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawSpectrum(ST7789_Handle_t *lcd,
                        const float *fft_db, uint16_t bins,
                        float bw_lo_ratio, float bw_hi_ratio,
                        McHF_UI_State_t *ui)
{
  if(!bins) return;
  const uint16_t W = DISP_W;
  const uint16_t H = ZONE_SPEC_H;
  const float db_range = 80.0f, db_min = -80.0f;

  /* Map bins → pixel columns (stretch bins across DISP_W pixels) */
  const float bin_per_px = (float)bins / (float)W;

  /* Precompute bar height cho mỗi PIXEL COLUMN */
  uint16_t bar_h[256U];
  if (W > 256U) { return; } /* safety */
  for (uint16_t x = 0U; x < W; x++)
  {
    uint16_t bi = (uint16_t)((float)x * bin_per_px);
    if (bi >= bins) bi = bins - 1U;
    float db = fft_db[bi];
    if (db < db_min) db = db_min;
    if (db > 0.0f)   db = 0.0f;
    float norm = (db - db_min) / db_range;
    bar_h[x] = (uint16_t)(norm * (float)H);
  }

  /* Grid horizontal lines (25%, 50%, 75% dB) */
  uint16_t g20 = (uint16_t)(H - (uint16_t)(0.75f * (float)H));
  uint16_t g40 = (uint16_t)(H - (uint16_t)(0.50f * (float)H));
  uint16_t g60 = (uint16_t)(H - (uint16_t)(0.25f * (float)H));

  /* Center marker (DC = middle of spectrum after fftshift) */
  uint16_t cx_px = W / 2U;

  /* Bandwidth markers: tính riêng cho mỗi phía
   *   bw_lo_ratio = offset / Fs về phía TRÁI center (âm frequency)
   *   bw_hi_ratio = offset / Fs về phía PHẢI center (dương frequency)
   *   Pixel offset = ratio * W (vì full spectrum span Fs = W pixel)
   *   → nửa spectrum (Fs/2) = W/2 pixel
   *   → offset pixel = (offset_Hz / Fs_Hz) * W */
  bool bw_lo_valid = (bw_lo_ratio > 0.0001f);
  bool bw_hi_valid = (bw_hi_ratio > 0.0001f);

  uint16_t bw_lo_px = 0U, bw_hi_px = 0U;
  if (bw_lo_valid)
  {
    uint16_t off = (uint16_t)(bw_lo_ratio * (float)W + 0.5f);
    if (off == 0U) off = 1U;
    bw_lo_px = (cx_px > off) ? (uint16_t)(cx_px - off) : 0U;
  }
  if (bw_hi_valid)
  {
    uint16_t off = (uint16_t)(bw_hi_ratio * (float)W + 0.5f);
    if (off == 0U) off = 1U;
    bw_hi_px = cx_px + off;
    if (bw_hi_px >= W) bw_hi_px = W - 1U;
  }

  for (uint16_t y = 0U; y < H; y++)
  {
    bool is_grid = (y == g20 || y == g40 || y == g60);
    uint16_t bg_sw = is_grid ? SWAP16(MCH_SPEC_GRID) : SWAP16(MCH_SPEC_BG);
    for (uint16_t x = 0U; x < LCD_W; x++) s_line[x] = bg_sw;

    if (!is_grid)
    {
      for (uint16_t gx = DISP_X; gx < LCD_W; gx += 40U)
        s_line[gx] = SWAP16(MCH_SPEC_GRID);
    }

    /* Bandwidth markers (cyan) - vẽ TRƯỚC spectrum bars */
    if (bw_lo_valid)
    {
      uint16_t px = (uint16_t)(DISP_X + bw_lo_px);
      if (px < LCD_W) s_line[px] = SWAP16(MCH_SPEC_BW);
    }
    if (bw_hi_valid)
    {
      uint16_t px = (uint16_t)(DISP_X + bw_hi_px);
      if (px < LCD_W) s_line[px] = SWAP16(MCH_SPEC_BW);
    }

    /* Center marker (magenta) */
    uint16_t cxabs = (uint16_t)(DISP_X + cx_px);
    if (cxabs < LCD_W) s_line[cxabs] = SWAP16(MCH_SPEC_CENTER);

    /* Spectrum bars */
    uint16_t y_from_bot = (uint16_t)(H - 1U - y);
    for (uint16_t x = 0U; x < W; x++)
    {
      if (bar_h[x] > 0U && y_from_bot < bar_h[x])
      {
        float norm_v = (float)y_from_bot / (float)H;
        uint16_t sc  = SWAP16(spec_color_mchf(norm_v));
        uint16_t px  = (uint16_t)(DISP_X + x);
        if (px < LCD_W) s_line[px] = sc;
      }
    }

    push_disp_scanline(lcd, (uint16_t)(ZONE_SPEC_Y + y), s_line);
  }
  (void)ui;
}

/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawWaterfall  –  1 dòng waterfall, DISP_W px, 1 DMA call
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawWaterfall(ST7789_Handle_t *lcd,
                         const float *fft_db, uint16_t bins)
{
  if(!bins) return;
  const float xs=(float)DISP_W/(float)bins;
  const float db_min=-80.0f, db_range=80.0f;
  for(uint16_t x=0;x<DISP_W;x++){
    uint16_t bi=(uint16_t)((float)x/xs); if(bi>=bins)bi=bins-1U;
    float db=fft_db[bi]; if(db<db_min)db=db_min;
    float norm=(db-db_min)/db_range;
    s_wfline[x]=wf_color_mchf(norm);
  }
  uint16_t draw_y=(uint16_t)(ZONE_WF_Y+lcd->wf_row);
  dma_wait(lcd);
  hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,DISP_X); hw_d16(lcd,LCD_W-1U);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,draw_y); hw_d16(lcd,draw_y);
  hw_cmd(lcd,ST7789_RAMWR);
  _CSL(lcd); _DCdat(lcd);
  dma_push(lcd,(const uint8_t*)s_wfline,(uint32_t)DISP_W*2U);
  dma_wait(lcd); _CSH(lcd);
  lcd->wf_row=(uint16_t)((lcd->wf_row+1U)%ZONE_WF_H);
}

/* ════════════════════════════════════════════════════════════════
 *  McHF_DrawFuncBar  –  40 dòng × 7 nút, PushBlock
 * ════════════════════════════════════════════════════════════════ */
void McHF_DrawFuncBar(ST7789_Handle_t *lcd, const McHF_UI_State_t *ui)
{ (void)lcd; (void)ui; /* removed */ }


/* Public wrappers so menu.c can call internal helpers */
void dma_wait_pub(ST7789_Handle_t *lcd) { dma_wait(lcd); }
void cs_high_pub(ST7789_Handle_t *lcd)  { _CSH(lcd); }

/* Also expose s_line for menu.c scanline writing */
uint16_t *ST7789_GetLineBuf(void) { return s_line; }

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
