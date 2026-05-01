/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7789.c
  * @brief   ST7789VW SPI/DMA driver – init, window, push, font primitives.
  *
  *  Pure hardware layer.  No UI layout, no colour palette, no app state.
  *  UI lives in sdr_ui.h / sdr_ui.c.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "st7789.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TO  20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define _CSL(h)   HAL_GPIO_WritePin((h)->cs_port,(h)->cs_pin,GPIO_PIN_RESET)
#define _CSH(h)   HAL_GPIO_WritePin((h)->cs_port,(h)->cs_pin,GPIO_PIN_SET)
#define _DCcmd(h) HAL_GPIO_WritePin((h)->dc_port,(h)->dc_pin,GPIO_PIN_RESET)
#define _DCdat(h) HAL_GPIO_WritePin((h)->dc_port,(h)->dc_pin,GPIO_PIN_SET)
#define _RSTL(h)  HAL_GPIO_WritePin((h)->rst_port,(h)->rst_pin,GPIO_PIN_RESET)
#define _RSTH(h)  HAL_GPIO_WritePin((h)->rst_port,(h)->rst_pin,GPIO_PIN_SET)
/* USER CODE END PM */

/* ── DMA Buffers ─────────────────────────────────────────────────────────── */

/* Working line: 320×2 = 640B */
static uint16_t s_line[LCD_W]
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
  while(h->dma_busy){
    if((HAL_GetTick()-t)>500U){
      h->dma_busy=false;
      if(h->cs_held){ _CSH(h); h->cs_held=false; }
      break;
    }
  }
}

static void dma_push(ST7789_Handle_t *h, const uint8_t *buf, uint32_t bytes)
{ SCB_CleanDCache_by_Addr((uint32_t*)(uintptr_t)buf,(int32_t)((bytes+31U)&~31U));
  h->dma_busy=true;
  HAL_SPI_Transmit_DMA(h->hspi,(uint8_t*)(uintptr_t)buf,(uint16_t)(bytes&0xFFFF)); }

/* ── LCD_LineFill / Char / Str / Rect ── */
void LCD_LineFill(uint16_t *ln, uint16_t x0, uint16_t w, uint16_t color)
{ uint16_t sw=SWAP16(color);
  uint16_t x1=(uint16_t)(x0+w); if(x1>LCD_W)x1=LCD_W;
  for(uint16_t x=x0;x<x1;x++) ln[x]=sw; }

void LCD_LineChar(uint16_t *ln, uint16_t x, uint16_t frow,
                  char c, const Font_t *f, uint16_t fg, uint16_t bg)
{ if(c>='a'&&c<='z') c=(char)(c-32);
  if(c==':') c=';';
  if((uint8_t)c<32U||(uint8_t)c>90U) c=' ';
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

/* USER CODE END 0 */

/* ════════════════════════════════════════════════════════════════
 *  Core LCD
 * ════════════════════════════════════════════════════════════════ */

void ST7789_DMA_TxCpltCallback(ST7789_Handle_t *lcd){
  lcd->dma_busy=false;
  if(lcd->cs_held){ _CSH(lcd); lcd->cs_held=false; }
}

void ST7789_Init(ST7789_Handle_t *lcd)
{
  lcd->dma_busy=false; lcd->cs_held=false;

  _RSTL(lcd); HAL_Delay(10); _RSTH(lcd); HAL_Delay(120);
  hw_cmd(lcd,ST7789_SWRESET); HAL_Delay(150);
  hw_cmd(lcd,ST7789_SLPOUT);  HAL_Delay(10);
  hw_cmd(lcd,ST7789_COLMOD); hw_d8(lcd,0x55);
  hw_cmd(lcd,ST7789_MADCTL); hw_d8(lcd,ST7789_MADCTL_LANDSCAPE);
  hw_cmd(lcd,ST7789_PORCTRL);
  hw_d8(lcd,0x0C);hw_d8(lcd,0x0C);hw_d8(lcd,0x00);hw_d8(lcd,0x33);hw_d8(lcd,0x33);
  hw_cmd(lcd,ST7789_GCTRL);    hw_d8(lcd,0x35);
  hw_cmd(lcd,ST7789_VCOMS);    hw_d8(lcd,0x19);
  hw_cmd(lcd,ST7789_LCMCTRL);  hw_d8(lcd,0x2C);
  hw_cmd(lcd,ST7789_VDVVRHEN); hw_d8(lcd,0x01);
  hw_cmd(lcd,ST7789_VRHS);     hw_d8(lcd,0x12);
  hw_cmd(lcd,ST7789_VDVS);     hw_d8(lcd,0x20);
  hw_cmd(lcd,ST7789_FRCTRL2);  hw_d8(lcd,0x0F);
  hw_cmd(lcd,ST7789_PWCTRL1);  hw_d8(lcd,0xA4);hw_d8(lcd,0xA1);
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
  hw_cmd(lcd,ST7789_INVON);  HAL_Delay(10);
  hw_cmd(lcd,ST7789_NORON);  HAL_Delay(10);
  hw_cmd(lcd,ST7789_DISPON); HAL_Delay(10);
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
 *  Scanline / block / window primitives
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

void ST7789_PushWindow(ST7789_Handle_t *lcd,
                       uint16_t x0, uint16_t x1,
                       uint16_t y0, uint16_t y1,
                       const uint16_t *buf)
{ uint16_t w=(uint16_t)(x1-x0+1U);
  uint16_t h=(uint16_t)(y1-y0+1U);
  uint32_t bytes=(uint32_t)w*(uint32_t)h*2U;
  dma_wait(lcd);
  hw_cmd(lcd,ST7789_CASET); hw_d16(lcd,x0); hw_d16(lcd,x1);
  hw_cmd(lcd,ST7789_RASET); hw_d16(lcd,y0); hw_d16(lcd,y1);
  hw_cmd(lcd,ST7789_RAMWR);
  _CSL(lcd); _DCdat(lcd);
  dma_push(lcd,(const uint8_t*)buf,bytes);
  dma_wait(lcd); _CSH(lcd); }

/* ── Helpers exposed to UI layer ── */
void      dma_wait_pub(ST7789_Handle_t *lcd) { dma_wait(lcd); }
void      cs_high_pub(ST7789_Handle_t *lcd)  { _CSH(lcd); }
uint16_t *ST7789_GetLineBuf(void)            { return s_line; }

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
