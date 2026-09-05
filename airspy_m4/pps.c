#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/cm3/nvic.h>

#include "airspy_stream.h"
#include "pps.h"

#define PPS_TIMER      TIMER3
#define PPS_TIMER_IRQ  NVIC_TIMER3_IRQ
#define PPS_PIN        P2_2
#define PPS_PIN_FUNC   SCU_CONF_FUNCTION6 /* T3_CAP2 */
#define PPS_CCR        (TIMER_CCR_CAP2RE | TIMER_CCR_CAP2I)
#define PPS_CR         TIMER_CR2(PPS_TIMER)
#define PPS_IR_CAP     TIMER_IR_CR2INT
#define PPS_GIMA_SEL   GIMA_CAP3_2_IN
#define PPS_GIMA_SELECT (3)


static volatile uint32_t wraps;
static uint64_t anchor;           /* 64-bit timer count at the ADC trigger */
static uint32_t ratio_num;
static uint32_t ratio_den;
static volatile uint8_t armed;

static volatile airspy_stream_state_t* const stream = AIRSPY_STREAM_STATE;

static uint64_t timer_now64(void)
{
  uint32_t h1, h2, tc, pending;
  do
  {
    h1 = wraps;
    tc = TIMER_TC(PPS_TIMER);
    pending = TIMER_IR(PPS_TIMER) & TIMER_IR_MR0INT;
    h2 = wraps;
  } while(h1 != h2);
  if(pending && tc < 0x80000000)
    h1++;
  return ((uint64_t)h1 << 32) | tc;
}

void pps_init(void)
{
  CCU1_CLK_M4_TIMER3_CFG |= 1;
  while((CCU1_CLK_M4_TIMER3_STAT & 1) == 0);

  TIMER_TCR(PPS_TIMER) = TIMER_TCR_CRST;
  TIMER_TCR(PPS_TIMER) = 0;
  TIMER_PR(PPS_TIMER) = 0;
  TIMER_MR0(PPS_TIMER) = 0xFFFFFFFF;
  TIMER_MCR(PPS_TIMER) = TIMER_MCR_MR0I;
  TIMER_CCR(PPS_TIMER) = PPS_CCR;
  TIMER_IR(PPS_TIMER) = 0xFF;

  PPS_GIMA_SEL = PPS_GIMA_SELECT << 4;
  scu_pinmux(PPS_PIN, PPS_PIN_FUNC | SCU_CONF_EPD_EN_PULLDOWN | SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EZI_EN_IN_BUFFER);

  wraps = 0;
  armed = 0;
  stream->pps_seq = 0;
  stream->pps_count = 0;
  nvic_set_priority(PPS_TIMER_IRQ, 0);
  nvic_enable_irq(PPS_TIMER_IRQ);
  TIMER_TCR(PPS_TIMER) = TIMER_TCR_CEN;
}

static uint32_t gcd32(uint32_t a, uint32_t b)
{
  while(b)
  {
    uint32_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

void pps_stream_start(uint32_t sample_rate_hz, uint32_t timer_clock_hz)
{
  uint32_t g = gcd32(sample_rate_hz, timer_clock_hz);
  ratio_num = sample_rate_hz / g;
  ratio_den = timer_clock_hz / g;

  stream->pps_seq = (stream->pps_seq + 1) | 1;
  stream->pps_count = 0;
  stream->pps_index_lo = 0;
  stream->pps_index_hi = 0;
  stream->pps_fraction = 0;
  stream->pps_seq = stream->pps_seq + 1;

  anchor = timer_now64();
  armed = 1;
}

void pps_stream_stop(void)
{
  armed = 0;
}

void timer3_isr(void)
{
  uint32_t ir = TIMER_IR(PPS_TIMER);

  if(ir & TIMER_IR_MR0INT)
    wraps++;

  if(ir & PPS_IR_CAP)
  {
    uint32_t cap = PPS_CR;
    uint32_t h = wraps;

    if((ir & TIMER_IR_MR0INT) && cap >= 0x80000000)
      h--;
    else if(!(ir & TIMER_IR_MR0INT) && (TIMER_IR(PPS_TIMER) & TIMER_IR_MR0INT) && cap < 0x80000000)
      h++;

    if(armed)
    {
      uint64_t ticks = (((uint64_t)h << 32) | cap) - anchor;
      uint64_t q = ticks / ratio_den;
      uint32_t r = (uint32_t)(ticks % ratio_den);
      uint32_t rn = r * ratio_num;
      uint64_t index = q * ratio_num + rn / ratio_den;
      uint32_t fraction = (uint32_t)((((uint64_t)(rn % ratio_den)) << 32) / ratio_den);

      stream->pps_seq = (stream->pps_seq + 1) | 1;
      stream->pps_index_lo = (uint32_t)index;
      stream->pps_index_hi = (uint32_t)(index >> 32);
      stream->pps_fraction = fraction;
      stream->pps_count++;
      stream->pps_seq = stream->pps_seq + 1;
    }
  }

  TIMER_IR(PPS_TIMER) = ir;
}
