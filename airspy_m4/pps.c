#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/cm3/nvic.h>

#include "airspy_conf.h"
#include "airspy_stream.h"
#include "pps.h"

typedef struct
{
  uint32_t timer;             /* TIMERn base */
  uint32_t irq;
  volatile uint32_t* ccu_cfg;
  uint32_t pin;
  uint32_t pin_func;
  uint32_t ccr;
  uint32_t ir_cap;            /* IR bit of the capture channel */
  uint32_t cr_offset;         /* capture register of the channel */
  volatile uint32_t* gima;    /* CAPn_m_IN multiplexer */
  uint32_t gima_select;
} pps_input_t;

static const pps_input_t pps_r2 = {
  TIMER3, NVIC_TIMER3_IRQ, &CCU1_CLK_M4_TIMER3_CFG,
  P2_2, SCU_CONF_FUNCTION6,
  TIMER_CCR_CAP2RE | TIMER_CCR_CAP2I, TIMER_IR_CR2INT, 0x034,
  &GIMA_CAP3_2_IN, 3
};

static const pps_input_t pps_mini = {
  TIMER0, NVIC_TIMER0_IRQ, &CCU1_CLK_M4_TIMER0_CFG,
  P1_17, SCU_CONF_FUNCTION4,
  TIMER_CCR_CAP3RE | TIMER_CCR_CAP3I, TIMER_IR_CR3INT, 0x038,
  &GIMA_CAP0_3_IN, 1
};

static const pps_input_t* in;

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
    tc = TIMER_TC(in->timer);
    pending = TIMER_IR(in->timer) & TIMER_IR_MR0INT;
    h2 = wraps;
  } while(h1 != h2);
  if(pending && tc < 0x80000000)
    h1++;
  return ((uint64_t)h1 << 32) | tc;
}

void pps_init(void)
{
  in = AIRSPY_HW_MINI_PINS(airspy_conf->conf_hw.hardware_type) ? &pps_mini : &pps_r2;

  in->ccu_cfg[0] |= 1;
  while((in->ccu_cfg[1] & 1) == 0);

  TIMER_TCR(in->timer) = TIMER_TCR_CRST;
  TIMER_TCR(in->timer) = 0;
  TIMER_PR(in->timer) = 0;
  TIMER_MR0(in->timer) = 0xFFFFFFFF;
  TIMER_MCR(in->timer) = TIMER_MCR_MR0I;
  TIMER_CCR(in->timer) = in->ccr;
  TIMER_IR(in->timer) = 0xFF;

  *in->gima = in->gima_select << 4;
  scu_pinmux(in->pin, in->pin_func | SCU_CONF_EPD_EN_PULLDOWN | SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EZI_EN_IN_BUFFER);

  wraps = 0;
  armed = 0;
  stream->pps_seq = 0;
  stream->pps_count = 0;
  nvic_set_priority(in->irq, 0);
  nvic_enable_irq(in->irq);
  TIMER_TCR(in->timer) = TIMER_TCR_CEN;
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

static void pps_isr(void)
{
  uint32_t ir = TIMER_IR(in->timer);

  if(ir & TIMER_IR_MR0INT)
    wraps++;

  if(ir & in->ir_cap)
  {
    uint32_t cap = MMIO32(in->timer + in->cr_offset);
    uint32_t h = wraps;

    if((ir & TIMER_IR_MR0INT) && cap >= 0x80000000)
      h--;
    else if(!(ir & TIMER_IR_MR0INT) && (TIMER_IR(in->timer) & TIMER_IR_MR0INT) && cap < 0x80000000)
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

  TIMER_IR(in->timer) = ir;
}

void timer0_isr(void)
{
  pps_isr();
}

void timer3_isr(void)
{
  pps_isr();
}
