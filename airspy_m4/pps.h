#ifndef __PPS_H__
#define __PPS_H__

#include <stdint.h>

void pps_init(void);
/* Call with interrupts disabled immediately before the ADC trigger */
void pps_stream_start(uint32_t sample_rate_hz, uint32_t timer_clock_hz);
void pps_stream_stop(void);

#endif
