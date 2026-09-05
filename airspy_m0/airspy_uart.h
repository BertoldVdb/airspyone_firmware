/* Serial pass-through for a GNSS module (or anything serial) */
#ifndef __AIRSPY_UART_H__
#define __AIRSPY_UART_H__

#include <stdint.h>

#define AIRSPY_UART_DEFAULT_BAUD (9600)

int airspy_uart_available(void);
void airspy_uart_init(uint32_t baud);
void airspy_uart_flush_rx(void);
uint32_t airspy_uart_read(uint8_t* dst, uint32_t max);  /* main loop only */
uint32_t airspy_uart_write(const uint8_t* src, uint32_t len); /* returns bytes accepted */
uint32_t airspy_uart_baud(void);

#endif
