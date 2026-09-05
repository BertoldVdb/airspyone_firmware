/* Debug access from the host */
#ifndef __AIRSPY_DEBUG_H__
#define __AIRSPY_DEBUG_H__

#include <stdint.h>

typedef struct
{
  volatile uint32_t pending;  /* 1 while the M4 has not run it yet */
  volatile uint32_t address; /* function address, Thumb bit added by the callee */
  volatile uint32_t args[4];
  volatile uint32_t result;   /* R0 after the call */
} airspy_debug_mailbox_t;

extern uint32_t cm4_data_share; /* linker script */
#define AIRSPY_DEBUG_MAILBOX_OFFSET (0x300)
#define AIRSPY_DEBUG_MAILBOX ((volatile airspy_debug_mailbox_t *)((uint8_t *)&cm4_data_share + AIRSPY_DEBUG_MAILBOX_OFFSET))

#endif
