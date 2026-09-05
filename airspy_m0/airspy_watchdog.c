/*
 * Copyright 2026 Bertold Van den Bergh <vandenbergh@bertold.org>
 *
 * This file is part of AirSpy.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "airspy_watchdog.h"

#ifdef AIRSPY_HOST_WATCHDOG

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/wwdt.h>
#include <libopencm3/lpc43xx/m0/nvic.h>

#define WWDT_MOD_WDEN     (1 << 0)
#define WWDT_MOD_WDRESET  (1 << 1) /* time-out resets the chip */
#define WWDT_MOD_WDTOF    (1 << 2)
#define WWDT_MOD_WDINT    (1 << 3)

#define WATCHDOG_COUNTS_PER_MS (3000)
#define WATCHDOG_TIMEOUT_COUNTS (0xFFFFFF)
#define WATCHDOG_WARNING_COUNTS (0x3FF)

static uint32_t watchdog_flags = 0;
static volatile uint8_t host_contact = 0;
static volatile uint8_t self_feeds_left = AIRSPY_WATCHDOG_SELF_FEEDS;

void watchdog_feed(void)
{
  cm_disable_interrupts();
  WWDT_FEED = 0xAA;
  WWDT_FEED = 0x55;
  cm_enable_interrupts();
}

void watchdog_host_contact(void)
{
  host_contact = 1;
}

void ritimer_or_wwdt_isr(void)
{
  WWDT_MOD = (WWDT_MOD & ~WWDT_MOD_WDTOF) | WWDT_MOD_WDINT;
  if(host_contact == 0 && self_feeds_left > 0)
  {
    self_feeds_left--;
    watchdog_feed();
  }
}

void watchdog_init(void)
{
  CCU1_CLK_M4_WWDT_CFG |= 1;

  watchdog_flags = AIRSPY_WATCHDOG_FLAG_ARMED;
  if(WWDT_MOD & WWDT_MOD_WDTOF)
  {
    watchdog_flags |= AIRSPY_WATCHDOG_FLAG_RESET_BY_WATCHDOG;
  }

  host_contact = 0;
  self_feeds_left = AIRSPY_WATCHDOG_SELF_FEEDS;

  WWDT_TC = WATCHDOG_TIMEOUT_COUNTS;
  WWDT_MOD = WWDT_MOD_WDEN | WWDT_MOD_WDRESET; /* writing WDTOF as 0 clears it */
  watchdog_feed(); /* starts the countdown from TC */

  WWDT_WARNINT = WATCHDOG_WARNING_COUNTS;
  WWDT_MOD = (WWDT_MOD & ~WWDT_MOD_WDTOF) | WWDT_MOD_WDINT;
  nvic_set_priority(NVIC_RITIMER_OR_WWDT_IRQ, 0);
  nvic_enable_irq(NVIC_RITIMER_OR_WWDT_IRQ);
}

void watchdog_get_status(airspy_watchdog_status_t* status)
{
  status->flags = watchdog_flags | (host_contact ? AIRSPY_WATCHDOG_FLAG_HOST_OWNED : 0);
  status->timeout_ms = WATCHDOG_TIMEOUT_COUNTS / WATCHDOG_COUNTS_PER_MS;
  status->remaining_ms = WWDT_TV / WATCHDOG_COUNTS_PER_MS;
  status->self_feeds_left = self_feeds_left;
}

#endif /* AIRSPY_HOST_WATCHDOG */
