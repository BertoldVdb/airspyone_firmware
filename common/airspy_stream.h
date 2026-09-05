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

#ifndef __AIRSPY_STREAM_H
#define __AIRSPY_STREAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ADC -> USB ring buffer, shared by the M4 */
#define AIRSPY_STREAM_RING_ADDR   (0x10008000)
#define AIRSPY_STREAM_RING_SIZE   (98304) /* 96KB */

/* One GPDMA linked-list item fills one slot */
#define AIRSPY_STREAM_SLOT_SIZE   (8192)
#define AIRSPY_STREAM_NUM_SLOTS   (AIRSPY_STREAM_RING_SIZE / AIRSPY_STREAM_SLOT_SIZE) /* 12 */

/* Unpacked: one chunk is two slots of 16-bit samples */
#define AIRSPY_STREAM_CHUNK_BYTES_UNPACKED  (2 * AIRSPY_STREAM_SLOT_SIZE)
/* Packed: one chunk is one slot */
#define AIRSPY_STREAM_CHUNK_BYTES_PACKED    ((AIRSPY_STREAM_SLOT_SIZE / 4) * 3)

/* Stream state, placed at cm4_data_share (AHB SRAM visible to both cores) */
typedef struct
{
  volatile uint32_t captured; /* chunks ready for USB: DMA done and, when packing, packed */
  volatile uint32_t chunk_bytes;  /* bytes per chunk as sent over USB */
  volatile uint32_t chunk_stride; /* bytes between consecutive chunk starts in the ring */
  volatile uint32_t ring_chunks;  /* number of chunks the ring holds */
  volatile uint32_t overruns;
  volatile uint32_t backlog_max; /* highest number of chunks pending at the device */
  volatile uint32_t m0_lag_max; /* highest (produced - queued) seen at a DMA chunk completion */
  volatile uint32_t delivered; /* chunks whose USB transfer completed (USB ISR) */
  volatile uint32_t lost; /* chunks never queued because the DMA overwrote them first (main loop) */
  volatile uint32_t queued; /* chunks handed to the USB controller (main loop) */
} airspy_stream_state_t;

extern uint32_t cm4_data_share; /* defined in linker script */
#define AIRSPY_STREAM_STATE ((volatile airspy_stream_state_t *)&cm4_data_share)

#ifdef __cplusplus
}
#endif

#endif /* __AIRSPY_STREAM_H */
