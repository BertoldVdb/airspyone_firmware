/*
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
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

#ifndef __AIRSPY_COMMANDS_H__
#define __AIRSPY_COMMANDS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
  RECEIVER_MODE_OFF = 0,
  RECEIVER_MODE_RX = 1
} receiver_mode_t;

// Commands (usb vendor request) shared between Firmware and Host.
#define AIRSPY_CMD_MAX (27)
#define AIRSPY_EXT_CMD_BASE (0x80)
#define AIRSPY_EXT_CMD_MAX (0x8A)
typedef enum
{
  AIRSPY_INVALID                    = 0 ,
  AIRSPY_RECEIVER_MODE              = 1 ,
  AIRSPY_SI5351C_WRITE              = 2 ,
  AIRSPY_SI5351C_READ               = 3 ,
  AIRSPY_R820T_WRITE                = 4 ,
  AIRSPY_R820T_READ                 = 5 ,
  AIRSPY_SPIFLASH_ERASE             = 6 ,
  AIRSPY_SPIFLASH_WRITE             = 7 ,
  AIRSPY_SPIFLASH_READ              = 8 ,
  AIRSPY_BOARD_ID_READ              = 9 ,
  AIRSPY_VERSION_STRING_READ        = 10,
  AIRSPY_BOARD_PARTID_SERIALNO_READ = 11,
  AIRSPY_SET_SAMPLERATE             = 12,
  AIRSPY_SET_FREQ                   = 13,
  AIRSPY_SET_LNA_GAIN               = 14,
  AIRSPY_SET_MIXER_GAIN             = 15,
  AIRSPY_SET_VGA_GAIN               = 16,
  AIRSPY_SET_LNA_AGC                = 17,
  AIRSPY_SET_MIXER_AGC              = 18,
  AIRSPY_MS_VENDOR_CMD              = 19,
  AIRSPY_SET_RF_BIAS_CMD            = 20,
  AIRSPY_GPIO_WRITE                 = 21,
  AIRSPY_GPIO_READ                  = 22,
  AIRSPY_GPIODIR_WRITE              = 23,
  AIRSPY_GPIODIR_READ               = 24,
  AIRSPY_GET_SAMPLERATES            = 25,
  AIRSPY_SET_PACKING                = 26,
  AIRSPY_SPIFLASH_ERASE_SECTOR      = AIRSPY_CMD_MAX,
  /* 28..33 are taken by HydraSDR (SET_RF_PORT, GET_CAPABILITIES, SET_BANDWIDTH, GET_BANDWIDTHS,
     GET_TEMPERATURE, SET_GAIN): not implemented here, they stall */
  /* Stream metadata, timing and debug commands, the same numbers as on the HydraSDR RFOne */
  AIRSPY_GET_STREAM_STATUS          = AIRSPY_EXT_CMD_BASE + 0,
  AIRSPY_SET_FRAMING                = AIRSPY_EXT_CMD_BASE + 1,
  AIRSPY_WATCHDOG                   = AIRSPY_EXT_CMD_BASE + 2,
  AIRSPY_SET_UART_BAUD              = AIRSPY_EXT_CMD_BASE + 3, /* wValue = baud & 0xFFFF, wIndex = baud >> 16 */
  AIRSPY_UART_WRITE                 = AIRSPY_EXT_CMD_BASE + 4, /* OUT data = bytes to transmit (max 64) */
  AIRSPY_SET_CALIBRATION            = AIRSPY_EXT_CMD_BASE + 5, /* wValue|wIndex<<16 = crystal correction in ppb (int32), applied at once */
  AIRSPY_GET_CALIBRATION            = AIRSPY_EXT_CMD_BASE + 6, /* IN: airspy_calibration_t */
  /* Debug access, see airspy_debug: */
  AIRSPY_MEM_READ                   = AIRSPY_EXT_CMD_BASE + 7, /* IN: wValue | wIndex << 16 = address, wLength <= 64 bytes */
  AIRSPY_MEM_WRITE                  = AIRSPY_EXT_CMD_BASE + 8, /* OUT: same addressing, data = bytes to write */
  AIRSPY_CALL                       = AIRSPY_EXT_CMD_BASE + 9, /* OUT: airspy_call_request_t runs a function; IN: airspy_call_result_t */
  AIRSPY_SET_SOF_DIVIDER            = AIRSPY_EXT_CMD_MAX /* wValue | wIndex << 16 = divider: tag the first SOF of every USB frame whose number is a multiple of it, 0 = off; cleared at every stream stop */
} airspy_vendor_request;

/* Reply to AIRSPY_GET_STREAM_STATUS: 8 little-endian uint32 */
/* Crystal correction in effect: the flash block's value */
#define AIRSPY_CALIBRATION_SOURCE_NONE  (0)
#define AIRSPY_CALIBRATION_SOURCE_FLASH (1)
#define AIRSPY_CALIBRATION_SOURCE_HOST  (2)
typedef struct
{
  uint32_t core; /* 0 = M0 (runs in the USB request handler), 1 = M4 (runs in its main loop) */
  uint32_t address; /* function address; the Thumb bit is added by the device */
  uint32_t r[4];    /* arguments */
} airspy_call_request_t;

typedef struct
{
  uint32_t status; /* 0 = done (r0 valid), 1 = still running on the M4, 2 = bad request */
  uint32_t r0;
} airspy_call_result_t;

typedef struct
{
  int32_t correction_ppb;
  uint32_t source; /* AIRSPY_CALIBRATION_SOURCE_* */
} airspy_calibration_t;

typedef struct
{
  uint32_t captured;      /* chunks captured by the ADC DMA */
  uint32_t delivered; /* chunks whose USB transfer to the host completed */
  uint32_t lost; /* chunks overwritten before they could be sent: the sample stream has gaps */
  uint32_t overruns;
  uint32_t backlog_max; /* worst number of chunks pending at the device */
  uint32_t ring_chunks;   /* chunks the device ring buffer holds */
  uint32_t chunk_bytes;   /* bytes per chunk on USB */
  uint32_t chunk_samples; /* real ADC samples per chunk */
  uint32_t m0_lag_max; /* worst lag of the device's own USB queuing behind the ADC, in chunks */
  uint32_t dma_errors; /* ADC DMA bus errors: the DMA could not write part of the ring */
  uint32_t usb_errors; /* bulk transfers the USB controller retired with an error */
  uint32_t adc_overflows; /* chunks during which the ADC FIFO overflowed */
  uint32_t pps_count; /* PPS edges captured since the stream started */
  uint32_t sof_count; /* USB SOFs tagged since the stream started */
  uint32_t sof_edges; /* USB SOFs captured since the stream started, tagged or not */
  uint32_t sof_divider; /* AIRSPY_SET_SOF_DIVIDER value in effect, 0 = off */
} airspy_stream_status_t;

#define AIRSPY_FRAME_HEADER_SIZE (96)
#define AIRSPY_FRAME_MAGIC (0x42445642)
#define AIRSPY_FRAME_UART_BYTES (15) /* UART bytes per chunk header */
#define AIRSPY_FRAME_FLAG_PACKED (1 << 0) /* 12-bit packed samples (packing 1) */
#define AIRSPY_FRAME_FLAG_8BIT   (1 << 1) /* one byte per sample, the top 8 bits (packing 3) */

typedef struct
{
  uint64_t sample_index; /* index of this chunk's first sample in the ADC sample stream */
  uint32_t magic;          /* AIRSPY_FRAME_MAGIC */
  uint32_t chunk_index;    /* chunk number since stream start */
  uint32_t sample_count;   /* real samples in this chunk */
  uint32_t flags;          /* AIRSPY_FRAME_FLAG_* */
  uint32_t lost_chunks; /* chunks the device lost so far, see airspy_stream_status_t */
  uint32_t overrun_chunks; /* chunks the device delivered corrupted so far */
  uint32_t freq_hz; /* tuner frequency the device was set to when this chunk was queued */
  uint32_t pps_sample_index_lo; /* sample index of the last PPS edge captured since the stream started */
  uint32_t pps_sample_index_hi;
  uint32_t pps_fraction; /* position of that edge within the sample, in 1/2^32 sample units */
  uint32_t pps_count; /* PPS edges captured since the stream started */
  uint8_t uart_len; /* bytes received on the auxiliary UART (GNSS module) carried in this chunk */
  uint8_t uart_data[AIRSPY_FRAME_UART_BYTES];
  /* USB Start-Of-Frame tagging (AIRSPY_SET_SOF_DIVIDER), independent of the PPS input */
  uint32_t sof_sample_index_lo; /* sample index of the last tagged SOF */
  uint32_t sof_sample_index_hi;
  uint32_t sof_fraction; /* position of that SOF within the sample, in 1/2^32 sample units */
  uint32_t sof_frame; /* its USB frame number, extended past the 11-bit wrap by the device */
  uint32_t sof_count; /* SOFs tagged since the stream started */
  uint32_t reserved[2];    /* zero */
} airspy_frame_header_t;   /* AIRSPY_FRAME_HEADER_SIZE bytes */

/* Chunk size on the wire, framed or not */
#define AIRSPY_FRAME_WIRE_UNPACKED (16384)
#define AIRSPY_FRAME_WIRE_PACKED (6144)
#define AIRSPY_FRAME_WIRE_8BIT (4096)

/* AIRSPY_WATCHDOG: wIndex 1 feeds the watchdog and reports */
#define AIRSPY_WATCHDOG_FLAG_ARMED (1 << 0)             /* the watchdog is running */
#define AIRSPY_WATCHDOG_FLAG_RESET_BY_WATCHDOG (1 << 1) /* the last reset was a watchdog time-out (best effort) */
#define AIRSPY_WATCHDOG_FLAG_HOST_OWNED (1 << 2) /* strict mode: the host has made contact, only the host feeds */
#define AIRSPY_WATCHDOG_FLAG_STRICT (1 << 3) /* firmware built with WATCHDOG=1: the host protocol applies */
#define AIRSPY_WATCHDOG_SELF_FEEDS (6) /* strict mode: self feeds before contact, about 5.6 s each */

typedef struct
{
  uint32_t flags;           /* AIRSPY_WATCHDOG_FLAG_* */
  uint32_t timeout_ms;      /* time between feeds the device tolerates */
  uint32_t remaining_ms; /* time left before a reset when the reply was built */
  uint32_t self_feeds_left; /* strict mode: self feeds left before contact; 0xFFFFFFFF when unlimited */
} airspy_watchdog_status_t;

typedef enum
{
  GPIO_PORT0 = 0,
  GPIO_PORT1 = 1,
  GPIO_PORT2 = 2,
  GPIO_PORT3 = 3,
  GPIO_PORT4 = 4,
  GPIO_PORT5 = 5,
  GPIO_PORT6 = 6,
  GPIO_PORT7 = 7
} airspy_gpio_port_t;

typedef enum
{
  GPIO_PIN0 = 0,
  GPIO_PIN1 = 1,
  GPIO_PIN2 = 2,
  GPIO_PIN3 = 3,
  GPIO_PIN4 = 4,
  GPIO_PIN5 = 5,
  GPIO_PIN6 = 6,
  GPIO_PIN7 = 7,
  GPIO_PIN8 = 8,
  GPIO_PIN9 = 9,
  GPIO_PIN10 = 10,
  GPIO_PIN11 = 11,
  GPIO_PIN12 = 12,
  GPIO_PIN13 = 13,
  GPIO_PIN14 = 14,
  GPIO_PIN15 = 15,
  GPIO_PIN16 = 16,
  GPIO_PIN17 = 17,
  GPIO_PIN18 = 18,
  GPIO_PIN19 = 19,
  GPIO_PIN20 = 20,
  GPIO_PIN21 = 21,
  GPIO_PIN22 = 22,
  GPIO_PIN23 = 23,
  GPIO_PIN24 = 24,
  GPIO_PIN25 = 25,
  GPIO_PIN26 = 26,
  GPIO_PIN27 = 27,
  GPIO_PIN28 = 28,
  GPIO_PIN29 = 29,
  GPIO_PIN30 = 30,
  GPIO_PIN31 = 31
} airspy_gpio_pin_t;

#ifdef __cplusplus
} // __cplusplus defined.
#endif

#endif//__AIRSPY_COMMANDS_H__
