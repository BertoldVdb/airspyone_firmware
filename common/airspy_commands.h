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
#define AIRSPY_CMD_MAX (37)
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
  AIRSPY_SPIFLASH_ERASE_SECTOR      = 27,
  AIRSPY_GET_STREAM_STATUS          = 28, /* IN: airspy_stream_status_t */
  AIRSPY_SET_FRAMING                = 29, /* wIndex 1 = on, 0 = off; cleared at every stream stop */
  AIRSPY_MEM_READ                   = 35, /* IN: wValue | wIndex << 16 = address, wLength <= 64 bytes */
  AIRSPY_MEM_WRITE                  = 36, /* OUT: same addressing, data = bytes to write */
  AIRSPY_CALL                       = AIRSPY_CMD_MAX /* OUT: airspy_call_request_t runs a function; IN: airspy_call_result_t */
} airspy_vendor_request;

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


/* Reply to AIRSPY_GET_STREAM_STATUS: 8 little-endian uint32 */
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
} airspy_stream_status_t;

#define AIRSPY_FRAME_HEADER_SIZE (96)
#define AIRSPY_FRAME_MAGIC (0x59505341) /* "ASPY" */
#define AIRSPY_FRAME_FLAG_PACKED (1 << 0)

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
  uint32_t reserved[15];   /* zero */
} airspy_frame_header_t;   /* AIRSPY_FRAME_HEADER_SIZE bytes */

/* Chunk size on the wire, framed or not */
#define AIRSPY_FRAME_WIRE_UNPACKED (16384)
#define AIRSPY_FRAME_WIRE_PACKED (6144)

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
