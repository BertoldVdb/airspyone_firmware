/*
 * Copyright 2012 Jared Boone
 * Copyright 2013-2015 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
 *
 * This file is part of AirSpy (based on HackRF project).
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

#include <string.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/i2c.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m0/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>

#include <airspy_core.h>
#include <si5351c.h>
#include <r820t.h>
#include <w25q80bv.h>
#include <rom_iap.h>
#include <signal_mcu.h>

#include "usb.h"
#include "usb_standard_request.h"

#include "usb_device.h"
#include "usb_endpoint.h"
#include "usb_descriptor.h"

#include "airspy_conf.h"
#include "airspy_usb_req.h"
#include "airspy_commands.h"
#include "airspy_rx.h"
#include "r820t.h"
#include "airspy_m0.h"
#include "airspy_stream.h"
#include "airspy_m0.hdr"

extern uint32_t cm0_data_share; /* defined in linker script */

volatile unsigned int phase = 0;

volatile airspy_stream_state_t * const stream = AIRSPY_STREAM_STATE;
static volatile uint32_t stream_epoch = 0;

volatile airspy_mcore_t *start_adchs = (airspy_mcore_t *)(&cm0_data_share);
volatile airspy_mcore_t *set_samplerate = (airspy_mcore_t *)((&cm0_data_share)+1);
volatile airspy_mcore_t *set_packing = (airspy_mcore_t *)((&cm0_data_share)+2);
volatile airspy_mcore_t *set_framing = (airspy_mcore_t *)((&cm0_data_share)+3);

#define MASTER_TXEV_FLAG  ((uint32_t *) 0x40043130)
#define MASTER_TXEV_QUIT()  { *MASTER_TXEV_FLAG = 0x0; }

uint8_t* const usb_bulk_buffer = (uint8_t*)AIRSPY_STREAM_RING_ADDR;

const char version_string[] = " " AIRSPY_FW_GIT_TAG " " AIRSPY_FW_CHECKIN_DATE;

typedef struct {
  uint32_t freq_hz;
  uint32_t divider;
} set_sample_r_params_t;

set_sample_r_params_t set_sample_r_params;

__attribute__ ((always_inline)) static inline void start_stop_adchs_m4(uint8_t conf_num, uint8_t command)
{
  start_adchs->conf = conf_num;
  start_adchs->cmd = command;

  signal_sev();

  /* Wait until M4 have finished executing the command (it set the data to 0) */
  while(1)
  {
    if(start_adchs->raw == 0)
      break;
  }
}

void set_samplerate_m4(uint8_t conf_num)
{
  set_samplerate->conf = conf_num;
  set_samplerate->cmd = SET_SAMPLERATE_CMD;

  signal_sev();

  /* Wait until M4 have finished executing the command (it set the data to 0) */
  while(1)
  {
    if(set_samplerate->raw == 0)
      break;
  }
}

void set_packing_m4(uint8_t state)
{
  set_packing->conf = state;
  set_packing->cmd = SET_PACKING_CMD;
  
  signal_sev();
  
  while(1)
  {
    if(set_packing->raw == 0)
      break;
  }
}

void set_framing_m4(uint8_t state)
{
  set_framing->conf = state;
  set_framing->cmd = SET_FRAMING_CMD;

  signal_sev();

  while(1)
  {
    if(set_framing->raw == 0)
      break;
  }
}

static void stream_write_header(uint8_t* chunk, uint32_t chunk_index, uint32_t chunk_samples, uint32_t chunk_bytes)
{
  airspy_frame_header_t* h = (airspy_frame_header_t*)chunk;
  uint32_t i;

  h->sample_index = (uint64_t)chunk_index * chunk_samples;
  h->magic = AIRSPY_FRAME_MAGIC;
  h->chunk_index = chunk_index;
  h->sample_count = chunk_samples;
  h->flags = (chunk_bytes == AIRSPY_STREAM_CHUNK_BYTES_PACKED) ? AIRSPY_FRAME_FLAG_PACKED : 0;
  h->lost_chunks = stream->lost;
  h->overrun_chunks = stream->overruns;
  h->freq_hz = set_freq_params.freq_hz;
  for(i = 0; i < sizeof(h->reserved) / sizeof(h->reserved[0]); i++)
    h->reserved[i] = 0;
}

void usb_configuration_changed(usb_device_t* const device)
{
  if( device->configuration->number )
  {
    /* RECEIVER ON */
    set_receiver_mode(get_receiver_mode());
  } else
  {
    /* RECEIVER OFF */
    /* Configuration number equal 0 means usb bus reset. */
    set_receiver_mode(RECEIVER_MODE_OFF);
  }
}

void ADCHS_start(uint8_t conf_num)
{
  stream->delivered = 0;
  stream->lost = 0;
  stream->queued = 0;

  start_stop_adchs_m4(conf_num, START_ADCHS_CMD);

  //enable_r820t_power();

  /* Re-Init I2C0 & I2C1 after PLL1 frequency is modified (for I2C1 also because PowerOn on R820T) */
  i2c0_init(airspy_conf->i2c_conf.i2c0_pll1_ls_hs_conf_val); /* Si5351C I2C peripheral */
  i2c1_init(airspy_conf->i2c_conf.i2c1_pll1_hs_conf_val); /* R820T I2C peripheral */

  if((conf_num & AIRSPY_SAMPLERATE_CONF_ALT) == AIRSPY_SAMPLERATE_CONF_ALT)
  {
    conf_num = conf_num & (~AIRSPY_SAMPLERATE_CONF_ALT);
    r820t_init(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_alt_conf[conf_num].airspy_m0_conf.r820t_if_freq);
    r820t_set_if_bandwidth(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_alt_conf[conf_num].airspy_m0_conf.r820t_if_bw);
  }else
  {
    r820t_init(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_conf[conf_num].airspy_m0_conf.r820t_if_freq);
    r820t_set_if_bandwidth(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_conf[conf_num].airspy_m0_conf.r820t_if_bw);
  }
  phase = 1;
  stream_epoch++;
}

void ADCHS_stop(uint8_t conf_num)
{
  r820t_standby();
  start_stop_adchs_m4(conf_num, STOP_ADCHS_CMD);
  set_framing_m4(0);

  /* Re-Init I2C0 & I2C1 after PLL1 frequency is modified */
  i2c0_init(airspy_conf->i2c_conf.i2c0_pll1_ls_hs_conf_val); /* Si5351C I2C peripheral */
  i2c1_init(airspy_conf->i2c_conf.i2c1_pll1_ls_conf_val); /* R820T I2C peripheral */
  stream_epoch++;
}

void usb_bulk_in_transfer_complete(usb_endpoint_t* const endpoint)
{
  uint32_t before = usb_queue_active_count(endpoint);
  usb_queue_transfer_complete(endpoint);
  stream->delivered += before - usb_queue_active_count(endpoint);
  signal_sev();
}

static void stream_queue_chunks(uint32_t *queued_p)
{
  uint32_t queued = *queued_p;
  uint32_t captured = stream->captured;
  uint32_t ring_chunks = stream->ring_chunks;
  uint32_t chunk_stride = stream->chunk_stride;
  uint32_t chunk_bytes = stream->chunk_bytes;
  uint32_t header_bytes = stream->header_bytes;
  uint32_t chunk_samples = stream->chunk_samples;
  int32_t pending = (int32_t)(captured - queued);

  if(pending < 0 || ring_chunks == 0)
    return;

  if((uint32_t)pending > ring_chunks - 1)
  {
    uint32_t skip = (uint32_t)pending - (ring_chunks - 1);
    stream->lost += skip;
    queued += skip;
  }

  while((int32_t)(captured - queued) > 0)
  {
    uint32_t offset = (queued % ring_chunks) * chunk_stride;
    if(header_bytes)
      stream_write_header(&usb_bulk_buffer[offset], queued, chunk_samples, chunk_bytes);
    if(usb_transfer_schedule(&usb_endpoint_bulk_in, &usb_bulk_buffer[offset], chunk_bytes) != 0)
      break;
    queued++;
  }

  stream->queued = queued;
  *queued_p = queued;
}

/* adchs_isr managed by M4 */
void m4core_isr(void)
{
  MASTER_TXEV_QUIT();
}

/*
M0 Core Manage USB 
*/
int main(void)
{
  iap_cmd_res_t iap_cmd_res;
  usb_descriptor_serial_number_t serial_number;
  airspy_usb_req_init();

  /* R820T Startup */
  r820t_startup(&airspy_conf->r820t_conf_rw);

  usb_set_configuration_changed_cb(usb_configuration_changed);
  usb_peripheral_reset();

  usb_device_init(0, &usb_device);

  usb_queue_init(&usb_endpoint_control_out_queue);
  usb_queue_init(&usb_endpoint_control_in_queue);
  usb_queue_init(&usb_endpoint_bulk_out_queue);
  usb_queue_init(&usb_endpoint_bulk_in_queue);

  usb_endpoint_init(&usb_endpoint_control_out);
  usb_endpoint_init(&usb_endpoint_control_in);

  /* Read IAP Serial Number Identification */
  iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_SERIAL_NO;
  iap_cmd_call(&iap_cmd_res);
  if(iap_cmd_res.status_res.status_ret == CMD_SUCCESS)
  {
    /* Only retrieve 2 last 32bits for Serial Number */
    serial_number.sn_32b[0] = iap_cmd_res.status_res.iap_result[2];
    serial_number.sn_32b[1] = iap_cmd_res.status_res.iap_result[3];
    usb_descriptor_fill_string_serial_number(serial_number);
  }

  nvic_set_priority(NVIC_USB0_IRQ, 255);
  
  nvic_set_priority(NVIC_M4CORE_IRQ, 1);
  nvic_enable_irq(NVIC_M4CORE_IRQ);

  usb_run(&usb_device);

  uint32_t epoch = stream_epoch;
  uint32_t queued = 0;

  while(true)
  {
    signal_wfe();

    if(epoch != stream_epoch)
    {
      epoch = stream_epoch;
      queued = 0;
      if(get_receiver_mode() == RECEIVER_MODE_RX)
      {
        usb_endpoint_flush(&usb_endpoint_bulk_in);
        queued = stream->captured;
        stream->delivered = queued;
        stream->lost = 0;
        stream->queued = queued;
      }
      continue;
    }

    if(get_receiver_mode() != RECEIVER_MODE_RX)
      continue;

    stream_queue_chunks(&queued);
  }
}
