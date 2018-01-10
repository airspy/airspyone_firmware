/*
 * Copyright 2012 Jared Boone
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
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
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m0/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>

#include <airspy_core.h>
#include <si5351c.h>
#include <r820t.h>
#include <w25q80bv.h>
#include <rom_iap.h>

#include "usb.h"
#include "usb_device.h"
#include "usb_endpoint.h"
#include <usb_standard_request.h>
#include <usb_queue.h>
#include "usb_descriptor.h"
#include "airspy_usb_req.h"

#include "airspy_m0.h"
#include "airspy_commands.h"
#include "airspy_rx.h"
#include "r820t.h"

#include "airspy_conf.h"

#define ADDR_ALIGN_32BITS (3)

extern const char version_string[];

/* Allocate aligned buffer on 4bytes for 32bits store (this buffer shall be not less than 256bytes) */
uint8_t spiflash_buffer[W25Q80BV_PAGE_LEN] __attribute__ ((aligned(4)));
/* spiflash_buffer used for:
 * spiflash read/write.
 * samplerates_buffer shall not exceed AIRSPY_CONF_NB_MAX.
 * version_string.
*/

typedef struct {
  uint32_t freq_hz;
} set_freq_params_t;

set_freq_params_t set_freq_params;
uint8_t sample_rate_conf_no;

usb_request_status_t usb_vendor_request(usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage);

const usb_request_handlers_t usb_request_handlers = {
  .standard = usb_standard_request,
  .class = 0,
  .vendor = usb_vendor_request,
  .reserved = 0,
};

__inline__ void gpio_set(uint32_t gpioport, uint32_t gpios)
{
  GPIO_SET(gpioport) = gpios;
}

__inline__ void gpio_clear(uint32_t gpioport, uint32_t gpios)
{
  GPIO_CLR(gpioport) = gpios;
}

__inline__ uint32_t gpio_get(uint32_t gpioport, uint32_t gpios)
{
  return (GPIO_PIN(gpioport) & gpios) != 0;
}

void usb_streaming_disable(void)
{
  usb_endpoint_disable(&usb_endpoint_bulk_in);
  usb_endpoint_disable(&usb_endpoint_bulk_out);
}

usb_request_status_t usb_vendor_request_set_receiver_mode(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage
)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    switch( endpoint->setup.value )
    {
      case RECEIVER_MODE_OFF:
      case RECEIVER_MODE_RX:
        set_receiver_mode(endpoint->setup.value);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
      default:
        return USB_REQUEST_STATUS_STALL;
    }
  } else
  {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_reset(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  if (stage == USB_TRANSFER_STAGE_SETUP)
  {
    //usb_transfer_schedule_ack(endpoint->in);
    cpu_reset();
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_write_si5351c(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      if( endpoint->setup.value < 256 )
      {
        si5351c_write_single(endpoint->setup.index, endpoint->setup.value);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
      }
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_read_si5351c(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      const uint8_t value = si5351c_read_single(endpoint->setup.index);
      endpoint->buffer[0] = value;
      usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
      usb_transfer_schedule_ack(endpoint->out);
      return USB_REQUEST_STATUS_OK;
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_write_r820t(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      if( endpoint->setup.value < 256 )
      {
        airspy_r820t_write_single(&airspy_conf->r820t_conf_rw, endpoint->setup.index, endpoint->setup.value);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
      }
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_read_r820t(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      const uint8_t value = airspy_r820t_read_single(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
      endpoint->buffer[0] = value;
      usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
      usb_transfer_schedule_ack(endpoint->out);
      return USB_REQUEST_STATUS_OK;
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_erase_spiflash(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  if (stage == USB_TRANSFER_STAGE_SETUP)
  {
    w25q80bv_setup();
    w25q80bv_sector_erase(0); /* Erase 1st sector 64KB */
    usb_transfer_schedule_ack(endpoint->in);
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_write_spiflash(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  uint32_t addr = 0;
  uint16_t len = 0;

  if (stage == USB_TRANSFER_STAGE_SETUP)
  {
    addr = (endpoint->setup.value << 16) | endpoint->setup.index;
    len = endpoint->setup.length;
    if ((len > W25Q80BV_PAGE_LEN) || (addr > W25Q80BV_NUM_BYTES)
        || ((addr + len) > W25Q80BV_NUM_BYTES))
    {
      return USB_REQUEST_STATUS_STALL;
    } else
    {
      usb_transfer_schedule_block(endpoint->out, &spiflash_buffer[0], len);
      w25q80bv_setup();
      return USB_REQUEST_STATUS_OK;
    }
  } else if (stage == USB_TRANSFER_STAGE_DATA)
  {
    addr = (endpoint->setup.value << 16) | endpoint->setup.index;
    len = endpoint->setup.length;
    /* This check is redundant but makes me feel better. */
    if ((len > W25Q80BV_PAGE_LEN) || (addr > W25Q80BV_NUM_BYTES)
        || ((addr + len) > W25Q80BV_NUM_BYTES))
    {
      return USB_REQUEST_STATUS_STALL;
    } else {
      w25q80bv_program(addr, len, &spiflash_buffer[0]);
      usb_transfer_schedule_ack(endpoint->in);
      return USB_REQUEST_STATUS_OK;
    }
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_read_spiflash(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  uint32_t i;
  uint32_t addr;
  uint16_t len;
  uint8_t* u8_addr_pt;
  uint32_t* u32_addr_pt;
  uint32_t* u32_dest_pt;

  if (stage == USB_TRANSFER_STAGE_SETUP) 
  {
    addr = (endpoint->setup.value << 16) | endpoint->setup.index;
    len = endpoint->setup.length;

    if(len > W25Q80BV_PAGE_LEN)
    {
      return USB_REQUEST_STATUS_STALL;
    } else 
    {
      if((addr + len) > W25Q80BV_NUM_BYTES)
      {
        if( (len >= 4) &&
            ((len & ADDR_ALIGN_32BITS) == 0) &&
            ((addr & ADDR_ALIGN_32BITS) == 0)
          )
        {
          u32_addr_pt = (uint32_t*)addr;
          u32_dest_pt = (uint32_t*)&spiflash_buffer[0];
          for(i=0; i<(len/4); i++)
          {
            u32_dest_pt[i] = u32_addr_pt[i];
          }
        } else
        {
          u8_addr_pt = (uint8_t*)addr;
          for(i=0; i<len; i++)
          {
            spiflash_buffer[i] = u8_addr_pt[i];
          }
        }
      } else
      {
        w25q80bv_read(addr, len, &spiflash_buffer[0]);
      }
      usb_transfer_schedule_block(endpoint->in, &spiflash_buffer[0], len);
      return USB_REQUEST_STATUS_OK;
    }
  } else if (stage == USB_TRANSFER_STAGE_DATA) 
  {
    addr = (endpoint->setup.value << 16) | endpoint->setup.index;
    len = endpoint->setup.length;
    /* This check is redundant but makes me feel better. */
    if(len > W25Q80BV_PAGE_LEN) 
    {
      return USB_REQUEST_STATUS_STALL;
    } else
    {
      usb_transfer_schedule_ack(endpoint->out);
      return USB_REQUEST_STATUS_OK;
    }
  } else 
  {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_read_board_id(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  if (stage == USB_TRANSFER_STAGE_SETUP) {
    endpoint->buffer[0] = BOARD_ID;
    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_read_version_string(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  unsigned int i;
  int version_string_len;

  if (stage == USB_TRANSFER_STAGE_SETUP) {
    for(i = 0; i < sizeof(spiflash_buffer); i++)
      spiflash_buffer[i] = 0;
    
    strcpy((char *)spiflash_buffer, (char *)airspy_conf->conf_hw.version);
    version_string_len = strlen((char *)spiflash_buffer);
    strcpy((char *)&spiflash_buffer[version_string_len], version_string);
    version_string_len = strlen((char *)spiflash_buffer) + 1;
    version_string_len = (version_string_len + 3) & ~0x03; /* Round to a multiple of 4 */

    usb_transfer_schedule_block(endpoint->in, spiflash_buffer, version_string_len);
    usb_transfer_schedule_ack(endpoint->out);
  }
  return USB_REQUEST_STATUS_OK;
}

typedef struct {
  uint32_t part_id[2];
  uint32_t serial_no[4];
} read_partid_serialno_t;

usb_request_status_t usb_vendor_request_read_partid_serialno(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  uint8_t length;
  read_partid_serialno_t read_partid_serialno;
  iap_cmd_res_t iap_cmd_res;

  if (stage == USB_TRANSFER_STAGE_SETUP) 
  {
    /* Read IAP Part Number Identification */
    iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_PART_ID_NO;
    iap_cmd_call(&iap_cmd_res);
    if(iap_cmd_res.status_res.status_ret != CMD_SUCCESS)
      return USB_REQUEST_STATUS_STALL;

    read_partid_serialno.part_id[0] = iap_cmd_res.status_res.iap_result[0];
    read_partid_serialno.part_id[1] = iap_cmd_res.status_res.iap_result[1];
    
    /* Read IAP Serial Number Identification */
    iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_SERIAL_NO;
    iap_cmd_call(&iap_cmd_res);
    if(iap_cmd_res.status_res.status_ret != CMD_SUCCESS)
      return USB_REQUEST_STATUS_STALL;

    read_partid_serialno.serial_no[0] = iap_cmd_res.status_res.iap_result[0];
    read_partid_serialno.serial_no[1] = iap_cmd_res.status_res.iap_result[1];
    read_partid_serialno.serial_no[2] = iap_cmd_res.status_res.iap_result[2];
    read_partid_serialno.serial_no[3] = iap_cmd_res.status_res.iap_result[3];
    
    length = (uint8_t)sizeof(read_partid_serialno_t);
    usb_transfer_schedule_block(endpoint->in, &read_partid_serialno, length);
    usb_transfer_schedule_ack(endpoint->out);
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_packing_command(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage) 
{
  receiver_mode_t rx_mode;
  uint8_t state;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if(endpoint->setup.index > 2)
    {
      return USB_REQUEST_STATUS_STALL;
    }else
    {
      state = endpoint->setup.index;
    }

    rx_mode = get_receiver_mode();
    if(rx_mode == RECEIVER_MODE_RX)
    {
      ADCHS_stop(sample_rate_conf_no);
    }

    set_packing_m4(state);

    if(rx_mode == RECEIVER_MODE_RX)
    {
      ADCHS_start(sample_rate_conf_no);
    }

    endpoint->buffer[0] = 1;
    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_samplerate(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage) 
{
  int i;
  uint16_t airspy_conf_nb;
  receiver_mode_t rx_mode;
  uint16_t conf_no;
  uint32_t conf_hz;
  uint32_t freq_hz;
  bool conf_found;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    conf_no = endpoint->setup.index;
    if(conf_no < AIRSPY_CONF_NB_MAX)
    {
      airspy_conf_nb = airspy_conf->nb_airspy_m0_m4_conf_t;
      if(conf_no > (airspy_conf_nb-1))
      {
          return USB_REQUEST_STATUS_STALL;
      }else
      {
        sample_rate_conf_no = conf_no;
      }
    }else
    {
      conf_found = false;
      conf_hz = (uint32_t)(conf_no) * 1000;

      for(i = 0; i < airspy_conf->nb_airspy_m0_m4_conf_t; i++)
      {
        freq_hz = (airspy_conf->airspy_m0_m4_conf[i].airspy_m0_conf.r820t_if_freq * 4);
        if(freq_hz == conf_hz)
        {
          sample_rate_conf_no = i;
          conf_found = true;
          break;
        }
      }

      if(conf_found == false)
      {
        for(i = 0; i < airspy_conf->nb_airspy_m0_m4_alt_conf_t; i++)
        {
          freq_hz = (airspy_conf->airspy_m0_m4_alt_conf[i].airspy_m0_conf.r820t_if_freq * 4);
          if(freq_hz == conf_hz)
          {
            sample_rate_conf_no = AIRSPY_SAMPLERATE_CONF_ALT | i;
            conf_found = true;
            break;
          }
        }
      }

      if(conf_found == false)
      {
        return USB_REQUEST_STATUS_STALL;
      }
    }

    rx_mode = get_receiver_mode();
    if(rx_mode == RECEIVER_MODE_RX)
    {
      ADCHS_stop(sample_rate_conf_no);
    }

    set_samplerate_m4(sample_rate_conf_no);

    if(rx_mode == RECEIVER_MODE_RX)
    {
      ADCHS_start(sample_rate_conf_no);
    }

    endpoint->buffer[0] = 1;
    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_freq(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage) 
{
  if (stage == USB_TRANSFER_STAGE_SETUP) 
  {
    usb_transfer_schedule_block(endpoint->out, &set_freq_params, sizeof(set_freq_params_t));
    return USB_REQUEST_STATUS_OK;
  } else if (stage == USB_TRANSFER_STAGE_DATA) 
  {
    r820t_set_freq(&airspy_conf->r820t_conf_rw, set_freq_params.freq_hz);
    usb_transfer_schedule_ack(endpoint->in);
    return USB_REQUEST_STATUS_OK;
  } else
  {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_set_lna_gain(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    value = r820t_set_lna_gain(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
    endpoint->buffer[0] = value;

    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_mixer_gain(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    value = r820t_set_mixer_gain(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
    endpoint->buffer[0] = value;

    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_vga_gain(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    value = r820t_set_vga_gain(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
    endpoint->buffer[0] = value;

    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_lna_agc(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    value = r820t_set_lna_agc(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
    endpoint->buffer[0] = value;

    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_mixer_agc(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    value = r820t_set_mixer_agc(&airspy_conf->r820t_conf_rw, endpoint->setup.index);
    endpoint->buffer[0] = value;

    usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_ms_vendor_command(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
     if (endpoint->setup.index == 0x04)
     {  
        usb_transfer_schedule_block(endpoint->in, &usb_descriptor_CompatIDDescriptor, endpoint->setup.length);
        usb_transfer_schedule_ack(endpoint->out);
        return USB_REQUEST_STATUS_OK;
     }
     if (endpoint->setup.index == 0x05)
     {
        usb_transfer_schedule_block(endpoint->in, &usb_descriptor_ExtProps, endpoint->setup.length);
        usb_transfer_schedule_ack(endpoint->out);
        return USB_REQUEST_STATUS_OK;
     }  
    return USB_REQUEST_STATUS_STALL;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_rf_bias_command(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if(endpoint->setup.index == 1)
    {
      enable_biast_power();
    }else
    {
      disable_biast_power();
    }
    usb_transfer_schedule_ack(endpoint->in);
    return USB_REQUEST_STATUS_OK;
  }
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_write_gpio_command(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  uint32_t port_num;
  uint32_t pin_num;
  uint16_t index;
  uint16_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    index = endpoint->setup.index;
    if( index < 256 )
    {
      value = endpoint->setup.value;
      if( value < 2 )
      {
        port_num = index >> 5;
        port_num = (GPIO_PORT_BASE + 0x2000 + (port_num * 4));

        pin_num = index & 0x1F;
        pin_num = (1 << pin_num);

        if(value == 1)
        {
          gpio_set(port_num, pin_num);
        }else
        {
          gpio_clear(port_num, pin_num);
        }
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
      }
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_read_gpio_command(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  uint32_t port_num;
  uint32_t pin_num;
  uint8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      port_num = endpoint->setup.index >> 5;
      port_num = (GPIO_PORT_BASE + 0x2000 + (port_num * 4));

      pin_num = endpoint->setup.index & 0x1F;
      pin_num = (1 << pin_num);
      
      /* If GPIO DIR is set to OUT read the GPIO_SET reg else just read GPIO PIN */
      if( (GPIO_DIR(port_num) & pin_num) )
      {
        value = ((GPIO_SET(port_num) & pin_num) != 0);
      }else
      {
        value = gpio_get(port_num, pin_num);
      }
      endpoint->buffer[0] = value;
      usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
      usb_transfer_schedule_ack(endpoint->out);
      return USB_REQUEST_STATUS_OK;
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_gpiodir_write_command(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  uint32_t port_num;
  uint32_t pin_num;
  uint16_t index;
  uint16_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    index = endpoint->setup.index;
    if( index < 256 )
    {
      value = endpoint->setup.value;
      if( value < 2 )
      {
        port_num = index >> 5;
        port_num = (GPIO_PORT_BASE + 0x2000 + (port_num * 4));

        pin_num = index & 0x1F;
        pin_num = (1 << pin_num);

        if(value == 1)
        {
          GPIO_DIR(port_num) = (GPIO_DIR(port_num) | pin_num);
        }else
        {
          GPIO_DIR(port_num) = (GPIO_DIR(port_num) & (~pin_num));
        }
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
      }
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_gpiodir_read_command(
usb_endpoint_t* const endpoint,
const usb_transfer_stage_t stage)
{
  uint32_t port_num;
  uint32_t pin_num;
  uint8_t value;

  if( stage == USB_TRANSFER_STAGE_SETUP )
  {
    if( endpoint->setup.index < 256 )
    {
      port_num = endpoint->setup.index >> 5;
      port_num = (GPIO_PORT_BASE + 0x2000 + (port_num * 4));

      pin_num = endpoint->setup.index & 0x1F;
      pin_num = (1 << pin_num);
      
      if( (GPIO_DIR(port_num) & pin_num) )
      {
        value = 1;
      }else
      {
        value = 0;
      }
      endpoint->buffer[0] = value;
      usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 1);
      usb_transfer_schedule_ack(endpoint->out);
      return USB_REQUEST_STATUS_OK;
    }
    return USB_REQUEST_STATUS_STALL;
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_get_samplerates_command(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  int i;
  uint16_t nb_samplerate;
  uint32_t schedule_block_len;
  uint16_t airspy_conf_nb;
  uint32_t* samplerates_buffer;

  if (stage == USB_TRANSFER_STAGE_SETUP) 
  {
    nb_samplerate = endpoint->setup.index;
    airspy_conf_nb = airspy_conf->nb_airspy_m0_m4_conf_t;

    if(nb_samplerate > airspy_conf_nb)
    {
      nb_samplerate = airspy_conf_nb;
    }
    if(nb_samplerate > AIRSPY_CONF_NB_MAX)
    {
      nb_samplerate = AIRSPY_CONF_NB_MAX;
    }

    samplerates_buffer = (uint32_t*)&spiflash_buffer[0];
    if(nb_samplerate == 0)
    {
      /* Return the number of samplerates available */
      samplerates_buffer[0] = airspy_conf_nb;
      usb_transfer_schedule_block(endpoint->in, &samplerates_buffer[0], 4);
    } else
    {
      /* Return each samplerate available */
      for(i = 0; i < nb_samplerate; i++)
      {
        samplerates_buffer[i] = airspy_conf->airspy_m0_m4_conf[i].airspy_m0_conf.r820t_if_freq * 2; /* samplerate = IF_freq * 2 */
      }
      schedule_block_len = nb_samplerate * sizeof(uint32_t);
      usb_transfer_schedule_block(endpoint->in, &samplerates_buffer[0], schedule_block_len);
    }
    usb_transfer_schedule_ack(endpoint->out);
    return USB_REQUEST_STATUS_OK;
  } else 
  {
    return USB_REQUEST_STATUS_OK;
  }
}

usb_request_status_t usb_vendor_request_erase_sector_spiflash(
usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  uint32_t addr;
  uint32_t sector = 0;
  #define MIN_SECTOR (2) /* 128KB Reserved (Firmware) */
  #define MAX_SECTOR (13) /* 128KB Reserved + (13 * 64KB) Conf = 1MB Flash */
  #define MAX_SECTOR_SIZE_64KB (64 * 1024)

  if (stage == USB_TRANSFER_STAGE_SETUP)
  {
    sector = (uint32_t)endpoint->setup.value;
    /* Check if we exceed Flash Size */
    if(sector < MIN_SECTOR || sector > MAX_SECTOR)
    {
      return USB_REQUEST_STATUS_STALL;
    }
    w25q80bv_setup();
    addr = (sector * MAX_SECTOR_SIZE_64KB);
    w25q80bv_sector_erase(addr); /* Erase 64KB */
    usb_transfer_schedule_ack(endpoint->in);
  }
  return USB_REQUEST_STATUS_OK;
}

/* ID 1 to X corresponds to user endpoint->setup.request */
usb_request_handler_fn vendor_request_handler[AIRSPY_CMD_MAX+1];

void airspy_usb_req_init(void)
{
  /* Init default sample_rate conf */
  sample_rate_conf_no = AIRSPY_SAMPLERATE_DEFAULT_CONF;

  /* Init default value to 100.0MHz */
  set_freq_params.freq_hz = 100000000;

  /* TODO remove this code, for test => INVALID => RESET */
  vendor_request_handler[AIRSPY_INVALID] = usb_vendor_request_reset;

  vendor_request_handler[AIRSPY_RECEIVER_MODE] = usb_vendor_request_set_receiver_mode;

  vendor_request_handler[AIRSPY_SI5351C_WRITE] = usb_vendor_request_write_si5351c;
  vendor_request_handler[AIRSPY_SI5351C_READ] = usb_vendor_request_read_si5351c;

  vendor_request_handler[AIRSPY_R820T_WRITE] = usb_vendor_request_write_r820t;
  vendor_request_handler[AIRSPY_R820T_READ] = usb_vendor_request_read_r820t;

  vendor_request_handler[AIRSPY_SPIFLASH_ERASE] = usb_vendor_request_erase_spiflash;
  vendor_request_handler[AIRSPY_SPIFLASH_WRITE] = usb_vendor_request_write_spiflash;
  vendor_request_handler[AIRSPY_SPIFLASH_READ] = usb_vendor_request_read_spiflash;

  vendor_request_handler[AIRSPY_BOARD_ID_READ] = usb_vendor_request_read_board_id;
  vendor_request_handler[AIRSPY_VERSION_STRING_READ] = usb_vendor_request_read_version_string;
  vendor_request_handler[AIRSPY_BOARD_PARTID_SERIALNO_READ] = usb_vendor_request_read_partid_serialno;

  vendor_request_handler[AIRSPY_SET_SAMPLERATE] = usb_vendor_request_set_samplerate;

  vendor_request_handler[AIRSPY_SET_FREQ] = usb_vendor_request_set_freq;

  vendor_request_handler[AIRSPY_SET_LNA_GAIN] = usb_vendor_request_set_lna_gain;
  vendor_request_handler[AIRSPY_SET_MIXER_GAIN] = usb_vendor_request_set_mixer_gain;
  vendor_request_handler[AIRSPY_SET_VGA_GAIN] = usb_vendor_request_set_vga_gain;

  vendor_request_handler[AIRSPY_SET_LNA_AGC] = usb_vendor_request_set_lna_agc;
  vendor_request_handler[AIRSPY_SET_MIXER_AGC] = usb_vendor_request_set_mixer_agc;
  
  vendor_request_handler[AIRSPY_MS_VENDOR_CMD] = usb_vendor_request_ms_vendor_command;

  vendor_request_handler[AIRSPY_SET_RF_BIAS_CMD] = usb_vendor_request_set_rf_bias_command;

  vendor_request_handler[AIRSPY_GPIO_WRITE] = usb_vendor_request_write_gpio_command;
  vendor_request_handler[AIRSPY_GPIO_READ] = usb_vendor_request_read_gpio_command;

  vendor_request_handler[AIRSPY_GPIODIR_WRITE] = usb_vendor_request_gpiodir_write_command;
  vendor_request_handler[AIRSPY_GPIODIR_READ] = usb_vendor_request_gpiodir_read_command;

  vendor_request_handler[AIRSPY_GET_SAMPLERATES] = usb_vendor_request_get_samplerates_command;
  vendor_request_handler[AIRSPY_SET_PACKING] = usb_vendor_request_set_packing_command;

  vendor_request_handler[AIRSPY_SPIFLASH_ERASE_SECTOR] = usb_vendor_request_erase_sector_spiflash;
}

usb_request_status_t usb_vendor_request(usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
  usb_request_status_t status = USB_REQUEST_STATUS_STALL;
  
  if( endpoint->setup.request <= AIRSPY_CMD_MAX )
  {
    usb_request_handler_fn handler = vendor_request_handler[endpoint->setup.request];
    if( handler )
    {
      status = handler(endpoint, stage);
    }
  }else
  {
    if( stage == USB_TRANSFER_STAGE_SETUP )
    {
    status = USB_REQUEST_STATUS_STALL;
  } else
  {
    status = USB_REQUEST_STATUS_OK;
  }
  }
  return status;
}

