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

#include <stdint.h>

#include "usb_type.h"
#include "airspy_commands.h"
#include "usb_descriptor.h"

#define USB_VENDOR_ID  (0x1D50)
#define USB_PRODUCT_ID (0x60A1)

#define USB_WORD(x) (x & 0xFF), ((x >> 8) & 0xFF)

#define USB_MAX_PACKET0        (64)
#define USB_MAX_PACKET_BULK_FS (64)
#define USB_MAX_PACKET_BULK_HS (512)

#define USB_BULK_IN_EP_ADDR  (0x81)
#define USB_BULK_OUT_EP_ADDR (0x02)

#define USB_STRING_LANGID (0x0409)

uint8_t usb_descriptor_device[] =
{
  18,                                // bLength
  USB_DESCRIPTOR_TYPE_DEVICE,        // bDescriptorType
  USB_WORD(0x0200),                  // bcdUSB
  0x00,                              // bDeviceClass
  0x00,                              // bDeviceSubClass
  0x00,                              // bDeviceProtocol
  USB_MAX_PACKET0,                   // bMaxPacketSize0
  USB_WORD(USB_VENDOR_ID),           // idVendor
  USB_WORD(USB_PRODUCT_ID),          // idProduct
  USB_WORD(0x0100),                  // bcdDevice
  0x01,                              // iManufacturer
  0x02,                              // iProduct
  0x03,                              // iSerialNumber
  0x01                               // bNumConfigurations
};

uint8_t usb_descriptor_device_qualifier[] =
{
  10,                         // bLength
  USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, // bDescriptorType
  USB_WORD(0x0200),                     // bcdUSB
  0x00,                                 // bDeviceClass
  0x00,                                 // bDeviceSubClass
  0x00,                                 // bDeviceProtocol
  64,                                   // bMaxPacketSize0
  0x01,                                 // bNumOtherSpeedConfigurations
  0x00                                  // bReserved
};

uint8_t usb_descriptor_configuration_full_speed[] =
{
  9,                      // bLength
  USB_DESCRIPTOR_TYPE_CONFIGURATION,  // bDescriptorType
  USB_WORD(32),           // wTotalLength
  0x01,                   // bNumInterfaces
  0x01,                   // bConfigurationValue
  0x00,                   // iConfiguration
  0x80,                   // bmAttributes: USB-powered
  250,                    // bMaxPower: 500mA
  
  9,                      // bLength
  USB_DESCRIPTOR_TYPE_INTERFACE,    // bDescriptorType
  0x00,                   // bInterfaceNumber
  0x00,                   // bAlternateSetting
  0x02,                   // bNumEndpoints
  0xFF,                   // bInterfaceClass: vendor-specific
  0xFF,                   // bInterfaceSubClass
  0xFF,                   // bInterfaceProtocol: vendor-specific
  0x00,                   // iInterface
  
  7,                      // bLength
  USB_DESCRIPTOR_TYPE_ENDPOINT,   // bDescriptorType
  USB_BULK_IN_EP_ADDR,    // bEndpointAddress
  0x02,                   // bmAttributes: BULK
  USB_WORD(USB_MAX_PACKET_BULK_FS),   // wMaxPacketSize
  0x00,                   // bInterval: no NAK
  
  7,                      // bLength
  USB_DESCRIPTOR_TYPE_ENDPOINT,   // bDescriptorType
  USB_BULK_OUT_EP_ADDR,   // bEndpointAddress
  0x02,                   // bmAttributes: BULK
  USB_WORD(USB_MAX_PACKET_BULK_FS),   // wMaxPacketSize
  0x00,                   // bInterval: no NAK

  0,                  // TERMINATOR
};

uint8_t usb_descriptor_configuration_high_speed[] =
{
  9,                      // bLength
  USB_DESCRIPTOR_TYPE_CONFIGURATION,  // bDescriptorType
  USB_WORD(32),           // wTotalLength
  0x01,                   // bNumInterfaces
  0x01,                   // bConfigurationValue
  0x00,                   // iConfiguration
  0x80,                   // bmAttributes: USB-powered
  250,                    // bMaxPower: 500mA
  
  9,                      // bLength
  USB_DESCRIPTOR_TYPE_INTERFACE,    // bDescriptorType
  0x00,                   // bInterfaceNumber
  0x00,                   // bAlternateSetting
  0x02,                   // bNumEndpoints
  0xFF,                   // bInterfaceClass: vendor-specific
  0xFF,                   // bInterfaceSubClass
  0xFF,                   // bInterfaceProtocol: vendor-specific
  0x00,                   // iInterface
  
  7,                      // bLength
  USB_DESCRIPTOR_TYPE_ENDPOINT,   // bDescriptorType
  USB_BULK_IN_EP_ADDR,            // bEndpointAddress
  0x02,                   // bmAttributes: BULK
  USB_WORD(USB_MAX_PACKET_BULK_HS),   // wMaxPacketSize
  0x00,                   // bInterval: no NAK
  
  7,                      // bLength
  USB_DESCRIPTOR_TYPE_ENDPOINT,   // bDescriptorType
  USB_BULK_OUT_EP_ADDR,           // bEndpointAddress
  0x02,                   // bmAttributes: BULK
  USB_WORD(USB_MAX_PACKET_BULK_HS),   // wMaxPacketSize
  0x00,                   // bInterval: no NAK

  0,                  // TERMINATOR
};

uint8_t usb_descriptor_string_languages[] =
{
  0x04,                        // bLength
  USB_DESCRIPTOR_TYPE_STRING,  // bDescriptorType
  USB_WORD(USB_STRING_LANGID), // wLANGID
};

uint8_t usb_descriptor_string_manufacturer[] =
{
  30,                         // bLength
  USB_DESCRIPTOR_TYPE_STRING, // bDescriptorType
  'w', 0x00,
  'w', 0x00,
  'w', 0x00,
  '.', 0x00,
  'a', 0x00,
  'i', 0x00,
  'r', 0x00,
  's', 0x00,
  'p', 0x00,
  'y', 0x00,
  '.', 0x00,
  'c', 0x00,
  'o', 0x00,
  'm', 0x00
};

uint8_t usb_descriptor_string_product[] =
{
  14,                         // bLength
  USB_DESCRIPTOR_TYPE_STRING, // bDescriptorType
  'A', 0x00,
  'I', 0x00,
  'R', 0x00,
  'S', 0x00,
  'P', 0x00,
  'Y', 0x00
};

/* 64bits Serial Number from SPIFI */
#define USB_DESCRIPTOR_SN_POS (22)
uint8_t usb_descriptor_string_serial_number[] =
{
  54,                         // bLength
  USB_DESCRIPTOR_TYPE_STRING, // bDescriptorType
  'A', 0x00,
  'I', 0x00,
  'R', 0x00,
  'S', 0x00,
  'P', 0x00,
  'Y', 0x00,
  ' ', 0x00,
  'S', 0x00,
  'N', 0x00,
  ':', 0x00,
  /* Data filled by CPU with 64bits Serial Number from SPIFI to ASCII HEX */
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00,
  ' ', 0x00
};

static const uint8_t htoa[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void usb_descriptor_fill_string_serial_number(usb_descriptor_serial_number_t serial_number)
{
  int i, j;
  uint32_t data_u32;
  uint8_t data_u8;

  j = 0;
  for(i=0; i<2; i++)
  {
    data_u32 = serial_number.sn_32b[i];

    data_u8 = (data_u32 & 0xFF000000) >> 24;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0xF0) >> 4];
    j+=2;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0x0F)];
    j+=2;

    data_u8 = (data_u32 & 0x00FF0000) >> 16;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0xF0) >> 4];
    j+=2;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0x0F)];
    j+=2;

    data_u8 = (data_u32 & 0x0000FF00) >> 8;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0xF0) >> 4];
    j+=2;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0x0F)];
    j+=2;

    data_u8 = (data_u32 & 0x000000FF);
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0xF0) >> 4];
    j+=2;
    usb_descriptor_string_serial_number[USB_DESCRIPTOR_SN_POS + j] = htoa[(data_u8 & 0x0F)];
    j+=2;
  }
}

uint8_t* const usb_descriptor_strings[] =
{
  usb_descriptor_string_languages,
  usb_descriptor_string_manufacturer,
  usb_descriptor_string_product,
  usb_descriptor_string_serial_number,
  0, // TERMINATOR
};

uint8_t usb_descriptor_MSDescriptor[] =
{
  0x12, /* Length */
  0x03, /* Descriptor Type */ 
  0x4D,0x00,0x53,0x00,0x46,0x00,0x54,0x00,0x31,0x00,0x30,0x00,0x30,0x00, /* 'MSFT100' */
  AIRSPY_MS_VENDOR_CMD, /* Vendor Specific to retrieve OS feature descriptors (shall be > 0) */
  0x00 /* Pad field */
};

uint8_t usb_descriptor_CompatIDDescriptor[] =
{
  0x28,0x00,0x00,0x00, //dwLength
  0x00,0x01,   //bcdVersion
  0x04,0x00,   //wIndex (Fixed:INDEX_CONFIG_DESCRIPTOR)
  0x01,        //bCount (Count of device functions - Must be 1)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00, //Reserved
  0x01,        //bFirstInterfaceNumber (First interface for this function)
  0x01,        //Reserved
  'W' ,'I' ,'N' ,'U' ,'S' ,'B' ,0x00,0x00, //Compatible ID, padded with zeros
  'W' ,'I' ,'N' ,'U' ,'S' ,'B' ,0x00,0x00, //Sub-compatible ID, padded with zeros
  0x00,0x00,0x00,0x00,0x00,0x00 //Reserved
};

uint8_t usb_descriptor_ExtProps[] =
{
  0x8e,0x00,0x00,0x00,                         //dwLength
  0x00,0x01,                     //bcdVersion 0x02
  0x05,0x00,                     //wIndex (Fixed:5) 0x02
  0x01,0x00,                 //wCount  0x02
  //0x00,0x00,0x00,0x00,0x00,0x00,0x00,          //Reserved 0x07
  ////////////////////////////////////////////////////////
  //Custom Property 1 
  ////////////////////////////////////////////////////////  
  0x84,0x00,0x00,0x00,                         //dwLength //0x04
  0x01,0x00,0x00,0x00,                         //dwPropertyDataType   //0x04
  0x28,0x00,                                   //wPropertyNameLength  //0x02
  'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,'r',0,'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,0,0,//bPropertyName = 0x28
  0x4e,0x00,0x00,0x00,                         //dwPropertyDataLength //0x04
  '{',0,'4',0,'A',0,'F',0,'4',0,'1',0,'8',0,'6'
     ,0,'5',0,'-',0,'B',0,'C',0,'1',0,'E',0,'-'
     ,0,'7',0,'4',0,'5',0,'A',0,'-',0,'9',0,'8'
     ,0,'2',0,'6',0,'-',0,'9',0,'7',0,'C',0,'A'
     ,0,'A',0,'8',0,'E',0,'E',0,'0',0,'4',0,'7'
     ,0,'F',0,'}',0,0,0 //0x4e //bPropertyData
};
