/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements a the USB Communications Device Class - Abstract
 * Control Model (CDC-ACM) as defined in CDC PSTN subclass 1.2.
 *
 * The device's unique id is used as the USB serial number string.
 */
#ifndef __CDCACM_H
#define __CDCACM_H

#include <libopencm3/usb/usbd.h>

#define CDCACM_PACKET_SIZE 	64

extern usbd_device *usbdev;
extern int cdcacm_configured;

void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_dtr(void);

#define	_UEP_BULK(addr, interv)	{ 			\
	.bLength = USB_DT_ENDPOINT_SIZE,		\
	.bDescriptorType = USB_DT_ENDPOINT,		\
	.bEndpointAddress = (addr),			\
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,		\
	.wMaxPacketSize = 64,				\
	.bInterval = (interv),				\
    }
#define	_UEP_INTR(addr)		{ 			\
	.bLength = USB_DT_ENDPOINT_SIZE,		\
	.bDescriptorType = USB_DT_ENDPOINT,		\
	.bEndpointAddress = (addr),			\
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,	\
	.wMaxPacketSize = 16,				\
	.bInterval = 255,				\
    }

#define _UIF_FUND(commi,datai)							\
    .header = {									\
	.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),		\
	.bDescriptorType = CS_INTERFACE,					\
	.bDescriptorSubtype = USB_CDC_TYPE_HEADER,				\
	.bcdCDC = 0x0110,							\
    },										\
    .call_mgmt = {								\
	.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),	\
	.bDescriptorType = CS_INTERFACE,					\
	.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,			\
	.bmCapabilities = 0,							\
	.bDataInterface = (datai),						\
    },										\
    .acm = {									\
	.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),		\
	.bDescriptorType = CS_INTERFACE,					\
	.bDescriptorSubtype = USB_CDC_TYPE_ACM,					\
	.bmCapabilities = 2, /* SET_LINE_CODING supported*/			\
    },										\
    .cdc_union = {								\
	.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),		\
	.bDescriptorType = CS_INTERFACE,					\
	.bDescriptorSubtype = USB_CDC_TYPE_UNION,				\
	.bControlInterface = (commi),						\
	.bSubordinateInterface0 = (datai),					\
    }

#define	_UIF_COMM(ifn, ep, fd, istr)	{		\
	.bLength = USB_DT_INTERFACE_SIZE,		\
	.bDescriptorType = USB_DT_INTERFACE,		\
	.bInterfaceNumber = (ifn),			\
	.bAlternateSetting = 0,				\
	.bNumEndpoints = 1,				\
	.bInterfaceClass = USB_CLASS_CDC,		\
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,	\
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,	\
	.iInterface = (istr),				\
	.endpoint = (ep),				\
	.extra = &(fd),					\
	.extralen = sizeof((fd))			\
    }

#define	_UIF_DATA(ifn, ep)	{			\
	.bLength = USB_DT_INTERFACE_SIZE,		\
	.bDescriptorType = USB_DT_INTERFACE,		\
	.bInterfaceNumber = (ifn),			\
	.bAlternateSetting = 0,				\
	.bNumEndpoints = 2,				\
	.bInterfaceClass = USB_CLASS_DATA,		\
	.bInterfaceSubClass = 0,			\
	.bInterfaceProtocol = 0,			\
	.iInterface = 0,				\
	.endpoint = (ep),				\
    }

#define	_UIF_ASSO(ifn)						\
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,		\
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,	\
	.bFirstInterface = (ifn),				\
	.bInterfaceCount = 2,					\
	.bFunctionClass = USB_CLASS_CDC,			\
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,		\
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,		\
	.iFunction = 0,

#define	_UIF_CDC(asso, commi, datai)		{		\
	.num_altsetting = 1,					\
	.iface_assoc = &(asso),					\
	.altsetting = (commi),					\
    }, {							\
	.num_altsetting = 1,					\
	.altsetting = (datai),					\
    }

#endif