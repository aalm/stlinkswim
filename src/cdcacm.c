/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <stdlib.h>

extern void	stlink_data_rx_cb(usbd_device *, uint8_t);
static void	cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd);

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[256];

usbd_device *usbdev;

int cdcacm_configured;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
#if 1
	.bDeviceClass = 0xEF,		/* Miscellaneous Device */
	.bDeviceSubClass = 2,		/* Common Class */
	.bDeviceProtocol = 1,		/* Interface Association */
#else
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
#endif
	.bMaxPacketSize0 = 64,
#if 0
	.idVendor = 0x1D50,	/* ? */
	.idProduct = 0x6018,	/* ? */
#else
	.idVendor = 0x0483,		/* STMicroelectronics */
	.idProduct = 0x3748,		/* ST-LINK/V2 */
#endif
	.bcdDevice = 0x0100,
	.iManufacturer = 1,	/* "STMicroelectronics "*/
	.iProduct = 2,		/* BOARD_IDENT from platform.h */
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor stlink_data_endp[] = {
	_UEP_BULK(0x81, 0),
	_UEP_BULK(0x02, 0),
	_UEP_BULK(0x83, 0)
};
static const struct usb_interface_descriptor stlink_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 3,
	.bInterfaceClass = 255,		/* vendor specific class */
	.bInterfaceSubClass = 255,	/* vendor specific subclass */
	.bInterfaceProtocol = 255,	/* vendor specific protocol */
	.iInterface = 4,	/* "ST Link" */
	.endpoint = stlink_data_endp,
}};

/* Serial ACM interfaces */
static const struct usb_endpoint_descriptor uart_comm_endp1[] = {
	_UEP_INTR(0x86)
};

static const struct usb_endpoint_descriptor uart_data_endp1[] = {
	_UEP_BULK(0x05, 1),
	_UEP_BULK(0x85, 1)
};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptors1 = {
	_UIF_FUND(1, 2)
};

static const struct usb_interface_descriptor uart_comm_iface1[] = {
	_UIF_COMM(1, uart_comm_endp1, uart_cdcacm_functional_descriptors1, 5)
};

static const struct usb_interface_descriptor uart_data_iface1[] = {
	_UIF_DATA(2, uart_data_endp1)
};

static const struct usb_iface_assoc_descriptor uart_assoc1 = {
	_UIF_ASSO(1)
};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = stlink_data_iface,
},
	_UIF_CDC(uart_assoc1, uart_comm_iface1, uart_data_iface1)
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = sizeof(ifaces) / sizeof(ifaces[0]),
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
#if 1
	.bMaxPower = 0x32,
#else
	.bMaxPower = 0x64,
#endif

	.interface = ifaces,
};

char serial_no[9];

static const char *usb_strings[] = {
	"STMicroelectronics",
	BOARD_IDENT,
	serial_no,
	"ST Link",
	"USART1 RX:PA10-TX:PA9",
};

#if 0
static int
cdcacm_control_request(usbd_device *dev, struct usb_setup_data *req,
    uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device *dev, struct usb_setup_data *req))
#else
static enum usbd_request_return_codes
cdcacm_control_request(usbd_device *dev, struct usb_setup_data *req,
    uint8_t **buf, uint16_t *len,
    usbd_control_complete_callback *complete)
#endif
{
	(void)complete;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		cdcacm_set_modem_state(dev, req->wIndex, true, true);
		/* Ignore since is not for GDB interface */
		return USBD_REQ_HANDLED;
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;
#if 0
		switch (req->wIndex) {
		case 0:
			usbuart_set_line_coding((struct usb_cdc_line_coding *)*buf, USART1);
			return 1;
#if 0
		case 0:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf, USART2);
			return 1;
		case 2:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf, USART3);
			return 1;
		case 4:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf, USART1);
			return 1;
#endif
		default:
			return 0;
		}
#else
		usbuart_set_line_coding((struct usb_cdc_line_coding *)*buf);
		return USBD_REQ_HANDLED;
#endif
	}
	return USBD_REQ_NOTSUPP;
}

static void
cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd)
{
	char buf[10];
	struct usb_cdc_notification *notif = (void *)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
	buf[9] = 0;
	usbd_ep_write_packet(dev, /* XXX */0x85 + iface, buf, 10);
}

static void
cdcacm_set_config(usbd_device *_dev, uint16_t wValue)
{

	cdcacm_configured = wValue;

	usbd_ep_setup(_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, stlink_data_rx_cb);
	usbd_ep_setup(_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	/* Serial interface */
	usbd_ep_setup(_dev, 0x05, USB_ENDPOINT_ATTR_BULK, 64, usbuart1_usb_out_cb);
	usbd_ep_setup(_dev, 0x85, USB_ENDPOINT_ATTR_BULK, 64, usbuart_usb_in_cb);
	usbd_ep_setup(_dev, 0x86, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(_dev,
	    USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
	    USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
	    cdcacm_control_request);

	/* Notify the host that DCD is asserted.
	 * Allows the use of /dev/tty* devices on *BSD/MacOS
	 */
	cdcacm_set_modem_state(_dev, 1/*was 0*/, true, true);
}

static char *
serialno_read(char *s)
{
	volatile uint32_t *unique_id_p = (volatile uint32_t *)0x1FFFF7E8;
	uint32_t unique_id = *unique_id_p + *(unique_id_p + 1) + *(unique_id_p + 2);
	uint8_t i;

	/* Fetch serial number from chip's unique ID */
	for (i = 0; i < 8; i++) {
		s[7 - i] = ((unique_id >> (4 * i)) & 0xF) + '0';
	}
	for (i = 0; i < 8; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;
	s[8] = 0;

	return s;
}

void
cdcacm_init(void)
{

	serialno_read(serial_no);

	usbdev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
	    usb_strings, sizeof(usb_strings) / sizeof(char *),
	    usbd_control_buffer, sizeof(usbd_control_buffer));

	cdcacm_configured = 0;
	usbd_register_set_config_callback(usbdev, cdcacm_set_config);

	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

void
usb_lp_can_rx0_isr(void)
{
	usbd_poll(usbdev);
}