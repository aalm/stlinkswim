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

/* Provides main entry point.  Initialise subsystems and enter GDB
 * protocol loop.
 */

#include "general.h"
#include "swim_hwcfg.h"
#include "swim.h"
#include "cdcacm.h"
#include "usbuart.h"

#include <libopencm3/stm32/rcc.h>

// STLINK usb interface
#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

// swim commands
#define STLINK_SWIM_ENTER               0x00
#define STLINK_SWIM_EXIT                0x01
#define STLINK_SWIM_READ_CAP            0x02
#define STLINK_SWIM_SPEED               0x03
#define STLINK_SWIM_ENTER_SEQ           0x04
#define STLINK_SWIM_GEN_RST             0x05
#define STLINK_SWIM_RESET               0x06
#define STLINK_SWIM_ASSERT_RESET        0x07
#define STLINK_SWIM_DEASSERT_RESET      0x08
#define STLINK_SWIM_READSTATUS          0x09
#define STLINK_SWIM_WRITEMEM            0x0a
#define STLINK_SWIM_READMEM             0x0b
#define STLINK_SWIM_READBUF             0x0c
#define STLINK_SWIM_READ_BUFFERSIZE     0x0d

// swim error codes
#define STLINK_SWIM_OK          0x00
#define STLINK_SWIM_BUSY        0x01
#define STLINK_SWIM_NO_RESPONSE 0x04 // Target did not respond. SWIM not active?
#define STLINK_SWIM_BAD_STATE   0x05 // ??

// for handling the stlink protocol
#define STLINK_STATE_CMD    0
#define STLINK_STATE_READ   1
#define STLINK_STATE_WRITE  2

void	stlink_data_rx_cb(usbd_device *, uint8_t);

static struct {
	uint8_t state;
	uint8_t mode;
	uint16_t totalBytes;
	uint16_t curBytes;
	uint32_t address;
} stlink_status;

static uint8_t ep_buf[64];
static uint8_t swim_buffer[SWIM_BUFFERSIZE];

#if 0
static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0, // filled in by usb_standard.c
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x64,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "STMicroelectronics",
  "STM32 STLink",
  "PÿnPfVW59",
  "ST Link",
};
#endif

#ifdef SWIM_DEBUG
static void
dbg_usart_setup(void)
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3); /* XXX _MODE_TX used */

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);
}

int _write(int fd, char *ptr, int len);
int
_write(int fd, char *ptr, int len)
{
	int i = 0;

	/*
	 * Write "len" of char from "ptr" to file id "fd"
	 * Return number of char written.
	 * Only work for STDOUT, STDIN, and STDERR
	 */
	if (fd > 2) {
		return -1;
	}
	while (*ptr && (i < len)) {
		usart_send_blocking(USART1, *ptr);
		if (*ptr == '\n') {
			usart_send_blocking(USART1, '\r');
		}
		i++;
		ptr++;
	}
	return i;
}
#endif /* SWIM_DEBUG */

void
stlink_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	int len = usbd_ep_read_packet(usbd_dev, 0x02, ep_buf, 64);
#ifdef SWIM_DEBUG
	printf("cb:%d-%d\n", ep_buf[0], ep_buf[1]);
	fflush(stdout);
#endif
	switch (stlink_status.state) {
	case STLINK_STATE_CMD: // get a new command
		// TODO check if we get exactly 16 bytes
		switch (ep_buf[0]) {
		case STLINK_GET_VERSION:
			// version = [0]<<8 | [1]
			// version[b0..b5] = SWIM version (7)
			// version[b6..b11] = JTAG version (37)
			// version[b12..b15] = STLINK version (2)
			ep_buf[0] = 0x29;
			ep_buf[1] = 0x47;
			ep_buf[2] = 0x83; // 2 bytes VID
			ep_buf[3] = 0x04;
			ep_buf[4] = 0x48; // 2 bytes PID
			ep_buf[5] = 0x37;
			usbd_ep_write_packet(usbd_dev, 0x81, ep_buf, 6);
			break;
		case STLINK_GET_CURRENT_MODE:
			memset(ep_buf, 0, 2);
			ep_buf[0] = stlink_status.mode;
			ep_buf[1] = 1; // TODO : what's this byte?
			usbd_ep_write_packet(usbd_dev, 0x81, ep_buf, 2);
			break;
		case STLINK_SWIM_COMMAND:
			switch (ep_buf[1]) { // swim subcommand
			case STLINK_SWIM_ENTER:
				stlink_status.mode = STLINK_DEV_SWIM_MODE;
				swim_init(true);
				break;
			case STLINK_SWIM_EXIT:
				swim_exit();
				break;
			case STLINK_SWIM_READ_CAP:
				// ignore for now, need to send 8 bytes, that's all i know
				memset(ep_buf, 0, 8);
				usbd_ep_write_packet(usbd_dev, 0x81, ep_buf, 8);
				break;
			case STLINK_SWIM_SPEED:
				swim_setHighSpeed(ep_buf[2] != 0);
				break;
			case STLINK_SWIM_ENTER_SEQ:
				swim_doEntrySequence();
				break;
			case STLINK_SWIM_GEN_RST:
				swim_srst();
				break;
			case STLINK_SWIM_RESET:
				swim_commsReset();
				break;
			case STLINK_SWIM_ASSERT_RESET:
				swim_assertReset();
				break;
			case STLINK_SWIM_DEASSERT_RESET:
				swim_deassertReset();
				break;
			case STLINK_SWIM_READSTATUS: {
				swimStatusAsync_t swimStatus;
				swim_readStatus(&swimStatus);
				switch (swimStatus.state) {
				case STATE_READY:
					ep_buf[0] = STLINK_SWIM_OK;
					break;
				case STATE_ERROR:
					ep_buf[0] = STLINK_SWIM_NO_RESPONSE;
					break;
				default:
					ep_buf[0] = STLINK_SWIM_BUSY;
					break;
				}
				ep_buf[1] = swimStatus.curBytes & 0xFF;
				ep_buf[2] = (swimStatus.curBytes >> 8) & 0xFF;
				ep_buf[3] = 0;
				usbd_ep_write_packet(usbd_dev, 0x81, ep_buf, 4);
				break;
			}
			case STLINK_SWIM_WRITEMEM:
				stlink_status.totalBytes = ((uint16_t)ep_buf[2] << 8) + ep_buf[3];
				stlink_status.address = (uint32_t)ep_buf[7] + ((uint32_t)ep_buf[6] << 8);
				if (stlink_status.totalBytes > 8) { // more bytes to follow
					stlink_status.state = STLINK_STATE_WRITE;
					stlink_status.curBytes = 8; // maybe len-8 is safer, but len==16 always here
					memcpy(swim_buffer, ep_buf + 8, 8);
				} else { // we have all bytes, so can initiate the WOTF transaction
					memcpy(swim_buffer, ep_buf + 8, stlink_status.totalBytes);
					swim_wotf(stlink_status.address, stlink_status.totalBytes, swim_buffer);
				}
				break;
			case STLINK_SWIM_READMEM:
				stlink_status.totalBytes = ((uint16_t)ep_buf[2] << 8) + ep_buf[3];
				stlink_status.address = (uint32_t)ep_buf[7] + ((uint32_t)ep_buf[6] << 8);
				stlink_status.curBytes = 0;
				swim_rotf(stlink_status.address, stlink_status.totalBytes, swim_buffer);
				break;
			case STLINK_SWIM_READBUF: {
				swimStatusAsync_t swimStatus;
				swim_readStatus(&swimStatus);
				if (swimStatus.state != STATE_READY)
					break; // not ready, so we don't return anything

				if (stlink_status.totalBytes <= 64) {
					usbd_ep_write_packet(usbd_dev, 0x81, swim_buffer, stlink_status.totalBytes);
				} else {
					usbd_ep_write_packet(usbd_dev, 0x81, swim_buffer, 64);
					stlink_status.state = STLINK_STATE_READ;
					stlink_status.curBytes = 64;
					// we haven't returned all data yet, we send the remainder from the main loop
				}
				break;
			}
			case STLINK_SWIM_READ_BUFFERSIZE:
				memset(ep_buf, 0, 2);
				ep_buf[0] = SWIM_BUFFERSIZE & 0xFF;
				ep_buf[1] = (SWIM_BUFFERSIZE >> 8) & 0xFF;
				usbd_ep_write_packet(usbd_dev, 0x81, ep_buf, 2);
				break;
			}
			break;
		case STLINK_DFU_COMMAND:
		default:
			break;
		}
		break;
	case STLINK_STATE_READ:
		// for now we don't buffer commands while we are sending out data from a READBUF
		// or we could simply abort the READBUF operation?
		break;
	case STLINK_STATE_WRITE:
		// we already received STLINK_SWIM_WRITEMEM, but expect more bytes to write
		memcpy(swim_buffer + stlink_status.curBytes, ep_buf, len);
		stlink_status.curBytes += len;
		if (stlink_status.curBytes >= stlink_status.totalBytes) {
			// we have all bytes, so can initiate the WOTF transaction
			stlink_status.state = STLINK_STATE_CMD;
			swim_wotf(stlink_status.address, stlink_status.totalBytes, swim_buffer);
		}
		break;
	}
}
void
swim_poll_read(void)
{

	if (stlink_status.state != STLINK_STATE_READ)
		return;

	uint16_t nbytes = stlink_status.totalBytes - stlink_status.curBytes; /* number of bytes left to read */
	if (nbytes > 64)
		nbytes = 64;
	nbytes = usbd_ep_write_packet(usbdev, 0x81, swim_buffer + stlink_status.curBytes, nbytes);
	/* nbytes = actual number sent to host, will be 0 if ep is busy */
	stlink_status.curBytes += nbytes;
	if (stlink_status.curBytes >= stlink_status.totalBytes)
		stlink_status.state = STLINK_STATE_CMD; /* read is finished */
}

int
main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	/* clock setup */
	/*rcc_clock_setup_in_hse_8mhz_out_72mhz();*/
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* enable peripherals */
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA); /* usb & uart */
	rcc_periph_clock_enable(RCC_GPIOB); /* swim lines */
	rcc_periph_clock_enable(RCC_GPIOC); /* led */

	platform_init();
	platform_timing_init();
#ifdef SWIM_DEBUG
	dbg_usart_setup();
#endif
	cdcacm_init();
	usbuart_init();

	while (true) {
		/*usbd_poll(usbdev);*/
		swim_update();
		swim_poll_read();
	}

	return 0;	/* Should never get here */
}