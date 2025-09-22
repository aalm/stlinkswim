/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
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

#include "general.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>

#include "cdcacm.h"
#include "usbuart.h"

#define	USBUART_TIMER_FREQ_HZ	1000000U	/* 1us per tick */
#define	USBUART_RUN_FREQ_HZ	5000U		/* 200us (or 100 characters at 2Mbps) */

#define	FIFO_SIZE	128

/* RX Fifo buffer */
static uint8_t buf_rx1[FIFO_SIZE];

/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx1_in;

/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx1_out;

static void usbuart_run(uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx);

void
usbuart_init(void)
{
	rcc_periph_clock_enable(RCC_USART1);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, /*9600*/1200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USARTs. */
	usart_enable(USART1);

	/* Enable interrupts */
	USART1_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(NVIC_USART1_IRQ, IRQ_PRI_USBUSART);
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup timer for running deferred FIFO processing */
	rcc_periph_clock_enable(RCC_TIM3);
	/*timer_reset(TIM3);*/
	rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST);
	rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST);

	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_set_prescaler(TIM3, rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);

	timer_set_period(TIM3, USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);

	/* Setup update interrupt in NVIC */
	nvic_set_priority(NVIC_TIM3_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_enable_irq(NVIC_TIM3_IRQ);

	/* turn the timer on */
	timer_enable_counter(TIM3);
}

/*
 * Runs deferred processing for usb uart rx, draining RX FIFO by sending
 * characters to host PC via CDCACM.  Allowed to read from FIFO in pointer,
 * but not write to it. Allowed to write to FIFO out pointer.
 */
static void
usbuart_run(uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	/* forcibly empty fifo if no USB endpoint */
	if (cdcacm_configured != 1)
		*buf_rx_out = *buf_rx_in;

	/* if fifo empty, nothing further to do */
	if (*buf_rx_in == *buf_rx_out) {
		/* turn off LED, disable IRQ */
		timer_disable_irq(TIM3, TIM_DIER_UIE);
		gpio_clear(LED_PORT_UART, LED_UART);
	} else {
		uint8_t pkt_buf[64];
		uint8_t pkt_size = 0;
		uint8_t buf_out = *buf_rx_out;

		/* copy from uart FIFO into local usb packet buffer */
		while (*buf_rx_in != buf_out && pkt_size < 64) {
			pkt_buf[pkt_size++] = buf_rx[buf_out++];

			/* wrap out pointer */
			if (buf_out >= FIFO_SIZE)
				buf_out = 0;
		}

		/* advance fifo out pointer by amount written */
		*buf_rx_out += usbd_ep_write_packet(usbdev, 0x05, pkt_buf, pkt_size);
		*buf_rx_out %= FIFO_SIZE;
	}
}

void
usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{

	usart_set_baudrate(USART1, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USART1, coding->bDataBits + 1);
	else
		usart_set_databits(USART1, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USART1, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USART1, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(USART1, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USART1, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USART1, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(USART1, USART_PARITY_EVEN);
		break;
	}
}

static void
usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(dev, 0x05, buf, 64);

	gpio_set(LED_PORT_UART, LED_UART);
	for (int i = 0; i < len; i++)
		usart_send_blocking(USART1, buf[i]);
	gpio_clear(LED_PORT_UART, LED_UART);
}

void
usbuart1_usb_out_cb(usbd_device *dev, uint8_t ep)
{
	usbuart_usb_out_cb(dev, ep);
}

void
usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
	(void)dev;
	(void)ep;
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
static void
usart_isr(uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	uint32_t err = USART_SR(USART1);
	char c = usart_recv(USART1);
	if (err & (USART_SR_ORE | USART_SR_FE))
		return;

	/* Turn on LED */
	gpio_set(LED_PORT_UART, LED_UART);

	/* If the next increment of rx_in would put it at the same point
	* as rx_out, the FIFO is considered full.
	*/
	if (((*buf_rx_in + 1) % FIFO_SIZE) != *buf_rx_out) {
		/* insert into FIFO */
		buf_rx[(*buf_rx_in)++] = c;

		/* wrap out pointer */
		if (*buf_rx_in >= FIFO_SIZE) {
			*buf_rx_in = 0;
		}

		/* enable deferred processing if we put data in the FIFO */
		timer_enable_irq(TIM3, TIM_DIER_UIE);
	}
}

void
usart1_isr(void)
{
	usart_isr(&buf_rx1_out, &buf_rx1_in, buf_rx1);
}

void
tim3_isr(void)
{
	/* need to clear timer update event */
	timer_clear_flag(TIM3, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run(&buf_rx1_out, &buf_rx1_in, buf_rx1);
}