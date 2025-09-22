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

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

static volatile uint32_t time_ms;
static volatile uint32_t _tick_inc;

void
platform_init(void)
{

	/* Relocate interrupt vector table here */
	extern int vector_table;
	SCB_VTOR = (uint32_t)&vector_table;
}

void
platform_timing_init(void)
{

	time_ms = 0;
	_tick_inc = 1;

	/* Setup heartbeat timer */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
#if 0
	systick_set_reload(900000);	/* Interrupt us at 10 Hz */
#else
#if 0
	uint32_t _reload = systick_get_calib() / 10; /* XXX _reload - 1 ? */
	systick_set_reload(_reload/*rcc_ahb_frequency / (8 * 1000) - 1*/);
#else
	if (systick_set_frequency(1000, rcc_ahb_frequency)) {
		_tick_inc = 1;
	} else
	if (systick_set_frequency(100, rcc_ahb_frequency)) {
		_tick_inc = 10;
	} else
	if (systick_set_frequency(10, rcc_ahb_frequency)) {
		_tick_inc = 100;
	} else {
		systick_set_reload(900000);	/* Interrupt us at 10 Hz */
		_tick_inc = 100;
	}
#endif
	systick_clear();
#endif
#if 0
	SCB_SHPR(SCB_SHPR_PRI_15_SYSTICK) &= ~((15 << 4) & 0xff);
	SCB_SHPR(SCB_SHPR_PRI_15_SYSTICK) |= ((14 << 4) & 0xff);
#else
	nvic_set_priority(NVIC_SYSTICK_IRQ, 14 << 4);
#endif
	systick_interrupt_enable();
	systick_counter_enable();
}

void
sys_tick_handler(void)
{

#if 0
	time_ms += 1/*00*/;
#else
	time_ms += _tick_inc;
#endif
}

uint32_t
platform_time_ms(void)
{
	return time_ms;
}