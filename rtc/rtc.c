/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Lord James <lordjames@y7mail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/nvic.h>

static void clock_setup(void)
{
	// rcc_clock_setup_in_hse_8mhz_out_24mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	rcc_set_rtc_clock_source( RCC_LSI );

}

static void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	// gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,   GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);


	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup GPIO pins for USART1 receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	// gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);

	/* Setup USART1 TX and RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO10);

	/* Setup UART parameters. */
	// usart_set_baudrate(USART1, 38400);
	/* TODO usart_set_baudrate() doesn't support 24MHz clock (yet). */
	/* This is the equivalent: */
	// USART_BRR(USART1) = (uint16_t)((24000000 << 4) / (38400 * 16));
	usart_set_baudrate(USART1, 19200);

	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	/* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
	// gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,    GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
		gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

}

static void nvic_setup(void)
{
	/* Without this the RTC interrupt routine will never be called. */
	nvic_enable_irq(NVIC_RTC_IRQ);
	nvic_set_priority(NVIC_RTC_IRQ, 1);
}

void rtc_isr(void)
{
	volatile uint32_t j = 0, c = 0;

	/* The interrupt flag isn't cleared by hardware, we have to do it. */
	// rtc_clear_flag(RTC_SEC);

	/* Visual output. */
	gpio_toggle(GPIOA, GPIO8);

	// c = rtc_get_counter_val();

	// /* Display the current counter value in binary via USART1. */
	// for (j = 0; j < 32; j++) {
	// 	if ((c & (0x80000000 >> j)) != 0)
	// 		usart_send_blocking(USART1, '1');
	// 	else
	// 		usart_send_blocking(USART1, '0');
	// }
	// usart_send_blocking(USART1, '\n');
	// usart_send_blocking(USART1, '\r');
}

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();

	usart_send_blocking(USART1, '>');

	rcc_osc_ready_int_clear(RCC_LSI );
	
	/*
	 * If the RTC is pre-configured just allow access, don't reconfigure.
	 * Otherwise enable it with the LSE as clock source and 0x7fff as
	 * prescale value.
	 */
	// rtc_auto_awake(RCC_LSI, 0x7f);
	rtc_unlock();
	dis_alrma();

	rtc_modify_alrma();
	enable_alrma_irq();
	enable_alrma();
	rtc_lock();


	// RTC_CR |= RTC_CR_WUTE;
	// RTC_CR |= RTC_CR_WUCLKSEL_RTC_DIV4;

	// rtc_set_wakeup_time( 0x7fff,RTC_CR_WUCLKSEL_SPRE );
	rtc_set_wakeup_time( 0x10,RTC_CR_WUCLKSEL_SPRE ); // 

	rtc_clear_wakeup_flag();
	 // rtc_set_wakeup_time();

	/* The above mode will not reset the RTC when you press the RST button.
	 * It will also continue to count while the MCU is held in reset. If
	 * you want it to reset, comment out the above and use the following:
	 */
	// rtc_awake_from_off(RCC_LSI);
	// rtc_set_prescale_val(0x7fff);
	// rtc_set_prescaler(256,128);

	/* Setup the RTC interrupt. */
	// nvic_setup();
	// rcc_osc_ready_int_enable(RCC_LSI);

	/* Enable the RTC interrupt to occur off the SEC flag. */
	// rtc_interrupt_enable(RTC_SEC);

	while(1);

	return 0;
}
