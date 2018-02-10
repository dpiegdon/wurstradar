/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Karl Palsson <karlp@tweak.net.au>
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

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	// Setup pll and system clock to 168 MHz.
	rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC2);

	rcc_periph_clock_enable(RCC_DAC);
}

static void led_setup(void)
{
	// Setup LED1 on PA8.
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
}

static void usart_setup(void)
{
	// Setup USART ouput 115200baud 8N1 on pin PA2.
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);
}

/**
 * Use USART2 as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void adc_setup(void)
{
	// Setup ADC1_IN0 on PA0 and ADC2_IN8 on PB0.
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_power_off(ADC2);
	adc_disable_scan_mode(ADC1);
	adc_disable_scan_mode(ADC2);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);
	adc_power_on(ADC2);
}

static void dac_setup(void)
{
	// Setup DAC output on PA4
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
	dac_disable(CHANNEL_1);
	dac_disable_waveform_generation(CHANNEL_1);
	dac_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_SW);
}

static void platform_init(void)
{
	clock_setup();
	led_setup();
	usart_setup();
	adc_setup();
	dac_setup();
}

static void led_toggle(void)
{
	/* LED on/off */
	gpio_toggle(GPIOA, GPIO8);
}

#if 0
static uint16_t read_adc_naiive(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}
#endif

int main(void)
{
	int i;
#if 0
	int j = 0;
#endif

	platform_init();

	printf("ETAGE5 WURSTRADAR\nbasic hardware initialized.\n");

	while (1) {
		led_toggle();
#if 0
		uint16_t input_adc0 = read_adc_naiive(0);
		uint16_t target = input_adc0 / 2;
		dac_load_data_buffer_single(target, RIGHT12, CHANNEL_2);
		dac_software_trigger(CHANNEL_2);
		uint16_t input_adc1 = read_adc_naiive(1);
		printf("tick: %d: adc0= %u, target adc1=%d, adc1=%d\n",
			j++, input_adc0, target, input_adc1);


#endif
		for (i = 0; i < 1000000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}

	return 0;
}

