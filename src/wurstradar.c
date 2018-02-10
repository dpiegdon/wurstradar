
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/iwdg.h>

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	// Setup pll and system clock to 168 MHz.
	rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_ADC1);

	rcc_periph_clock_enable(RCC_DAC);
}

static void wdt_setup(void)
{
	iwdg_set_period_ms(500);
	iwdg_start();
}

static void wdt_trigger(void)
{
	iwdg_reset();
}

static void led_setup(void)
{
	// Setup LED1 on PA8.
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
}

static void led_toggle(void)
{
	/* LED on/off */
	gpio_toggle(GPIOA, GPIO8);
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
	// Setup ADC1_IN0 on PA0 and ADC1_IN1 on PA1.
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);

	// FIXME:
	// Setup ADC DMA transfers:
	// ADC1 via DMA2 Stream 0 Channel 0
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
	wdt_setup();
	led_setup();
	usart_setup();
	adc_setup();
	dac_setup();
}

int main(void)
{
	uint64_t i;

	platform_init();

	printf("ETAGE5 WURSTRADAR\nbasic hardware initialized.\n");

	while (1) {
		wdt_trigger();
		led_toggle();
		for (i = 0; i < 2000000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}

	return 0;
}

