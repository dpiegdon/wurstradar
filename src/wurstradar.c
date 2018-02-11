
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
	// Setup PLL and system clock to 168 MHz.
	rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC2);

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
	// Setup ADC1_IN0 on PA0 and ADC2_IN8 on PB0.
	uint8_t adc1_channels[] = { ADC_CHANNEL0 };
	uint8_t adc2_channels[] = { ADC_CHANNEL8 };

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_power_off(ADC2);
	adc_power_off(ADC3);

	adc_disable_scan_mode(ADC1);
	adc_disable_scan_mode(ADC2);

	adc_disable_external_trigger_regular(ADC1); // or use TIMx? then only on ADC1. see RefMan pg 401.
	adc_disable_external_trigger_regular(ADC2);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_28CYC);

	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);

	adc_set_left_aligned(ADC1);
	adc_set_left_aligned(ADC2);

	adc_set_regular_sequence(ADC1, 1, adc1_channels);
	adc_set_regular_sequence(ADC2, 1, adc2_channels);

	adc_set_continuous_conversion_mode(ADC1); // XXX
	adc_set_continuous_conversion_mode(ADC2); // XXX

	// Setup dual multi mode for ADC1+ADC2 with DMA
	ADC_CCR &= (~ADC_CCR_MULTI_MASK) | (~ADC_CCR_DMA_MASK);
	ADC_CCR |= ADC_CCR_MULTI_DUAL_REGULAR_SIMUL | ADC_CCR_DMA_MODE_2;
	// XXX => let DMA read data from ADC_CDR; read status from ADC_CSR. see RefMan pg 401.

	// enable automatic DMA requests after conversion
	adc_enable_dma(ADC1);

	// continue sending DMA requests even if DMA controller is no longer
	// configured for DMA. needed to support circular buffers
	adc_set_dma_continue(ADC1);

	// FIXME:
	// Setup ADC DMA transfers:
	// ADC1 via DMA2 Stream 0 Channel 0


	// XXX for debugging:
	adc_enable_eoc_interrupt(ADC1);
	// XXX for debugging.

	adc_power_on(ADC1);
	adc_power_on(ADC2);
}

static unsigned adcvalue; // XXX for debugging

void adc_isr(void)
{
	ADC_SR(ADC1) = 0;
	adcvalue = ADC_CDR;
	led_toggle();
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

static void nvic_setup(void)
{
	nvic_enable_irq(NVIC_ADC_IRQ);
}

static void platform_init(void)
{
	clock_setup();
	wdt_setup();
	led_setup();
	usart_setup();
	adc_setup();
	dac_setup();
	nvic_setup();
}

int main(void)
{
	uint64_t i;

	platform_init();

	adc_start_conversion_regular(ADC1);

	printf("ETAGE5 WURSTRADAR\nbasic hardware initialized.\n");

	while (1) {
		wdt_trigger();
		//led_toggle();
		for (i = 0; i < 2000000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
		printf("0x%08x\n", adcvalue);
	}

	return 0;
}

