
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/timer.h>

#include <arm_math.h>
#include <arm_const_structs.h>

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
	rcc_periph_clock_enable(RCC_DMA2);

	rcc_periph_clock_enable(RCC_TIM4);
	rcc_periph_clock_enable(RCC_DAC);
}

static void wdt_setup(void)
{
	iwdg_set_period_ms(1000);
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

	// additional GPIOs for debugging output
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
}

static void led_toggle(void)
{
	/* LED on/off */
	gpio_toggle(GPIOA, GPIO8);
}

static void pb13_toggle(void)
{
	gpio_toggle(GPIOB, GPIO13);
}

static void pb14_toggle(void)
{
	gpio_toggle(GPIOB, GPIO14);
}

static void pb15_toggle(void)
{
	gpio_toggle(GPIOB, GPIO15);
}

static void usart_setup(void)
{
	// Setup USART ouput on pin PA2.
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	usart_set_baudrate(USART2, 921600);
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

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define WAVESIZE 4096
static uint32_t waveform1[WAVESIZE];
static uint32_t waveform2[WAVESIZE];
static uint32_t * waveform_to_process = NULL;
static volatile int waveform_ready = 0;

static void dma_setup(void)
{
	// Setup ADC1 DMA transfers via DMA2 Stream 0 Channel 0
	dma_stream_reset(DMA2, DMA_STREAM0);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
	dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_MEDIUM);

	dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

	dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_32BIT);
	dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) &ADC_CDR);
	dma_set_number_of_data(DMA2, DMA_STREAM0, WAVESIZE);

	dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
	dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t) waveform1);
	dma_set_memory_address_1(DMA2, DMA_STREAM0, (uint32_t) waveform2);
	dma_set_initial_target(DMA2, DMA_STREAM0, 0);
	dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM0);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);

	dma_enable_circular_mode(DMA2, DMA_STREAM0);
	dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);

	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);

	dma_enable_stream(DMA2, DMA_STREAM0);
}

static void adc_setup(void)
{
	// Setup ADC1_IN0 on PA0 and ADC2_IN8 on PB0.
	// sampling at ~42688 SPS.
	// (actually should be 168/2 / 4 / 480 = 43750 SPS)
	uint8_t adc1_channels[] = { ADC_CHANNEL0 };
	uint8_t adc2_channels[] = { ADC_CHANNEL8 };

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

	adc_power_off(ADC1);
	adc_power_off(ADC2);

	adc_disable_scan_mode(ADC1);
	adc_disable_scan_mode(ADC2);

	adc_disable_external_trigger_regular(ADC1);
	adc_disable_external_trigger_regular(ADC2);

	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY4);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_480CYC);
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_480CYC);

	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);

	adc_set_right_aligned(ADC1);
	adc_set_right_aligned(ADC2);

	adc_set_regular_sequence(ADC1, ARRAY_SIZE(adc1_channels), adc1_channels);
	adc_set_regular_sequence(ADC2, ARRAY_SIZE(adc2_channels), adc2_channels);

	adc_set_continuous_conversion_mode(ADC1);
	adc_set_continuous_conversion_mode(ADC2);

	// Setup dual multi mode for ADC1+ADC2 with DMA
	ADC_CCR &= (~ADC_CCR_MULTI_MASK) | (~ADC_CCR_DMA_MASK);
	ADC_CCR |= ADC_CCR_MULTI_DUAL_REGULAR_SIMUL | ADC_CCR_DMA_MODE_2;

	// enable automatic DMA requests after conversion
	adc_enable_dma(ADC1);

	// continue sending DMA requests even if DMA controller is no longer
	// configured for DMA. needed to support circular buffers
	adc_set_dma_continue(ADC1);

	dma_setup();

	adc_power_on(ADC1);
	adc_power_on(ADC2);
}

uint32_t dma_sample_todo = 0;
void dma2_stream0_isr(void)
{
	dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
	pb13_toggle();

	dma_sample_todo = DMA2_SNDTR(DMA_STREAM0);

	if(0 == dma_get_target(DMA2, DMA_STREAM0))
		waveform_to_process = waveform2;
	else
		waveform_to_process = waveform1;
	waveform_ready = 1;

}

static void pwm_setup(void)
{
	// configure PWM on TIM4CHAN1 on PB6
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOB, GPIO_AF2, GPIO6);

	timer_reset(TIM4);
	// Timer global mode:
	// - Sampling clock divider 1
	// - Alignment edge
	// - Direction up
	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	// Prescaler for input clock (half of the core clock aka 84MHz)
	timer_set_prescaler(TIM4, 1);
	// Enable preload.
	timer_disable_preload(TIM4);
	// Continous mode.
	timer_continuous_mode(TIM4);
	// Period (84MHz/1/65535) = ~ 1282 Hz
	timer_set_period(TIM4, 0xFFFF);
	// Disable outputs.
	timer_disable_oc_output(TIM4, TIM_OC1);
	timer_disable_oc_output(TIM4, TIM_OC2);
	timer_disable_oc_output(TIM4, TIM_OC3);
	timer_disable_oc_output(TIM4, TIM_OC4);
	// OC1 configuration
	// Configure global mode of line 1.
	timer_disable_oc_clear(TIM4, TIM_OC1);
	timer_enable_oc_preload(TIM4, TIM_OC1);
	timer_set_oc_slow_mode(TIM4, TIM_OC1);
	timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(TIM4, TIM_OC1);
	// Set the capture compare value for OC1 to half of the period to achieve 50% duty cycle.
	timer_set_oc_value(TIM4, TIM_OC1, 0xFFFF/2);
	timer_enable_oc_output(TIM4, TIM_OC1);
	timer_enable_preload(TIM4);
	// Counter enable.
	timer_enable_counter(TIM4);
}

static void pwm_output(uint16_t value)
{
	// set duty cycle
	timer_set_oc_value(TIM4, TIM_OC1, value);
}

static void platform_init(void)
{
	clock_setup();
	wdt_setup();
	led_setup();
	usart_setup();
	adc_setup();
	pwm_setup();
}

struct fft_length_config_entry {
	uint32_t len;
	const arm_cfft_instance_q15 * config;
};

static void process_waveform(void)
{
	unsigned i;

	assert(WAVESIZE == 4096);
	arm_cfft_q15(&arm_cfft_sR_q15_len4096, (q15_t*)waveform_to_process, 0, 1);

	int16_t *wav = (int16_t*)waveform_to_process;
	int16_t max_mag=0, max_index=0;
	// NOTE: when direction of movement is required, use i=20..WAVESIZE-20 !
	for(i = 20; i < WAVESIZE/2; ++i) { // omit DC (index 0-3)
		int16_t mag = wav[2*i+0] * wav[2*i+0]  +  wav[2*i+1] * wav[2*i+1];
		if(max_mag < mag) {
			max_mag = mag;
			max_index = i;
		}
	}

	unsigned int speed = max_index;
	uint16_t out = max_index;
	if(max_mag < 0x10)
		out = 0;


// duty cycle value that will show up as peak on analog out
#define PEAK_OUTPUT 0xeb00

#if 0
//  calculations for 45 degree to moving target:
#   define DOPPER_HZ_PER_POINT1_KPH	314
#else
//  calculations for  0 degree to moving target:
#   define DOPPER_HZ_PER_POINT1_KPH	444
#endif

#define HZ_PER_BIN			( 42688. / WAVESIZE )
#define KPH_PER_BIN			( (HZ_PER_BIN) / (DOPPER_HZ_PER_POINT1_KPH/10.) )
#define MAX_BIN_240KPH			( 240. / KPH_PER_BIN )

	speed = (max_index * (int)(HZ_PER_BIN*10) ) / (int)DOPPER_HZ_PER_POINT1_KPH;
	if(out > MAX_BIN_240KPH)
		out = MAX_BIN_240KPH;
	out *= PEAK_OUTPUT / MAX_BIN_240KPH;

#if 1
	printf("fft bin %4d mag^2 %04x speed %u kph out 0x%04x\n", max_index, max_mag, speed, out);
#endif

	pwm_output(out);
}

int main(void)
{
	platform_init();

	printf("ETAGE5 WURSTRADAR\n\n");

	adc_start_conversion_regular(ADC1);
	adc_start_conversion_regular(ADC2);

	while (1) {
		if(waveform_ready) {
			pb13_toggle();
			pb14_toggle();
			waveform_ready = 0;
			wdt_trigger();
			process_waveform();
			pb14_toggle();
			pb15_toggle();
			led_toggle();
		}
	}

	return 0;
}

