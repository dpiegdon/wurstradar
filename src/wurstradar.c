
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

#include <CMSIS_5/CMSIS/DSP/Include/arm_math.h>
#include <CMSIS_5/CMSIS/DSP/Include/arm_const_structs.h>

#define DEBUG


int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	// Setup PLL and system clock to 168 MHz.
	rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART2);

	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_DMA2);

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
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

static void led_toggle(void)
{
	/* LED on/off */
	gpio_toggle(GPIOA, GPIO8);
}

static void pb6_toggle(void)
{
	gpio_toggle(GPIOB, GPIO6);
}

static void pb14_toggle(void)
{
	gpio_toggle(GPIOB, GPIO14);
}

static void usart_setup(void)
{
	// Setup USART ouput 2Mbaud 8N1 on pin PA2.
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	usart_set_baudrate(USART2, 2000000);
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

static void adc_setup(void)
{
	// Setup ADC1_IN0 on PA0 and ADC2_IN8 on PB0.
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

	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_28CYC);

	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);

	adc_set_left_aligned(ADC1);
	adc_set_left_aligned(ADC2);

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

#ifdef DEBUG
	adc_eoc_after_each(ADC1);
	nvic_enable_irq(NVIC_ADC_IRQ);
	adc_enable_eoc_interrupt(ADC1);
#endif

	// Setup ADC1 DMA transfers via DMA2 Stream 0 Channel 0
	dma_stream_reset(DMA2, DMA_STREAM0);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
	dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_MEDIUM);

	dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

	dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);
	//dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) &ADC_CDR);
	dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) &ADC_DR(ADC1));
	dma_set_number_of_data(DMA2, DMA_STREAM0, WAVESIZE);

	dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
	dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t) waveform1);
	dma_set_memory_address_1(DMA2, DMA_STREAM0, (uint32_t) waveform2);
	dma_set_initial_target(DMA2, DMA_STREAM0, 0);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);

	dma_enable_circular_mode(DMA2, DMA_STREAM0);
	dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);

	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);

	dma_enable_stream(DMA2, DMA_STREAM0);

	adc_power_on(ADC1);
	adc_power_on(ADC2);
}

#ifdef DEBUG
uint32_t adc_sample_counter = 0;
uint32_t adc_csr_flags = 0;
void adc_isr(void)
{
	adc_csr_flags = ADC_CSR;
	ADC_SR(ADC1) = 0;
	pb6_toggle();
	adc_sample_counter += 1;
}

uint32_t dma_sample_counter = 0;
#endif

uint32_t dma_sample_todo = 0;
void dma2_stream0_isr(void)
{
	dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
	pb14_toggle();

	if(0 == dma_get_target(DMA2, DMA_STREAM0))
		waveform_to_process = waveform2;
	else
		waveform_to_process = waveform1;
	waveform_ready = 1;

#ifdef DEBUG
	dma_sample_counter = adc_sample_counter;
	adc_sample_counter = 0;
#endif
	dma_sample_todo = DMA2_SNDTR(DMA_STREAM0);
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

static void dac_output(uint16_t value)
{
	dac_load_data_buffer_single(value, LEFT12, CHANNEL_1);
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

static uint32_t perform_fft(uin32_t * waveform, const uin32_t len)
{
	/* CMSIS DSP FFT requires power-of-two lenghts.
	 * Hence, we create a LUT to select the correct initializer from */
	static const struct {
		uint32_t len;
		arm_cfft_instance_q15* config;
	} len_to_config[] = {
		{16, arm_cfft_sR_q15_len16},
		{32, arm_cfft_sR_q15_len32},
		{64, arm_cfft_sR_q15_len64},
		{128, arm_cfft_sR_q15_len128},
		{256, arm_cfft_sR_q15_len256},
		{512, arm_cfft_sR_q15_len512},
		{1024, arm_cfft_sR_q15_len1024},
		{2048, arm_cfft_sR_q15_len2048},
		{4096, arm_cfft_sR_q15_len4096}
	};

	uint32_t i = 0;
	for(; len_to_config[i].len < len && len <= 4096; ++i);
	arm_cfft_instance_q15* fft_config = len_to_config[i].config;

	/* I understand waveform1/2 should have I and Q samples interleaved inside
	 * each 32bit word once the ADC/DMA contraption is working with double-
	 * buffering enabled.
	 *
	 * The CMSIS DSP FFT thingy requires just this - how convenient.
	 * Thus, we may reinterprete those as little-endian int16_t and pass those
	 * onto cfft function.
	 */
	arm_cfft_q15(fft_config, (q15_t*)waveform, 0, 0);

	return len_to_config[i].len;
}

static uint16_t waveform_magnitudes[WAVESIZE];
static void process_waveform(void)
{
	unsigned i;

	printf("\nbuffer:%p\n", waveform_to_process);
#ifdef DEBUG
	printf("adc csr: %08lx\n", adc_csr_flags);
	printf("got samples: %ld\n", dma_sample_counter);
#endif
	printf("todo: %ld\n", dma_sample_todo);
	for(i = 0; i < 16; ++i)
		printf(" %08lx\n", waveform_to_process[i]);

	/* let's obtain magnitude squares of fft */
	uin32_t processed_len = perform_fft(waveform_to_process, dma_sample_todo);
	arm_cmplx_mag_squared_q15((q15_t*)waveform_to_process,
		waveform_magnitudes, processed_len);

	/* now we need to find those local maxima inside the vector in an efficent
	 * way.
	 */
	printf("fft mag:\n");
	for(i = 0; i < 16; ++i)
		printf(" %08lx\n", waveform_magnitudes[i]);
}

int main(void)
{
	platform_init();

	printf("ETAGE5 WURSTRADAR\n\n");

	adc_start_conversion_regular(ADC1);
	adc_start_conversion_regular(ADC2);

	while (1) {
		if(waveform_ready) {
			waveform_ready = 0;
			wdt_trigger();
			led_toggle();
			process_waveform();
		}
	}

	return 0;
}

