/* Real Time Clock (RTC)
   Read ADC every 10 ms
   
   by: Tutla Ayatullah
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mcuio.h"
#include "miniprintf.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>

#define USE_USB		1		// Set to 1 for USB

static TaskHandle_t 		h_task2=0;
static SemaphoreHandle_t 	h_mutex;

static volatile unsigned 
	rtc_isr_count = 0u,			// Times rtc_isr() called
	rtc_alarm_count = 0u,		// Times alarm detected
	rtc_overflow_count = 0u,	// Times overflow occurred
	alarm =0;

int adc0 = 0;

static void adc_setup(void){
	
	/*Setup ADC*/
	rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);
	adc_power_off(ADC1);
	rcc_peripheral_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
	rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);	// Set. 12MHz, Max. 14MHz
	adc_set_dual_mode(ADC_CR1_DUALMOD_IND);		// Independent mode
	adc_disable_scan_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1,ADC_CHANNEL_TEMP,ADC_SMPR_SMP_239DOT5CYC);
	adc_set_sample_time(ADC1,ADC_CHANNEL_VREF,ADC_SMPR_SMP_239DOT5CYC);
	adc_enable_temperature_sensor();
	adc_power_on(ADC1);
	adc_reset_calibration(ADC1);
	adc_calibrate_async(ADC1);
	while ( adc_is_calibrating(ADC1) );
}

static uint16_t read_adc(uint8_t channel) {

	/*Read ADC*/
	adc_set_sample_time(ADC1,channel,ADC_SMPR_SMP_239DOT5CYC);
	adc_set_regular_sequence(ADC1,1,&channel);
	adc_start_conversion_direct(ADC1);
	while ( !adc_eoc(ADC1) )
		taskYIELD();
	return adc_read_regular(ADC1);
}

static void rtc_setup(void) {

	rcc_enable_rtc_clock();
	rtc_interrupt_disable(RTC_SEC);
	rtc_interrupt_disable(RTC_ALR);
	rtc_interrupt_disable(RTC_OW);

	// Avaiable Clock Source: RCC_HSE, RCC_LSE, RCC_LSI
	rtc_awake_from_off(RCC_HSE); 
	rtc_set_prescale_val(625);
	rtc_set_counter_val(0x00000000);

	nvic_enable_irq(NVIC_RTC_IRQ);

	cm_disable_interrupts();
	rtc_clear_flag(RTC_SEC);
	rtc_clear_flag(RTC_ALR);
	rtc_clear_flag(RTC_OW);
	rtc_interrupt_enable(RTC_SEC);
	rtc_interrupt_enable(RTC_ALR);
	rtc_interrupt_enable(RTC_OW);
	cm_enable_interrupts();
}

static void mutex_lock(void) {
	xSemaphoreTake(h_mutex,portMAX_DELAY);
}

static void mutex_unlock(void) {
	xSemaphoreGive(h_mutex);
}

void rtc_isr(void) {
	
	/*Interrupt Services Routine */
	UBaseType_t intstatus;
	BaseType_t woken = pdFALSE;

	++rtc_isr_count;
	if ( rtc_check_flag(RTC_OW) ) {
		// Timer overflowed:
		++rtc_overflow_count;
		rtc_clear_flag(RTC_OW);
		if ( !alarm ) // If no alarm pending, clear ALRF
			rtc_clear_flag(RTC_ALR);
	} 

	if ( rtc_check_flag(RTC_SEC) ) {

		rtc_clear_flag(RTC_SEC); 					// RTC tick interrupt
		intstatus = taskENTER_CRITICAL_FROM_ISR();	// increment time
		
		adc0 = read_adc(0) * 330 / 4095;			/*ISR main program*/
		
		taskEXIT_CRITICAL_FROM_ISR(intstatus);

		vTaskNotifyGiveFromISR(h_task2,&woken);		// wake task2 to print adc value
		portYIELD_FROM_ISR(woken);
		return;
	}

}


/*********************************************************************
 * Task 2 : Toggle LED and report time
 *********************************************************************/

static void task2(void *args __attribute__((unused))) {

	for (;;) {
		
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);					// Block execution until notified

		gpio_toggle(GPIOC,GPIO13);								// Toggle LED

		mutex_lock();
		std_printf("Voltage: %d.%02d V\n",adc0/100,adc0%100);	// print data after ISR
		mutex_unlock();
	}
}

/*********************************************************************
 * Wait until the user presses a key on the terminal program.
 *********************************************************************/

static void wait_terminal(void) {
	TickType_t ticks0, ticks;

	ticks0 = xTaskGetTickCount();			// similar to millis() arduino

	for (;;) {
		ticks = xTaskGetTickCount();
		if ( ticks - ticks0 > 1000 ) { 		// Every 2 seconds
			std_printf("Press any key to start...\n");
			ticks0 = ticks;
		}
		if ( std_peek() >= 1 ) { 			// Key data pending?
			while ( std_peek() >= 1 )
				std_getc(); 				// Eat pending chars
			return;
		}
		taskYIELD();						// Give up CPU
	}
}

/*********************************************************************
 * Task 1 : The user console task
 *********************************************************************/

static void task1(void *args __attribute__((unused))) {

	wait_terminal();
	std_printf("Started!\n\n");

	rtc_setup();	// Start RTC interrupts
	taskYIELD();

	for (;;) {

	}
}

static void gpio_setup(void){
	/*
		LED = PC13 (digital output)
		Joystick = PA0 (adc)
	*/
	
	rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"
	
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO0);
	
}

int main(void) {

	gpio_setup();
	adc_setup();

	h_mutex = xSemaphoreCreateMutex();
	xTaskCreate(task1,"task1",350,NULL,1,NULL);
	xTaskCreate(task2,"task2",400,NULL,3,&h_task2);

	gpio_clear(GPIOC,GPIO13);

#if USE_USB
	usb_start(1,1);
	std_set_device(mcu_usb);							// Use USB for std I/O
#else
	rcc_periph_clock_enable(RCC_GPIOA);					// TX=A9, RX=A10, CTS=A11, RTS=A12
	rcc_periph_clock_enable(RCC_USART1);
	
	gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO9|GPIO11);
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,GPIO10|GPIO12);
	open_uart(1,115200,"8N1","rw",1,1);					// UART1 with RTS/CTS flow control
	std_set_device(mcu_uart1);							// Use UART1 for std I/O
#endif

	vTaskStartScheduler();
	for (;;);

	return 0;
}
