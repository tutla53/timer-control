/* Real Time Clock (RTC) Demo 1
 * Warren Gay Sat Nov 11 10:52:32 2017
 * Uses single rtc_isr() routine.
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

static TaskHandle_t h_task2=0, h_task3=0;
static SemaphoreHandle_t h_mutex;

static volatile unsigned 
	rtc_isr_count = 0u,		// Times rtc_isr() called
	rtc_alarm_count = 0u,		// Times alarm detected
	rtc_overflow_count = 0u;	// Times overflow occurred

static volatile unsigned
	days=0,
	hours=0, minutes=0, seconds=0,
	alarm=0;			// != 0 when alarm is pending
	
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



/*********************************************************************
 * Lock mutex:
 *********************************************************************/

static void
mutex_lock(void) {
	xSemaphoreTake(h_mutex,portMAX_DELAY);
}

/*********************************************************************
 * Unlock mutex:
 *********************************************************************/

static void
mutex_unlock(void) {
	xSemaphoreGive(h_mutex);
}

/*********************************************************************
 * RTC Interrupt Service Routine
 *********************************************************************/

void
rtc_isr(void) {
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
		// RTC tick interrupt:
		rtc_clear_flag(RTC_SEC);

		// Increment time:
		intstatus = taskENTER_CRITICAL_FROM_ISR();
		
		adc0 = read_adc(0) * 330 / 4095;
		
		taskEXIT_CRITICAL_FROM_ISR(intstatus);

		// Wake task2 if we can:
		vTaskNotifyGiveFromISR(h_task2,&woken);
		portYIELD_FROM_ISR(woken);
		return;
	}

	if ( rtc_check_flag(RTC_ALR) ) {
		// Alarm interrupt:
		++rtc_alarm_count;
		rtc_clear_flag(RTC_ALR);

		// Wake task3 if we can:
		vTaskNotifyGiveFromISR(h_task3,&woken);
		portYIELD_FROM_ISR(woken);
		return;
	}
}

/*********************************************************************
 * Set an alarm n seconds into the future
 *********************************************************************/

static void set_alarm(unsigned secs) {
	alarm = (rtc_get_counter_val() + secs) & 0xFFFFFFFF;

	rtc_disable_alarm();
	rtc_set_alarm_time(rtc_get_counter_val() + secs);
	rtc_enable_alarm();
}

/*********************************************************************
 * Task 3 : Alarm
 *********************************************************************/

static void task3(void *args __attribute__((unused))) {

	for (;;) {
		// Block execution until notified
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);		

		mutex_lock();
		std_printf("ALARM at %3u days %02u:%02u:%02u\n",
			days,hours,minutes,seconds);
		mutex_unlock();
	}
}

/*********************************************************************
 * Task 2 : Toggle LED and report time
 *********************************************************************/

static void task2(void *args __attribute__((unused))) {

	for (;;) {
		// Block execution until notified
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);		

		// Toggle LED
		gpio_toggle(GPIOC,GPIO13);

		mutex_lock();
		std_printf("Test: %d.%02d V\n",adc0/100,adc0%100);
		mutex_unlock();
	}
}

/*********************************************************************
 * Initialize RTC for Interrupt processing
 *********************************************************************/

static void
rtc_setup(void) {

	rcc_enable_rtc_clock();
	rtc_interrupt_disable(RTC_SEC);
	rtc_interrupt_disable(RTC_ALR);
	rtc_interrupt_disable(RTC_OW);

	// RCC_HSE, RCC_LSE, RCC_LSI
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

/*********************************************************************
 * Wait until the user presses a key on the terminal program.
 *********************************************************************/

static void
wait_terminal(void) {
	TickType_t ticks0, ticks;

	ticks0 = xTaskGetTickCount();

	for (;;) {
		ticks = xTaskGetTickCount();
		if ( ticks - ticks0 > 2000 ) { // Every 2 seconds
			std_printf("Press any key to start...\n");
			ticks0 = ticks;
		}
		if ( std_peek() >= 1 ) { // Key data pending?
			while ( std_peek() >= 1 )
				std_getc(); // Eat pending chars
			return;
		}
		taskYIELD();		// Give up CPU
	}
}

/*********************************************************************
 * Task 1 : The user console task
 *********************************************************************/

static void
task1(void *args __attribute__((unused))) {
	char ch;

	wait_terminal();
	std_printf("Started!\n\n");

	rtc_setup();	// Start RTC interrupts
	taskYIELD();

	for (;;) {
		mutex_lock();
		std_printf("\nPress 'A' to set 10 second alarm,\n"
			"else any key to read time.\n\n");
		mutex_unlock();
		
		ch = std_getc();

		if ( ch == 'a' || ch == 'A' ) {
			mutex_lock();
			std_printf("\nAlarm configured for 10 seconds from now.\n");
			mutex_unlock();
			set_alarm(10u);
		}
	}
}

/*********************************************************************
 * Main routine : startup
 *********************************************************************/

int
main(void) {

	rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,GPIO_CNF_INPUT_ANALOG,GPIO0);
	adc_setup();

	h_mutex = xSemaphoreCreateMutex();
	xTaskCreate(task1,"task1",350,NULL,1,NULL);
	xTaskCreate(task2,"task2",400,NULL,3,&h_task2);
	xTaskCreate(task3,"task3",400,NULL,3,&h_task3);

	gpio_clear(GPIOC,GPIO13);

#if USE_USB
	usb_start(1,1);
	std_set_device(mcu_usb);		// Use USB for std I/O
#else
	rcc_periph_clock_enable(RCC_GPIOA);	// TX=A9, RX=A10, CTS=A11, RTS=A12
	rcc_periph_clock_enable(RCC_USART1);
	
	gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO9|GPIO11);
	gpio_set_mode(GPIOA,GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,GPIO10|GPIO12);
	open_uart(1,115200,"8N1","rw",1,1);	// UART1 with RTS/CTS flow control
	// open_uart(1,9600,"8N1","rw",0,0);	// UART1 at 9600 baud with no flow control
	std_set_device(mcu_uart1);		// Use UART1 for std I/O
#endif

	vTaskStartScheduler();
	for (;;);

	return 0;
}

// End main.c
