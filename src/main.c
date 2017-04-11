// ----------------------------------------------------------------------------
//
// Standalone STM32F1 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
//#pragma GCC diagnostic ignored "-Wmissing-declarations"
//#pragma GCC diagnostic ignored "-Wreturn-type"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "task_measuretimer.h"
#include "task_uart.h"
#include "task_adc.h"
int main(void) {
	// At this stage the system clock should have already been configured
	// at high speed.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &gpio);

	SystemInit();
	xTaskCreate(vUart,"UART", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(vADC,"ADC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vMeasureTimer, "MeasT", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	vTaskStartScheduler();
	// Infinite loop
	while (1) {
		// Add your code here.
	}
}
void vApplicationIdleHook( void )
{
}
//#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
