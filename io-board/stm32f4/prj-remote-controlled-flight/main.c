/**
  ******************************************************************************
  * @file    main/main.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Startup do projeto.
  *
  *	TODO
  *
  * \todo	 1 - Terminar gpio_common.
  * \todo	 2 - Implementar comunicação com servos.
  * \todo	 3 - Implementar comunicação com ESCs.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Std includes */
#include <stdio.h>
#include <math.h>
#include "inttypes.h"

/* Hardware includes. */
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"

/* FreeRTOS+Trace Includes */
#include "trcUser.h"

/* ProVANT Modules */
#include "pv_module_rc.h"
#include "pv_module_io.h"

/* Common Components, FOR TESTING */
#include "c_common_gpio.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIOPin LED;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
	LED = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);

    while(1) {
        c_common_gpio_toggle(LED);
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

// Module RC task
void module_rc_task(void *pvParameters)
{
	while(1) {
		module_rc_run();
	}
}

// Module IO taske
void module_io_task(void *pvParameters)
{
	while(1) {
		module_io_run();
	}
}

void blctrl_task(void *pvParameters)
{
  char str[30];
  c_io_blctrl_setSpeed(BLCTRL_ADDR, 0);
  while(1)
  {
    vTaskDelay(10/portTICK_RATE_MS);
    c_io_blctrl_updateBuffer(BLCTRL_ADDR);

    sprintf(str, "esc rpm: %d \n\r",(int)c_io_blctrl_readSpeed(BLCTRL_ADDR) );
    c_common_usart_puts(USART2, str);

    sprintf(str, "esc Voltage: %d \n\r",(int)c_io_blctrl_readVoltage(BLCTRL_ADDR) );
    c_common_usart_puts(USART2, str);

    vTaskDelay(100/portTICK_RATE_MS);
    c_io_blctrl_setSpeed(BLCTRL_ADDR, 195);
  }
}

void sonar_task(void *pvParameters)
{

  while(1)
  {
    char str[64];
    sprintf(str, "Distance: %d \n\r", c_io_sonar_read());
    c_common_usart_puts(USART2, str);
    vTaskDelay(300/portTICK_RATE_MS);
  }
}


/* PRV -----------------------------------------------------------------------*/
void prvHardwareInit()
{
	c_common_i2c_init();
	c_common_usart2_init(9600);
	c_io_sonar_init();
	//c_io_rx24f_init(1000000);
	//c_rc_receiver_init();
	LED = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Init system and trace */
	SystemInit();

	vTraceInitTraceData();

	vTraceConsoleMessage("Starting application...");
	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");

	/* Init modules */
	module_io_init(); //IO precisa ser inicializado antes de outros.
	module_rc_init();

	/* Connect modules */
	pv_interface_rc.oAngularRefs = pv_interface_io.iServoSetpoints;

	c_common_usart_puts(USART2, "Iniciado!\n\r");

	/* create tasks */
	xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(module_rc_task, (signed char *)"module_rc", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(module_io_task, (signed char *)"module_io", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(sonar_task, (signed char *)"Sonar task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
