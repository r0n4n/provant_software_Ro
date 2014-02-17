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
#define ARM_MATH_CM4
#include "arm_math.h"

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
#include "c_io_blctrl.h"
#include "c_io_sonar.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIOPin LED_builtin;
GPIOPin LED_Green, LED_Red;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float32_t largetarget_f32[36];

float32_t large_f32[36] =
{
	0.8147,    0.2785,    0.9572,    0.7922,    0.6787,    0.7060,
	0.9058,    0.5469,    0.4854,    0.9595,    0.7577,    0.0318,
	0.1270,    0.9575,    0.8003,    0.6557,    0.7431,    0.2769,
	0.9134,    0.9649,    0.1419,    0.0357,    0.3922,    0.0462,
	0.6324,    0.1576,    0.4218,    0.8491,    0.6555,    0.0971,
	0.0975,    0.9706,    0.9157,    0.9340,    0.1712,    0.8235
};

/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};


void FPU_init(){
	/* Enable FPU.*/
	__asm("  LDR.W R0, =0xE000ED88\n"
		"  LDR R1, [R0]\n"
		"  ORR R1, R1, #(0xF << 20)\n"
		"  STR R1, [R0]");

	//#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//#endif
}

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
	LED_builtin = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);

	char str[64];
	int  channels[4];

    while(1) {
        c_common_gpio_toggle(LED_builtin);
        vTaskDelay(100/portTICK_RATE_MS);
    }
}

void servo_task(void *pvParameters)
{
	char str[64];

    while(1)
    {
    	vTaskDelay(500/portTICK_RATE_MS);
    	sprintf(str, "move 10 -> %d\n\r",c_io_rx24f_move(2,10));
    	c_common_usart_puts(USART2, str); 
    	vTaskDelay(500/portTICK_RATE_MS);
    	sprintf(str, "move 50 -> %d\n\r",c_io_rx24f_move(2,50));
    	c_common_usart_puts(USART2, str); 
    	vTaskDelay(500/portTICK_RATE_MS);
    	c_common_usart_flush(USART6);
    	vTaskDelay(2/portTICK_RATE_MS);
    	sprintf(str, "lido -> %d\n\r",c_io_rx24f_readPosition(2));
        c_common_usart_puts(USART2, str);        
    }

}

// Module RC task
void module_rc_task(void *pvParameters)
{
	module_rc_run();
}

// Module IO taske
void module_io_task(void *pvParameters)
{
	module_io_run();
}

void blctrl_task(void *pvParameters)
{
  char str[30];
  c_common_usart_puts(USART2, "esc task !!\n\r");
  c_io_blctrl_setSpeed(1, 0);
  while(1)
  {
	c_common_usart_puts(USART2, "Starting blctrl_task main loop!");

    vTaskDelay(10/portTICK_RATE_MS);
    c_io_blctrl_updateBuffer(1);

    sprintf(str, "esc rpm: %d \n\r",(int)c_io_blctrl_readSpeed(1) );
    c_common_usart_puts(USART2, str);

    sprintf(str, "esc Voltage: %d \n\r",(int)c_io_blctrl_readVoltage(1) );
    c_common_usart_puts(USART2, str);

    vTaskDelay(100/portTICK_RATE_MS);
    c_io_blctrl_setSpeed(BLCTRL_ADDR, 500);
    //c_io_blctrl_setSpeed(BLCTRL_ADDR+1, 500);
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

void matrix_task(void *pvParameters)
{
	arm_matrix_instance_f32 l;
	arm_matrix_instance_f32 lt;

	arm_mat_init_f32(&l, 6, 6, (float32_t *)large_f32);
	arm_mat_init_f32(&lt, 6, 6, largetarget_f32);
	vu32 it, it2;

	while(1) {
		it = CORE_GetSysTick();
		arm_mat_inverse_f32(&l, &lt);
		it2 = CORE_GetSysTick() - it;
	}
}


/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Init system and trace */
	SystemInit();
	FPU_init();

	vTraceInitTraceData();
	vTraceConsoleMessage("Starting application...");
	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");

	/* Init modules */
	module_io_init(); //IO precisa ser inicializado antes de outros.
	module_rc_init();

	/* Connect modules: interface1.o* = interface2.i* */
	pv_interface_io.oAttitude  = pv_interface_rc.iAttitude;
	pv_interface_rc.oActuation = pv_interface_io.iActuation;

	c_common_usart_puts(USART2, "Iniciando!\n\r");

	/* create tasks */
	xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(module_rc_task, (signed char *)"module_rc", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(module_io_task, (signed char *)"module_io", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	//xTaskCreate(sonar_task, (signed char *)"Sonar task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(matrix_task   , (signed char *)"matrixinv", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(blctrl_task, (signed char *)"blctr", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(servo_task   , (signed char *)"servo", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
