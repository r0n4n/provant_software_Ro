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

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIOPin LED;

/* Private define ------------------------------------------------------------*/
// Sys-Tick Counter - Messen der Anzahl der Befehle des Prozessors:
#define CORE_SysTickEn()    (*((u32*)0xE0001000)) = 0x40000001
#define CORE_SysTickDis()   (*((u32*)0xE0001000)) = 0x40000000
#define CORE_GetSysTick()   (*((u32*)0xE0001004))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float32_t A_f32[4] =
{
  3.0,     5.0,
  9.0,     7.0
};

float32_t B_f32[4] =
{
  1.0,     1.0,
  1.0,     1.0
};

float32_t C_f32[4];
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

void matrix_task(void *pvParameters)
{
	arm_matrix_instance_f32 l;
	arm_matrix_instance_f32 lt;

	arm_mat_init_f32(&l, 6, 6, (float32_t *)large_f32);
	arm_mat_init_f32(&lt, 6, 6, largetarget_f32);

	while(1) {
		vu32 it = CORE_GetSysTick();
		arm_mat_inverse_f32(&l, &lt);
		vu32 it2 = CORE_GetSysTick() - it;
	}
}


arm_matrix_instance_f32 A;
arm_matrix_instance_f32 B;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 l;
arm_matrix_instance_f32 lt;

/* Main ----------------------------------------------------------------------*/
int main(void)
{
  /* Enable FPU.*/
  __asm("  LDR.W R0, =0xE000ED88\n"
		"  LDR R1, [R0]\n"
		"  ORR R1, R1, #(0xF << 20)\n"
		"  STR R1, [R0]");

	/* Init system and trace */
	SystemInit();

	//*(uint32_t *) (0xE000ED88) |= 0X00F00000;
	//__DSB();
	//__ISB();
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	/******MESSAROUND******************/
	float f = 1.01f;
	int   i = 53425;
	CORE_SysTickEn();
	vu32 it = CORE_GetSysTick();
	float f2 = f + 2.29f;
	vu32 it2 = CORE_GetSysTick() - it;
	float f3 = f / 2.29f;
	vu32 it3 = CORE_GetSysTick() - it - it2;



	arm_mat_init_f32(&A, 2, 2, (float32_t *)A_f32);
	arm_mat_init_f32(&B, 2, 2, (float32_t *)B_f32);
	arm_mat_init_f32(&C, 2, 2, C_f32);

	arm_mat_init_f32(&l, 6, 6, (float32_t *)large_f32);
	arm_mat_init_f32(&lt, 6, 6, largetarget_f32);

	arm_mat_add_f32(&A, &B, &C);
	arm_mat_inverse_f32(&A, &C);

	//it = CORE_GetSysTick();
	//arm_mat_inverse_f32(&l, &lt);
	//it2 = CORE_GetSysTick() - it;
	/******MESSAROUND******************/

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
	xTaskCreate(matrix_task   , (signed char *)"matrixinv", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
