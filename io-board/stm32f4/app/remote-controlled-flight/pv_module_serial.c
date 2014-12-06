/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_serial.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    10/11/2014
  * @brief   implementacao do modulo de envio de dados de teste via USB
  ******************************************************************************/

#include "pv_module_serial.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_do
  * \brief Módulo responsavel por transmitir dados.
  *
  * Definição do módulo de transmissão de dados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	     100//ms
//#define USART_BAUDRATE     460800
#define USART_BAUDRATE     115200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;

extern xQueueHandle iEscQueueData;
pv_msg_esc iEscMsgData;
char serial_msg[300];
int32_t msg_size;// = sizeof(pv_msg_esc);
//pv_msg_input iInputData;
//pv_msg_controlOutput iControlOutputData;
//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_serial_init()
{
	/* Inicia a usart2 */
	c_common_usart6_init(USART_BAUDRATE);

	/* Reserva o local de memoria compartilhado */
	iEscQueueData = xQueueCreate(1, sizeof(pv_msg_esc));





	/* Pin for debug */
	//debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_serial_run()
{
	unsigned int heartBeat=0;
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

		/* toggle pin for debug */
		//c_common_gpio_toggle(debugPin);

		//xQueueReceive(iEscQueueData, &iEscMsgData, 0);

		//serialize();
		const char str[] = "Ola PC!";
		strcpy(serial_msg,str);
		msg_size=sizeof(serial_msg);
		for (int i = 0; i < msg_size ; ++i)
		    c_common_usart_putchar(USART6,serial_msg[i]);
		//get_raw_String();

		/* toggle pin for debug */
		//c_common_gpio_toggle(debugPin);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}


void serialize_esc_msg() {
	memcpy(serial_msg,&iEscMsgData,msg_size);
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
