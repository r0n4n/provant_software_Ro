/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_servo.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    06/12/2014
  * @brief   Modulo de testes do servos Herkulex DRS0201
  ******************************************************************************/



#include "pv_module_servo.h"
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
#define MODULE_PERIOD	     12//ms
#define USART_BAUDRATE     115200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
USART_TypeDef *USARTn = USART6;

extern xQueueHandle iEscQueueData;
//pv_msg_esc iEscMsgData;
char DATA[100];
int32_t size;// = sizeof(pv_msg_esc);
uint8_t servo_id;

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
void module_servo_init()
{
	servo_id=253;
	/* Inicia a usart2 */
	c_io_herkulex_init(USARTn,USART_BAUDRATE);
	c_common_utils_delayms(12);
	c_io_herkulex_clear(servo_id);
	c_common_utils_delayms(12);
	c_io_herkulex_reboot(servo_id);
	c_common_utils_delayms(1000);

	//char *data = (char*)malloc(2*sizeof(char));

	c_io_herkulex_set_torque_control(servo_id,TORQUE_FREE);//torque free

	DATA[0]=1;
		//only reply to read commands
	c_io_herkulex_config_ack_policy(servo_id,1);

	//set no acceleration time
	DATA[0]=0;
	c_io_herkulex_write(RAM,servo_id,REG_MAX_ACC_TIME,1,DATA);

	DATA[0]=0;
	c_io_herkulex_write(RAM,servo_id,REG_PWM_OFFSET,1,DATA);

	//min pwm = 0
	DATA[0]=0;
	c_io_herkulex_write(RAM,servo_id,REG_MIN_PWM,1,DATA);

	//max pwm >1023 -> no max pwm
	DATA[1]=0x03;//little endian 0x03FF sent
	DATA[0]=0xFF;
	c_io_herkulex_write(RAM,servo_id,REG_MAX_PWM,2,DATA);

	//Acceleration Ratio = MAX
	DATA[0]=0xFF;
	c_io_herkulex_write(RAM,servo_id,REG_ACC_RATIO,1,DATA);

		//0x7FFE max. overload pwm
	DATA[1]=0xFE;//little endian
	DATA[0]=0x7F;
	c_io_herkulex_write(RAM,servo_id,REG_MAX_PWM,2,DATA);

	c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);//torque on




	/* */
	// Reserva o local de memoria compartilhado
	//iEscQueueData = xQueueCreate(1, sizeof(pv_msg_esc));
	// Pin for debug
	//debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_servo_run()
{
	unsigned int heartBeat=0;
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

		c_io_herkulex_set_torque(servo_id, 200);
		/* toggle pin for debug */
		//c_common_gpio_toggle(debugPin);

		//xQueueReceive(iEscQueueData, &iEscMsgData, 0);

		//serialize();
		/*const char str[] = "Ola PC!";
		strcpy(serial_msg,str);
		msg_size=sizeof(serial_msg);
		for (int i = 0; i < msg_size ; ++i)
		    c_common_usart_putchar(USART2,serial_msg[i]);
		//get_raw_String();
*/
		float vel = 0;
		vel = c_io_herkulex_read_velocity(servo_id);

		/* toggle pin for debug */
		//c_common_gpio_toggle(debugPin);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}

/*
void serialize_esc_msg() {
	memcpy(serial_msg,&iEscMsgData,msg_size);
}*/
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
