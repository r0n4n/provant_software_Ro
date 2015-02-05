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
#define MODULE_PERIOD	    120//ms
#define USART_BAUDRATE     115200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
portTickType wakeTime;
USART_TypeDef *USARTn = USART6;

pv_msg_servo oServoMsg;
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
	pv_interface_servo.oServoOutput = xQueueCreate(50,sizeof(pv_msg_servo));
	servo_id=253;
	/* Inicia a usart */
	c_io_herkulex_init(USARTn,USART_BAUDRATE);
	//c_common_utils_delayms(12);
	c_io_herkulex_clear(servo_id);
	//c_common_utils_delayms(12);
	c_io_herkulex_reboot(servo_id);
	c_common_utils_delayms(1000);

	//char *data = (char*)malloc(2*sizeof(char));

	c_io_herkulex_set_torque_control(servo_id,TORQUE_FREE);//torque free

	DATA[0]=1;
	//only reply to read commands
	c_io_herkulex_config_ack_policy(servo_id,1);

	//Acceleration Ratio = 0
	DATA[0]=0;
	c_io_herkulex_write(RAM,servo_id,REG_ACC_RATIO,1,DATA);

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



	//0x7FFE max. pwm
	DATA[1]=0x03;//little endian
	DATA[0]=0xFF;//maximo em 1023
	c_io_herkulex_write(RAM,servo_id,REG_MAX_PWM,2,DATA);

	/** set overload pwm register, if overload_pwm>1023, overload is never
	 * activated this is good for data acquisition, but may not be the case for
	 * the tilt-rotor actualy flying.
	 */
	DATA[0]=0xFF;
	DATA[1]=0x03;//little endian, 2048 sent
	c_io_herkulex_write(RAM,servo_id,REG_OVERLOAD_PWM_THRESHOLD,1,DATA);

	//seta Kp, ki,kd and 1st and 2nd feedforward gains
	//uint8_t i, n=10;// n is the number of bytes to written
	//for(i=0;i<n;i++) {
	//	DATA[i]=0;
	//}
	//c_io_herkulex_write(RAM,servo_id,REG_KP,n,DATA);



	c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);//set torque on




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

	uint8_t status_error=0, status_detail=0;
	uint16_t pwm = 0;
	uint8_t inc = 1;
	float vel=0;
	c_io_herkulex_set_torque(servo_id, pwm);
	//c_common_utils_delayms(12);
	portBASE_TYPE xStatus;
	uint8_t counter = 0;

	while(1)
	{
		wakeTime = xTaskGetTickCount();
		heartBeat++;


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

		if ((heartBeat%40)==0) {
			if (counter == 5) inc=0;
			else if (counter == 0) inc=1;
			if (inc) counter++;
			else counter--;
			pwm=counter*200;
		}
		c_io_herkulex_set_torque(servo_id, pwm);
		//c_common_utils_delayms(1);
		//vel = 0;
		//oServoMsg.angularSpeed=1024;
		//oServoMsg.angularSpeed = c_io_herkulex_read_velocity(servo_id);
		//oServoMsg.pwm=pwm;
		//oServoMsg.heartBeat=heartBeat;
		//oServoMsg.sampleTime=wakeTime-lastWakeTime;
		lastWakeTime=wakeTime;
		//c_io_herkulex_stat(servo_id);
		receive(12);
		status_error=c_io_herkulex_get_status_error();
		status_detail=c_io_herkulex_get_status_detail();
		if (status_error!=0 || status_detail!= 0) {
			c_io_herkulex_clear(servo_id);
		}

		//if(pv_interface_servo.oServoOutput != 0)
			//xStatus = xQueueOverwrite(pv_interface_servo.oServoOutput, &oServoMsg);

		//xStatus = xQueueSend(pv_interface_servo.oServoOutput,&oServoMsg,1/portTICK_RATE_MS);
		xStatus = xQueueSend(pv_interface_servo.oServoOutput,&oServoMsg,1/portTICK_RATE_MS);
		/* toggle pin for debug */
		//c_common_gpio_toggle(debugPin);

		//c_common_utils_delayms(1);
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
