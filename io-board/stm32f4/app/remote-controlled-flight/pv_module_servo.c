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
#include <assert.h>

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
#define MODULE_PERIOD	    12//ms
#define USART_BAUDRATE     115200
#define QUEUE_SIZE 200

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
int16_t get_stepped_pwm(int heartBeat, int16_t pwm);
int16_t velocity_controller(float r, float y);
int16_t velocity_feedforward(float r);
int16_t saturate(float x, const float max);
/* Private functions ---------------------------------------------------------*/
int16_t velocity_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;

	u=u+619.4*e-318.4955*e_old;
	e_old=e;
	//saturacao
	int16_t out = saturate(u,1023);

	assert(out>=(-1023) && out<=1023);

	return out;
}

int16_t velocity_feedforward(float r)
{
	static float y = 0, r_old = 0;
	y=0.5142*(y+r_old);
	r_old=r;

	return saturate(y,9);
}

int16_t saturate(float x, const float max)
{
	if (x>max) x=max;
	else if (x<(-max)) x=-max;

	return (int16_t)(x+0.5);
}

int16_t get_stepped_pwm(int heartBeat, int16_t pwm)
{
  static int8_t counter = 0, inc=1;

  //parametros da escada
  const int STEP_LENGTH = 50; //tamanho do degraus em ptos de amostragem
  const int STEP_SIZE = 200; //amplitude dos degraus
  const int NUM_STEPS = 5; //numero de degraus

  if ((heartBeat%STEP_LENGTH)==0)
  {
    if (counter == NUM_STEPS) inc=0;
    else if (counter == -NUM_STEPS) inc=1;
    if (inc) counter++;
    else counter--;
    pwm=counter*200;
    //if (pwm) pwm=0;
    //else pwm=1000;
  }

  return pwm;
}
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_servo_init()
{
	pv_interface_servo.oServoOutput = xQueueCreate(QUEUE_SIZE,sizeof(pv_msg_servo));
	servo_id=253;
	/* Inicia a usart */
	c_io_herkulex_init(USARTn,USART_BAUDRATE);
	//c_common_utils_delayms(12);
	c_io_herkulex_clear(servo_id);
	//c_common_utils_delayms(12);
	c_io_herkulex_reboot(servo_id);
	c_common_utils_delayms(1000);

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
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_servo_run()
{
	unsigned int heartBeat=0;
	uint8_t status = 0;

	uint8_t status_error=0, status_detail=0;
	uint16_t pwm = 0, pos = 0;
	float vel=0;
	//c_io_herkulex_set_torque(servo_id, pwm);
	//c_io_herkulex_set_goal_position(servo_id,pos);
	//c_common_utils_delayms(12);
	portBASE_TYPE xStatus;


	while(1)
	{
		wakeTime = xTaskGetTickCount();
		heartBeat++;


		//pwm=get_stepped_pwm(heartBeat,pwm);
		//pwm=500;
		//c_io_herkulex_set_torque(servo_id, pwm);

		/* Teste do controle de posição interno do servo.
		if (heartBeat%200 == 0)
		{
			pos+=5;
			c_io_herkulex_set_goal_position(servo_id,pos);
		}
		*/

		/**
		 * Leitura de dados
		 */
		//oServoMsg.angularSpeed=1024;
		oServoMsg.angularSpeed = c_io_herkulex_read_velocity(servo_id);
		//oServoMsg.position = c_io_herkulex_read_position(servo_id);

		/**
		 * Controle
		 */
		float ref = 1;
		//pwm=500;
		pwm = velocity_controller(velocity_feedforward(ref),oServoMsg.angularSpeed);
		c_io_herkulex_set_torque(servo_id, pwm);


		/**
		 * Envio dos dados para o modulo de comunicação
		 * Verificação da integridade dos pacotes recebidos
		 */
		status = c_io_herkulex_get_status();//status verifica meramente erros de comunicação
		if (status)
		{
			oServoMsg.pwm=pwm;
			oServoMsg.heartBeat=heartBeat;
			//ServoMsg.angularSpeed = 0;
			xStatus = xQueueSend(pv_interface_servo.oServoOutput,&oServoMsg,1/portTICK_RATE_MS);
		} else
		{
			c_io_herkulex_stat(servo_id);
			status_error=c_io_herkulex_get_status_error();
			status_detail=c_io_herkulex_get_status_detail();
			if (status_error!=0 || status_detail!= 0)
			{
				c_io_herkulex_clear(servo_id);
			}
		}


		/**
		 * Para utilização em testes finitos dos servos.
		 * O tamanho da fila(xQueue) indica o numero de pontos acumulados
		 */
		//c_common_utils_delayms(1);=1000
		uint16_t queue_size = uxQueueMessagesWaiting(pv_interface_servo.oServoOutput);
		if (queue_size<QUEUE_SIZE)
		{
			lastWakeTime=wakeTime;
			vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
		} else
		{
			break;
		}
	}
	c_io_herkulex_set_torque(servo_id, 0);
	c_io_herkulex_set_torque_control(servo_id,TORQUE_BREAK);//set torque free
	vTaskDelete(NULL);
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


