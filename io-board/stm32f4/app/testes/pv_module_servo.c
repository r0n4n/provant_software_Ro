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
#define QUEUE_SIZE 500

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
float position_controller(float r, float y);
float velocity_controller(float r, float y);
float velocity_feedforward(float r);
int16_t saturate(float x, const float max);
int16_t get_stepped_pwm(int heartBeat, int16_t pwm);
int check_outlier(int new,int sec);
/* Private functions ---------------------------------------------------------*/
float position_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	//int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x
	//int K1 = 22.0272, K2=20.6867;// slow motherfucker PI
	int K1 = 11.1838, K2 = -10.8214;
	u=u+K1*e-K2*e_old; //intermediario
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x20.6867
	//int K1 = 15.966, K2=7.9196;// slow motherfucker PI

	u=u+K1*e-K2*e_old;
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_feedforward(float r)
{
	static float y = 0;
	//y=0.5142*(y+r); //original
	y=0.6439*y+0.3561*r; //slow

	return y;
}

int16_t saturate(float x, const float max)
{
	if (x>max)
		x=max;
	else if (x<(-max))
		x=-max;
	if (x>0) x+=0.5;
	if (x<0) x-=0.5;


	return (int16_t)x;
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
//new value, and secure value
int check_outlier(int new, int sec)
{
	int limite = 0.6;
	return ((sec>=0 && (new>=sec*limite)) ||
			(sec<0 && (new<=sec*limite)));
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
	pv_interface_servo.oServoOutput = xQueueCreate(QUEUE_SIZE,
			sizeof(pv_msg_servo));
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
	c_common_utils_delayms(50);
}

/** \brief Função principal do módulo de data out.
  * @param  None			last_vel=new_vel;
  * @retval None
  *
  */
void module_servo_run()
{
	uint32_t heartBeat=0;
	uint8_t status = 0;
	int st =0, el=0;
	uint8_t status_error=0, status_detail=0;
	int16_t pwm = 0;
	float new_vel=0, new_pos = 0, sec_vel = 0, sec_pos = 0;
	c_io_herkulex_set_torque(servo_id, pwm);
	//c_common_utils_delayms(100);
	int32_t data_counter=0;
	int32_t queue_data_counter = 0;
	int32_t lost_data_counter = 0;
	uint8_t data_received = 0;
	c_io_herkulex_set_goal_position(servo_id,0);

	c_io_herkulex_read_data(servo_id);
	float initial_position = c_io_herkulex_get_position(servo_id);

	float ref_pos = initial_position + 5*PI/180;
	portBASE_TYPE xStatus;
	c_common_utils_delayms(1);

	while(1)
	{
		wakeTime = xTaskGetTickCount();
		heartBeat++;


		/**
		 * Teste das escadas
		 */
		//pwm=get_stepped_pwm(heartBeat,pwm);
		//c_io_herkulex_set_torque(servo_id, pwm);

		/**
		 *  Teste do controle de posição interno do servo.
		 */
		/*
		if (heartBeat%50 == 0)
		{
			ref_pos+=5;
			c_io_herkulex_set_goal_position(servo_id,ref_pos);
		}*/


		//resposta ao pulso unitario pwm=1 antes do loop
		//c_io_herkulex_set_torque(servo_id,pwm);
		//pwm=0;

		/**
		 * Leitura de dados
		 */
#if !SERVO_IN_TEST
		if (c_io_herkulex_read_data(servo_id))
		{
			new_vel = c_io_herkulex_get_velocity(servo_id);
			oServoMsg.angularSpeed = new_vel;
			new_pos = c_io_herkulex_get_position(servo_id);
			oServoMsg.position = new_pos;
			oServoMsg.status=1;
			data_counter++;
			data_received=1;
			status_error = c_io_herkulex_get_status_error();
			status_detail = c_io_herkulex_get_status_detail();
			if (status_error) {
				c_io_herkulex_clear(servo_id);
			}
		} else
		{
			data_received = 0;
			oServoMsg.angularSpeed=0;
			oServoMsg.position=0;
			oServoMsg.heartBeat=heartBeat;
			oServoMsg.pwm=0;
			oServoMsg.status=0;
		}


		//float ref_vel=5;

		if (data_received)
		{	//somente atualiza o pwm se receber dados.
			if (check_outlier(new_vel,sec_vel)) {//checar precedencia
		 	 	//Controle de velocidade - funciona, dependendo do controlador
				//
				//float rv = velocity_feedforward(ref_vel);
				//pwm = saturate(velocity_controller(ref_vel,new_vel),1023);
				sec_vel=new_vel;

				 // Controle de posicao

				//float ref_vel = velocity_feedforward(position_controller(ref_pos,
						//oServoMsg.position));
				if (check_outlier(new_pos,sec_pos))
				{
					sec_pos=new_pos;
					float ref_vel = position_controller(ref_pos,new_pos);
					pwm = saturate(velocity_controller(ref_vel,	new_vel),1023);
				}

			}
		}

		//pwm=1023;
		c_io_herkulex_set_torque(servo_id, pwm);


#endif
		//pwm=200;

#if !SERVO_IN_TEST

		/**
		 * Envio dos dados para o modulo de comunicação
		 * Verificação da integridade dos pacotes recebidos
		 */
		status = c_io_herkulex_get_status();//indica erros de comunicação

		if (status)
		{
			oServoMsg.pwm=pwm;
			oServoMsg.heartBeat=heartBeat;
			xStatus = xQueueSend(pv_interface_servo.oServoOutput,
					&oServoMsg,1/portTICK_RATE_MS);
			if (!xStatus) xQueueSend(pv_interface_servo.oServoOutput,
					&oServoMsg,1/portTICK_RATE_MS);

			queue_data_counter++;
		} else
		{
			c_io_herkulex_stat(servo_id);
			status_error=c_io_herkulex_get_status_error();
			status_detail=c_io_herkulex_get_status_detail();
			if (status_error!=0 || status_detail!= 0)
			{
				c_io_herkulex_clear(servo_id);
			}
			lost_data_counter++;
		}


		/**
		 * Para utilização em testes finitos dos servos.
		 * O tamanho da fila(xQueue) indica o numero de pontos acumulados
		 */
		//c_common_utils_delayms(1);=1000
		uint16_t queue_size = uxQueueMessagesWaiting(
				pv_interface_servo.oServoOutput);
		if (queue_size<QUEUE_SIZE)
		{
#endif
			lastWakeTime=wakeTime;
			vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
#if !SERVO_IN_TEST
		} else
		{
			break;
		}
#endif
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


