/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_serial.c
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    10/11/2014
  * @brief   implementacao do modulo de envio de dados de teste via UART
  ******************************************************************************/

#include "pv_module_serial.h"
#include "c_io_herkulex.h"
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
//#define USART_BAUDRATE     460800
#define USART_BAUDRATE     115200


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
USART_TypeDef *USARTx = USART2;
//extern xQueueHandle iEscQueueData;
pv_msg_servo iServoOutput;
uint8_t BUFFER[100];
int32_t msg_size;// = sizeof(pv_msg_esc);
//pv_msg_input iInputData;
//pv_msg_controlOutput iControlOutputData;
//GPIOPin debugPin;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void send_data(uint8_t size);
void serialize_servo_msg(pv_msg_servo msg);
uint8_t checksum(uint8_t buffer[]);
void stub();
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
	if (USARTx==USART1) {
		c_common_usart1_init(USART_BAUDRATE);
	} else if (USARTx==USART2) {
		c_common_usart2_init(USART_BAUDRATE);
	} else if (USARTx==USART3) {
		c_common_usart3_init(USART_BAUDRATE);
	} else if (USARTx==USART6) {
		c_common_usart6_init(USART_BAUDRATE);
	}
	//stub();
	c_io_herkulex_init(USARTx,USART_BAUDRATE);



	/* Reserva o local de memoria compartilhado */
	//iEscQueueData = xQueueCreate(1, sizeof(pv_msg_esc));

	/* Pin for debug */
	//debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);
}

void stub() {
	//codigo de teste
	BUFFER[0]=0xFF;
	BUFFER[1]=0xFF;
	BUFFER[2]=0x0E;//13 valores fora o
	float value=  3.14159265;
	float value1=  3.14159265/2.0;
	float value2=  3.14159265/4.0;
	memcpy((BUFFER+3),&value,4);
	memcpy((BUFFER+7),&value1,4);
	memcpy((BUFFER+11),&value2,4);
	BUFFER[15]=checksum(BUFFER);//0 a 14, 15 bytes
}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_serial_run()
{
	unsigned int heartBeat=0;
	portBASE_TYPE xStatus;
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;
#if SERIAL_TESTE
		//stub();
		//send_data(BUFFER[2]+2);
		uint8_t r = receive(12);
		xStatus=r;
#else
		xStatus = xQueueReceive(pv_interface_serial.iServoOutput,&iServoOutput,1/portTICK_RATE_MS);
		/*if (xStatus!= pdPASS) {
			BUFFER[0]=0xFF;
			BUFFER[1]=0xFF;
			BUFFER[2]=5;
			BUFFER[3]=3;
			BUFFER[4]=14;
			BUFFER[5]=15;
			BUFFER[6]=checksum(BUFFER);
			send_data(BUFFER[2]+2);// tamanha dos dados mais cabeçalho
		}*/
		//iServoOutput.angularSpeed=10.0;
		//iServoOutput.heartBeat=53;
		//iServoOutput.sampleTime=12;
		if (xStatus == pdPASS) {
			serialize_servo_msg(iServoOutput);
			send_data(BUFFER[2]+2);
		}
#endif

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}






uint8_t checksum(uint8_t buffer[]) {
  uint8_t i, chksum=0;
  uint8_t n = BUFFER[2]-1+2;//-1-> tira o checksum  2->por começar em buffer[2]
  /* o buffer é passado inteiro, com o cabeçalho */
  for(i=2;i<n;i++) {
    chksum=chksum^buffer[i];
  }

  return chksum&0xFE;
}


void serialize_servo_msg(pv_msg_servo msg) {
	int msg_size = sizeof(pv_msg_servo);
	BUFFER[0]=0xFF;
	BUFFER[1]=0xFF;
	BUFFER[2]=msg_size+2;
	memcpy((BUFFER+3),&msg,msg_size);
	BUFFER[msg_size+3]=checksum(BUFFER);
}

void send_data(uint8_t size) {
	for (int i = 0; i < size ; ++i)
		c_common_usart_putchar(USARTx,BUFFER[i]);
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
