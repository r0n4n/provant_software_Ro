/**
  ******************************************************************************
  * @file    modules/io/c_io_herkulex.h
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    01/12/2014
  * @brief   Implementação da Servo Herkulex DRS-0201
  *****************************************************************************/

#include "c_io_herkulex.h"
#include "c_common_uart.h"

#include <math.h>

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_Herkulex
  *	\brief Componente para o uso dos servos Herkulex DRS- 0201
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* #define KV (0.325*PI/(0.0112*180))
#define TIME_OUT 10
#define INTER_PKG_TIME 0.000105
#define HEADER 0xFF
 status error register, 48
#define EXCEED_INPUT_VOLT_LIMIT 0x01
#define EXCEED_ALLOWED_POT_LIMIT 0x02
#define EXCEED_TEMP_LIMIT 0x04
#define INVALID_PACKET 0x08
#define OVERLOAD 0x10
#define DRIVER_FAULT 0x20
#define EEP_REG_DISTORTED 0x40

/* status error register, 49
#define MOVING_FLAG 0x01
#define INPOSITION_FLAG 0x02
/* invalid packet errors
#define CHECKSUM_ERROR 0x04
#define UNKNOWM_COMMAND 0x08
#define EXCEED_REG_RANGE 0x10
#define GARBAGE_DETECTED 0x20
/* end of packet errors
#define MOTOR_ON_FLAG 0x40

/* Address of registers in RAM
#define REG_SERVO_ID 0
#define REG_ACK_POLICY 1
#define REG_ALARM_LED_POLICY 2
#define REG_TORQUE_POLICY 3
#define REG_MAX_TEMP 5
#define REG_MIN_VOLT 6
#define REG_MAX_VOLT 7
#define REG_ACC_RATIO 8
#define REG_MAX_ACC_TIME 9
#define REG_DEAD_ZONE 10
#define REG_SATURATOR_OFFSET 11
#define REG_SATURATOR_SLOPE 12
#define REG_PWM_OFFSET 14
#define REG_MIN_PWM 15
#define REG_MAX_PWM 16
#define REG_OVERLOAD_PWM_THRESHOLD 18
#define REG_MIN_POS 20
#define REG_MAX_POS 22

/* some other registers from voltatile RAM
#define REG_INPOSITION_MARGIN 44
#define REG_CALIBRATION_DIFF 47
#define REG_STATUS_ERROR 48
#define REG_STATUS_DETAIL 49
#define REG_TORQUE_CONTROL 52
#define REG_LED_CONTROL 53
#define REG_VOLT 54
#define REG_TEMP 55
#define REG_CURRENT_CONTROL_MODE 56
#define REG_TICK 57
#define REG_CALIBRATED_POS 58
#define REG_ABSOLUTE_POS 60
#define REG_DIFFERENTIAL_POS 62
#define REG_PWM 64
#define REG_ABSOLUTE_GOAL_POS 68
#define REG_DESIRED_VELOCITY 70

/* commands to servo
#define EEP_WRITE 0x01
#define EEP_READ 0x02
#define RAM_WRITE 0x03
#define RAM_READ 0x04
#define I_JOG 0x05
#define S_JOG 0x06
#define STAT 0x07
#define ROLLBACK 0x08
#define REBOOT 0x09

/* ack responses
#define ACK_RAM_WRITE 0x43
#define ACK_RAM_READ 0x44
#define ACK_I_JOG 0x45
#define ACK_S_JOG 0x46
#define ACK_STAT 0x47
#define ACK_ROLLBACK 0x48
#define ACK_REBOOT 0x49

#define EEP_BAUD_RATE 0x04

#define RAM 0
#define EEP 1
#define TORQUE_ON 0x60
#define TORQUE_FREE 0x00
#define TORQUE_BREAK 0x40

/* #define BROADCAST_ADDR 0xFE

/* Leds
#define LED_GREEN 1
#define LED_BLUE 2
#define LED_RED 4
*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
USART_TypeDef *usartx;
/* variaveis para decodificação de pacotes */
uint8_t size;			//tamanho do pacote
uint8_t pid;			//servo ID ou 0xFE para enviar a todos os servos.
uint8_t cmd;			// Comando
uint8_t csum1;			//checksum1
uint8_t csum2;			//checksum2
uint8_t dataAddr;		// endereço do registrador
uint8_t dataLength;		//numero de bytes lidos
uint8_t statusError;
uint8_t statusDetail;
float position;
float velocity;

/** informa se pacotes foram recebidos e se estão inteiros
 * status =
 * 1- ok, recebido
 * 0- erro: ou o pacote nao foi recebido, ou o pacote foi corrompido.
 */
uint8_t status;/* informa se pacotes foram recebidos, */
uint8_t BUFFER[30];		// Buffer de dados a serem enviados
uint8_t DATA[20];		// Buffer dados crus recebidos sem cabeçalho ou soma de verificação



/* Private function prototypes -----------------------------------------------*/
void c_io_herkulex_clearBuffer(void);
int16_t c_io_herkulex_getRawPosition(uint8_t data[]);
int16_t c_io_herkulex_getRawVelocity(uint8_t data[]);
float c_io_herkulex_raw2pos(int16_t raw_data);
float c_io_herkulex_raw2vel(int16_t raw_data);
uint8_t c_io_herkulex_unpack(uint8_t *new_buffer);
uint8_t c_io_herkulex_serializeRw(void);
uint8_t c_io_herkulex_serializeSjog(pv_sjog_herkulex sjog[], uint8_t num_jogs, uint8_t ptime);
uint8_t c_io_herkulex_serializeIjog(pv_ijog_herkulex ijog[], uint8_t num_jogs);
void c_io_herkulex_send(void);
uint8_t c_io_herkulex_receive(void);
uint8_t c_io_herkulex_checksum1(uint8_t *buffer, uint8_t size);
uint8_t c_io_herkulex_checksum2(uint8_t cksum1);
int16_t c_io_herkulex_deg2raw(float position_deg);
pv_ijog_herkulex c_io_herkulex_createIjog(uint8_t servo_id, int16_t data, uint8_t stop, uint8_t mode, uint8_t led, uint8_t ptime);
pv_sjog_herkulex c_io_herkulex_createSjog(uint8_t servo_id, int16_t data, uint8_t stop, uint8_t mode, uint8_t led, uint8_t no_action);


/* Private functions ---------------------------------------------------------*/
/** \brief Zera o buffer de saída. */
void c_io_herkulex_clearBuffer(void)
{
	int i;

	for (i = 0; i < 50; i++)
		BUFFER[i] = 0;
}

int16_t c_io_herkulex_getRawPosition(uint8_t data[])
{
	return (int16_t)(((data[1] & 0x03) << 8) | data[0]);
}
int16_t c_io_herkulex_getRawVelocity(uint8_t data[])
{
	return (int16_t)(((data[1] & 0xFF) << 8) | data[0]);
}

/* convert raw_data to position in rad */
float c_io_herkulex_raw2pos(int16_t raw_data)
{
	return (((float)raw_data) * 0.325 - 166.65) * PI / 180;
}

/* convert raw_data to velocity in rad/s */
float c_io_herkulex_raw2vel(int16_t raw_data)
{
	return ((float)raw_data) * 29.09 * PI / 180.0;
}

uint8_t c_io_herkulex_unpack(uint8_t *new_buffer)
{
	uint8_t new_cksum1, new_cksum2;

	size = (unsigned char)BUFFER[2];
	pid = BUFFER[3];
	cmd = BUFFER[4];
	csum1 = BUFFER[5];
	new_cksum1 = c_io_herkulex_checksum1(BUFFER, size);
	csum2 = BUFFER[6];
	new_cksum2 = c_io_herkulex_checksum2(new_cksum1);
	if ((new_cksum1 != csum1) || (new_cksum2 != csum2))
		return 0;

	if (size > 7) {
		if (cmd == ACK_STAT) {
			statusError = BUFFER[7];
			statusDetail = BUFFER[8];
		} else {
			dataAddr = BUFFER[7];
			dataLength = BUFFER[8];
		}
	}

	if (size > 9) {
		memcpy(DATA, BUFFER + 9, dataLength);
		statusError = BUFFER[size - 2];
		statusDetail = BUFFER[size - 1];
	}

	return 1;
}

/* so retorna 0 se size <=0, ou seja não há dados a serem serializados */
uint8_t c_io_herkulex_serializeRw(void)
{
	if (size <= 0)
		return 0;

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	BUFFER[2] = size;
	BUFFER[3] = pid;
	BUFFER[4] = cmd;

	if (size > 7) {
		BUFFER[7] = dataAddr;
		BUFFER[8] = dataLength;
	}

	uint8_t i;

	if (size > 9) {
		for (i = 0; i < dataLength; i++)
			BUFFER[9 + i] = DATA[i];
	}


	if ((size-dataLength) > 9) {
		BUFFER[size - 2] = statusError;
		BUFFER[size - 1] = statusDetail;
	}

	BUFFER[5] = c_io_herkulex_checksum1(BUFFER, size);
	BUFFER[6] = c_io_herkulex_checksum2(BUFFER[5]);
	csum1 = BUFFER[5];
	csum2 = BUFFER[6];

	return 1;
}

uint8_t c_io_herkulex_serializeSjog(pv_sjog_herkulex sjog[], uint8_t num_jogs, uint8_t ptime)
{
	int n = sizeof(pv_sjog_herkulex);

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	/* numero_sjogs*bytes_por_sjog+bytes_de_cabecalho */
	BUFFER[2] = num_jogs * n + 8;
	/* if (num_jogs>1)
		BUFFER[3] = 0xFE;
	else */
	BUFFER[3] = sjog[0].ucID;
	BUFFER[4] = S_JOG;
	BUFFER[7] = ptime;

	memcpy((BUFFER+8), (void *) sjog, n * num_jogs);

	csum1 = c_io_herkulex_checksum1(BUFFER, BUFFER[2]); /* por que usar csum??????? */
	csum2 = c_io_herkulex_checksum2(csum1);
	BUFFER[5] = csum1; /* checksum1(BUFFER, BUFFER[2]); */
	BUFFER[6] = csum2; /* checksum2(BUFFER[5]); */

	return 1;
}

uint8_t c_io_herkulex_serializeIjog(pv_ijog_herkulex ijog[], uint8_t num_jogs)
{
	int n = sizeof(ijog);

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	/* numero_sjogs * bytes_por_sjog + bytes_de_cabecalho */
	BUFFER[2] = num_jogs * n + 7;
	/*if (num_jogs>1)
		BUFFER[3] = 0xFE;
	else */
	BUFFER[3] = ijog[0].ucID;
	BUFFER[4] = I_JOG;

	memcpy((BUFFER + 7), (void *) ijog, n * num_jogs);

	/* csum1=checksum1(BUFFER,BUFFER[2]); /* por que usar csum??????? */
	/* csum2=checksum2(csum1); */
	BUFFER[5] = c_io_herkulex_checksum1(BUFFER, BUFFER[2]);
	BUFFER[6] = c_io_herkulex_checksum2(BUFFER[5]);

	return 1;
}

void c_io_herkulex_send(void)
{
	int i;

	for (i = 0; i < size ; ++i)
		c_common_usart_putchar(usartx, BUFFER[i]);
}

uint8_t c_io_herkulex_receive(void)
{
	int i = 0;
	uint8_t lastByte = 0, inByte = 0, ok = 0, size = 30;
	unsigned long last_now = 0, now = 0, timeOut;

	last_now = now;
	now = c_common_utils_millis();
	timeOut = 3 + now;
	/* timeOut = 3; while (now - last_now <= timeOut && i < size) { */
	while (now <= timeOut && i < size) {
		while (!c_common_usart_available(usartx) && now <= timeOut)
			now = c_common_utils_millis();
		lastByte = inByte;
		inByte = c_common_usart_read(usartx);
		if (!ok && inByte == 0xFF && lastByte == 0xFF) {
			BUFFER[0] = 0xFF;
			i = 1;
			ok = 1;
		}
		if (ok) {
			BUFFER[i] = inByte;
			if (i == 2)
				size = BUFFER[2];
			i++;
		}
	}
	if (now >= timeOut)
		return 0;
	return 1;
}

uint8_t c_io_herkulex_checksum1(uint8_t *buffer, uint8_t size)
{
	uint8_t i;
	char chksum = 0;

	for (i = 2; i < 5; i++)
		chksum = chksum ^ buffer[i];

	for (i = 7; i < size; i++)
		chksum = chksum ^ buffer[i];
	/* printf("i=%d\n",i);
	assert(i==size); */
	chksum = chksum & 0xFE;

	return chksum;
}

uint8_t c_io_herkulex_checksum2(uint8_t checksum1)
{
	return (~checksum1) & 0xFE;
}

/**
 * Converte de graus para um inteiro aceito pelo servo.
 */
int16_t c_io_herkulex_deg2raw(float position_deg)
{
	return ((int16_t)(position_deg / 0.325 + 0.5)) + 512; /* 0.5 -> round */
}

pv_ijog_herkulex c_io_herkulex_createIjog(uint8_t servo_id, int16_t data,
		uint8_t stop, uint8_t mode, uint8_t led, uint8_t ptime)
{
	pv_ijog_herkulex ijog;

	ijog.iJogData = data;
	ijog.uiStop = stop;
	ijog.uiMode = mode;
	ijog.uiLed = led;
	ijog.ucID = servo_id;
	ijog.uiReserved1 = 0;
	ijog.uiJogInvalid = 0;
	ijog.uiReserved2 = 0;
	ijog.ucJogTime_ms = ptime;


	return ijog;
}

pv_sjog_herkulex c_io_herkulex_createSjog(uint8_t servo_id, int16_t data,
		uint8_t stop, uint8_t mode, uint8_t led, uint8_t no_action)
{
	pv_sjog_herkulex sjog;

	sjog.iJogData = data;
	sjog.uiStop = stop;
	sjog.uiMode = mode;
	sjog.uiLed = led;
	sjog.ucID = servo_id;
	sjog.uiReserved1 = 0;
	sjog.uiJogInvalid = no_action;
	sjog.uiReserved2 = 0;

	return sjog;
}



/* Exported functions definitions --------------------------------------------*/

/* Direct servo commands */
/** \brief Lê o valor dado um endereço de memoria.
  * Retorna o valor em caso de sucesso.
  *
  * @param mem tipo de memória, ROM ou RAM
  * @param  servo_id ID do servo.
  * @param reg_addr Endereço do registrador do servo
  * @param data_length Tamanho dos dados a serem escritos
  * @retval ponteiro para os dados lidos.
  */
uint8_t  c_io_herkulex_read(char mem, char servo_id, char reg_addr,
	unsigned char datalength)
{
	pid = servo_id;
	size = 9;
	if (mem == EEP)
		mem = EEP_READ;
	else
		mem = RAM_READ;

	cmd = mem;
	dataAddr = reg_addr;
	dataLength = datalength;
	status = 1;
	c_io_herkulex_serializeRw();
	c_io_herkulex_send();
	c_io_herkulex_clearBuffer();
	status = c_io_herkulex_receive();
	if (status) /* se for corrompido, faz status = 0 */
		status = c_io_herkulex_unpack(BUFFER);

	return status;
}




/** \brief Escreve o valor dado um endereço de memoria, entre 0 e 0x18.
  * Retorna o valor em caso de sucesso.
  *
  * @param mem tipo de memória, ROM ou RAM
  * @param  servo_id ID do servo.
  * @param reg_addr Endereço do registrador do servo
  * @param data_length Tamanho dos dados a serem escritos
  * @param data Ponteiro para os dados a serem escritos
  * @retval bool - 1 se dados foram enviados, 0 se não foram.
  */
uint8_t c_io_herkulex_write(char mem, char servo_id, char reg_addr,
	unsigned char datalength, char *data)
{
	pid = servo_id;
	size = 0x09 + datalength;
	if (mem == EEP)
		mem = EEP_WRITE;
	else
		mem = RAM_WRITE;

	cmd = mem;
	dataAddr = reg_addr;
	status = 1;
	dataLength = datalength;
	if (data != NULL) {
		memcpy(DATA, data, datalength);
		c_io_herkulex_serializeRw();
		c_io_herkulex_send();
		return 1;
	} else {
		return 0;
	}
}




void c_io_herkulex_sendIjog(pv_ijog_herkulex ijog[], uint8_t num_servos)
{
	c_io_herkulex_serializeIjog(ijog, num_servos);
	c_io_herkulex_send();
}

void c_io_herkulex_sendSjog(pv_sjog_herkulex sjog[], uint8_t num_servos,
	uint8_t ptime)
{
	c_io_herkulex_serializeSjog(sjog, num_servos, ptime);
	c_io_herkulex_send();
}

uint8_t c_io_herkulex_readStatus(uint8_t servo_id)
{
	pid = servo_id;
	cmd = STAT;
	size = 7;
	status = 1;
	c_io_herkulex_serializeRw();
	c_io_herkulex_send();

	return c_io_herkulex_receive();
}

void c_io_herkulex_reboot(uint8_t servo_id)
{
	pid = servo_id;
	cmd = REBOOT;
	size = 7;
	status = 1;
	c_io_herkulex_serializeRw();
	c_io_herkulex_send();
}




/* Indirect commands */
/** \brief Inicializa o usart para comunicacao serial entre servo e discovery
  *
  *
  * @param  None
  * @retval None
  */
void c_io_herkulex_init(USART_TypeDef *usartn, int baudrate)
{
	usartx = usartn;
	/* Inicia a usart2 */
	if (usartx == USART1)
		c_common_usart1_init(baudrate);
	else if (usartx == USART2)
		c_common_usart2_init(baudrate);
	else if (usartx == USART3)
		c_common_usart3_init(baudrate);
	else if (usartx == USART6)
		c_common_usart6_init(baudrate);
}

void c_io_herkulex_config(uint8_t servoId)
{
	c_io_herkulex_clear(servoId);
	c_io_herkulex_reboot(servoId);
	c_common_utils_delayms(1000);

	DATA[0]=1;
	//only reply to read commands
	c_io_herkulex_configAckPolicy(servoId,1);

	//Acceleration Ratio = 0
	DATA[0]=0;
	c_io_herkulex_write(RAM,servoId,REG_ACC_RATIO,1,DATA);

	//set no acceleration time
	DATA[0]=0;
	c_io_herkulex_write(RAM,servoId,REG_MAX_ACC_TIME,1,DATA);

	DATA[0]=0;
	c_io_herkulex_write(RAM,servoId,REG_PWM_OFFSET,1,DATA);

	//min pwm = 0
	DATA[0]=0;
	c_io_herkulex_write(RAM,servoId,REG_MIN_PWM,1,DATA);

	//max pwm >1023 -> no max pwm
	DATA[1]=0x03;//little endian 0x03FF sent
	DATA[0]=0xFF;
	c_io_herkulex_write(RAM,servoId,REG_MAX_PWM,2,DATA);

	/** set overload pwm register, if overload_pwm>1023, overload is never
	 * activated this is good for data acquisition, but may not be the case for
	 * the tilt-rotor actualy flying.
	 */
	DATA[0]=0xFF;
	DATA[1]=0x03;//little endian, 2048 sent
	c_io_herkulex_write(RAM,servoId,REG_OVERLOAD_PWM_THRESHOLD,2,DATA);

	//configura Kp, ki,kd and 1st and 2nd feedforward gains
	//uint8_t i, n=10;// n is the number of bytes to written
	//for(i=0;i<n;i++) {
	//	DATA[i]=0;
	//}
	//c_io_herkulex_write(RAM,servo1_id,REG_KP,n,DATA);
	c_io_herkulex_setTorqueControl(servoId, TORQUE_ON);
	c_io_herkulex_clear(servoId);
}

void c_io_herkulex_configAckPolicy(char servo_id, char policy)
{
	DATA[0] = policy;
	c_io_herkulex_write(RAM, servo_id, REG_ACK_POLICY, 1, DATA);
}
void c_io_herkulex_configLedPolicy(char servo_id, char policy)
{
	DATA[0] = policy;
	c_io_herkulex_write(RAM, servo_id, REG_ACK_POLICY, 1, DATA);
}
void c_io_herkulex_ledControl(char servo_id, char led)
{
	DATA[0] = led;
	c_io_herkulex_write(RAM, servo_id, REG_LED_CONTROL, 1, DATA);
}
void c_io_herkulex_clear(uint8_t servo_id)
{
	DATA[0] = 0;
	DATA[1] = 0;
	c_io_herkulex_write(RAM, servo_id, REG_STATUS_ERROR, 2, DATA);
}

/*void c_io_herkulex_change_mode(uint8_t servo_id, uint8_t mode)
{
	/* c_io_herkulex_set_torque_control(servo_id,TORQUE_FREE);
	pv_sjog_herkulex jog = c_io_herkulex_create_sjog(servo_id,0,0,mode,0,1);
	c_io_herkulex_sjog(&jog, 1, 0);
c_io_herkulex_setTorqueControl(servo_id, TORQUE_ON);
}*/

void c_io_herkulex_setTorqueControl(uint8_t servo_id, uint8_t control)
{
	DATA[0] = control;
	c_io_herkulex_write(RAM, servo_id, REG_TORQUE_CONTROL, 1, DATA);
}

void c_io_herkulex_setBaudRate(uint8_t servo_id, int baudrate)
{
	uint8_t baud;

	switch (baudrate) {
	case 57600:
		baudrate = 0x22;
		break;
	case 115200:
		baudrate = 0x10;
		break;
	case 200000:
		baudrate = 0x09;
		break;
	case 250000:
		baudrate = 0x07;
		break;
	case 400000:
		baudrate = 0x04;
		break;
	case 500000:
		baudrate = 0x03;
		break;
	case 666666:
		baudrate = 0x02;
		break;
	}
	DATA[0] = baudrate;
	c_io_herkulex_write(EEP, servo_id, EEP_BAUD_RATE, 1, DATA);
}



/** Control Interface
	 *
	 * Functions for feedback, the 1st read the current absolute
	 * position, the second read the angular velocity. The outputs
	 * are converted to degrees and rad/s respectively
	 */
float c_io_herkulex_readPosition(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM, servo_id, REG_ABSOLUTE_POS, 2))
		return -1;

	int16_t rawValue;

	rawValue = ((DATA[1] & 0x03) << 8) | DATA[0];

	return (((float)rawValue) * 0.325 - 166.65) * PI / 180;
}

float c_io_herkulex_readVelocity(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM, servo_id, REG_DIFFERENTIAL_POS, 2))
		return -1;

	int16_t rawValue = 0;

	rawValue = ((DATA[1] & 0xFF) << 8) | DATA[0];

	/* return ((float)rawValue)*0.325*PI/(0.0112*180.0); */
	float vel = ((float)rawValue) * 29.09 * PI / 180.0;
	return vel;
}

/**
 * Read position and velocity in rad and rad/s
 */

int8_t c_io_herkulex_readData(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM, servo_id, REG_ABSOLUTE_POS, 4))
			return 0;
	int16_t raw_position = c_io_herkulex_getRawPosition(DATA);
	int16_t raw_velocity = c_io_herkulex_getRawVelocity(DATA + 2);

	velocity = c_io_herkulex_raw2vel(raw_velocity);
	position = c_io_herkulex_raw2pos(raw_position);
	return 1;
}

float c_io_herkulex_getPosition(uint8_t servo_id)
{
	return position;
}

float c_io_herkulex_getVelocity(uint8_t servo_id)
{
	return velocity;
}

/* set input toque to servo */
void c_io_herkulex_setTorque(uint8_t servo_id, int16_t pwm)
{
	int8_t sign = 0;

	if (pwm < 0) {
		pwm = pwm * -1;
		sign = 1;
	}
	if (pwm > 8191)
		pwm = 8191;
	/*
	if (torque_status[translate_servo_id(servo_id)]!=TORQUE_ON)
	{
		c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);
	}
	*/
	pwm |= sign << 14;
	pv_sjog_herkulex sjog =  c_io_herkulex_createSjog(servo_id, pwm, 0,
			ROTATION_MODE, 0, 0);
	c_io_herkulex_sendSjog(&sjog, 1, 0);
}

/* set input toque to servo */
void c_io_herkulex_setTorque2Servos(uint8_t servo1_id, int16_t pwm1,
	uint8_t servo2_id, int16_t pwm2)
{
	int8_t sign1 = 0, sign2 = 0;

	if (pwm1 < 0) {
		pwm1 = pwm1 * -1;
		sign1 = 1;
	}
	if (pwm1 > 8191)
		pwm1 = 8191;

	if (pwm2 < 0) {
		pwm2 = pwm2 * -1;
		sign1 = 1;
	}
	if (pwm2 > 8191)
		pwm2 = 8191;
	/*
	if (torque_status[translate_servo_id(servo_id)]!=TORQUE_ON)
	{
		c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);
	}
	*/
	pwm1 |= sign1 << 14;
	pv_sjog_herkulex sjog[2];

	sjog[0] = c_io_herkulex_createSjog(servo1_id, pwm1, 0, ROTATION_MODE,
		0, 0);

	pwm2 |= sign2 << 14;
	sjog[1] =  c_io_herkulex_createSjog(servo2_id, pwm2, 0, ROTATION_MODE,
		0, 0);
	c_io_herkulex_sendSjog(sjog, 2, 0);
}


void c_io_herkulex_setPosition(uint8_t servo_id, float position_deg)
{
	int16_t raw_pos = c_io_herkulex_deg2raw(position_deg);
	pv_sjog_herkulex sjog = c_io_herkulex_createSjog(servo_id, raw_pos, 0,
			POSITION_MODE, 0, 0);
	c_io_herkulex_sendSjog(&sjog, 1, 0);
}

void c_io_herkulex_setPositionRad(uint8_t servo_id, float position_rad)
{
	c_io_herkulex_setPosition(servo_id, rad2deg(position_rad));
}

void c_io_herkulex_setPosition2Servos(uint8_t servo1_id, float pos1_deg,
		uint8_t servo2_id, float pos2_deg)
{
	int16_t raw_pos1 = c_io_herkulex_deg2raw(pos1_deg);
	int16_t raw_pos2 = c_io_herkulex_deg2raw(pos2_deg);
	pv_sjog_herkulex sjog[2];

	sjog[0] = c_io_herkulex_createSjog(servo1_id, raw_pos1, 0,
		POSITION_MODE, 0, 0);
	sjog[1] = c_io_herkulex_createSjog(servo2_id, raw_pos2, 0,
		POSITION_MODE, 0, 0);
	c_io_herkulex_sendSjog(sjog, 2, 0);
}

uint8_t c_io_herkulex_getStatusError(void)
{
	return statusError;
}

uint8_t c_io_herkulex_getStatusDetail(void)
{
	return statusDetail;
}

uint8_t c_io_herkulex_getStatus(void)
{
	return status;
}

/**
 * Auxiliary functions
 */

void c_io_herkulex_decodeError(char *dest, int8_t status_error, int8_t status_detail)
{
	int n;

	dest[0] = 0;
	if (status_error & 0x01)
		strcat(dest, "Exceed input voltage limit, ");

	if (status_error & 0x02)
		strcat(dest, "Exceed alowed POT limit, ");

	if (status_error & 0x04)
		strcat(dest, "Exceed temperature limit, ");

	if (status_error & 0x08) {
		strcat(dest, "Invalid packet: ");
		if (status_detail & 0x04)
			strcat(dest, "Checksum Error, ");
		if (status_detail & 0x08)
			strcat(dest, "Unknown Command, ");
		if (status_detail & 0x10)
			strcat(dest, "Exceed REG range, ");
		if (status_detail & 0x20)
			strcat(dest, "Garbage detected, ");
	}
	if (status_error & 0x10)
		strcat(dest, "Overload detected, ");

	if (status_error & 0x20)
		strcat(dest, "Driver fault detected, ");

	if (status_error & 0x40)
		strcat(dest, "EEP reg distorted, ");

	n = strlen(dest);

	dest[n - 2] = 0;
}

/* @}
 * @}
 */
