/*
 * c_io_herkulex.c
 *
 *  Created on: 01/12/2014
 *      Author: iuro
 */

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
#define KV 0.325*PI/(0.0112*180)
#define TIME_OUT 10
#define INTER_PKG_TIME 0.000105
#define HEADER 0xFF
// status error register, 48
#define EXCEED_INPUT_VOLT_LIMIT 0x01
#define EXCEED_ALLOWED_POT_LIMIT 0x02
#define EXCEED_TEMP_LIMIT 0x04
#define INVALID_PACKET 0x08
#define OVERLOAD 0x10
#define DRIVER_FAULT 0x20
#define EEP_REG_DISTORTED 0x40

// status error register, 49
#define MOVING_FLAG 0x01
#define INPOSITION_FLAG 0x02
//invalid packet errors
#define CHECKSUM_ERROR 0x04
#define UNKNOWM_COMMAND 0x08
#define EXCEED_REG_RANGE 0x10
#define GARBAGE_DETECTED 0x20
//end of packet errors
#define MOTOR_ON_FLAG 0x40

//Address of registers in RAM
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

//some other registers from voltatile RAM
#define REG_INPOSITION_MARGIN 44
#define REG_CALIBRATION _DIFF 47
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

//commands to servo
#define EEP_WRITE 0x01
#define EEP_READ 0x02
#define RAM_WRITE 0x03
#define RAM_READ 0x04
#define I_JOG 0x05
#define S_JOG 0x06
#define STAT 0x07
#define ROLLBACK 0x08
#define REBOOT 0x09

//ack responses
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

#define BROADCAST_ADDR 0xFE;

//Leds
#define LED_GREEN 1
#define LED_BLUE 2
#define LED_RED 4


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//da classe 'packet'
USART_TypeDef *usartx = 0;

uint8_t size;
uint8_t pid;
uint8_t cmd;
uint8_t csum1;
uint8_t csum2;
uint8_t data_addr;
uint8_t data_length;
uint8_t status_error;
uint8_t status_detail;

//for use with read_data function
float position;
float velocity;
/** informa se pacotes foram recebidos e se estão inteiros
 * status =
 * 1- ok, recebido
 * 0- erro: ou o pacote nao foi recebido, ou o pacote foi corrompido.
 */
uint8_t status;//informa se pacotes foram recebidos,

uint8_t BUFFER[100];
uint8_t DATA[100];

//Pacote de
pv_sjog_herkulex jog_packet;
uint8_t torque_status[2];
uint8_t servo_ids[2];
uint8_t play_time;

/* Private function prototypes -----------------------------------------------*/
uint8_t raw2packet(uint8_t* new_buffer);
uint8_t serialize_io();
uint8_t receive();
void send();
uint8_t serialize_sjog(pv_sjog_herkulex sjog[], uint8_t num_jogs, uint8_t ptime);
uint8_t serialize_ijog(pv_ijog_herkulex ijog[], uint8_t num_jogs, uint8_t ptime);
uint8_t translate_servo_id(uint8_t servo_id);
uint8_t checksum1(uint8_t *buffer, uint8_t size);
uint8_t checksum2(uint8_t cksum1);
void zera_buffer();
int16_t get_raw_position(uint8_t data[]);
int16_t get_raw_velocity(uint8_t data[]);
float convert_position(int16_t raw_data);
float convert_velocity(int16_t raw_data);
static inline int16_t c_io_herkulex_deg2raw(float position_deg);

/* Private functions ---------------------------------------------------------*/
void zera_buffer()
{
	int i;
	for(i=0;i<50;i++)
		BUFFER[i]=0;
}
int16_t get_raw_position(uint8_t data[])
{
	return (int16_t)(((data[1]&0x03)<<8) | data[0]);
}
int16_t get_raw_velocity(uint8_t data[])
{
	return (int16_t)(((data[1]&0xFF)<<8) | data[0]);
}

//convert raw_data to position in rad
float convert_position(int16_t raw_data)
{
	return (((float)raw_data)*0.325-166.65)*PI/180;
}

//convert raw_data to velocity in rad/s
float convert_velocity(int16_t raw_data)
{
	return ((float)raw_data)*29.09*PI/180.0;
}
/* Exported functions definitions --------------------------------------------*/

//Direct servo commands

/** \brief Lê o valor dado um endereço de memoria.
  * Retorna o valor em caso de sucesso.
  *
  * @param mem tipo de memória, ROM ou RAM
  * @param  servo_id ID do servo.
  * @param reg_addr Endereço do registrador do servo
  * @param data_length Tamanho dos dados a serem escritos
  * @retval ponteiro para os dados lidos.
  */
uint8_t  c_io_herkulex_read(char mem, char servo_id, char reg_addr, unsigned char datalength)
{
	pid = servo_id;
	size=9;
	if (mem==EEP) {
		mem=EEP_READ;
	} else {
		mem=RAM_READ;
	}
	cmd=mem;
	data_addr = reg_addr;
	data_length = datalength;
	status=1;
	//send() envia comando de requisição de leitura
	serialize_io();
	send();
	//le os dados enviados pelo servo
	zera_buffer();
	status = receive();
	if (status)
	{ //se for corrompido, faz status = 0
		status = raw2packet(BUFFER);
	}

	return status;
}


void send()
{
	for (int i=0; i < size ; ++i)
		c_common_usart_putchar(usartx,BUFFER[i]);
}

uint8_t receive()
{// precisa de um timeout
	int i=0;
	uint8_t lastByte = 0, inByte = 0, ok=0, size = 30;
	long now = c_common_utils_millis();
	long timeOut = now + 10;
	//while(c_common_usart_read(usartx)!=0xFF);
	//if (c_common_usart_read(usartx) != 0xFF) return 0;
	//BUFFER[0]=0xFF;
	//BUFFER[1]=0xFF;
	//i=2;
	while (now<=timeOut && i<size) {
		//verifica quando o primeiro byte chegou
		while (!c_common_usart_available(usartx) && now<=timeOut) now=c_common_utils_millis();
		lastByte=inByte;
		inByte = c_common_usart_read(usartx);
		if (!ok && inByte == 0xFF && lastByte == 0xFF ) {
			BUFFER[0]=0xFF;
			i=1;
			ok=1;
		}
		if (ok) {
			BUFFER[i] = inByte;
			if (i==2) size=BUFFER[2];
			i++;
		}
	}
	if (now>=timeOut) {
		return 0;
	} else {
		return 1;
	}

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
uint8_t c_io_herkulex_write(char mem, char servo_id, char reg_addr, unsigned char datalength, char *data)
{
	pid=servo_id;
	size=0x09+datalength;
	if (mem==EEP) {
		mem=EEP_WRITE;
	} else {
		mem=RAM_WRITE;
	}
	cmd = mem;
	data_addr = reg_addr;
	status = 1;
	data_length=datalength;
	if (data!=NULL) {
		memcpy(DATA,data,datalength);
		serialize_io();
		send();
		return 1;
	} else {
		return 0;
	}
}


pv_ijog_herkulex c_io_herkulex_create_ijog(uint8_t servo_id, int16_t data, uint8_t stop, uint8_t mode, uint8_t led, uint8_t ptime)
{
	pv_ijog_herkulex ijog;
	ijog.iJogData = data;
	ijog.uiStop = stop;
	ijog.uiMode = mode;
	ijog.uiLed = led;
	ijog.ucID = servo_id;
	ijog.uiReserved1=0;
	ijog.uiJogInvalid=0;
	ijog.uiReserved2=0;
	ijog.ucJogTime_ms;


	return ijog;
}

pv_sjog_herkulex c_io_herkulex_create_sjog(uint8_t servo_id, int16_t data,
		uint8_t stop, uint8_t mode, uint8_t led)
{
	pv_sjog_herkulex sjog;
	sjog.iJogData = data;
	sjog.uiStop = stop;
	sjog.uiMode = mode;
	sjog.uiLed = led;
	sjog.ucID = servo_id;
	sjog.uiReserved1=0;
	sjog.uiJogInvalid=0;
	sjog.uiReserved2=0;

	return sjog;
}

void c_io_herkulex_ijog(pv_ijog_herkulex ijog[], uint8_t num_servos, uint8_t ptime)
{
	serialize_ijog(ijog,num_servos,ptime);
	send();
}

void c_io_herkulex_sjog(pv_sjog_herkulex sjog[], uint8_t num_servos, uint8_t ptime)
{
	serialize_sjog(sjog,num_servos,ptime);
	send();
}

uint8_t c_io_herkulex_stat(uint8_t servo_id)
{
	pid=servo_id;
	cmd=STAT;
	size=7;
	status=1;
	serialize_io();
	send();
	//uint8_t *error = (uint8_t*)malloc(2*sizeof(char));
	//uint8_t error[2];
	//error[0]=status_error;
	//error[1]=status_detail;
	//decodeError(error);
	return receive();
}

void c_io_herkulex_rollback()
{

}

void c_io_herkulex_reboot(uint8_t servo_id)
{
	pid=servo_id;
	cmd=REBOOT;
	size=7;
	status=1;
	serialize_io();
	send();
}

//Indirect commands
/** \brief Inicializa o usart para comunicacao serial entre servo e discovery
  *
  *
  * @param  None
  * @retval None
  */
void c_io_herkulex_init(USART_TypeDef *usartn, int baudrate)
{
	usartx=usartn;
	/* Inicia a usart2 */
	if (usartx==USART1) {
		c_common_usart1_init(baudrate);
	} else if (usartx==USART2) {
		c_common_usart2_init(baudrate);
	} else if (usartx==USART3) {
		c_common_usart3_init(baudrate);
	} else if (usartx==USART6) {
		c_common_usart6_init(baudrate);
	}

	torque_status[0]=0;
	torque_status[1]=0;
}

void c_io_herkulex_config_ack_policy(char servo_id, char policy)
{
	DATA[0]=policy;
	c_io_herkulex_write(RAM,servo_id,REG_ACK_POLICY,1,DATA);
}
void c_io_herkulex_config_led_policy(char servo_id, char policy)
{
	DATA[0]=policy;
	c_io_herkulex_write(RAM,servo_id,REG_ACK_POLICY,1,DATA);
}
void c_io_herkulex_led_control(char servo_id, char led)
{
	DATA[0]=led;
	c_io_herkulex_write(RAM,servo_id,REG_LED_CONTROL,1,DATA);
}
void c_io_herkulex_clear(uint8_t servo_id)
{
	DATA[0]=0;
	DATA[1]=0;
	c_io_herkulex_write(RAM,servo_id,REG_STATUS_ERROR,2,DATA);
}


/** \brief Muda o modo de funcionamento do servomotor entre rotação/posição
 *
 * @param servo_id ID do servo
 * @param mode Modo de funcionamento. Usar ROTATION_MODE para rotação ou \
 * 	POSITION_MODE para posição
 */
void c_io_herkulex_change_mode(uint8_t servo_id,uint8_t mode)
{
	c_io_herkulex_set_torque_control(servo_id,TORQUE_FREE);
	pv_sjog_herkulex jog = c_io_herkulex_create_sjog(servo_id,0,0,mode,0);
	c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);
}

void c_io_herkulex_set_torque_control(uint8_t servo_id, uint8_t control)
{
	DATA[0]=control;
	torque_status[translate_servo_id(servo_id)] = control;
	c_io_herkulex_write(RAM,servo_id,REG_TORQUE_CONTROL,1,DATA);
}

/** Control Interface
	 *
	 * Functions for feedback, the 1st read the current absolute
	 * position, the second read the angular velocity. The outputs
	 * are converted to degrees and rad/s respectively
	 */
float c_io_herkulex_read_position(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM,servo_id,REG_ABSOLUTE_POS,2)) {
		return -1;
	}
	int16_t rawValue;
	rawValue=((DATA[1]&0x03)<<8) | DATA[0];

	return (((float)rawValue)*0.325-166.65)*PI/180;
}

float c_io_herkulex_read_velocity(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM,servo_id,REG_DIFFERENTIAL_POS,2)) {
		return -1;
	}
	int16_t rawValue = 0;
	rawValue = ((DATA[1]&0xFF)<<8) | DATA[0];

	//return ((float)rawValue)*0.325*PI/(0.0112*180.0);
	float vel = ((float)rawValue)*29.09*PI/180.0;
	return vel;
}

/**
 * Read position and velocity in rad and rad/s
 */

int8_t c_io_herkulex_read_data(uint8_t servo_id)
{
	if (!c_io_herkulex_read(RAM,servo_id,REG_ABSOLUTE_POS,4)) {
			return 0;
	}
	int16_t raw_position = get_raw_position(DATA);
	int16_t raw_velocity = get_raw_velocity(DATA+2);
	velocity = convert_velocity(raw_velocity);
	position = convert_position(raw_position);
	return 1;
}

float c_io_herkulex_get_position(uint8_t servo_id)
{
	return position;
}

float c_io_herkulex_get_velocity(uint8_t servo_id)
{
	return velocity;
}

//set input toque to servo
void c_io_herkulex_set_torque(uint8_t servo_id, int16_t pwm)
{
	int8_t sign = 0;

	if (pwm<0)
	{
		pwm=pwm*-1;
		sign=1;
	}
	if (pwm>8191)
		pwm=8191;
	/*
	if (torque_status[translate_servo_id(servo_id)]!=TORQUE_ON)
	{
		c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);
	}
	*/
	pwm|=sign<<14;
	pv_sjog_herkulex sjog =  c_io_herkulex_create_sjog(servo_id,pwm,0,ROTATION_MODE,0);
	c_io_herkulex_sjog(&sjog,1,0);
}

//set input toque to servo
void c_io_herkulex_set_torque2(uint8_t servo1_id, int16_t pwm1, uint8_t servo2_id, int16_t pwm2)
{
	int8_t sign1 = 0, sign2 = 0;

	if (pwm1<0)
	{
		pwm1=pwm1*-1;
		sign1=1;
	}
	if (pwm1>8191)
		pwm1=8191;

	if (pwm2<0)
	{
		pwm2=pwm2*-1;
		sign1=1;
	}
	if (pwm2>8191)
		pwm2=8191;
	/*
	if (torque_status[translate_servo_id(servo_id)]!=TORQUE_ON)
	{
		c_io_herkulex_set_torque_control(servo_id,TORQUE_ON);
	}
	*/
	pwm1|=sign1<<14;
	pv_sjog_herkulex sjog[2];
	sjog[0] =  c_io_herkulex_create_sjog(servo1_id,pwm1,0,ROTATION_MODE,0);

	pwm2|=sign2<<14;
	sjog[1] =  c_io_herkulex_create_sjog(servo2_id,pwm2,0,ROTATION_MODE,0);
	c_io_herkulex_sjog(sjog,2,0);
}


static inline void c_io_herkulex_set_goal_position_rad(uint8_t servo_id, float position_rad)
{
	c_io_herkulex_set_goal_position(servo_id,r2d(position_rad));
}

void c_io_herkulex_set_goal_position(uint8_t servo_id, float position_deg)
{
	int16_t raw_pos = c_io_herkulex_deg2raw(position_deg);
	pv_sjog_herkulex sjog = c_io_herkulex_create_sjog(servo_id,raw_pos,0,POSITION_MODE,0);
	c_io_herkulex_sjog(&sjog,1,0);
}

void c_io_herkulex_set_goal_position2(uint8_t servo1_id, float pos1_deg, uint8_t servo2_id, float pos2_deg)
{
	int16_t raw_pos1 = c_io_herkulex_deg2raw(pos1_deg);
	int16_t raw_pos2 = c_io_herkulex_deg2raw(pos2_deg);
	pv_sjog_herkulex sjog[2];
	sjog[0] = c_io_herkulex_create_sjog(servo1_id, raw_pos1,0, POSITION_MODE, 0);
	sjog[1] = c_io_herkulex_create_sjog(servo2_id, raw_pos2,0, POSITION_MODE, 0);
	c_io_herkulex_sjog(sjog, 2, 0);
}

/**
 * Converte de graus para um inteiro aceito pelo servo.
 */
static inline int16_t c_io_herkulex_deg2raw(float position_deg)
{
	return ((int16_t)(position_deg/0.325 + 0.5))+512;//0.5 -> round
}

//so retorna 0 se size <=0, ou seja não há dados a serem serializados
uint8_t serialize_io()
{
	//header
	if (size <= 0) {
		return 0;
	}

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	BUFFER[2] = size;
	BUFFER[3] = pid;
	BUFFER[4] = cmd;

	//data
	if (size>7) {
		BUFFER[7] = data_addr;
		BUFFER[8] = data_length;
	}

	unsigned char i;
	if (size>9) {
		for (i=0;i<(data_length);i++) {
			BUFFER[9+i] = DATA[i];
		}
	}


	if ((size-data_length)>9) {
		BUFFER[size-2]=status_error;
		BUFFER[size-1]=status_detail;
	}
	//checksums

	BUFFER[5] = checksum1(BUFFER, size);
	BUFFER[6] = checksum2(BUFFER[5]);
	csum1=BUFFER[5];
	csum2=BUFFER[6];

	return 1;
}

uint8_t raw2packet(uint8_t* new_buffer)
{
	uint8_t new_cksum1, new_cksum2;
	size = (unsigned char)BUFFER[2];
	pid = BUFFER[3];
	cmd = BUFFER[4];
	csum1 = BUFFER[5];
	new_cksum1=checksum1(BUFFER,size);
	csum2 = BUFFER[6];
	new_cksum2=checksum2(new_cksum1);
	if ((new_cksum1!=csum1) || (new_cksum2!=csum2)) return 0;

	if (size> 7)
	{
		if (cmd==ACK_STAT)
		{
			status_error=BUFFER[7];
			status_detail=BUFFER[8];
		} else
		{
			data_addr=BUFFER[7];
			data_length=BUFFER[8];
		}
	}

	if (size>9)
	{
		memcpy(DATA,BUFFER+9,data_length);
		status_error=BUFFER[size-2];
		status_detail=BUFFER[size-1];
	}

	return 1;
}

uint8_t translate_servo_id(uint8_t servo_id)
{
	if (servo_id==servo_ids[0])
	{
		return 0;
	} else
	{
		return 1;
	}
}

uint8_t checksum1(uint8_t *buffer, uint8_t size)
{
	uint8_t i;
	char chksum=0;

	for(i=2;i<5;i++) {
		chksum=chksum^buffer[i];
	}

	for(i=7;i<size;i++) {
		chksum=chksum^buffer[i];
	}
	//printf("i=%d\n",i);
	//assert(i==size);
	chksum=chksum&0xFE;

	return chksum;
}

uint8_t checksum2(uint8_t checksum1)
{
	return (~checksum1) & 0xFE;
}

uint8_t c_io_herkulex_get_status_error()
{
	return status_error;
}

uint8_t c_io_herkulex_get_status_detail()
{
	return status_detail;
}

uint8_t c_io_herkulex_get_status()
{
	return status;
}

uint8_t serialize_sjog(pv_sjog_herkulex sjog[], uint8_t num_jogs, uint8_t ptime)
{
	if (size < 0)
		return 0;
	int n = sizeof(pv_sjog_herkulex);
	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	BUFFER[2] = num_jogs * n + 8; /* numero_sjogs*bytes_por_sjog+bytes_de_cabecalho */
	BUFFER[3] = sjog[0].ucID;
	BUFFER[4] = S_JOG;
	BUFFER[7] = ptime;

	memcpy((BUFFER+8), (void*) sjog, n * num_jogs);

	//csum1=checksum1(BUFFER,BUFFER[2]); /* por que usar csum??????? */
	//csum2=checksum2(csum1);
	BUFFER[5] = checksum1(BUFFER, BUFFER[2]);
	BUFFER[6] = checksum2(BUFFER[5]);

	return 1;
}

uint8_t serialize_ijog(pv_ijog_herkulex ijog[], uint8_t num_jogs, uint8_t ptime)
{
	if (size < 0)
		return 0;
	int n = sizeof(ijog);
	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	BUFFER[2] = num_jogs * n + 7; /* numero_sjogs*bytes_por_sjog+bytes_de_cabecalho */
	BUFFER[3] = ijog[0].ucID;
	BUFFER[4] = I_JOG;

	memcpy((BUFFER+7), (void*) ijog, n * num_jogs);

	//csum1=checksum1(BUFFER,BUFFER[2]); /* por que usar csum??????? */
	//csum2=checksum2(csum1);
	BUFFER[5] = checksum1(BUFFER, BUFFER[2]);
	BUFFER[6] = checksum2(BUFFER[5]);

	return 1;
}

/* @}
 * @}
 */
