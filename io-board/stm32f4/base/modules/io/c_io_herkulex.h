/*
 * c_io_herkulex.h
 *
 *  Created on: 01/12/2014
 *      Author: iuro
 */

#ifndef BASE_MODULES_IO_C_IO_HERKULEX_H_
#define BASE_MODULES_IO_C_IO_HERKULEX_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "c_common_utils.h"
#include "pv_typedefs.h"


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define BLCTRL_ADDR  0x29

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


//Direct servo commands
uint8_t  c_io_herkulex_read(char mem, char servo_id, char reg_addr, unsigned char data_length);
uint8_t c_io_herkulex_write(char mem, char servo_id, char reg_addr, unsigned char datalength, char *data);
void c_io_herkulex_ijog();
void c_io_herkulex_sjog(char size, char servo_id, uint16_t data, char stop, char mode, char led, char ptime);
uint8_t c_io_herkulex_stat(uint8_t servo_id);
void c_io_herkulex_rollback();
void c_io_herkulex_reboot(uint8_t servo_id);

//Indirect commands
void c_io_herkulex_init();
void c_io_herkulex_config_ack_policy(char servo_id, char policy);
void c_io_herkulex_config_led_policy(char servo_id, char policy);
void c_io_herkulex_led_control(char servo_id, char led);
void c_io_herkulex_clear(uint8_t servo_id);
void c_io_herkulex_set_torque_control(char servo_id, char control);

/** Control Interface
	 *
	 * Functions for feedback, the 1st read the current absolute
	 * position, the second read the angular velocity. The outputs
	 * are converted to degrees and rad/s respectively
	 */
float c_io_herkulex_read_position(uint8_t servo_id);
float c_io_herkulex_read_velocity(uint8_t servo_id);
//set input toque to servo
void c_io_herkulex_set_torque(uint8_t servo_id, int16_t pwm);


#ifdef __cplusplus
}
#endif

#endif /* BASE_MODULES_IO_C_IO_HERKULEX_H_ */
