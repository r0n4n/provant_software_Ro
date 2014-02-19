/**
  ******************************************************************************
  * @file    modules/common/c_common_i2c.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    17-Dezember-2013
  * @brief   Funcões para configuração de I2C, para uso em outros módulos.
  *	TODO
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_I2C_H
#define C_COMMON_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_common_i2c_init();
void c_common_i2c_start(uint8_t address, uint8_t direction);
void c_common_i2c_write(uint8_t data);
void c_common_i2c_stop();

void c_common_i2c_writeBit(uint8_t device, uint8_t address, uint8_t bit, bool value);

uint8_t c_common_i2c_readAck();
uint8_t c_common_i2c_readNack();

void c_common_i2c_readBytes(uint8_t device, uint8_t address, char bytesToRead, uint8_t * recvBuffer);
void c_common_i2c_writeByte(uint8_t device, uint8_t address, uint8_t byteToWrite);

/* Header-defined wrapper functions ----------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif //C_COMMON_I2C_H
