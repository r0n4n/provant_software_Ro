/**
  ******************************************************************************
  * @file    modules/io/c_io_rx24f.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do servo RX-24F.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_RX24F_H
#define C_IO_RX24F_H

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
void c_io_rx24f_init(int baudrate);
int  c_io_rx24f_move(unsigned char ID, int position);
int  c_io_rx24f_readPosition(unsigned char ID);
int  c_io_rx24f_setLed(unsigned char ID, unsigned char value);

#ifdef __cplusplus
}
#endif

#endif //C_IO_RX24F_H
