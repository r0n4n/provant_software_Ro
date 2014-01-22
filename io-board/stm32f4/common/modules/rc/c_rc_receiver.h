/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implementação do receiver do controle de rádio manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_RECEIVER_H
#define C_RC_RECEIVER_H

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

 void c_rc_receiver_init();
 int  c_rc_receiver_get_channel(int channel_n);

#ifdef __cplusplus
}
#endif

#endif //C_RC_RECEIVER_H
