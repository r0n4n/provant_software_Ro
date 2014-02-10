/**
  ******************************************************************************
  * @file    modules/common/pv_typedefs.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Definições de tipos e estruturas de mensagens para o projeto.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_TYPEDEFS_H
#define PV_TYPEDEFS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /** @addtogroup Common_Components
   * @{
   */

 /** @addtogroup PV_Typedefs
   *
   * Definições de tipagem e estruras de mensagem para o projeto.
   * @{
   */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** \brief Um par de floats. */
typedef struct {
	int channel[8];
} pv_type_receiverChannels;

/** \brief Estrutura de mensagens para os dois RX24f.*/
typedef struct {
	float leftAngle;
	float leftTorque;
	float rightAngle;
	float rightTorque;
} pv_msg_io_servoSetpoints;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Header-defined wrapper functions ----------------------------------------- */

 /**
   * @}
   */
 /**
   * @}
   */

#ifdef __cplusplus
}
#endif

#endif //PV_TYPEDEFS_H

