/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_rc.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_RC
  * \brief Módulo com as principais funcionalidades para operação em modo rádio controlado.
  *
  * Definição do módulo de controle e comunicação via rádio manual.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pv_msg_io_servoSetpoints setp;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de RC.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * o controle remoto e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_rc_init() {
	c_rc_receiver_init();
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  */
void module_rc_run() {
	setp.leftAngle = 180.0;
	setp.rightAngle = 300.0;

	if(pv_interface_rc.oAngularRefs != 0)
		xQueueOverwrite(pv_interface_rc.oAngularRefs, &setp);
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
