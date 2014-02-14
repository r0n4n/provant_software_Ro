/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_io.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_IO
  * \brief Componentes para atuação e sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de I/O do VANT.
  * Leituras de todos os sensores, comandos para atuadores. O processamento destes
  * dados brutos NÃO é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int  accRaw[3], gyroRaw[3], magRaw[3];
char str[64];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores e atuadores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_io_init() {
	c_common_i2c_init();

	c_common_usart2_init(9600);
	c_io_rx24f_init(1000000);

	//for(int i=0; i<0xFFFFFF; i++) { __asm("NOP"); }
	C_COMMON_UTILS_1MS_DELAY //delay para inicialização da IMU
	c_io_imu_init();

	c_io_blctrl_init();
	// Init queues
	//pv_interface_io.iServoSetpoints = xQueueCreate(1, sizeof(pv_msg_io_servoSetpoints));

	//if(pv_interface_io.iServoSetpoints == 0) {
	//	vTraceConsoleMessage("Could not create queue in pv_interface_io!");
	//	while(1);
	//}
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
void module_io_run() {
	float rpy[3];

	while(1) {
		//xQueueReceive(pv_interface_io.iServoSetpoints, &recvsetp, 0);
		/*
		c_io_imu_getComplimentaryRPY(rpy);
		//sprintf(str, "RP: %d %d\n\r", (int)(100.0*rpy[0]), (int)(100.0*rpy[1]));
		//c_common_usart_puts(USART2, str);
		*/
		vTaskDelay(250/portTICK_RATE_MS);
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
