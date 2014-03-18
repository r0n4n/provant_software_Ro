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
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
int  accRaw[3], gyroRaw[3], magRaw[3];
char str[64];

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;

/* Outboxes buffers*/
pv_msg_datapr_attitude oAttitude;
pv_msg_datapr_position oPosition;

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
	/* Inicialização do hardware do módulo */
	c_common_i2c_init(I2C2); //imu
	c_common_i2c_init(I2C1); //esc

	c_common_usart2_init(115200);

	/* Inicializar do sonar */
	c_io_sonar_init();

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
	c_io_rx24f_move(1, 150);
	c_io_rx24f_move(2, 140);
	c_common_utils_delayms(1);
	c_io_rx24f_setSpeed(1, 20);
	c_io_rx24f_setSpeed(2, 20);

	c_common_utils_delayms(100);

	c_io_imu_init(I2C1);   
	c_io_blctrl_init(I2C2);

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
	pv_interface_io.oAttitude = 0;
	pv_interface_io.oPosition = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_io.iActuation == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
void module_io_run() 
{
	float accRaw[3], gyrRaw[3], magRaw[3];
	char  ax[16], ay[16], az[16], r[16], p[16], y[16], dr[16], dp[16], dy[16];
	float rpy[] = {0,0,0,0,0,0};

	while(1)
	{
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

		//c_io_blctrl_setSpeed(0, 700);//1700-iActuation.escLeftSpeed);
		//c_io_blctrl_setSpeed(1, 700);//1700-iActuation.escLeftSpeed);
		
		
		taskENTER_CRITICAL();
		c_io_imu_getComplimentaryRPY(rpy);
		taskEXIT_CRITICAL();

		/// DEBUG
		#if 0    
		// imu data
		c_common_utils_floatToString(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG, r,  3);
		c_common_utils_floatToString(rpy[PV_IMU_PITCH ]*RAD_TO_DEG, p,  3);
		c_common_utils_floatToString(rpy[PV_IMU_YAW   ]*RAD_TO_DEG, y,  3);
		c_common_utils_floatToString(rpy[PV_IMU_DROLL ]*RAD_TO_DEG, dr, 3);
		c_common_utils_floatToString(rpy[PV_IMU_DPITCH]*RAD_TO_DEG, dp, 3);
		c_common_utils_floatToString(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG, dy, 3);
		sprintf(str, "imu -> \t %s \t %s \t %s \t %s \t %s \t %s\n\r", r, p, y, dr, dp, dy);
		c_common_usart_puts(USART2, str);
		#endif

		#if 0
		// control data
		c_common_utils_floatToString(iActuation.servoRight, r,  3);
		c_common_utils_floatToString(iActuation.servoLeft , p,  3);
		c_common_utils_floatToString(iActuation.escRightSpeed, y,  3);
		c_common_utils_floatToString(iActuation.escLeftSpeed , dr, 3);
		sprintf(str, "Control -> \t %s \t %s \t %s \t %s \n\r", r, p, y, dr);
		c_common_usart_puts(USART2, str);
		#endif

		/// SONAR
		#if 1
		sprintf(str, "Distance: %d \n\r", c_io_sonar_read());
    	c_common_usart_puts(USART2, str);
    	#endif

		/// SERVOS		
		#if 1
		// servo actuation
		taskENTER_CRITICAL();
		if(abs(iActuation.servoRight)<90)
			c_io_rx24f_move(2, 150 - iActuation.servoRight);
		if(abs(iActuation.servoLeft)<90)
			c_io_rx24f_move(1, 130 + iActuation.servoLeft);	
		taskEXIT_CRITICAL();
		#endif
		
		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];

		if(pv_interface_io.oAttitude != 0)
      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
		
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

