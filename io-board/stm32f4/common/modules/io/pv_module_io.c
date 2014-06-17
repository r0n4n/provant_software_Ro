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
#include "../datapr/c_datapr_MahonyAHRS.h"

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
//long last_IMU_time=0; /** Último valor do SysTick quando a função de leitura da IMU foi executada - para integracão numérica */
bool first_pv_io = true; // Primeira iteracao do codigo
char str[256];
float attitude_quaternion[4]={1,0,0,0};

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;

/* Outboxes buffers*/
pv_msg_datapr_attitude oAttitude;
pv_msg_datapr_position oPosition;
pv_msg_datapr_sensor_time oSensorTime;

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
//	float accRaw[3], gyrRaw[3], magRaw[3]; // TODO Tirar daqui junto com o init

	/* Inicialização do hardware do módulo */
	c_common_i2c_init(I2C1); //imu 
	//c_common_i2c_init(I2C2); //esc

	c_common_usart2_init(460800);

	/* Inicializar do sonar */
	c_io_sonar_init();

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
	c_io_rx24f_setSpeed(1, 20);
	c_io_rx24f_setSpeed(2, 20);
	c_common_utils_delayms(2);
	/* CCW Compliance Margin e CCW Compliance margin */
	c_io_rx24f_write(1, 0x1A,0x03);
	c_io_rx24f_write(1, 0x1B,0x03);
	c_io_rx24f_write(2, 0x1A,0x03);
	c_io_rx24f_write(2, 0x1B,0x03);
	c_common_utils_delayms(2);
	c_io_rx24f_move(1, 130);
	c_io_rx24f_move(2, 150);
	c_common_utils_delayms(100);

	c_io_imu_init(I2C1);   
	//c_io_blctrl_init(I2C1);

	//inicializacao dos PPM
	c_io_blctrl_init_ppm();

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
	pv_interface_io.oAttitude = 0;
	pv_interface_io.oPosition = 0;
	pv_interface_io.oSensorTime = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_io.iActuation == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Caso detecte overflow dos ticks do sistema, soma 25565 TODO rever valor */
long verifyOverflow(deltaT){
	if (deltaT < 0)
		deltaT = deltaT + 25565; //Valor que dá overflow - REVER valor

	return deltaT;
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
	float velAngular[3]={0,0,0};
	int iterations=0;


	while(1)
	{
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);
		
		/// IMU DATA
		#if 1
		 	taskENTER_CRITICAL();
		 	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);
			taskEXIT_CRITICAL();

//			system_time = c_common_utils_millis();
//			if (first_pv_io){
//				sample_time=0;
//				first_pv_io=false;
//			}
//			else
//				sample_time=(float)( verifyOverflow(system_time - last_IMU_time) ) /1000;
//
//			last_IMU_time = system_time;
			c_datapr_MahonyAHRSupdate(attitude_quaternion, velAngular, gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],magRaw[0],magRaw[1],magRaw[2]);
//			c_datapr_MahonyAHRSupdate(attitude_quaternion,gyrRaw[0],gyrRaw[1],gyrRaw[2],accRaw[0],accRaw[1],accRaw[2],0,0,0);
			c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
			c_io_imu_EulerMatrix(rpy, velAngular);
		#endif

		/// SERVOS
		#if 1
			if( (iActuation.servoRight*RAD_TO_DEG<70) && (iActuation.servoRight*RAD_TO_DEG>-70) )
				c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
			if( (iActuation.servoLeft*RAD_TO_DEG<70) && (iActuation.servoLeft*RAD_TO_DEG>-70) )
				c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);
		#endif


unsigned char velo_right, velo_left;
		/// ESCS
		#if 1
			// TODO ISTO É só para testes! TIRAR
//			if ((iActuation.escRightSpeed-6.0f) < 0)
//				iActuation.escRightSpeed = 0.0f;
//			else
//				iActuation.escRightSpeed=iActuation.escRightSpeed-6.0f;
//
//			if ((iActuation.escLeftSpeed-6.0f) < 0)
//				iActuation.escLeftSpeed = 0.0f;
//			else
//				iActuation.escLeftSpeed=iActuation.escLeftSpeed-6.0f;

			/* força para char
			 *  Foram retirados 2 retas, uma para valores baixos (<6) e uma para valores >6. Na verdade pode-se aproximar a curva inteira
			 *  por um polinomio de maior ordem
			 */

//			if (iActuation.escRightSpeed < 6)
				velo_right = (int)(20.332*iActuation.escRightSpeed +1.7466);
//			else
//				velo_right = (int)(12.256*iActuation.escRightSpeed - 39.441);

//			if (iActuation.escLeftSpeed < 6)
				velo_left = (int)(20.332*iActuation.escLeftSpeed +1.7466);
//			else
//				velo_left = (int)(12.256*iActuation.escLeftSpeed - 39.441);


//			if (velo_right > 30)
//				velo_right = 30;
//
//			if (velo_left > 30)
//				velo_left = 30;
			taskENTER_CRITICAL();
//			c_io_blctrl_setSpeed(0, velo_right );
//			c_common_utils_delayus(10);
//			c_io_blctrl_setSpeed(1, velo_left );

			// 100 iteracoes com a thread periodica de 10ms = 1segundo
			if (iterations < 200){
				c_io_blctrl_setSpeed(0, 10 );
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(1, 10 );

//				first_pv_io = false;
			}
			else{
				c_io_blctrl_setSpeed(1, velo_right );
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(0, velo_left );
			}
			taskEXIT_CRITICAL();
		#endif
		
		/// SONAR
		#if 0
			c_common_utils_floatToString(c_io_sonar_read(), r,  3);
			sprintf(str, "Distance: %s \n\r",r );
	    	c_common_usart_puts(USART2, str);
    	#endif

		/// DEBUG
		#if 1
//	    	running_time = running_time + sample_time;
			sprintf(str, "imu -> \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n\r" ,(int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG),
					(int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DROLL  ]*RAD_TO_DEG),
					(int)(rpy[PV_IMU_DPITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG),(int)(iActuation.servoLeft*RAD_TO_DEG*10),
					(int)(iActuation.servoRight*RAD_TO_DEG*10),(int)(iActuation.escLeftSpeed*10),(int)(iActuation.escRightSpeed*10),
					(int)velo_right, (int)velo_left);
			c_common_usart_puts(USART2, str);

		#endif

		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];
//		oAttitude={0,0,0,0,0,0};
//		oSensorTime.IMU_sample_time = sample_time;
		oSensorTime.IMU_sample_time = 0.010f;

		iterations++;

		if(pv_interface_io.oAttitude != 0){
      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);
      		xQueueOverwrite(pv_interface_io.oSensorTime, &oSensorTime);
		}

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

