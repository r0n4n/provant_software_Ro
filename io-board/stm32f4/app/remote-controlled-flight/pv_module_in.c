/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_in.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_in
  * \brief Componentes para o sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de input do VANT.
  * Leituras de todos os sensores. O processamento destes
  * dados brutos é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   5//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
char str[256];
GPIOPin LED_builtin_io;
//GPIOPin debugPin;
float attitude_quaternion[4]={1,0,0,0};
int securityStop=0; //Promove uma parada de seguranca - desliga os atuadores
int init=1; //Se 1 entao o UAV está em fase de inicializacao

/* Output Message */
pv_msg_input oInputData;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_in_init() 
{
	/* Inicialização do hardware do módulo */
	LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_15, GPIO_Mode_OUT);

	/* Inicialização da imu */
	c_common_i2c_init(I2C1); 
	c_io_imu_init(I2C1); 

  /* Inicializador do sonar */
  //c_io_sonar_init();

  /* Inicializador do receiver */
	c_rc_receiver_init();

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);

  /* Resevar o espaco para a variavel compartilhada */
	pv_interface_in.oInputData  = xQueueCreate(1, sizeof(pv_msg_input));
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores como necessário.
  *
  */
void module_in_run() 
{
  unsigned int heartBeat=0;
  /////////////////
  bool lock_increment_roll=false, lock_increment_pitch=false, lock_increment_yaw=false, enable_integration=false, lock_increment_z=false;
  	float rpy[6] = {0}, attitude_yaw_initial=0.0f, last_valid_sonar_raw=0.35f, position_reference_initial=0.0f;
  	int iterations=1, channel_flight_mode=0, sample=0;
  	float sonar_raw=0.0f, sonar_raw_real=0.0f, sonar_raw_filter=0.0f, sonar_corrected_debug=0.0f, sonar_corrected=0.0f, sonar_filtered=0.0f, dotZ=0.0f, dotZ_filtered=0.0f;
   	int valid_sonar_measurements=0;
  	int n_valid_samples=0;
  	long sample_time_gyro_us[1] ={0};
  ////////////////
  	/* Inicializa os dados da Roll Pitch Yaw*/
  	oInputData.attitude.roll  =0;
  	oInputData.attitude.pitch =0;
  	oInputData.attitude.yaw   =0;
	while(1)
	{
    oInputData.heartBeat=heartBeat+=1;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    /* Leitura do numero de ciclos atuais */
		lastWakeTime = xTaskGetTickCount();

	/*----------------------Tratamento da IMU---------------------*/
    /* Pega e trata os valores da imu */
	c_io_imu_getRaw(oInputData.imuOutput.accRaw, oInputData.imuOutput.gyrRaw, oInputData.imuOutput.magRaw,sample_time_gyro_us);
	c_datapr_MahonyAHRSupdate(attitude_quaternion,oInputData.imuOutput.gyrRaw[0],oInputData.imuOutput.gyrRaw[1],oInputData.imuOutput.gyrRaw[2],oInputData.imuOutput.accRaw[0],oInputData.imuOutput.accRaw[1],oInputData.imuOutput.accRaw[2],oInputData.imuOutput.magRaw[0],oInputData.imuOutput.magRaw[1],oInputData.imuOutput.magRaw[2],sample_time_gyro_us[0]);
	c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
	c_io_imu_EulerMatrix(rpy,oInputData.imuOutput.gyrRaw);
	oInputData.imuOutput.sampleTime =xTaskGetTickCount() -lastWakeTime;

    /* Saida dos dados de posição limitada a uma variaçao minima e velocidade angular*/
    if (abs2(rpy[0]-oInputData.attitude.roll)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.roll= rpy[0];
    if (abs2(rpy[1]-oInputData.attitude.pitch)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.pitch= rpy[PV_IMU_PITCH ];
    if (abs2(rpy[2]-oInputData.attitude.yaw)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.yaw= rpy[2];
    oInputData.attitude.dotRoll  = rpy[3];
    oInputData.attitude.dotPitch = rpy[4];
    oInputData.attitude.dotYaw   = rpy[5];

    /*----------------------Tratamento da Referencia---------------------*/

    /* Realiza a laitura dos canais do radio-controle */
	oInputData.receiverOutput.joystick[0]=c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);
	oInputData.receiverOutput.joystick[1]=c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
	oInputData.receiverOutput.joystick[2]=c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
	oInputData.receiverOutput.joystick[3]=c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
	oInputData.receiverOutput.vrPot		 =c_rc_receiver_getChannel(C_RC_CHANNEL_A);
	oInputData.receiverOutput.bButton    =c_rc_receiver_getChannel(C_RC_CHANNEL_B);
	oInputData.receiverOutput.sampleTime =xTaskGetTickCount();

	oInputData.reference.refroll = (REF_ROLL_MAX*oInputData.receiverOutput.joystick[2]/100)+REF_ROLL_BIAS;
	oInputData.reference.refpitch = REF_PITCH_MAX*oInputData.receiverOutput.joystick[1]/100+REF_PITCH_BIAS;
	oInputData.reference.refyaw   = attitude_yaw_initial;// + REF_YAW_MAX*channel_YAW/100;

	/*----------------------Tratamento do Sonar---------------------*/
	/* Executra a leitura do sonar */
	oInputData.sonarOutput.altitude  =c_io_sonar_read();
    oInputData.sonarOutput.sampleTime=xTaskGetTickCount() - lastWakeTime;
    oInputData.cicleTime             =xTaskGetTickCount() - lastWakeTime;

    /*----------------------Init-------------------------------------*/
    //Falta resolver

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    /* Realiza o trabalho de mutex */
	if(pv_interface_in.oInputData != 0)
		xQueueOverwrite(pv_interface_in.oInputData, &oInputData);

    /* A thread dorme ate o tempo final ser atingido */
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



