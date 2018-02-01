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
#define MODULE_PERIOD	   6//ms
#ifdef HIL
  #define USART2_BAUDRATE    1152000  // HIL communication
#endif

//
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType pv_module_in_lastWakeTime;
char str[256];
GPIOPin pv_module_in_LED4;

float attitude_quaternion[4]={1,0,0,0};


/* Output Message */
pv_msg_input oInputData;

#ifdef HIL
  pv_msg_controlOutput iOutputData;
  double startDelay ;
  double delay ;
  pv_type_datapr_position ref2 ;
#endif

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
 *
 * Initializes the hardware to communicate with the sensors (IMU, SONAR, RECEIVER, SERVOS).
 * Test routines still need to be performed.
 * @param  None
 * @retval None
 */
void module_in_init() 
{

  /* Inicialização do hardware do módulo */
  pv_module_in_LED4 = c_common_gpio_init(GPIOD, GPIO_Pin_12, GPIO_Mode_OUT); //LED4



#ifndef DISABLE_RC
  /* Inicializador do receiver */
   c_rc_receiver_init();
#endif

#ifdef HIL
  /* Inicia a usart2 */
  c_common_usart2_init(USART2_BAUDRATE);
  sincronization() ;
  pv_interface_in.iOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
#else
  /* Inicialização da imu */
  c_common_i2c_init(I2C1);
  c_io_imu_init(I2C1);

  /* Inicializador do sonar */
  c_io_sonar_init();

  /* Inicializador do servos */
  c_io_servos_init();

#endif

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);

  /* Resevar o espaco para a variavel compartilhada */
  pv_interface_in.oInputData  = xQueueCreate(1, sizeof(pv_msg_input));

  oInputData.init=1;
  oInputData.securityStop=0;
  oInputData.flightmode=0;


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

#ifdef HIL
  float state[16] ;
  float output[4] ; // set the output array
#endif


  unsigned int heartBeat=0;
  /////////////////
  bool lock_increment_roll=false, lock_increment_pitch=false, lock_increment_yaw=false, lock_increment_z=false;
  float rpy[6] = {0}, last_valid_sonar_raw=0.35f, position_reference_initial=0.0f;
  int iterations=1, channel_flight_mode=0, sample=0;
  float sonar_raw=0.0f, sonar_raw_real=0.0f, sonar_raw_filter=0.0f, sonar_corrected_debug=0.0f, sonar_corrected=0.0f, sonar_filtered=0.0f, dotZ=0.0f, dotZ_filtered=0.0f;
  float servo_filter_right=0, servo_filter_left=0;
  int n_valid_samples=0;
  long sample_time_gyro_us[1] ={0};
  int16_t torq;
  float barometer[2];
  float temperature;
  long pressure;
  bool pv_module_in_aButton_ant=0;
  int pv_module_cont=0;
  ////////////////

  /*Calibrar dados*/
  float  attitude_yaw_initial=0.0f, attitude_pitch_initial=0.0f,attitude_roll_initial=0.0f;
  float  yaw_aux;
  /*Dados usados no sonar*/
  float k1_1o_10Hz=0.7265, k2_1o_10Hz=0.1367, k3_1o_10Hz=0.1367;
  float k1_2o_10Hz=1.56102, k2_2o_10Hz=-0.64135, k3_2o_10Hz=0.02008, k4_2o_10Hz=0.04017, k5_2o_10Hz=0.02008;
  float sonar_raw_k_minus_1=0.0f, sonar_raw_k_minus_2=0.0f, sonar_filtered_k_minus_1=0.0f, sonar_filtered_k_minus_2=0.0f;
  float dotZ_filtered_k_minus_1=0.0f, dotZ_k_minus_1=0.0f;
  float last_reference_z=0;
  int valid_sonar_measurements=0;

  /*Dados usados na filtragem dos servos*/
  pv_type_datapr_servos servo_real, servo_filtered;
  float servo_r_raw_k_minus_1=0.0f, servo_r_raw_k_minus_2=0.0f, servo_r_filtered_k_minus_1=0.0f, servo_r_filtered_k_minus_2=0.0f;
  float servo_l_raw_k_minus_1=0.0f, servo_l_raw_k_minus_2=0.0f, servo_l_filtered_k_minus_1=0.0f, servo_l_filtered_k_minus_2=0.0f;

  /* Inicializa os dados dos servos*/
  servo_real.alphal=0;
  servo_real.alphar=0;
  servo_real.dotAlphal=0;
  servo_real.dotAlphar=0;

  servo_filtered.alphal=0;
  servo_filtered.alphar=0;
  servo_filtered.dotAlphal=0;
  servo_filtered.dotAlphar=0;

  /* Inicializa os dados da attitude*/
  oInputData.attitude.roll  = 0;
  oInputData.attitude.pitch = 0;
  oInputData.attitude.yaw   = 0;
  oInputData.attitude.dotRoll  = 0;
  oInputData.attitude.dotPitch = 0;
  oInputData.attitude.dotYaw   = 0;

  /* Inicializa os dados da posiçao*/
  oInputData.position.x = 0;
  oInputData.position.y = 0;
  oInputData.position.z = 0;
  oInputData.position.dotX = 0;
  oInputData.position.dotY = 0;
  oInputData.position.dotZ = 0;

  /*Inicializa as referencias*/
  oInputData.position_refrence.x = 0;
  oInputData.position_refrence.y = 0;
  oInputData.position_refrence.z = 0.5;
  oInputData.position_refrence.dotX = 0;
  oInputData.position_refrence.dotY = 0;
  oInputData.position_refrence.dotZ = 0;

  oInputData.attitude_reference.roll  = 0;
  oInputData.attitude_reference.pitch = 0;
  oInputData.attitude_reference.yaw   = 0;
  oInputData.attitude_reference.dotRoll  = 0;
  oInputData.attitude_reference.dotPitch = 0;
  oInputData.attitude_reference.dotYaw   = 0;
  oInputData.enableintegration== false ;


  iOutputData.actuation.servoRight = 0;
  iOutputData.actuation.servoLeft  = 0;
  iOutputData.actuation.escRightNewtons = 0;
  iOutputData.actuation.escLeftNewtons  = 0;
  iOutputData.HIL_mode = false ;

  ref2.x=0;
  ref2.y=0;
  ref2.z=oInputData.position_refrence.z ;
  ref2.dotX=0;
  ref2.dotY=0;
  ref2.dotZ=0;



  while(1)
    {

    /* Leitura do numero de ciclos atuais */
    pv_module_in_lastWakeTime = xTaskGetTickCount();

    if (uxQueueMessagesWaiting(pv_interface_in.iOutputData)!=0)
        xQueueReceive(pv_interface_in.iOutputData, &iOutputData, 0);


    oInputData.heartBeat=heartBeat+=1;


    /* Verifica init*/
    if (iterations > INIT_ITERATIONS)
      oInputData.init = 0; //Sai da fase de inicializacao


    /* toggle pin for debug */
    //c_common_gpio_toggle(LED_builtin_io);




#ifdef HIL
    //oInputData.position_refrence.z = 0.5; /// TO REMOVE THEN !!!!!




    if (oInputData.enableintegration== false) { // if no data needs to be sent we can read de serial port
      ReceiveData(16, state) ;

      // get the position
      oInputData.position.x = state[0];
      oInputData.position.y = state[1];
      oInputData.position.z = state[2];
      oInputData.position.dotX = state[8] ;
      oInputData.position.dotY = state [9] ;
      oInputData.position.dotZ = state[10] ;

      // get servo motor position
      oInputData.servosOutput.servo.alphar=state[6];
      oInputData.servosOutput.servo.alphal=state[7];
      oInputData.servosOutput.servo.dotAlphar=state[14];
      oInputData.servosOutput.servo.dotAlphal=state[15];

      /* get the attitude*/
      oInputData.attitude.roll  = state[3];
      oInputData.attitude.pitch = state[4];
      oInputData.attitude.yaw   = state[5];
      oInputData.attitude.dotRoll  = state[11];
      oInputData.attitude.dotPitch = state[12];
      oInputData.attitude.dotYaw   = state[13];

      oInputData.enableintegration= true ; // enable integration in the controller
    }

    if ( ( oInputData.position.z < oInputData.position_refrence.z*1.1)&&  ( oInputData.position.z > oInputData.position_refrence.z*0.9) ) {
          if (startDelay == 0 ) {
            startDelay = xTaskGetTickCount() ;
          }
          else {
            delay=xTaskGetTickCount()-startDelay ;

          }
        }
        else{
          startDelay = 0 ;
          delay = 0 ;
        }

        if (delay>3000 ) {
          oInputData.position_refrence = ref2 ;

        }

    if(iOutputData.HIL_mode) { // if the controller calculated the control outputs we can send them
      output[0]=iOutputData.actuation.escRightNewtons ;
      output[1]=iOutputData.actuation.escLeftNewtons ;
      output[2]=iOutputData.actuation.servoRight ;
      output[3]=iOutputData.actuation.servoLeft ;

     /* output[0]=oInputData.position_refrence.x;
      output[1]=oInputData.position_refrence.y;
      output[2]=oInputData.position_refrence.z;
      output[3]= oInputData.attitude_reference.yaw ;*/

      SendData(output,4) ;
      oInputData.enableintegration= false ;
      iOutputData.HIL_mode = false ;
    }


#endif


#ifdef ENABLE_IMU
    /*----------------------Tratamento da IMU---------------------*/
    /* Pega e trata os valores da imu */

    c_io_imu_getRaw(oInputData.imuOutput.accRaw, oInputData.imuOutput.gyrRaw, oInputData.imuOutput.magRaw,sample_time_gyro_us);
    c_datapr_MahonyAHRSupdate(attitude_quaternion,oInputData.imuOutput.gyrRaw[0],oInputData.imuOutput.gyrRaw[1],oInputData.imuOutput.gyrRaw[2],oInputData.imuOutput.accRaw[0],oInputData.imuOutput.accRaw[1],oInputData.imuOutput.accRaw[2],oInputData.imuOutput.magRaw[0],oInputData.imuOutput.magRaw[1],oInputData.imuOutput.magRaw[2],sample_time_gyro_us[0]);
    c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
    c_io_imu_EulerMatrix(rpy,oInputData.imuOutput.gyrRaw);
    oInputData.imuOutput.sampleTime =xTaskGetTickCount() -pv_module_in_lastWakeTime;

    /* Saida dos dados de posição limitada a uma variaçao minima */
    if (abs2(rpy[PV_IMU_ROLL]-oInputData.attitude.roll)>ATTITUDE_MINIMUM_STEP)
      oInputData.attitude.roll= rpy[PV_IMU_ROLL];
    if (abs2(rpy[PV_IMU_PITCH]-oInputData.attitude.pitch)>ATTITUDE_MINIMUM_STEP)
      oInputData.attitude.pitch= rpy[PV_IMU_PITCH];
    if (abs2(rpy[PV_IMU_YAW]-yaw_aux)>ATTITUDE_MINIMUM_STEP){
      yaw_aux= rpy[PV_IMU_YAW];
    }

    oInputData.attitude.yaw=yaw_aux-attitude_yaw_initial;

    /* Saida dos dados da velocidade angular*/
    oInputData.attitude.dotRoll  = rpy[PV_IMU_DROLL];
    oInputData.attitude.dotPitch = rpy[PV_IMU_DPITCH];
    oInputData.attitude.dotYaw   = rpy[PV_IMU_DYAW ];

    // A referencia é a orientacao que o UAV é iniciado
    if (oInputData.init){
      attitude_roll_initial = rpy[PV_IMU_ROLL];
      attitude_pitch_initial = rpy[PV_IMU_PITCH];
      attitude_yaw_initial = rpy[PV_IMU_YAW];
    }
#endif

#ifndef DISABLE_RC
    if (c_common_i2c_timeoutAck()==1){
      c_common_i2c_busReset(I2C1);
    }else if(c_common_i2c_timeoutAck()==2)
      c_common_i2c_busReset(I2C2);

    /*----------------------Tratamento da Referencia---------------------*/
    /* Realiza a leitura dos canais do radio-controle */
    oInputData.receiverOutput.joystick[0]=c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);//+100;
    oInputData.receiverOutput.joystick[1]=c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
    oInputData.receiverOutput.joystick[2]=c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
    oInputData.receiverOutput.joystick[3]=c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
    /*oInputData.receiverOutput.aButton	 =c_rc_receiver_getChannel(C_RC_CHANNEL_A);
    oInputData.receiverOutput.bButton    =c_rc_receiver_getChannel(C_RC_CHANNEL_B);*/
    //int aux=c_rc_receiver_getChannel(6);

    //	if (oInputData.receiverOutput.joystick[0] < 0)
    //			oInputData.receiverOutput.joystick[0] = 0;

    /*Referencia de attitude*/
    //oInputData.attitude_reference.roll  = ((float)oInputData.receiverOutput.joystick[2]/100)*REF_ROLL_MAX+REF_ROLL_BIAS;
    //oInputData.attitude_reference.pitch = ((float)oInputData.receiverOutput.joystick[1]/100)*REF_PITCH_MAX+REF_PITCH_BIAS;
    oInputData.attitude_reference.yaw   = oInputData.attitude_reference.yaw + ((float)oInputData.receiverOutput.joystick[3]/100)* REF_YAW_MAX ; // to change ?

    /*Como o canal YAW da valores -100 ou 100 */
    //	if (oInputData.receiverOutput.joystick[3]<0)/
    //		oInputData.flightmode=0;
    //	else{
    //		oInputData.flightmode=1;
    //		oInputData.position_refrence.refz = sonar_filtered;
    //	}

    /* Position reference */
    if (oInputData.receiverOutput.joystick[1]>3 || oInputData.receiverOutput.joystick[1]<-3)
      oInputData.position_refrence.x = oInputData.position_refrence.x + ((float)oInputData.receiverOutput.joystick[1]/100)*REF_X_INCREMENT;
    if (oInputData.receiverOutput.joystick[2]>3 || oInputData.receiverOutput.joystick[2]<-3)
      oInputData.position_refrence.y = oInputData.position_refrence.y+ ((float)oInputData.receiverOutput.joystick[2]/100)*REF_Y_INCREMENT;
    if (oInputData.receiverOutput.joystick[0]>3 || oInputData.receiverOutput.joystick[0]<-3)
      oInputData.position_refrence.z = oInputData.position_refrence.z +((float)oInputData.receiverOutput.joystick[0]/100)*REF_Z_INCREMENT;


    //SendData(output,4) ;


    //	/*Referencia de altitude*/
    //	//Se o canal 3 esta ligado ele muda a referencia de altura se nao esta ligado fica na referencia pasada
    //	// Trothel varia de -100 a 100 -> adiciono 100 para ficar 0-200 e divido para 200 para ficar 0->1
    /*if (oInputData.receiverOutput.joystick[3]<0){
      oInputData.flightmode=0;
    }
    else{
      oInputData.flightmode=1;
      oInputData.position_refrence.z=((float)(oInputData.receiverOutput.joystick[0]+100)/200)*1.5;
    }*/



    /*Como o canal B da valores 1 ou 100 */
   /* if (oInputData.receiverOutput.bButton>50)
      oInputData.enableintegration = true;
    else
      oInputData.enableintegration = false;*/
#endif

#ifdef ENABLE_ALTURA
    /*----------------------Tratamento do Sonar---------------------*/
    /* Executa a leitura do sonar */
    sonar_raw_real  =c_io_sonar_read();
    //	sonar_raw_real  =0;
    sonar_raw= sonar_raw_real/100;
    /////////////////////////////////

    /* Executa a leitura do barometro*/
    if (oInputData.init){
      //	sonar_raw_real=c_io_imu_getAltitude();
    }else{
      //	sonar_raw=c_io_imu_getAltitude()-sonar_raw_real;
    }
    /////////////////////////////////////

    sonar_corrected = (sonar_raw)*cos(oInputData.attitude.roll)*cos(oInputData.attitude.pitch);
    //sonar_corrected=sonar_raw;

    /*Filtrajem das amostras do sonar*/
#ifdef SONAR_FILTER_1_ORDER_10HZ
    //1st order filter with fc=10Hz
    sonar_filtered = k1_1o_10Hz*sonar_filtered_k_minus_1 + k2_1o_10Hz*sonar_corrected + k3_1o_10Hz*sonar_raw_k_minus_1;
    // Filter memory
    sonar_raw_k_minus_1 = sonar_corrected;
    sonar_filtered_k_minus_1 = sonar_filtered;
#elif defined SONAR_FILTER_2_ORDER_10HZ
    //1st order filter with fc=10Hz
    sonar_filtered = k1_2o_10Hz*sonar_filtered_k_minus_1 + k2_2o_10Hz*sonar_filtered_k_minus_2 + k3_2o_10Hz*sonar_corrected + k4_2o_10Hz*sonar_raw_k_minus_1 + k5_2o_10Hz*sonar_raw_k_minus_2;
    // Filter memory
    sonar_raw_k_minus_2 = sonar_raw_k_minus_1;
    sonar_raw_k_minus_1 = sonar_corrected;
    sonar_filtered_k_minus_2 = sonar_filtered_k_minus_1;
    sonar_filtered_k_minus_1 = sonar_filtered;
#else //If no filter is active, the result is the measurement
    sonar_filtered = sonar_corrected;
#endif

    // Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes) - fiz a derivada do sinal filtrado, REVER
    dotZ = (sonar_filtered - oInputData.position.z)*200.0;
    // 1st order filter with fc=10Hz
    dotZ_filtered = k1_1o_10Hz*dotZ_filtered_k_minus_1 + k2_1o_10Hz*dotZ + k3_1o_10Hz*dotZ_k_minus_1;
    // Filter memory
    dotZ_filtered_k_minus_1 = dotZ_filtered;
    dotZ_k_minus_1 = dotZ;

    //Filtered measurements
    oInputData.position.z = sonar_filtered;
    oInputData.position.dotZ = dotZ_filtered;
#endif

#ifdef ENABLE_SERVO
    /*----------------------Tratamento dos servos---------------------*/
    //Leitura da posicao e velocidade atual dos servo motores
    if (!oInputData.init){
      servo_real=c_io_servos_read();

      // Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes) - fiz a derivada do sinal filtrado, REVER
      //servo_real.dotAlphal = (servo_real.alphal - oInputData.servosOutput.servo.alphal)*200;
      //servo_real.dotAlphar = (servo_real.alphar - oInputData.servosOutput.servo.alphar)*200;

      /*Left servo filter*/
      //1st order filter with fc=10Hz
      servo_filtered.dotAlphal = k1_1o_10Hz*servo_l_filtered_k_minus_1 + k2_1o_10Hz*servo_real.dotAlphal + k3_1o_10Hz*servo_l_raw_k_minus_1;
      // Filter memory
      servo_l_raw_k_minus_1 = servo_real.dotAlphal;
      servo_l_filtered_k_minus_1 = servo_filtered.dotAlphal;

      /*Right servo filter*/
      //1st order filter with fc=10Hz
      servo_filtered.dotAlphar = k1_1o_10Hz*servo_r_filtered_k_minus_1 + k2_1o_10Hz*servo_real.dotAlphar + k3_1o_10Hz*servo_r_raw_k_minus_1;
      // Filter memory
      servo_r_raw_k_minus_1 = servo_real.dotAlphar;
      servo_r_filtered_k_minus_1 = servo_filtered.dotAlphar;
    }
    oInputData.servosOutput.servo.alphal=servo_real.alphal;
    oInputData.servosOutput.servo.alphar=servo_real.alphar;
    oInputData.servosOutput.servo.dotAlphal=servo_filtered.dotAlphal;
    oInputData.servosOutput.servo.dotAlphar=servo_filtered.dotAlphar;
#endif

    /*----------------------Seguranças-------------------------------------*/
    // Se o yaw está perto da zona de perigo a emergencia é acionada e o birotor é desligado
    if ( (rpy[PV_IMU_YAW]*RAD_TO_DEG < -160) || (rpy[PV_IMU_YAW]*RAD_TO_DEG > 160) )
      oInputData.securityStop=1;

    if (!oInputData.receiverOutput.aButton){
      if(pv_module_in_aButton_ant==oInputData.receiverOutput.aButton){
        pv_module_cont++;
      }
      if(pv_module_cont>=10){
        oInputData.securityStop = 1;
        pv_module_cont=0;
      }
      pv_module_in_aButton_ant=oInputData.receiverOutput.aButton;
    }
    else{
      if (oInputData.receiverOutput.aButton){
        oInputData.securityStop = 0;
        pv_module_cont=0;
        pv_module_in_aButton_ant=oInputData.receiverOutput.aButton;
      }
    }
    if (oInputData.init)
      iterations++;

    unsigned int timeNow=xTaskGetTickCount();
    oInputData.cicleTime = timeNow - pv_module_in_lastWakeTime;


    /* toggle pin for debug */
    c_common_gpio_toggle(pv_module_in_LED4);

    /* Realiza o trabalho de mutex */
    if(pv_interface_in.oInputData != 0)
      xQueueOverwrite(pv_interface_in.oInputData, &oInputData);

    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &pv_module_in_lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));

    }
}
/* IRQ handlers ------------------------------------------------------------- */

/**
 * @}
 */

/**
 * @}
 */



