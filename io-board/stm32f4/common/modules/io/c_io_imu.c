/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  *	\brief Componente para IMU.
  *
  *	Este componente é projetado para implementar funções da IMU da aeronave - leitura
  *	e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  *	outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define I2Cx_imu      I2C2 // i2c of imu

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
	#define GYRO_ADDR   0x68 // The address of ITG3205
	#define ACCL_ADDR   0x53 // The address of ADXL345
	#define MAGN_ADDR   0x1E // The address of HMC5883
	#define GYRO_X_ADDR 0x1D // Start address for x-axis
	#define ACCL_X_ADDR 0x32 // Start address for x-axis
	#define MAGN_X_ADDR 0x03 // Start address for x-axis
#elif defined C_IO_IMU_USE_MPU6050_HMC5883
	#include "c_io_imu_MPU6050.h"

	#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3
	#define HMC_POS_BIAS 1
	#define HMC_NEG_BIAS 2

	// HMC58X3 register map. For details see HMC58X3 datasheet
	#define HMC58X3_R_CONFA 0
	#define HMC58X3_R_CONFB 1
	#define HMC58X3_R_MODE 2
	#define HMC58X3_R_XM 3
	#define HMC58X3_R_XL 4
	#define HMC58X3_R_STATUS 9
	#define HMC58X3_R_IDA 10
	#define HMC58X3_R_IDB 11
	#define HMC58X3_R_IDC 12
#else
	#error "Define an IMU type in `c_io_imu.h`! C_IO_USE_ITG_ADXL, or other!"
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_TypeDef* I2Cx_imu;
uint8_t imuBuffer[16];
long lastIntegrationTime=0; /** Último valor do SysTick quando a função de filtragem foi chamada - para integracão numérica */
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

/* Private function prototypes -----------------------------------------------*/

void c_io_imu_initKalmanFilter();
void c_io_imu_CalculateTransitionMatrix(arm_matrix_instance_f32 StateVector, float * gyro_raw, float deltat);
void c_io_imu_Calculate_H(arm_matrix_instance_f32 H_k, arm_matrix_instance_f32 StateVector);
void c_io_imu_Calculate_h(arm_matrix_instance_f32 h_xk, arm_matrix_instance_f32 StateVector);
float c_io_imu_estimaYaw(float* acce_raw, float* magn_raw);
void c_io_imu_serialPrintData(); // Usado para salvar dados da IMU. Estes dados serão utilizados para inferir a variância dos ruidos dos sensores.

#define PV_IMU_SAMPLETIME  0.005
/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/


/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init(I2C_TypeDef* I2Cx)
{

  I2Cx_imu=I2Cx;

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
	// Get Accelerometer ID
	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, 0x00, 1, &ACCL_ID);

	// Accelerometer increase G-range (+/- 16G)
	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 0x0B);

  //  ADXL345 (Accel) pow_CTL
  c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2D, 8);

  // Gyro ID and setup
	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, 0x00, 1, &GYRO_ID);
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 24); //24 = 0b0001 1000

  // HMC5883 (Magn) Run in continuous mode
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x02, 0x00);
  // configure the B register to default value of Sensor Input Field Range: 1.2Ga
  // +/- 1.2Ga <-> +/- 2047
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x01, 0x20);
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883 //Inicialização para a IMU baseada na MPU6050
  // Clear the 'sleep' bit to start the sensor.
  c_common_i2c_writeByte(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);

  // Alocar o sub i2c -> desligar o I2C Master da MPU, habilitar I2C bypass
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN, 1);

  /** \todo Implementar e testar o enabling do bus secundário da MPU, para leitura do HMC.*/
  //c_common_i2c_writeByte(0x1E, 0x02, 0x00);
  //uint8_t hmcid[3];
  //c_common_i2c_readBytes(HMC58X3_ADDR, HMC58X3_R_IDA, 3, hmcid);
#endif
}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 * O output de cada um dos sensores é escrito nos buffers passados para a função. Os buffers passados
 * devem ter tamanho mínimo 3 (tipo float), e serão escritos sempre com valores dos eixos X, Y e Z
 * de cada sensor lido.
 * Os valores retornados para cada eixo de cada sensor estão em  \f$ g \f$ para o \b acelerômetro,
 * \f$ rad/s \f$ para o \b giroscópio, e \f$ rad \f$ para o \b magnetômetro.
 *
 * @param accRaw Buffer onde serão escritos os dados do acelerômetro.
 * @param gyrRaw Buffer onde serão escritos os dados do giroscópio.
 * @param magRaw Buffer onde serão escritos os dados do magnetômetro.
 */
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw) {

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
    // Read x, y, z acceleration, pack the data.
    uint8_t  buffer[14]={};
  	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);

    accRaw[0] = (int16_t)(imuBuffer[0] | (imuBuffer[1] << 8));
    accRaw[1] = (int16_t)(imuBuffer[2] | (imuBuffer[3] << 8));
    accRaw[2] = (int16_t)(imuBuffer[4] | (imuBuffer[5] << 8));

    // Read x, y, z from gyro, pack the data

    /** A sensitividade do acelerômetro da ITG3200 é dada pela tabela (extraída do datasheet):
     FS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | Reservado        | Reservado
    1       | Reservado        | Reservado
    2       | Reservado        | Reservado
    3       | 2,000°/seg       | 14.375 LSBs °/S
    ***********************************************/
    
    float accScale =14.375f;
    accScale = 0.0174532925f/accScale;//0.0174532925 = PI/180

  	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
  	gyrRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)))*accScale;
  	gyrRaw[1] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)))*accScale;
  	gyrRaw[2] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)))*accScale;

    // Read x, y, z from magnetometer;
    c_common_i2c_readBytes(I2Cx_imu, MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);
   
    magRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)));// X
    magRaw[1] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)));// Y
    magRaw[2] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)));// Z
    
    /** Como dito no link:  http://www.multiwii.com/forum/viewtopic.php?f=8&t=1387&p=10658
    * temosque encontrar os zeros do mag

         X   |     Y    |     Z
    ---------|----------|----------------
    -196/607 | -488/250 | -422/263
    ***********************************************/

    // -100/100
    
    magRaw[1] = (magRaw[1]-(250-488)/2)/3.69;
    magRaw[0] = (magRaw[0]-(607-196)/2)/4.015;
    magRaw[2] = (magRaw[2]-(263-422)/2)/3.425;
    
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883
    uint8_t  buffer[14];
    c_common_i2c_readBytes(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, buffer);

    /** A sensitividade do acelerômetro da MPU6050 é dada pela tabela (extraída do datasheet):
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | ±2g              | 16384 LSB/g
    1       | ±4g              | 8192 LSB/g
    2       | ±8g              | 4096 LSB/g
    3       | ±16g             | 2048 LSB/g
    ***********************************************/
    float accScale = 16384.0f;

    accRaw[0] = -1.0f*(float)((((signed char)buffer[0]) << 8) | ((uint8_t)buffer[1] & 0xFF))/accScale;
    accRaw[1] = -1.0f*(float)((((signed char)buffer[2]) << 8) | ((uint8_t)buffer[3] & 0xFF))/accScale;
    accRaw[2] =       (float)((((signed char)buffer[4]) << 8) | ((uint8_t)buffer[5] & 0xFF))/accScale;

    /** A sensitividade do giroscópio da MPU6050 é dada pela tabela (extraída do datasheet):
    FS_SEL | Full Scale Range | LSB Sensitivity
    -------|------------------|----------------
    0      | ± 250 °/s        | 131 LSB/°/s
    1      | ± 500 °/s        | 65.5 LSB/°/s
    2      | ± 1000 °/s       | 32.8 LSB/°/s
    3      | ± 2000 °/s       | 16.4 LSB/°/s
    ***********************************************/
    float gyrScale = 131.0f;

    gyrRaw[0] = (float)((((signed char)buffer[8])  << 8) | ((uint8_t)buffer[9]  & 0xFF))/gyrScale;
    gyrRaw[1] = (float)((((signed char)buffer[10]) << 8) | ((uint8_t)buffer[11] & 0xFF))/gyrScale;
    gyrRaw[2] = (float)((((signed char)buffer[12]) << 8) | ((uint8_t)buffer[13] & 0xFF))/gyrScale;
#endif

}

/** \brief Retorna os ângulos RPY através de um filtro complementar simples.
 *
 * Implementa apenas uma fusão simples de dados do Giroscópio e Acelerômetro usando a proposta de um filtro complementar.
 *
 * \image html complementary_filter_diagram.jpg "Diagrama de blocos simplificado para um filtro complementar." width=4cm
 */
float last_rpy[]={0,0,0,0,0,0};
void c_io_imu_getComplimentaryRPY(float * rpy) {
	float acce_raw[3], gyro_raw[3], magn_raw[3];
	float acce_rpy[3];

	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);


  #if 1
	//gyro_raw[X] = gyro_raw[X] - mean_gyro_raw[X];
	//gyro_raw[Y] = gyro_raw[Y] - mean_gyro_raw[Y];
	//gyro_raw[Z] = gyro_raw[Z] - mean_gyro_raw[Z];

  /*
	acce_rpy[PV_IMU_PITCH] = atan2( acce_raw[PV_IMU_X], sqrt(pow(acce_raw[PV_IMU_Y],2)
			+ pow(acce_raw[PV_IMU_Z],2)));// - mean_acce_rpy[ROLL] ;
	acce_rpy[PV_IMU_ROLL ] = atan2( acce_raw[PV_IMU_Y], sqrt(pow(acce_raw[PV_IMU_X],2)
			+ pow(acce_raw[PV_IMU_Z],2)));// - mean_acce_rpy[PITCH];
  */

  acce_rpy[PV_IMU_PITCH] = atan(acce_raw[PV_IMU_X]/sqrt(pow(acce_raw[PV_IMU_Y],2) + pow(acce_raw[PV_IMU_Z],2)));
  acce_rpy[PV_IMU_ROLL ] = atan(acce_raw[PV_IMU_Y]/sqrt(pow(acce_raw[PV_IMU_X],2) + pow(acce_raw[PV_IMU_Z],2)));

  float xh = magn_raw[PV_IMU_X]*cos(acce_rpy[PV_IMU_PITCH])+magn_raw[PV_IMU_Y]*sin(acce_rpy[PV_IMU_ROLL ])*sin(acce_rpy[PV_IMU_PITCH])-magn_raw[PV_IMU_Z]*cos(acce_rpy[PV_IMU_ROLL ])*sin(acce_rpy[PV_IMU_PITCH]);
  float yh = magn_raw[PV_IMU_Y]*cos(acce_rpy[PV_IMU_ROLL ])-magn_raw[PV_IMU_Z]*sin(acce_rpy[PV_IMU_ROLL ]);
  acce_rpy[PV_IMU_YAW ] = atan2(yh,xh);

  rpy[PV_IMU_ROLL ] = acce_rpy[PV_IMU_ROLL  ];
  rpy[PV_IMU_PITCH] = acce_rpy[PV_IMU_PITCH ];
  rpy[PV_IMU_YAW  ] = acce_rpy[PV_IMU_YAW   ];
	
  //Filtro complementar
	float a = 0.93;
  float b = 0.93;
  long  IntegrationTime = c_common_utils_millis();
  if(lastIntegrationTime==0) lastIntegrationTime=IntegrationTime+1;
  float IntegrationTimeDiff=(float)(((float)IntegrationTime- (float)lastIntegrationTime)/1000.0);

  rpy[PV_IMU_ROLL  ] =  a*(rpy[PV_IMU_ROLL ] + gyro_raw[PV_IMU_ROLL ]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_ROLL ];
	rpy[PV_IMU_PITCH ] =  a*(rpy[PV_IMU_PITCH] + gyro_raw[PV_IMU_PITCH]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_PITCH];
  rpy[PV_IMU_YAW   ] =  a*(rpy[PV_IMU_YAW  ] + gyro_raw[PV_IMU_YAW  ]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_YAW  ];
  
 
  rpy[PV_IMU_ROLL ] = (IntegrationTimeDiff/(a+IntegrationTimeDiff))*last_rpy[PV_IMU_ROLL ] + rpy[PV_IMU_ROLL ]*( 1-IntegrationTimeDiff/(a+IntegrationTimeDiff));
  rpy[PV_IMU_PITCH] = (IntegrationTimeDiff/(a+IntegrationTimeDiff))*last_rpy[PV_IMU_PITCH] + rpy[PV_IMU_PITCH]*( 1-IntegrationTimeDiff/(a+IntegrationTimeDiff));
  rpy[PV_IMU_YAW  ] = (IntegrationTimeDiff/(a+IntegrationTimeDiff))*last_rpy[PV_IMU_YAW  ] + rpy[PV_IMU_YAW  ]*( 1-IntegrationTimeDiff/(a+IntegrationTimeDiff));
  last_rpy[PV_IMU_ROLL ]  = rpy[PV_IMU_ROLL ];
  last_rpy[PV_IMU_PITCH]  = rpy[PV_IMU_PITCH];
  last_rpy[PV_IMU_YAW  ]  = rpy[PV_IMU_YAW  ];

  rpy[PV_IMU_DROLL ] = (IntegrationTimeDiff/(b+IntegrationTimeDiff))*last_rpy[PV_IMU_DROLL ] + gyro_raw[PV_IMU_ROLL ]*( 1-IntegrationTimeDiff/(b+IntegrationTimeDiff));
  rpy[PV_IMU_DPITCH] = (IntegrationTimeDiff/(b+IntegrationTimeDiff))*last_rpy[PV_IMU_DPITCH] + gyro_raw[PV_IMU_PITCH]*( 1-IntegrationTimeDiff/(b+IntegrationTimeDiff));
  rpy[PV_IMU_DYAW  ] = (IntegrationTimeDiff/(b+IntegrationTimeDiff))*last_rpy[PV_IMU_DYAW  ] + gyro_raw[PV_IMU_YAW  ]*( 1-IntegrationTimeDiff/(b+IntegrationTimeDiff));
  last_rpy[PV_IMU_DROLL ] = rpy[PV_IMU_DROLL ];
  last_rpy[PV_IMU_DPITCH] = rpy[PV_IMU_DPITCH];
  last_rpy[PV_IMU_DYAW  ] = rpy[PV_IMU_DYAW  ];
  

  /*
  rpy[PV_IMU_DPITCH] = gyro_raw[PV_IMU_PITCH];
  rpy[PV_IMU_DROLL ] = gyro_raw[PV_IMU_ROLL];
  rpy[PV_IMU_DYAW  ] = gyro_raw[PV_IMU_YAW];
  */
  
  /*
  last_rpy[PV_IMU_PITCH] = rpy[PV_IMU_PITCH];
  last_rpy[PV_IMU_ROLL ] = rpy[PV_IMU_ROLL ];
  last_rpy[PV_IMU_YAW  ] = rpy[PV_IMU_YAW  ];
  */


	lastIntegrationTime = IntegrationTime;
  #else
  rpy[0]=acce_raw[0];
  rpy[1]=acce_raw[1];
  rpy[2]=acce_raw[2];
  rpy[3]=gyro_raw[0];
  rpy[4]=gyro_raw[1];
  rpy[5]=gyro_raw[2];
  #endif

}

/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}




/** \brief
 *  Usado para salvar dados da IMU. Estes dados serão utilizados para inferir a variância dos ruidos dos sensores.
 *  São printados na orde: acce_x, acce_y, acce_z, gyro_p, gyro_q, gyro_r, magn_Hx, magn_Hy, magn_Hz, timestamp
 *  A frequencia com que os dados são salvos depende do setado para a thread de io, mas pode ser deduzida pelo timestamp
 */
int timestamp_seconds=-1;
int timestamp_minutes=0;
int lastTime=0;
void c_io_imu_serialPrintData(){
	float acce_raw[3], gyro_raw[3], magn_raw[3];
	int factor_acce=10000, factor_gyro=10000, factor_magn=10000, time, deltaT;
	char str[256];

	time=(int)c_common_utils_millis();

	deltaT=time-lastTime;
	if (deltaT < 0)
		deltaT+= 25565; //Valor que dá overflow - REVER valor

	if (timestamp_seconds == -1){
		timestamp_seconds=time;}//melhorar
	else
		timestamp_seconds+=deltaT;

	if (timestamp_seconds >= 60000){
		timestamp_minutes++;
		timestamp_seconds-=60000;}

	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);
	sprintf(str, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n\r" ,(int)(factor_acce*acce_raw[0]),
			(int)(factor_acce*acce_raw[1]), (int)(factor_acce*acce_raw[2]),(int)(factor_gyro*gyro_raw[0]),
			(int)(factor_gyro*gyro_raw[1]),(int)(factor_gyro*gyro_raw[2]), (int)(factor_magn*magn_raw[0]),
			(int)(factor_magn*magn_raw[1]),(int)(factor_magn*magn_raw[2]), timestamp_minutes, timestamp_seconds);

	c_common_usart_puts(USART2, str);

	lastTime=time;
}


/** \brief ---------------- Matrizes Globais utilizadas no Filtro de Kalman ------------------------------------
 * kminus1 = k-1, significa que o valor da variável é relativa a iteracão passada.
 * Abaixo as 3 matrizes que precisamos guardar entre as iteracões, logo foram declaradas como variáveis globais
 */
float32_t TransitionMatrix_kminus1_f32[7][7];
float32_t P_kminus1_f32[7][7];
float32_t StateVector_kminus1_f32[7]={1,0,0,0,1,1,1};
// Declaracão das structs para as Matrizes a serem utilizadas no filtro de kalman - alocacão de mamória
arm_matrix_instance_f32 TransitionMatrix_kminus1; //Matriz de Transicao Phi_(k-1)
arm_matrix_instance_f32 P_kminus1; // Matriz da covariancia do erro. Matriz P da funcao de Lyapunov
arm_matrix_instance_f32 StateVector_kminus1; //Vetor de estados x=[e0 e1 e2 e3 bp bq br]
/* ---------------- FIM DE: Matrizes Globais utilizadas no Filtro de Kalman ------------------------------------ */




/** \brief Calculo da Matriz de Transicao para o filtro de kalman
 *
 * Desenvolvido no software Mathematica com a inversa de Laplace para a funcao fb de acordo com
 * o artigo "Automation of small UAVs using a low cost MEMs sensor and Embedded Computing Platform".
 *
 * Sempre guarda os valores na matriz global TransictionMatrix_kminus1 - $\Phi_{k-1}$
 */
void c_io_imu_CalculateTransitionMatrix(arm_matrix_instance_f32 StateVector, float * gyro_raw, float deltat){
	float a,b,c,d,e0,e1,e2,e3;
	int m;
//arm_matrix_instance_f32
	// Definicoes
	e0=StateVector.pData[0]; e1=StateVector.pData[1]; e2=StateVector.pData[2]; e3=StateVector.pData[3];
	a=0.5*(-StateVector.pData[4] + gyro_raw[0]);//0.5(p-bp)
	b=0.5*(-StateVector.pData[5] + gyro_raw[1]);//0.5(q-bq)
	c=0.5*(-StateVector.pData[6] + gyro_raw[2]);//0.5(r-br)
	d=pow(a,2) + pow(b,2) + pow(c,2); //d=a^2+b^2+c^2

	//TM=TransitionMatrix_kminus1.pData;
	m=TransitionMatrix_kminus1.numCols;

	// Calculo da matriz de Transicao - relembrando que para acessar os elementos -> A(i,j)=pData[i*numCols+j]
	TransitionMatrix_kminus1.pData[0*m + 0]=cos(sqrt(d)*deltat);
	TransitionMatrix_kminus1.pData[0*m + 1]=-((a*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[0*m + 2]=-((b*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[0*m + 3]=-((c*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[0*m + 4]=(-((a*e0 - c*e2 + b*e3)*(-1 + cos(sqrt(d)*deltat))) +
	    sqrt(d)*e1*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[0*m + 5]=(-((b*e0 + c*e1 - a*e3)*(-1 + cos(sqrt(d)*deltat))) +
	    sqrt(d)*e2*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[0*m + 6]=(-((c*e0 - b*e1 + a*e2)*(-1 + cos(sqrt(d)*deltat))) +
	    sqrt(d)*e3*sin(sqrt(d)*deltat))/(2.*d);

	TransitionMatrix_kminus1.pData[1*m + 0]=(a*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[1*m + 1]=cos(sqrt(d)*deltat);
	TransitionMatrix_kminus1.pData[1*m + 2]=(c*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[1*m + 3]=-((b*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[1*m + 4]=-((a*e1 - b*e2 - c*e3)*(-1 + cos(sqrt(d)*deltat)) +
	     sqrt(d)*e0*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[1*m + 5]=((c*e0 - b*e1 - a*e2)*(-1 + cos(sqrt(d)*deltat)) +
	    sqrt(d)*e3*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[1*m + 6]=-((b*e0 + c*e1 + a*e3)*(-1 + cos(sqrt(d)*deltat)) +
	     sqrt(d)*e2*sin(sqrt(d)*deltat))/(2.*d);

	TransitionMatrix_kminus1.pData[2*m + 0]=(b*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[2*m + 1]=-((c*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[2*m + 2]=cos(sqrt(d)*deltat);
	TransitionMatrix_kminus1.pData[2*m + 3]=(a*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[2*m + 4]=-((c*e0 + b*e1 + a*e2)*(-1 + cos(sqrt(d)*deltat)) +
	     sqrt(d)*e3*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[2*m + 5]=((a*e1 - b*e2 + c*e3)*(-1 + cos(sqrt(d)*deltat)) -
	    sqrt(d)*e0*sin(sqrt(d)*deltat))/(2*d);
	TransitionMatrix_kminus1.pData[2*m + 6]=((a*e0 - c*e2 - b*e3)*(-1 + cos(sqrt(d)*deltat)) +
	    sqrt(d)*e1*sin(sqrt(d)*deltat))/(2.*d);

	TransitionMatrix_kminus1.pData[3*m + 0]=(c*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[3*m + 1]=(b*sin(sqrt(d)*deltat))/sqrt(d);
	TransitionMatrix_kminus1.pData[3*m + 2]=-((a*sin(sqrt(d)*deltat))/sqrt(d));
	TransitionMatrix_kminus1.pData[3*m + 3]=cos(sqrt(d)*deltat);
	TransitionMatrix_kminus1.pData[3*m + 4]=((b*e0 - c*e1 - a*e3)*(-1 + cos(sqrt(d)*deltat)) +
	    sqrt(d)*e2*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[3*m + 5]=-((a*e0 + c*e2 + b*e3)*(-1 + cos(sqrt(d)*deltat)) +
	     sqrt(d)*e1*sin(sqrt(d)*deltat))/(2.*d);
	TransitionMatrix_kminus1.pData[3*m + 6]=((a*e1 + b*e2 - c*e3)*(-1 + cos(sqrt(d)*deltat)) -
	    sqrt(d)*e0*sin(sqrt(d)*deltat))/(2.*d);

	TransitionMatrix_kminus1.pData[4*m + 0]=0;
	TransitionMatrix_kminus1.pData[4*m + 1]=0;
	TransitionMatrix_kminus1.pData[4*m + 2]=0;
	TransitionMatrix_kminus1.pData[4*m + 3]=0;
	TransitionMatrix_kminus1.pData[4*m + 4]=1;
	TransitionMatrix_kminus1.pData[4*m + 5]=0;
	TransitionMatrix_kminus1.pData[4*m + 6]=0;
	TransitionMatrix_kminus1.pData[5*m + 0]=0;
	TransitionMatrix_kminus1.pData[5*m + 1]=0;
	TransitionMatrix_kminus1.pData[5*m + 2]=0;
	TransitionMatrix_kminus1.pData[5*m + 3]=0;
	TransitionMatrix_kminus1.pData[5*m + 4]=0;
	TransitionMatrix_kminus1.pData[5*m + 5]=1;
	TransitionMatrix_kminus1.pData[5*m + 6]=0;
	TransitionMatrix_kminus1.pData[6*m + 0]=0;
	TransitionMatrix_kminus1.pData[6*m + 1]=0;
	TransitionMatrix_kminus1.pData[6*m + 2]=0;
	TransitionMatrix_kminus1.pData[6*m + 3]=0;
	TransitionMatrix_kminus1.pData[6*m + 4]=0;
	TransitionMatrix_kminus1.pData[6*m + 5]=0;
	TransitionMatrix_kminus1.pData[6*m + 6]=1;
}


/** \brief Calcula a matriz H
 *
 * \begin{equation}
 * 		H=\frac{\partial h(x)}{\partial x}\left | _{x=\hat{x}(-)}
 * \end{equation}
 *
 */
void c_io_imu_Calculate_H(arm_matrix_instance_f32 H_k, arm_matrix_instance_f32 StateVector){
	float e0,e1,e2,e3;
	// Definicoes
	e0=StateVector.pData[0]; e1=StateVector.pData[1]; e2=StateVector.pData[2]; e3=StateVector.pData[3];

	H_k.pData[0*H_k.numCols + 0]=-2*e2*G;
	H_k.pData[0*H_k.numCols + 1]=2*e3*G;
	H_k.pData[0*H_k.numCols + 2]=-2*e0*G;
	H_k.pData[0*H_k.numCols + 3]=2*e1*G;
	H_k.pData[0*H_k.numCols + 4]=0;
	H_k.pData[0*H_k.numCols + 5]=0;
	H_k.pData[0*H_k.numCols + 6]=0;

	H_k.pData[1*H_k.numCols + 0]=2*e1*G;
	H_k.pData[1*H_k.numCols + 1]=2*e0*G;
	H_k.pData[1*H_k.numCols + 2]=2*e3*G;
	H_k.pData[1*H_k.numCols + 3]=2*e2*G;
	H_k.pData[1*H_k.numCols + 4]=0;
	H_k.pData[1*H_k.numCols + 5]=0;
	H_k.pData[1*H_k.numCols + 6]=0;

	H_k.pData[2*H_k.numCols + 0]=2*e0*G;
	H_k.pData[2*H_k.numCols + 1]=-2*e1*G;
	H_k.pData[2*H_k.numCols + 2]=-2*e2*G;
	H_k.pData[2*H_k.numCols + 3]=2*e3*G;
	H_k.pData[2*H_k.numCols + 4]=0;
	H_k.pData[2*H_k.numCols + 5]=0;
	H_k.pData[2*H_k.numCols + 6]=0;

	H_k.pData[3*H_k.numCols + 0]=(-2*(2*e0*e1*e2 + (pow(e0,2) - pow(e1,2) + pow(e2,2))*e3 +
	      pow(e3,3)))/
	  ((pow(e0 + e2,2) + pow(e1 - e3,2))*
	    (pow(e0 - e2,2) + pow(e1 + e3,2)));
	H_k.pData[3*H_k.numCols + 1]=(-2*(-(pow(e0,2)*e2) + 2*e0*e1*e3 +
	      e2*(pow(e1,2) + pow(e2,2) + pow(e3,2))))/
	  ((pow(e0 + e2,2) + pow(e1 - e3,2))*
	    (pow(e0 - e2,2) + pow(e1 + e3,2)));
	H_k.pData[3*H_k.numCols + 2]=(2*e1*(pow(e0,2) + pow(e1,2) + pow(e2,2)) + 4*e0*e2*e3 -
	    2*e1*pow(e3,2))/
	  ((pow(e0 + e2,2) + pow(e1 - e3,2))*
	    (pow(e0 - e2,2) + pow(e1 + e3,2)));
	H_k.pData[3*H_k.numCols + 3]=(2*(pow(e0,3) + 2*e1*e2*e3 + e0*(pow(e1,2) - pow(e2,2) + pow(e3,2))))/
	  ((pow(e0 + e2,2) + pow(e1 - e3,2))*
	    (pow(e0 - e2,2) + pow(e1 + e3,2)));
	H_k.pData[3*H_k.numCols + 4]=0;
	H_k.pData[3*H_k.numCols + 5]=0;
	H_k.pData[3*H_k.numCols + 6]=0;
}


/** brief Funcão não linear da saida h(x), de acordo com o artigo previamente citado.
 *
 * \begin{equation}
 *  h_B(x_B) = \begin{bmatrix}
 *  		   2g(e_1 e_3 - e_0 e_2)\\
 *  		   2g(e_2 e_3 + e_0 e_1)\\
 * 			   g(e_0^2 - e_1^2 - e_2^2 + e_3^2)\\
 * 			   tan^{-1}\frac{2(e_1 e_2 + e_0 e_3)}{e_0^2 + e_1^2 + e_2^2 + e_3^2}
 * 			   \end{bmatrix}
 * \end{equation}
 *
 */
void c_io_imu_Calculate_h(arm_matrix_instance_f32 h_xk, arm_matrix_instance_f32 StateVector){
	h_xk.pData[0] = 2*(StateVector.pData[1]*StateVector.pData[3] - StateVector.pData[0]*StateVector.pData[2])*G;
	h_xk.pData[1] = 2*(StateVector.pData[2]*StateVector.pData[3] + StateVector.pData[0]*StateVector.pData[1])*G;
	h_xk.pData[2] = (pow(StateVector.pData[0],2) - pow(StateVector.pData[1],2) - pow(StateVector.pData[2],2) + pow(StateVector.pData[3],2))*G;
	h_xk.pData[3] = atan2(2*(StateVector.pData[1]*StateVector.pData[2] + StateVector.pData[0]*StateVector.pData[3]) ,
			(pow(StateVector.pData[0],2) + pow(StateVector.pData[1],2) - pow(StateVector.pData[2],2) - pow(StateVector.pData[3],2)));
}


/** \brief Faz a estimacão do angulo de guinada (Yaw-$\psi$) com as medicoes do acelerometro e do acelerometro.
 *
 *   Para conseguir estimar o angulo de guinada, precisamos primeiro estimar os angulos de rolagem e arfagem. Fazemos isto com as
 *   medicoes (ax,ay,az) do acelerometro de acordo com as seguintes formulas (segundo artigo "Application of UKF for MEMs and Fluxgate sensors based
 *   attitude and heading reference system of carriers"):
 * 		roll_estimado = atan(ay/ax)
 * 		pitch_estimado = asin(ay/g)
 *   Estas estimacões são utilizadas SOMENTE para calcular a guinada, não é utilizado em mais nenhum lugar do filtro de kalman. Com isto e
 *   com a medicão (mx,my,mz) do magnetometro, temos:
 *   	yaw_estimado = atan( (cos(roll)*my - sin(roll)*mz) / ( cos(pitch)*mx+sin(pitch)*sin(roll)*my+cos(roll)*sin(theta)*mz) )
 *
 *   	Estas formulas estão diferentes das utilizadas pelo Patrick no filtro complementar, logo uma discussão seria bom. As
 *   	equacões aqui usadas ao menos são facilmente derivadas.
 */
float c_io_imu_estimaYaw(float* acce_raw, float* magn_raw){
	float roll_est, pitch_est, Hx, Hy;

	roll_est = atan2(acce_raw[1],acce_raw[2]);
	pitch_est = atan2(acce_raw[0],G);

	Hx=cos(pitch_est)*magn_raw[0]+sin(roll_est)*sin(pitch_est)*magn_raw[1]+cos(roll_est)*sin(pitch_est)*magn_raw[2];
	Hy=cos(roll_est)*magn_raw[1] - sin(roll_est)*magn_raw[2];

	return atan2(Hy,Hx); //Retorna o Yaw estimado
}

/** \brief Inicializa as matrizes globais do Filtro de Kalman de acordo com a biblioteca de matrizes do CMSIS.
 *
 */
void c_io_imu_initKalmanFilter(){
	//float acce_raw,gyro_raw,magn_raw;
	//c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw); //para pegar a condicão inicial do gyro.
	float gyro_raw[3]={0,0,0}; // velocidades angulares iniciais consideradas iguais a zero

	// Inicializa P_{k-1} - os valores foram inicializados na declaracão da variável global
	// Inicializado em Zero - REVER SE DEVE SER ISTO
	arm_mat_init_f32(&P_kminus1, 7, 7, (float32_t *)P_kminus1_f32);

	// Inicializa o vetor de estados - os valores foram inicializados na declaracão da variável global
	arm_mat_init_f32(&StateVector_kminus1, 7, 1, (float32_t *)StateVector_kminus1_f32);

	// Inicializa a matriz de transicão (\Phi_{k-1}) para a condicao inicial do vetor de estado
	arm_mat_init_f32(&TransitionMatrix_kminus1, 7, 7, (float32_t *)TransitionMatrix_kminus1_f32);
	c_io_imu_CalculateTransitionMatrix(StateVector_kminus1, gyro_raw, 0.005); // REVER VALOR DELTA_T
}


/** \brief Algoritmo "Attitude Heading Reference System" baseado no artigo "Automation of small UAVS using a
 * low cost MEMS sensor and embedded computing platform" - J.S.Jang e D. Liccardo.
 *  Implementa um filtro de kalman para a atitude do VANT através da fusão das medidas provenientes dos gyroscopios,
 *  acelerometros e magnetometro.
 *  Tentou-se deixar o nome das variáveis o mais fiel possivel em relacão ao artigo.
 *  Variaveis com o sufixo "kminus1" (k-1): significa que o valor da variável é relativa a iteracão passada.
 *
 * Para o calculo de matrizes é utilizado a struct da biblioteca CMSIS. Isto é vantajoso pois as funcoes de matrizes são
 * otimizadas e utilizam a FPU do processador ARM. Abaixo sua estrutura:
 *
 *   typedef struct
 *   {
 *     uint16_t numRows;     // number of rows of the matrix.
 *     uint16_t numCols;     // number of columns of the matrix.
 *     float32_t *pData;     // points to the data of the matrix.
 *   } arm_matrix_instance_f32;
 *
 *   Para acessar o elemento (i,j) da matriz utiliza-se:
 *
 *   Matriz.pData[i*numCols + j]
 *
 *   A funcão
 *   void arm_mat_init_f32 	( 	arm_matrix_instance_f32 *  	S,
 *		uint16_t  	nRows,
 *		uint16_t  	nColumns,
 *		float32_t *  	pData
 *	)
 *	associa a sua matriz S um ponteiro aonde foi previamente alocada uma memória de tamanho nRows*NColumns*sizeof(float32_t)
 *
 *	Todas as operacões com matrizes desta biblioteca precisam de uma variável para guardar a resposta da operacão, como por exemplo
 *	arm_status arm_mat_mult_f32 	( 	const arm_matrix_instance_f32 *  	pSrcA,
 *		const arm_matrix_instance_f32 *  	pSrcB,
 *		arm_matrix_instance_f32 *  	pDst
 *	)
 *	a varivével pDst recebe o resultado da operacão. Infelizmente não conseguimos fazer operacões "nested", o que implica em várias variáveis
 *	intermediárias. No final decidiu-se utilizar 3 variáveis auxiliares, reutilizando-as entre as operacões. Também para deixar o código
 *	mais claro, foi criada uma matriz para cada variável presente no algoritmo (encontrado no artigo). Pode-se pensar em  mais tarde
 *	otimizar o código (perdendo legibilidade).
 *
 *	Em outro momento seria interessante implementar algumas funcões auxiliares, de modo que o programador consiga trabalhar em um nivel mais alto (por exemplo,
 *	uma pilha com operacões push/pop).
 */
void c_io_imu_getKalmanFilterRPY(float * rpy) {

	float acce_raw[3], gyro_raw[3], magn_raw[3]; // Medicoes da IMU
	float psi_mag;
	int error=0, error2=0; //flag setada em 1 caso ocorra erro em alguma operacao de matriz

	/*=======================================   DECLARACÃO DAS MATRIZES   ==========================================================*/

	/*  Para criar e inicializar uma matriz no contexto da biblioteca do CMSIS, 3 passos são necessários:
	 *  + Criar uma struct do tipo arm_matrix_instance_f32;
	 *  + Alocar a memória aonde os dados da matriz será guardada. Pode ser feito com: float_f32 exemplo[n][m], que já aloca a memória desejada.
	 *  + Linkar o ponteiro com a struct criada no primeiro passo com a funcão arm_mat_init_f32
	 */


	/* ----------------------------------  Criacão das structs "arm_matrix_instance_f32" ------------------------ */
	arm_matrix_instance_f32 H_k; // H_(k)
	arm_matrix_instance_f32 StateVector_k_minus; //X_(k)(-)
	//arm_matrix_instance_f32 StateVector_k_plus; //X_(k)(+)
	arm_matrix_instance_f32 P_k_minus; //P_(k)(-)
	//arm_matrix_instance_f32 P_k_plus; //P_(k)(+)
	arm_matrix_instance_f32 KalmanGain_k; //K_(k)
	arm_matrix_instance_f32 Q; // Matriz de covariancia do ruido dos estados, achada com o metodo de Variancia de Allan
	arm_matrix_instance_f32 R; // Matriz de covariancia do ruido das medicoes, achada com o metodo de Variancia de Allan
	arm_matrix_instance_f32 y_k; // Leituras do acelerometro concatenado com o angulo de yaw calculado com as medidas do magnetometro
	arm_matrix_instance_f32 h_xk_minus; // // h(x_k(-) - funcão nao linear que estima as aceleracoes lineares e o angulo de yaw calculados com a atitude estimada
	arm_matrix_instance_f32 auxA; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	arm_matrix_instance_f32 auxB; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	arm_matrix_instance_f32 auxC; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	/* --------------------------------- FIM DE: Criacão das structs "arm_matrix_instance_f32" ------------------------ */


	/* ----------------------------------  Alocacão de memória para as matrizes --------------------------------------- */
	// Declaracao das matrizes. Alocacão de memória para as matrizes. Os ponteiros pata tais memórias serão linkadas
	// com as structs "arm_matrix_instance_f32"
	float32_t H_k_f32[4][7]; //H_(k)
	float32_t StateVector_k_minus_f32[7][1]; //X_(k)(-)
	//float32_t y_k f32[7][1]; //X_(k)(+)
	float32_t P_k_minus_f32[7][7]; //P_(k)(-)
	//float32_t P_k_plus_f32[7][7]; //P_(k)(+)
	float32_t KalmanGain_k_f32[7][4]; //K_(k)
	float32_t y_k_f32[4]; // Leituras do acelerometro concatenado com o angulo de yaw calculado com as medidas do magnetometro
	float32_t h_xk_minus_f32[4]; // // h(x_k(-) - funcão nao linear que estima as aceleracoes lineares e o angulo de yaw calculados com a atitude estimada
	float32_t Q_f32[7]; // Matriz de covariancia do ruido dos estados, achada com o metodo de Variancia de Allan
	float32_t R_f32[7]; // Matriz de covariancia do ruido das medicoes, achada com o metodo de Variancia de Allan
	float32_t auxA_f32[7][7]; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	float32_t auxB_f32[7][7]; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	float32_t auxC_f32[7][7]; // Variavel auxiliar para o calculo com a biblioteca de matrizes do CMSIS
	/* --------------------------------- FIM DE: Alocacão de memória para as matrizes --------------------------------- */


	/* ------------------------------   Linkagem das memórias com as structs "arm_matrix_instance_f32"   ---------------------------------*/
	// Os ponteiro para a memória é guardado na váriavel *pData da struct.
	arm_mat_init_f32(&H_k, 4, 7, (float32_t *)H_k_f32);
	arm_mat_init_f32(&StateVector_k_minus, 7, 1, (float32_t *)StateVector_k_minus_f32);
	//arm_mat_init_f32(&StateVector_k_plus, 7, 1, (float32_t *)StateVector_k_plus_f32);
	arm_mat_init_f32(&P_k_minus, 7, 7, (float32_t *)P_k_minus_f32);
	//arm_mat_init_f32(&P_k_plus, 7, 7, (float32_t *)P_k_plus_f32);
	arm_mat_init_f32(&KalmanGain_k, 7, 4, (float32_t *)KalmanGain_k_f32);
	arm_mat_init_f32(&y_k, 4, 1, (float32_t *)y_k_f32);
	arm_mat_init_f32(&h_xk_minus, 4, 1, (float32_t *)h_xk_minus_f32);
	arm_mat_init_f32(&Q, 7, 7, (float32_t *)Q_f32);
	arm_mat_init_f32(&R, 4, 4, (float32_t *)R_f32);
	arm_mat_init_f32(&auxA, 7, 7, (float32_t *)auxA_f32);
	arm_mat_init_f32(&auxB, 7, 7, (float32_t *)auxB_f32);
	arm_mat_init_f32(&auxC, 7, 7, (float32_t *)auxC_f32);
	/* ----------------------------- FIM DE: Linkagem das memórias com as structs "arm_matrix_instance_f32"   ----------------------------*/

	/*=======================================   FIM DE: DECLARACÃO DAS MATRIZES  ==========================================================*/


	/*=======================================   FILTRO DE KALMAN   ========================================================================*/

	/* --------------------------------------   Condicões Iniciais   ---------------------------------------------------------------------*/
	// A funcão de inicializacão "c_io_imu_initKalmanFilter()" (a qual deveria ser executada antes de executar a Thread com o FK) já
	// inicializa as matrizes TransitionMatrix_kminus1, StateVector_kminus1 e P_kminus1
	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);

	// O vetor de saida é composto pelos acelerometros e pelo angulo psi calculado com as medidas do magnetometro
	psi_mag=atan2(magn_raw[1],magn_raw[0]);
	y_k.pData[0]=acce_raw[0]; y_k.pData[1]=acce_raw[1]; y_k.pData[2]=acce_raw[2]; y_k.pData[3]=c_io_imu_estimaYaw(acce_raw,magn_raw);
	/* --------------------------------------   FIM DE: Condicões Iniciais   -----------------------------------------------------------*/



	/* --------------------------------------   Fase de Predicão   ---------------------------------------------------------------------*/

	/* ----- \hat{x}(-) = \Phi_{k-1} * \hat{x}_{k-1}(+) ----------- */
	if (arm_mat_mult_f32(&TransitionMatrix_kminus1, &StateVector_kminus1, &StateVector_k_minus) != ARM_MATH_SUCCESS)
		error=-1;
	/* ----- FIM -------------------------------------------------- */

	/* ----- P_k(-) = \Phi_{k-1} * P_{k-1} * \Phi_{k-1}^T + Q ----- */
	// 1) auxA = \Phi_{k-1} * P_{k-1}
	if (arm_mat_mult_f32(&TransitionMatrix_kminus1, &P_kminus1, &auxA) != ARM_MATH_SUCCESS)
		error=-2;
	// 2) auxB = \Phi_{k-1}^T
	if (arm_mat_trans_f32(&TransitionMatrix_kminus1, &auxB) != ARM_MATH_SUCCESS)
		error=-3;
	// 3) auxC = auxA * auxB = (\Phi_{k-1} * P_{k-1}) * (\Phi_{k-1}^T)
	if (arm_mat_mult_f32(&auxA, &auxB, &auxC) != ARM_MATH_SUCCESS)
		error=-4;
	// 4) P_k(-) = auxC + Q = [(\Phi_{k-1} * P_{k-1}) * (\Phi_{k-1}^T)] + Q
	if (arm_mat_add_f32(&auxC, &Q, &P_k_minus) != ARM_MATH_SUCCESS)
		error=-5;
	/* ----- FIM -------------------------------------------------- */

	/* ----- \Phi_{k-1} = \Phi_k ---------------------------------- */
	c_io_imu_CalculateTransitionMatrix(StateVector_k_minus,gyro_raw,0.005); // REVER VALOR DELTA_T
	/* ----- FIM -------------------------------------------------- */


	/* --------------------------------------   Fase de Correcão   ---------------------------------------------------------------------*/

	// Calcula matrix H para x_k(-)
	c_io_imu_Calculate_H(H_k,StateVector_k_minus);
	// Calcula vetor h para x_k(-)
	c_io_imu_Calculate_h(h_xk_minus,StateVector_k_minus);

	/* ----- K_k = P_K(-) * H_k^T * [H_k * P_k(-) * H_k^T + R]^{-1} ---------------------------------- */
	// 1) auxA = H_k * P_k(-) - Dimensoes de A - 4x7
	//Ajusta o tamanho do buffer de saida (a memoria alocada continua sendo size(float_32t*7*7))
	auxA.numRows=4; auxA.numCols=7;
	if (arm_mat_mult_f32(&H_k, &P_k_minus, &auxA) != ARM_MATH_SUCCESS)
		error=-6;
	// 2) auxB = H_k^T - Dimensoes de B - 7x4
	auxB.numRows=7; auxB.numCols=4;
	if (arm_mat_trans_f32(&H_k, &auxB) != ARM_MATH_SUCCESS)
		error=-7;
	// 3) auxC = auxA * auxB =  H_k * P_k(-) * H_k^T - Dimensoes de C - 4x4
	auxC.numRows=4; auxC.numCols=4;
	if (arm_mat_mult_f32(&auxA, &auxB, &auxC) != ARM_MATH_SUCCESS)
		error=-8;
	// 4) auxA = auxC + R =  H_k * P_k(-) * H_k^T + R - Dimensoes de A - 4x4
	auxA.numRows=4; auxA.numCols=4;
	if (arm_mat_add_f32(&auxC, &R, &auxA) != ARM_MATH_SUCCESS)
		//error;
	// 5) auxB = auxA^{-1} =  [H_k * P_k(-) * H_k^T + R]^{-1}- Dimensoes de C - 4x4
		//auxC.numRows=4; auxC.numCols=4;
	if (arm_mat_inverse_f32(&auxA, &auxC) == ARM_MATH_SINGULAR)
		error2=-100;
	// 6) auxC = auxB * auxC =  H_k * [H_k * P_k(-) * H_k^T + R]^{-1} - Dimensoes de A - 7x4
		auxA.numRows=7; auxA.numCols=4;
	if (arm_mat_mult_f32(&auxB, &auxC, &auxA) != ARM_MATH_SUCCESS)
		error=-10;
	// 7) K_k = P_k(-) * auxA =  P_k9-) * H_k * [H_k * P_k(-) * H_k^T + R]^{-1} - Dimensoes de K_k - 7x4
	if (arm_mat_mult_f32(&P_k_minus, &auxA, &KalmanGain_k) != ARM_MATH_SUCCESS)
		error=-11;
	/* ----- FIM -------------------------------------------------- */


	/* ----- P_k(+) = P_k(-) - (K_k * H_k * P_k(-)) ---------------------------------- */
	// 1) auxA = H_k * P_k(-) - Dimensoes de A - 4x7
	auxA.numRows=4; auxA.numCols=7;
	if (arm_mat_mult_f32(&H_k, &P_k_minus, &auxA) != ARM_MATH_SUCCESS)
		error=-12;
	// 2) auxB = K_k * auxA = K_k * H_k * P_k(-) - Dimensoes de B - 7x7
	auxB.numRows=7; auxB.numCols=7;
	if (arm_mat_mult_f32(&KalmanGain_k, &auxA, &auxB) != ARM_MATH_SUCCESS)
		error=-13;
	// 3) P_k(+) = P_k(-) - auxB = P_k(-) - (K_k * H_k * P_k(-))
	if (arm_mat_sub_f32(&P_k_minus, &auxB, &P_kminus1) != ARM_MATH_SUCCESS)  //Já guardei no vetor P_{k-1} pois não usaremos mais em nenhum calculo
		error=-14;
	/* ----- FIM -------------------------------------------------- */


	/* ----- \hat{x}_{k}(+) = \hat{x}_{k}(-) + K_k * [y_k - h(\hat{x}_{k}(-))] ---------------------------------- */
	// 1) auxA = y_k - h(\hat{x}_{k}(-)) - Dimensoes de A - 4x1
	auxA.numRows=4; auxA.numCols=1;
	if (arm_mat_sub_f32(&y_k, &h_xk_minus, &auxA) != ARM_MATH_SUCCESS)
		error=-15;
	// 2) auxB = K_k * auxA = K_k * (y_k - h(\hat{x}_{k}(-))) - Dimensoes de A - 7x1
	auxB.numRows=7; auxB.numCols=1;
	if (arm_mat_mult_f32(&KalmanGain_k, &auxA, &auxB) != ARM_MATH_SUCCESS)
		error=-16;
	// 3) \hat{x}_{k}(+) = \hat{x}_{k}(-) + auxB = \hat{x}_{k}(-) + [K_k * (y_k - h(\hat{x}_{k}(-)))]
	if (arm_mat_add_f32(&StateVector_kminus1, &auxB, &auxB) != ARM_MATH_SUCCESS)  //Já guardei no vetor x_{k-1} pois não usaremos mais em nenhum calculo
		error=-17;
	/* ----- FIM -------------------------------------------------- */

	// Estou passando os angulos em QUATERNIONS para testar - FAZER conversao de quaternions para angulos de Euler
	rpy[0]=StateVector_kminus1.pData[0];
	rpy[1]=StateVector_kminus1.pData[1];
	rpy[2]=StateVector_kminus1.pData[2];
	rpy[3]=StateVector_kminus1.pData[3];
	/* FAZER - O Filtro de kalman estima somente a atitude - para estimar a velocidade angular, podemos fazer:
	 * $[\dot{\phi} \; \dot{\theta} \; \dot{\psi}]^T = \Eta^-1 * [p \; q \; r]^T$
	 */
	rpy[4]= error; //Por enquanto só mostra se teve erro
	rpy[5]= error2; //Por enquanto só mostra se teve erro
}


/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

