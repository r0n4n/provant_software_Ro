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

#define ROLL        0
#define PITCH       1
#define YAW         2

#define X           0
#define Y           1
#define Z           2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t imuBuffer[16];
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

float gyro_rpy[3], acce_rpy[3], filt_rpy[3];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init() {

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
	// Get Accelerometer ID
	c_common_i2c_readBytes(ACCL_ADDR, 0x00, 1, &ACCL_ID);

	// Accelerometer increase G-range (+/- 16G)
	c_common_i2c_writeByte(ACCL_ADDR, 0x31, 0b00001011);

    //  ADXL345 (Accel) POWER_CTL
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 0);
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 16);
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 8);

    // Gyro ID and setup
	c_common_i2c_readBytes(GYRO_ADDR, 0x00, 1, &GYRO_ID);
	c_common_i2c_writeByte(GYRO_ADDR, 22, 24);

    // HMC5883 (Magn) Run in continuous mode
    c_common_i2c_writeByte(MAGN_ADDR, 0x02, 0x00);
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883 //Inicialização para a IMU baseada na MPU6050
    // Clear the 'sleep' bit to start the sensor.
    c_common_i2c_writeByte(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);

    // Alocar o sub i2c -> desligar o I2C Master da MPU, habilitar I2C bypass
    c_common_i2c_writeBit(MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
    c_common_i2c_writeBit(MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN, 1);

    /** \todo Implementar e testar o enabling do bus secundário da MPU, para leitura do HMC.*/
    //c_common_i2c_writeByte(0x1E, 0x02, 0x00);
    //uint8_t hmcid[3];
    //c_common_i2c_readBytes(HMC58X3_ADDR, HMC58X3_R_IDA, 3, hmcid);
#endif
}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 */
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw) {

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
    // Read x, y, z acceleration, pack the data.
	c_common_i2c_readBytes(ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);
    accRaw[0] = ~(((char)imuBuffer[0] | ((char)imuBuffer[1] << 8))-1);
    accRaw[1] = ~(((char)imuBuffer[2] | ((char)imuBuffer[3] << 8))-1);
    accRaw[2] = ~(((char)imuBuffer[4] | ((char)imuBuffer[5] << 8))-1);

    // Read x, y, z from gyro, pack the data
	c_common_i2c_readBytes(GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
	gyrRaw[0] =  (int)imuBuffer[1] | ((int)imuBuffer[0] << 8);
	gyrRaw[1] = ((int)imuBuffer[3] | ((int)imuBuffer[2] << 8)) * -1;
	gyrRaw[2] = ((int)imuBuffer[5] | ((int)imuBuffer[4] << 8)) * -1;

    // Read x, y, z from magnetometer;
    c_common_i2c_readBytes(MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);
    for (unsigned char i =0; i < 3; i++) {
    	buffer[i] = (int)imuBuffer[(i * 2) + 1] | ((int)imuBuffer[i * 2] << 8);
    }
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883
    uint8_t  buffer[14];

    /*----------------------------------------------
    AFS_SEL | Full Scale Range | LSB Sensitivity
    0       | ±2g              | 16384 LSB/g
    1       | ±4g              | 8192 LSB/g
    2       | ±8g              | 4096 LSB/g
    3       | ±16g             | 2048 LSB/g
    ----------------------------------------------*/
    float accScale = 16384.0f;

    c_common_i2c_readBytes(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, buffer);

    accRaw[0] = -1.0f*(float)((((signed char)buffer[0]) << 8) | ((uint8_t)buffer[1] & 0xFF))/accScale;
    accRaw[1] = -1.0f*(float)((((signed char)buffer[2]) << 8) | ((uint8_t)buffer[3] & 0xFF))/accScale;
    accRaw[2] =       (float)((((signed char)buffer[4]) << 8) | ((uint8_t)buffer[5] & 0xFF))/accScale;

    gyrRaw[0] = (((signed char)buffer[8])  << 8) | ((uint8_t)buffer[9]  & 0xFF);
    gyrRaw[1] = (((signed char)buffer[10]) << 8) | ((uint8_t)buffer[11] & 0xFF);
    gyrRaw[2] = (((signed char)buffer[12]) << 8) | ((uint8_t)buffer[13] & 0xFF);

    //for(int i=0; i<3; i++)
    //	accRaw[i] = (0b1000000 & accBuf[i])? (-1*((int16_t)~(0b01111111 & accBuf[i]))) : (int16_t)accBuf[i];
#endif

}

/** \brief Retorna os ângulos RPY através de um filtro complementar simples.
 *
 *
 */
void c_io_imu_getComplimentaryRPY(float * rpy) {
	int acce_raw[3], gyro_raw[3], magn_raw[3];

	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);

	float32_t px, py, pz;

	//arm_sqrt_f32((float32_t)pow(acce_raw[Z],2), pz);

	acce_rpy[ROLL ] = atan2( acce_raw[X], sqrt(pow(acce_raw[Y],2) + pow(acce_raw[Z],2)));// - mean_acce_rpy[ROLL] ;
	acce_rpy[PITCH] = atan2( acce_raw[Y], sqrt(pow(acce_raw[X],2) + pow(acce_raw[Z],2)));// - mean_acce_rpy[PITCH];

	rpy = acce_rpy;
}

/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

