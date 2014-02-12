/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_IMU_H
#define C_IO_IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define C_IO_IMU_USE_ITG_ADXL_HMC

/* Exported functions ------------------------------------------------------- */
void c_io_imu_init();
void c_io_imu_getRaw(int * accRaw, int * gyroRaw, int * magRaw);
void c_io_imu_getRPY(float * rpy);
void c_io_imu_calibrate();

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
