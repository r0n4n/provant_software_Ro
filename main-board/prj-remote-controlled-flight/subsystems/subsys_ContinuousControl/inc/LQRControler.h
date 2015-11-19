/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   LQR Controler estabilization.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LQRCONTORLER_H_
#define LQRCONTORLER_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "math.h"
#include <iostream>
#include <chrono>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace LQR {

class LQRControler {
/* Exported functions ------------------------------------------------------- */
public:
	LQRControler();
	virtual ~LQRControler();
	Eigen::MatrixXf Controler(Eigen::MatrixXf states);
/* Private functions ------------------------------------------------------- */
private:
	Eigen::MatrixXf TrajetoryReference();
	Eigen::MatrixXf Gain(int a);
};

} /* namespace mpc */

#endif /* LQRCONTORLER_H_ */
