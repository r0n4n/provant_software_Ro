/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/MpcControle.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   LQR Controler estabilization.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "LQRControler.h"
/* Private typedef -----------------------------------------------------------*/
using namespace Eigen;
using namespace TRAJECTORY;

//std::chrono::steady_clock::time_point last;
namespace LQR {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MatrixXf Ke(4,18);
MatrixXf auxu(4,1);
MatrixXf u(4,1);
MatrixXf ur(4,1);
MatrixXf deltaU(4,1);
MatrixXf xsi(2,1);
MatrixXf xsiant(2,1);
MatrixXf deltaxsi(2,1);
MatrixXf deltaxsiant(2,1);
MatrixXf xs(16,1);
MatrixXf deltaxs(16,1);
MatrixXf xr(16,1);
MatrixXf xs_aumented(18,1);
int nWSR;
float ts;
/* Exported functions definitions --------------------------------------------*/

LQRControler::LQRControler() {
	trajectory=new ReferenceTrajectory();
	Ke=Gain(1);
	deltaxsiant.setZero();
	xsiant.setZero();
	ts=0.015;
}

LQRControler::~LQRControler() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf LQRControler::Controler(Eigen::MatrixXf states){


	//Vectors of reference trajectory and control
	xs<<0,0,3,states.block(3,0,5,1),0,0,0,states.block(11,0,5,1);
	xr=trajectory->TrajetoryReference_LQR();

	//Vector integration of error(Trapezoidal method)
	deltaxsi<<xs(2,0)-xr(2,0),xs(5,0)-xr(5,0);
	xsi=xsiant+ts*(deltaxsi+deltaxsiant)/2;

	// Error state vector
	deltaxs=xs-xr;
	// augmented error state vector
	xs_aumented<<deltaxs,xsi;

	//Control reference
	ur<<9857.54,9837.48,0,0;

	//Control action variation
	deltaU=Ke*xs_aumented;
	// Total control action
	auxu=ur+deltaU;
	//Variable update
	xsiant=xsi;
	deltaxsiant=deltaxsi;


	if(u(0,0)>15000 ){
		u(0,0)=15000;
	}
	if(u(1,0)>15000 ){
		u(1,0)=15000;
	}
	/*The mass in the mathematical model was taken in grams,
	for this reason the controller calculate the forces in g .m/s^2 and the torque in g .m^2/s^2.
	But, the actuators are in the international units N and N. m for this reason the controls
	actions are transforming from g to Kg*/
	u(0,0)=auxu(0,0)/1000;
	u(1,0)=auxu(1,0)/1000;
	u(2,0)=auxu(2,0)/1000;
	u(3,0)=auxu(3,0)/1000;
	return u;
}
/* Private functions ------------------------------------------------------- */

Eigen::MatrixXf LQRControler::Gain(int a){
	MatrixXf Ke(4,18);
	switch (a){
		case 1:
			Ke(0,0)=-0.244068;
			Ke(0,1)=-19.285325;
			Ke(0,2)=-69559.995180;
			Ke(0,3)=5354.312313;
			Ke(0,4)=-63.306232;
			Ke(0,5)=243.304657;
			Ke(0,6)=-114.804908;
			Ke(0,7)=62.424610;
			Ke(0,8)=-2.798968;
			Ke(0,9)=-226.085767;
			Ke(0,10)=-15989.214793;
			Ke(0,11)=1263.232411;
			Ke(0,12)=-6.563035;
			Ke(0,13)=80.057435;
			Ke(0,14)=-1.772429;
			Ke(0,15)=1.050946;
			Ke(0,16)=-114809.776972;
			Ke(0,17)=276.620055;
			Ke(1,0)=0.274290;
			Ke(1,1)=19.274533;
			Ke(1,2)=-69677.267138;
			Ke(1,3)=-5345.976465;
			Ke(1,4)=72.455893;
			Ke(1,5)=-242.134812;
			Ke(1,6)=114.385663;
			Ke(1,7)=-64.185459;
			Ke(1,8)=3.146925;
			Ke(1,9)=225.988761;
			Ke(1,10)=-15992.393903;
			Ke(1,11)=-1256.738676;
			Ke(1,12)=9.250577;
			Ke(1,13)=-79.622428;
			Ke(1,14)=1.772557;
			Ke(1,15)=-1.074450;
			Ke(1,16)=-115052.076994;
			Ke(1,17)=-275.552633;
			Ke(2,0)=-0.236719;
			Ke(2,1)=-0.004062;
			Ke(2,2)=-0.329982;
			Ke(2,3)=0.764044;
			Ke(2,4)=-65.397388;
			Ke(2,5)=-62.706269;
			Ke(2,6)=-177.047725;
			Ke(2,7)=-8.623446;
			Ke(2,8)=-2.758264;
			Ke(2,9)=-0.045055;
			Ke(2,10)=-0.043273;
			Ke(2,11)=-0.342024;
			Ke(2,12)=-13.718970;
			Ke(2,13)=-16.399598;
			Ke(2,14)=-8.166473;
			Ke(2,15)=-0.311702;
			Ke(2,16)=-0.610030;
			Ke(2,17)=-78.987895;
			Ke(3,0)=-0.237379;
			Ke(3,1)=0.010899;
			Ke(3,2)=-0.247997;
			Ke(3,3)=-2.748512;
			Ke(3,4)=-66.068602;
			Ke(3,5)=63.424713;
			Ke(3,6)=-8.919005;
			Ke(3,7)=-178.954435;
			Ke(3,8)=-2.763623;
			Ke(3,9)=0.125909;
			Ke(3,10)=-0.033473;
			Ke(3,11)=-0.268187;
			Ke(3,12)=-13.824924;
			Ke(3,13)=16.488429;
			Ke(3,14)=-0.314372;
			Ke(3,15)=-8.218297;
			Ke(3,16)=-0.439969;
			Ke(3,17)=80.794949;
			break;
		default:
			Ke.setZero();
			break;
	}
	return Ke;
}
} /* namespace mpc */
