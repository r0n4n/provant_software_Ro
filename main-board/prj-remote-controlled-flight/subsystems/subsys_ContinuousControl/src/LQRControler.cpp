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

//std::chrono::steady_clock::time_point last;
namespace LQR {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Eigen::MatrixXf Ke(4,16);
int nWSR;
/* Exported functions definitions --------------------------------------------*/

LQRControler::LQRControler() {
	Ke=Gain(1);
}

LQRControler::~LQRControler() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf LQRControler::Controler(Eigen::MatrixXf states){
	Eigen::MatrixXf auxu(4,1);
	Eigen::MatrixXf u(4,1);
	Eigen::MatrixXf ur(4,1);
	Eigen::MatrixXf deltaU(4,1);
	MatrixXf xs(16,1);
	MatrixXf deltaxs(16,1);
	MatrixXf xr(16,1);
	//Vectors of reference trajectory and control
	xs<<0,0,3,states.block(3,0,5,1),0,0,0,states.block(11,0,5,1);
	xr=TrajetoryReference();
	ur<<9857.54,9837.48,0,0;
	//Variation of estates
	deltaxs=xs-xr;
	//Control action variation
	deltaU=Ke*deltaxs;
	// Total control action
	auxu=ur+deltaU;
	u(0,0)=auxu(0,0)/1000;
	u(1,0)=auxu(1,0)/1000;
	u(2,0)=auxu(2,0)/1000;
	u(3,0)=auxu(3,0)/1000;
	return u;
}
/* Private functions ------------------------------------------------------- */
Eigen::MatrixXf LQRControler::TrajetoryReference(){
	MatrixXf R(16,1);
	double x,y,z,phi,theta,psi,alphal,alphar;
	double dot_x,dot_y,dot_z,dot_phi,dot_theta,dot_psi,dot_alphal,dot_alphar;
	R.setZero();
	x=0;
	y=0;
	z=3;
	phi=0.0000890176;
	theta=0.0154833;
	psi=0;
	alphar=-0.0154821;
	alphal=-0.0153665;
	dot_x=0;
	dot_y=0;
	dot_z=0;
	dot_phi=0;
	dot_theta=0;
	dot_psi=0;
	dot_alphar=0;
	dot_alphal=0;

	R<<x,y,z,phi,theta,psi,alphal,alphar,dot_x,dot_y,dot_z,dot_phi,
			dot_theta,dot_psi,dot_alphal,dot_alphar;
	return R;
}

Eigen::MatrixXf LQRControler::Gain(int a){
	MatrixXf Ke(4,16);
	switch (a){
		case 1:
			Ke(0,0)=-0.584737;
			Ke(0,1)=-33.650312;
			Ke(0,2)=-88571.117713;
			Ke(0,3)=5931.417158;
			Ke(0,4)=-117.419569;
			Ke(0,5)=111.329386;
			Ke(0,6)=-197.177504;
			Ke(0,7)=50.119926;
			Ke(0,8)=-5.153692;
			Ke(0,9)=-289.360851;
			Ke(0,10)=-25194.433096;
			Ke(0,11)=1337.180768;
			Ke(0,12)=-15.883536;
			Ke(0,13)=49.691133;
			Ke(0,14)=-2.972812;
			Ke(0,15)=0.506488;
			Ke(1,0)=0.272296;
			Ke(1,1)=33.497790;
			Ke(1,2)=-88629.919836;
			Ke(1,3)=-5899.637396;
			Ke(1,4)=41.918582;
			Ke(1,5)=-129.733880;
			Ke(1,6)=90.554377;
			Ke(1,7)=-114.201743;
			Ke(1,8)=2.525875;
			Ke(1,9)=288.063832;
			Ke(1,10)=-25191.001456;
			Ke(1,11)=-1322.111497;
			Ke(1,12)=2.687200;
			Ke(1,13)=-56.048803;
			Ke(1,14)=1.016633;
			Ke(1,15)=-1.704826;
			Ke(2,0)=-0.448041;
			Ke(2,1)=-0.009226;
			Ke(2,2)=-9.063586;
			Ke(2,3)=1.674962;
			Ke(2,4)=-76.009550;
			Ke(2,5)=-38.884090;
			Ke(2,6)=-184.044141;
			Ke(2,7)=-20.803348;
			Ke(2,8)=-3.730061;
			Ke(2,9)=-0.077270;
			Ke(2,10)=-2.733354;
			Ke(2,11)=-0.183371;
			Ke(2,12)=-15.410520;
			Ke(2,13)=-13.776350;
			Ke(2,14)=-9.015478;
			Ke(2,15)=-0.441553;
			Ke(3,0)=-0.449688;
			Ke(3,1)=0.022706;
			Ke(3,2)=-4.559402;
			Ke(3,3)=-3.764409;
			Ke(3,4)=-76.603533;
			Ke(3,5)=38.694762;
			Ke(3,6)=-20.888824;
			Ke(3,7)=-184.503052;
			Ke(3,8)=-3.743173;
			Ke(3,9)=0.185616;
			Ke(3,10)=-1.299671;
			Ke(3,11)=-0.493609;
			Ke(3,12)=-15.460798;
			Ke(3,13)=13.705059;
			Ke(3,14)=-0.442995;
			Ke(3,15)=-9.022101;
			break;
		default:
			Ke.setZero();
			break;
	}
	return Ke;
}
} /* namespace mpc */
