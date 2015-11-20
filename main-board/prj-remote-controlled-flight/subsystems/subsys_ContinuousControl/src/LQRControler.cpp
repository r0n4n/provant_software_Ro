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
			Ke(0,0)=-0.583861;
			Ke(0,1)=-34.451238;
			Ke(0,2)=-88599.920997;
			Ke(0,3)=5934.811132;
			Ke(0,4)=-115.028804;
			Ke(0,5)=115.309176;
			Ke(0,6)=-185.179454;
			Ke(0,7)=45.385091;
			Ke(0,8)=-5.072437;
			Ke(0,9)=-291.757104;
			Ke(0,10)=-25182.749088;
			Ke(0,11)=1337.337475;
			Ke(0,12)=-15.376982;
			Ke(0,13)=51.101061;
			Ke(0,14)=-2.373232;
			Ke(0,15)=0.252520;
			Ke(1,0)=0.293638;
			Ke(1,1)=34.295339;
			Ke(1,2)=-88658.975531;
			Ke(1,3)=-5903.072553;
			Ke(1,4)=44.267597;
			Ke(1,5)=-132.875156;
			Ke(1,6)=87.190696;
			Ke(1,7)=-104.325633;
			Ke(1,8)=2.666326;
			Ke(1,9)=290.451812;
			Ke(1,10)=-25179.415305;
			Ke(1,11)=-1322.264028;
			Ke(1,12)=3.152860;
			Ke(1,13)=-57.162852;
			Ke(1,14)=0.828654;
			Ke(1,15)=-1.231936;
			Ke(2,0)=-0.533889;
			Ke(2,1)=-0.011689;
			Ke(2,2)=-10.751614;
			Ke(2,3)=2.060808;
			Ke(2,4)=-88.401838;
			Ke(2,5)=-45.716597;
			Ke(2,6)=-212.831300;
			Ke(2,7)=-23.630630;
			Ke(2,8)=-4.376267;
			Ke(2,9)=-0.096426;
			Ke(2,10)=-3.228620;
			Ke(2,11)=-0.197769;
			Ke(2,12)=-17.793800;
			Ke(2,13)=-16.116675;
			Ke(2,14)=-10.202070;
			Ke(2,15)=-0.437549;
			Ke(3,0)=-0.535841;
			Ke(3,1)=0.027941;
			Ke(3,2)=-5.425104;
			Ke(3,3)=-4.527110;
			Ke(3,4)=-89.088430;
			Ke(3,5)=45.494667;
			Ke(3,6)=-23.718247;
			Ke(3,7)=-213.344365;
			Ke(3,8)=-4.391544;
			Ke(3,9)=0.225200;
			Ke(3,10)=-1.539787;
			Ke(3,11)=-0.593585;
			Ke(3,12)=-17.850631;
			Ke(3,13)=16.033379;
			Ke(3,14)=-0.439016;
			Ke(3,15)=-10.208529;
			break;
		default:
			Ke.setZero();
			break;
	}
	return Ke;
}
} /* namespace mpc */
