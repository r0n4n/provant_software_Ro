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
			Ke(0,0)=-0.309680;
			Ke(0,1)=-21.028894;
			Ke(0,2)=-106194.072335;
			Ke(0,3)=6224.613068;
			Ke(0,4)=-112.029178;
			Ke(0,5)=37.977102;
			Ke(0,6)=-372.653135;
			Ke(0,7)=247.291588;
			Ke(0,8)=-3.726269;
			Ke(0,9)=-236.933260;
			Ke(0,10)=-30039.638952;
			Ke(0,11)=1402.945154;
			Ke(0,12)=-14.077987;
			Ke(0,13)=23.403471;
			Ke(0,14)=-11.174013;
			Ke(0,15)=11.196897;
			Ke(1,0)=-0.141951;
			Ke(1,1)=20.889686;
			Ke(1,2)=-106270.519486;
			Ke(1,3)=-6181.647056;
			Ke(1,4)=-41.458771;
			Ke(1,5)=-73.415009;
			Ke(1,6)=70.174448;
			Ke(1,7)=-414.407051;
			Ke(1,8)=-1.175572;
			Ke(1,9)=235.408512;
			Ke(1,10)=-30036.337121;
			Ke(1,11)=-1383.774447;
			Ke(1,12)=-15.549848;
			Ke(1,13)=-36.230753;
			Ke(1,14)=0.039778;
			Ke(1,15)=-15.872808;
			Ke(2,0)=-0.078460;
			Ke(2,1)=-0.000544;
			Ke(2,2)=-2.104017;
			Ke(2,3)=0.212458;
			Ke(2,4)=-22.815676;
			Ke(2,5)=-11.690199;
			Ke(2,6)=-65.006450;
			Ke(2,7)=-1.676207;
			Ke(2,8)=-0.856499;
			Ke(2,9)=-0.006306;
			Ke(2,10)=-0.675407;
			Ke(2,11)=-0.105563;
			Ke(2,12)=-5.217765;
			Ke(2,13)=-4.532048;
			Ke(2,14)=-4.664804;
			Ke(2,15)=-0.022173;
			Ke(3,0)=-0.078770;
			Ke(3,1)=0.002476;
			Ke(3,2)=-0.755671;
			Ke(3,3)=-0.663380;
			Ke(3,4)=-23.033848;
			Ke(3,5)=11.650546;
			Ke(3,6)=-1.755716;
			Ke(3,7)=-65.271042;
			Ke(3,8)=-0.859769;
			Ke(3,9)=0.025472;
			Ke(3,10)=-0.228431;
			Ke(3,11)=-0.077955;
			Ke(3,12)=-5.241344;
			Ke(3,13)=4.515964;
			Ke(3,14)=-0.022985;
			Ke(3,15)=-4.673284;
			break;
		case 2:
			Ke(0,0)=2.097481;
			Ke(0,1)=-17.124932;
			Ke(0,2)=-64142.831806;
			Ke(0,3)=6368.499972;
			Ke(0,4)=1125.460910;
			Ke(0,5)=-285.978908;
			Ke(0,6)=5985.873857;
			Ke(0,7)=8438.876771;
			Ke(0,8)=30.333390;
			Ke(0,9)=-217.534416;
			Ke(0,10)=-19349.320302;
			Ke(0,11)=1414.929196;
			Ke(0,12)=504.607043;
			Ke(0,13)=-161.747997;
			Ke(0,14)=473.413372;
			Ke(0,15)=626.418252;
			Ke(1,0)=-2.146918;
			Ke(1,1)=17.111501;
			Ke(1,2)=-64217.045119;
			Ke(1,3)=-6350.644715;
			Ke(1,4)=-1147.116009;
			Ke(1,5)=230.891523;
			Ke(1,6)=-7061.424440;
			Ke(1,7)=-9110.838394;
			Ke(1,8)=-30.999761;
			Ke(1,9)=217.353228;
			Ke(1,10)=-19345.266748;
			Ke(1,11)=-1408.458157;
			Ke(1,12)=-514.118164;
			Ke(1,13)=132.936915;
			Ke(1,14)=-491.629575;
			Ke(1,15)=-604.985547;
			Ke(2,0)=-0.008753;
			Ke(2,1)=0.000500;
			Ke(2,2)=0.036798;
			Ke(2,3)=-0.212707;
			Ke(2,4)=-4.454171;
			Ke(2,5)=-2.431654;
			Ke(2,6)=-32.869580;
			Ke(2,7)=3.059859;
			Ke(2,8)=-0.126512;
			Ke(2,9)=0.006082;
			Ke(2,10)=-0.002688;
			Ke(2,11)=-0.070190;
			Ke(2,12)=-1.882739;
			Ke(2,13)=-1.493903;
			Ke(2,14)=-3.910832;
			Ke(2,15)=0.365366;
			Ke(3,0)=-0.008806;
			Ke(3,1)=-0.000610;
			Ke(3,2)=0.062524;
			Ke(3,3)=0.238745;
			Ke(3,4)=-4.559879;
			Ke(3,5)=2.436732;
			Ke(3,6)=2.880981;
			Ke(3,7)=-33.401976;
			Ke(3,8)=-0.127285;
			Ke(3,9)=-0.007384;
			Ke(3,10)=0.027195;
			Ke(3,11)=0.022364;
			Ke(3,12)=-1.907851;
			Ke(3,13)=1.500282;
			Ke(3,14)=0.363286;
			Ke(3,15)=-3.939714;
			break;
		default:
			Ke.setZero();
			break;
	}
	return Ke;
}
} /* namespace mpc */
