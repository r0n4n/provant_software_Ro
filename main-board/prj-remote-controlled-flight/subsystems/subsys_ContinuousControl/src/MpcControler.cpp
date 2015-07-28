/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/MpcControle.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "MpcControler.h"
/* Private typedef -----------------------------------------------------------*/
using namespace loadmodel;
using namespace Eigen;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LoadTranportationModel * Model;
int s; // Number of states
int p; // Inputs number
int q; // Outputs number
int N; // Prediction Horizon
int M; // Prodiction Control
int ts; //Sample time
int lastTime;
MatrixXf A(20,20);
MatrixXf B(20,4);
MatrixXf Wy; // Output weighting
MatrixXf Wu; //Control weighting
Vector4f ur;
namespace MPC {
/* Exported functions definitions --------------------------------------------*/

MpcControler::MpcControler() {
	MatrixXf SumRho(2,2);
	MatrixXf SumLambda(2,2);
	MatrixXf TerminalCost(2,2);
	s=20;
	p=4;
	q=20;
	N=5;
	M=1;
	Model=new LoadTranportationModel();
	Wy.setZero(q*N,q*N);
	Wu.setZero(p*M,p*M);
	//A=Model->MatrixA(0.5,0.5,0.5);
	//B=Model->MatrixB();
	//ur=Model->RefrenceControl(0,0,0);
	SumRho=Model->MatrixSumRho();
	SumLambda=Model->MatrixSumLambda();
	TerminalCost=Model->MatrixTerminalCost();
	for(int i=0;i<q*N;i+=q){
		if(i<q*(N-1)){
			Wy.block(i,i,q,q)=SumRho;
		}else{
			Wy.block(i,i,q,q)=TerminalCost;
		}
	}
	for(int i=0;i<p*M;i+=p){
		Wu.block(i,i,p,p)=SumLambda;
	}
	std::cout<<Wy<<std::endl;
	std::cout<<Wu<<std::endl;
}

MpcControler::~MpcControler() {
	// TODO Auto-generated destructor stub
}

} /* namespace mpc */
