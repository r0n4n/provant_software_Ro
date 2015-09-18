/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelataManager.cpp
 *  Implementação do gerenciador e instanciador de submodulos.
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 */

#include "ContinuousControlManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace Eigen;
using namespace loadmodel;
using namespace std;

ContinuousControlManager::ContinuousControlManager(std::string name) :
    interface(new ContinuousControlInterface("ContinuousControl:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(12),
    name_(name)
{
	mpc=new MPC::MpcControler();
}

ContinuousControlManager::~ContinuousControlManager()
{

}

void ContinuousControlManager::Init()
{
    //PROVANT.init("/dev/ttyUSB0", 460800);
    //DEBUG(LEVEL_INFO, "Connected");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void ContinuousControlManager::Run()
{
    Init();
    // Algumas variaveis... 
    proVant::atitude atitude;
    proVant::position position;
    proVant::servos_state servos;
    proVant::controlOutput actuation;
    proVant::debug2 debug;
    proVant::rcNormalize rcNormalize;
    int i=0;
    // Matrix class
    VectorXf xs(20);
    cout<<"init"<<endl;
    for (int j=0;j<7;j++)
    	rcNormalize.normChannels[j]=0;
    // Loop principal!
    while(1) {
    	if(interface->pop(atitude, &interface->q_atitude_in)){
    		/*Atitude*/
//    		cout<<"Atitude Received C"<<endl;
//    		cout<<"Roll= "<<atitude.roll<<endl;
//    		cout<<"Pitch= "<<atitude.pitch<<endl;
//    		cout<<"Yaw= "<<atitude.yaw<<endl;
//    		cout<<"dotRoll= "<<atitude.dotRoll<<endl;
//    		cout<<"dotPitch= "<<atitude.dotPitch<<endl;
//    		cout<<"dotYaw= "<<atitude.dotYaw<<endl;
    	}
    	if(interface->pop(position, &interface->q_position_in)){
    		/*Position*/
//    		cout<<"Position Received C"<<endl;
//    		cout<<"X= "<<position.x<<endl;
//    		cout<<"Y= "<<position.y<<endl;
//    		cout<<"Z= "<<position.z<<endl;
//    		cout<<"dotX= "<<position.dotX<<endl;
//    		cout<<"dotY= "<<position.dotY<<endl;
//    		cout<<"dotZ= "<<position.dotZ<<endl;
    	}
    	if(interface->pop(servos, &interface->q_servos_in)){
    		/*Servos*/
//    		cout<<"Servos Received C"<<endl;
//    		cout<<"Alphal= "<<servos.alphal<<endl;
//    		cout<<"Alphar= "<<servos.alphar<<endl;
//    		cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
//    		cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	if(interface->pop(debug, &interface->q_debug_in)){
    		/*Servos*/
    	//  cout<<"Servos Received C"<<endl;
    	//  cout<<"Alphal= "<<servos.alphal<<endl;
    	//  cout<<"Alphar= "<<servos.alphar<<endl;
    	//  cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
    	//  cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	if(interface->pop(rcNormalize, &interface->q_rc_in)){
    	   	/*Servos*/
    	//	cout<<"channel[1]"<<rcNormalize.normChannels[1]<<endl;
    	//	cout<<"Alphar= "<<servos.alphar<<endl;
    	//  cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
    	//  cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	xs.setZero();
    	//mpc->Controler(xs);
    	actuation.servoLeft=((float)rcNormalize.normChannels[1]/84)*1023;
    	actuation.servoRight=((float)rcNormalize.normChannels[1]/84)*1023;
    	actuation.escLeftNewtons=19.19;
    	actuation.escRightNewtons=20.20;
    	actuation.escLeftSpeed=21.21;
    	actuation.escRightSpeed=22.22;
//    	cout<<"Control"<<endl;
//    	cout<<"C-k="<<i<<endl;
//    	cout<<"C-Cannel 1:"<<rcNormalize.normChannels[1]<<endl;
//    	cout<<"C-actuation-left:"<<actuation.servoLeft<<endl;
//    	cout<<"C-actutation-right:"<<actuation.servoRight<<endl;
//    	cout<<"C-sample:"<<ms_sample_time<<endl;

    	interface->push(actuation, interface->q_actuation_out_);
    	interface->push(actuation, interface->q_actuation2_out_);
    	i++;
	    boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void ContinuousControlManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

