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
    ms_sample_time(10),
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
    std::string msg("Hello!");
    proVant::atitude atitude;
    proVant::altitude altitude;
    atitude.roll = 0;
    atitude.pitch = 0;
    atitude.yaw = 0;
    atitude.dotRoll = 0;
    atitude.dotPitch = 0;
    atitude.dotYaw = 0;
    int i = 0, n;
    char *buffer;   
    // Matrix class
    VectorXf xs(20);
    cout<<"init"<<endl;

    // Loop principal!
    while(1) {
    	if(interface->pop(atitude, &interface->q_atitude_in)){
    		//DEBUG(LEVEL_INFO, "Recive message from ") << name_;
    		printf("roll=%f, pitch=%f, yaw=%f\n", atitude.roll, atitude.pitch, atitude.yaw);
    	}
    	if(interface->pop(altitude, &interface->q_altitude_in)){
    		//DEBUG(LEVEL_INFO, "Recive message from ") << name_;
    		printf("reference=%d, altura=%d\n", altitude.estAlt, altitude.vario);
    	}
    	xs.setZero();
    	//mpc->Controler(xs);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void ContinuousControlManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

