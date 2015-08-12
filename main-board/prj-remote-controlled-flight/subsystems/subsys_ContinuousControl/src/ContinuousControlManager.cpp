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
    ms_sample_time(500),
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
    proVant::atitude atitude, atd;
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
	if(interface->pop(atd, &interface->q_atitude_in)){        
		DEBUG(LEVEL_INFO, "Recive message from ") << name_;
		printf("%f\n", atd.roll);
		xs.setZero();
		mpc->Controler(xs);

	}
        //i++;
	//Send and recive uart
	//PROVANT.updateData();
        //DEBUG(LEVEL_INFO, "Data updated ");
	//atitude = PROVANT.getVantData().getAtitude();
	//atitude.roll = i;
        //DEBUG(LEVEL_INFO, "Sending message from ") << name_;
        //interface->push(atitude, interface->q_atitude_out_);

        boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void ContinuousControlManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

