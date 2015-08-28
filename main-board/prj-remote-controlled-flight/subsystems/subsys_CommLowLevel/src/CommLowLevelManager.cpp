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

#include "CommLowLevelManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;

CommLowLevelManager::CommLowLevelManager(std::string name) :
    interface(new CommLowLevelInterface("CommLowLevel:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(10),
    name_(name)
{

}

CommLowLevelManager::~CommLowLevelManager()
{

}

void CommLowLevelManager::Init()
{
    //PROVANT.init("/dev/ttyO1", 460800);
	PROVANT.init("/dev/ttyO1", 921600);
    DEBUG(LEVEL_INFO, "Connected 1");

    PROVANT2.init("/dev/ttyO2", 460800);
    DEBUG(LEVEL_INFO, "Connected 0");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void CommLowLevelManager::Run()
{
    Init();
    // Algumas variaveis... 
    proVant::atitude atitude, atd;
    proVant::altitude altitude;
    atitude.roll = 0;
    atitude.pitch = 0;
    atitude.yaw = 0;
    atitude.dotRoll = 0;
    atitude.dotPitch = 0;
    atitude.dotYaw = 0;
    int i = 0, n;
    char *buffer;   

    // Loop principal!
    while(1) {
    	//Send and recive uart
    	PROVANT.updateData();
    	atitude = PROVANT.getVantData().getAtitude();
    	cout<<"roll="<<atitude.roll<<",pitch="<<atitude.pitch<<",yaw="<<atitude.yaw<<endl;

    	PROVANT2.multwii_attitude(atitude.roll,atitude.pitch,atitude.yaw);
    	PROVANT2.multwii_sendstack();

    	interface->push(atitude, interface->q_atitude_out_);
    	interface->push(altitude, interface->q_altitude_out_);
    	boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void CommLowLevelManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

