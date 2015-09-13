/*
 *-----------------------------------------------------------------------------
 *  Filename:    DataProcessingManager.cpp
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

#include "DataProcessingManager.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>

using namespace std;

DataProcessingManager::DataProcessingManager(std::string name) :
    interface(new DataProcessingInterface("DataProcessing:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(10),
    name_(name)
{

}

DataProcessingManager::~DataProcessingManager()
{

}

void DataProcessingManager::Init()
{
	PROVANT2.init("/dev/ttyO2", 460800);
	DEBUG(LEVEL_INFO, "Connected 0");
    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...


    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void DataProcessingManager::Run()
{
    Init();

    // Algumas variaveis...
    proVant::atitude atitude;
    proVant::position position;
    proVant::servos_state servos;
    proVant::controlOutput actuation;
    float rpy[3];
    float drpy[3];
    float trajectory[3];
    float velocity[3];
    float alpha[2];
    float dalpha[2];
    int aux[2];
    float aux2[3];
    float servoTorque[2];
    float escForce[2];

    int i = 0;

    // Loop principal!
    while(1) {
        //...
    	auto start2 = std::chrono::steady_clock::now();
    	if(interface->pop(atitude, &interface->q_atitude_in)){
    		/*Atitude*/
    		rpy[0]=atitude.roll;
    		rpy[1]=atitude.pitch;
    		rpy[2]=atitude.yaw;
    		drpy[0]=atitude.dotRoll;
    		drpy[1]=atitude.dotPitch;
    		drpy[2]=atitude.dotYaw;
    	}
    	if(interface->pop(position, &interface->q_position_in)){
    		/*Position*/
    		trajectory[0]=position.x;
    		trajectory[1]=position.y;
    		trajectory[2]=position.z;
    		velocity[0]=position.dotX;
    		velocity[1]=position.dotY;
    		velocity[2]=position.dotZ;
    	}
    	if(interface->pop(servos, &interface->q_servos_in)){
    		/*Servos*/
    		alpha[0]=servos.alphal;
    		alpha[1]=servos.alphar;
    		dalpha[0]=servos.dotAlphal;
    		dalpha[1]=servos.dotAlphar;
    	}
    	if(interface->pop(actuation, &interface->q_actuation_in)){
    		/*Control*/
    		servoTorque[0]=actuation.servoLeft;
    		servoTorque[1]=actuation.servoRight;
    		escForce[0]=actuation.escLeftNewtons;
    		escForce[1]=actuation.escRightNewtons;
    		aux2[0]=actuation.escLeftSpeed;
    		aux2[1]=actuation.escRightSpeed;
    	}


    	PROVANT2.multwii_attitude(rpy[0]*RAD_TO_DEG,rpy[1]*RAD_TO_DEG,rpy[2]*RAD_TO_DEG);
    	PROVANT2.multwii2_sendControldatain(rpy,drpy,trajectory,velocity);
    	PROVANT2.multwii_sendstack();
    	PROVANT2.multwii2_sendEscdata(aux,alpha,dalpha);
    	PROVANT2.multwii2_sendControldataout(servoTorque,escForce,aux2);
    	PROVANT2.multwii_sendstack();
    	auto end2 = std::chrono::steady_clock::now();
    	auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);
    	std::cout << "It took me 2: " << (float)(elapsed2.count()/1000) << " miliseconds." << std::endl;
    	boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void DataProcessingManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

