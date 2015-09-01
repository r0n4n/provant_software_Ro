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
    PROVANT.init("/dev/ttyO1", 921600);
    DEBUG(LEVEL_INFO, "Connected 1");

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
    proVant::atitude atitude;
    proVant::position position;
    proVant::servos_state servos;
    proVant::controlOutput actuation;
    proVant::controlOutput actuation2;
    float data1[2];
    float data2[2];
    float data3[2];
    int i = 0, n;
    float esln, esls, esrn, esrs, serl, serr;
    // Loop principal!
    while(1) {

    	//Recive states from Discovery
    	PROVANT.updateData();
    	atitude = PROVANT.getVantData().getAtitude();
    	position= PROVANT.getVantData().getPosition();
    	servos= PROVANT.getVantData().getServoState();    //Function made to save curren as alpha and
    	                                                  //voltage as dotalpha in class vant
    	//Send Control to Discovery
    	if(interface->pop(actuation, &interface->q_actuation_in)){
    		/*Control*/
    		data1[0]= actuation.servoLeft;
    		data1[1]= actuation.servoRight;
    		data3[0]= actuation.escLeftNewtons;
    		data3[1]= actuation.escRightNewtons;
    		data2[0]= actuation.escLeftSpeed;
    		data2[1]= actuation.escRightSpeed;
    		PROVANT.multwii2_sendControldataout(data1,data3,data2);
    		PROVANT.multwii_sendstack();
    	}

    	//Test: receives control sent to discovery from discovery
		PROVANT.updateData();
		actuation2=PROVANT.getVantData().getActuation();

//		//Print screen of data received
//		/*Atitude*/
		cout<<"Atitude Received"<<endl;
//		cout<<"Roll= "<<atitude.roll<<endl;
//		cout<<"Pitch= "<<atitude.pitch<<endl;
//		cout<<"Yaw= "<<atitude.yaw<<endl;
//		cout<<"dotRoll= "<<atitude.dotRoll<<endl;
//		cout<<"dotPitch= "<<atitude.dotPitch<<endl;
//		cout<<"dotYaw= "<<atitude.dotYaw<<endl;
//		/*Position*/
		cout<<"Position Received"<<endl;
//		cout<<"X= "<<position.x<<endl;
//		cout<<"Y= "<<position.y<<endl;
//		cout<<"Z= "<<position.z<<endl;
//		cout<<"dotX= "<<position.dotX<<endl;
//		cout<<"dotY= "<<position.dotY<<endl;
//		cout<<"dotZ= "<<position.dotZ<<endl;
//		/*Servos*/
		cout<<"Servos Received"<<endl;
//		cout<<"Alphal= "<<servos.alphal<<endl;
//		cout<<"Alphar= "<<servos.alphar<<endl;
//		cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
//		cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
//		/*Control*/
		cout<<"Control Received"<<endl;
//		cout<<"EscLeftNew= "<<actuation2.escLeftNewtons<<endl;
//		cout<<"EscRightNew= "<<actuation2.escRightNewtons<<endl;
//		cout<<"ServLeft= "<<actuation2.servoLeft<<endl;
//		cout<<"ServRight= "<<actuation2.servoRight<<endl;

		interface->push(position, interface->q_position_out_);
    	interface->push(atitude, interface->q_atitude_out_);
    	interface->push(servos, interface->q_servos_out_);

    	interface->push(position, interface->q_position2_out_);
    	interface->push(atitude, interface->q_atitude2_out_);
    	interface->push(servos, interface->q_servos2_out_);

    	boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void CommLowLevelManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

