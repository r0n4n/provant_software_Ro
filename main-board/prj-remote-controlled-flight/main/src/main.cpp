/*
 *-----------------------------------------------------------------------------
 *  Filename:    main.cpp
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 */

#include <iostream>

//#include "DataProcessingManager.h"
#include "ContinuousControlManager.h"
#include "CommLowLevelManager.h"
//#include "ModuleManager.h"

// STL
#include <iostream>

// Boost
#include <boost/thread/thread.hpp>

class mainManager {
public:
    mainManager() {}
    ~mainManager() {}

    //ModuleManager* mm;
    //DataProcessingManager* dm;
};

int main(int argc, char ** argv) {
    //DataProcessingManager DataProcessing("DataProcessing:Manager");
    ContinuousControlManager   ContinuousControl("ContinuousControl:Manager");  //AQUI
    CommLowLevelManager        CommLowLevel("CommLowLevel:Manager");  //AQUI
    //ModuleManager         GenericModule("GenericModule:Manager");

    //DataProcessing.interface->q_out_ = &GenericModule.interface->q_in;
    CommLowLevel.interface->q_atitude_out_ = &ContinuousControl.interface->q_atitude_in; //Aqui

    boost::thread th1( boost::bind( &CommLowLevelManager::Run, CommLowLevel) ); //AQUI
    boost::thread th2( boost::bind( &ContinuousControlManager::Run, ContinuousControl) ); //AQUI
    //boost::thread th2( boost::bind( &ModuleManager::Run,         GenericModule ) );

    th1.join();
    th2.join();
}
