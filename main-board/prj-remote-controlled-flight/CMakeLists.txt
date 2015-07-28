#/*
#-----------------------------------------------------------------------------
#   Filename:    CMakeLists.txt
#   Template default para projetos.
#-----------------------------------------------------------------------------
#      ___                           _
#     / _ \_ __ ___/\   /\__ _ _ __ | |_
#    / /_)/ '__/ _ \ \ / / _` | '_ \| __|
#   / ___/| | | (_) \ V / (_| | | | | |_
#   \/    |_|  \___/ \_/ \__,_|_| |_|\__|
#
#-----------------------------------------------------------------------------
#
#   Compilando em release/debug (chamar pelo terminal)
#   cmake -DCMAKE_BUILD_TYPE=Debug ..
#
#   Para obter informacoes sobre um modulo (ex, FindBoost)
#   cmake --help-module FindBoost
#*/

# GO FROM THAT http://www.cmake.org/cmake/help/examples.html 

cmake_minimum_required(VERSION 2.8.3)

# CONFIGURATIONS FOR PROJECT  ------------------------------------------------
set(PROJECT_NAME 	provant_main_v1)

set(MAIN_DIR		"${CMAKE_CURRENT_LIST_DIR}/main")
set(PROJECT_BUILD_DIR	"${CMAKE_CURRENT_LIST_DIR}/build")
set(PROJECT_CONFIG_DIR	"${MAIN_DIR}/config")
set(PROJECT_SOURCE_DIR	"${MAIN_DIR}/src")
set(PROJECT_INCLUDE_DIR	"${MAIN_DIR}/inc")
set(PROJECT_DOC_DIR	"${MAIN_DIR}/doc")

# Project subsystems and common dirs
set(PROJECT_SUBSYS_DIR 		"${CMAKE_CURRENT_LIST_DIR}/subsystems")
set(PROJECT_COMMON_DIR		"${CMAKE_CURRENT_LIST_DIR}/common")
#set(PROJECT_COMMON_DIR_SUBSYS	"${CMAKE_CURRENT_LIST_DIR}/subsystems/common")

# Building the project tree
file(MAKE_DIRECTORY ${PROJECT_CONFIG_DIR})
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR})
file(MAKE_DIRECTORY ${PROJECT_INCLUDE_DIR})
file(MAKE_DIRECTORY ${PROJECT_DOC_DIR})

# Adding modules via subsystem folders
#add_subdirectory("${PROJECT_SUBSYS_DIR}/subsys_DataProcessing")
add_subdirectory("${PROJECT_SUBSYS_DIR}/subsys_ContinuousControl")
add_subdirectory("${PROJECT_SUBSYS_DIR}/subsys_CommLowLevel")




# Set the source files
set(SRCS
        ${PROJECT_SOURCE_DIR}/main.cpp
	${PROJECT_COMMON_DIR}/inc/AbstractMessageInterface.h
        ${PROJECT_COMMON_DIR}/inc/AbstractModuleManager.h
        ${PROJECT_COMMON_DIR}/inc/debug.h
)

# Project common libraries, includes and sources
# set(CMM_SRCS
# 	${PROJECT_COMMON_DIR}/inc/

# CREATING CONFIG HEADER FILE WITH SOME BUILD PARAMS ------------------------
#configure_file (
#  "${PROJECT_CONFIG_DIR}/buildParams.h.in"
#  "${PROJECT_CONFIG_DIR}/buildParams.h"
#)

# CONFIGURATIONS FOR PROJECT  ------------------------------------------------
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "")
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -Wall -ftime-report")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_INSTALL_PREFIX    "${CMAKE_CURRENT_BINARY_DIR}/dist")

# specify the cross compiler
#set(CMAKE_C_COMPILER	/usr/bin/arm-linux-gnueabi-gcc)
#set(CMAKE_CXX_COMPILER	/usr/bin/arm-linux-gnueabi-g++)
set(CMAKE_C_COMPILER	arm-linux-gnueabi-gcc-4.7)
set(CMAKE_CXX_COMPILER	arm-linux-gnueabi-g++-4.7)

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=armv7-a -mtune=cortex-a8 -mfloat-abi=softfp -mfpu=neon" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -march=armv7-a -mtune=cortex-a8 -mfloat-abi=softfp -mfpu=neon" )
#set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pg" )

# FINDING PACKAGES -----------------------------------------------------------
find_package(PkgConfig REQUIRED)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK) #linkar a biblioteca de Logging dinamicamente
#set(Boost_LIBRARIES    "/home/martin/lib/boost_1_54_0/install/lib")
#set(Boost_INCLUDE_DIRS "/home/martin/lib/boost_1_54_0/install/include")
#set(Boost_LIBRARIES    "/usr/local/Cellar/boost/1.55.0_1/lib")
#set(Boost_INCLUDE_DIRS "/usr/local/Cellar/boost/1.55.0_1/include")
set(Boost_INCLUDE_DIRS  "/home/richard/lib/boost_1_58_0/spirit/include")
set(Boost_LIBRARIES   	"/home/richard/lib/boost_1_58_0/stage/lib")


# DEBUG MESSAGES -------------------------------------------------------------
# message(status "** Boost Include:   ${Boost_INCLUDE_DIR}")
# message(status "** Boost Libraries: ${Boost_LIBRARIES}")
# message(status "** Boost Dirs:      ${Boost_LIBRARY_DIRS}")
#set(Boost_INCLUDE_DIRS    	  "/home/fernando/lib/boost_1_55_0/spirit/include")
#set(Boost_LIBRARIES   		  "/home/fernando/lib/boost_1_55_0/stage/lib")
set(Boost_INCLUDE_DIRS    	  "/home/richard/lib/boost_1_58_0/spirit/include")
set(Boost_LIBRARIES   		  "/home/richard/lib/boost_1_58_0/stage/lib")


# BUILDING IT -----------------------------------------------------------------
include_directories(
        ${PROJECT_INCLUDE_DIR}
        ${PROJECT_CONFIG_DIR}
        ${Boost_INCLUDE_DIRS}
        "${PROJECT_COMMON_DIR}/inc"
        "${PROJECT_SUBSYS_DIR}/common/inc"
	"${PROJECT_SUBSYS_DIR}/common"
	
	# Include all `ext` directories in each subsystem
        #"${PROJECT_SUBSYS_DIR}/subsys_DataProcessing/inc"
        "${PROJECT_SUBSYS_DIR}/subsys_ContinuousControl/inc"
        "${PROJECT_SUBSYS_DIR}/subsys_CommLowLevel/inc"
)

link_directories(
        ${Boost_LIBRARIES}
        #"${PROJECT_SUBSYS_DIR}/subsys_DataProcessing/build"
        "${PROJECT_SUBSYS_DIR}/subsys_ContinuousControl/build"
        "${PROJECT_SUBSYS_DIR}/subsys_CommLowLevel/build"
	
)

add_executable(${PROJECT_NAME} ${HDRS} ${SRCS})

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(${PROJECT_NAME}
                        # Subsystems
                        #subsys_DataProcessing_bundle
                        subsys_ContinuousControl_bundle
			subsys_CommLowLevel_bundle
			# System libraries
                        pthread
                        boost_system
                        boost_thread
                        boost_log
                        boost_log_setup
			
			
 )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BUILD_DIR})


