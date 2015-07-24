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

cmake_minimum_required(VERSION 2.8.3)

# CONFIGURATIONS FOR PROJECT  ------------------------------------------------
set(PROJECT_NAME 	Math_Neon)
set(PROJECT_BUILD_DIR	"${CMAKE_CURRENT_LIST_DIR}/build")
set(PROJECT_CONFIG_DIR	"${CMAKE_CURRENT_LIST_DIR}/config")
set(PROJECT_SOURCE_DIR	"${CMAKE_CURRENT_LIST_DIR}/src")
set(PROJECT_INCLUDE_DIR	"${CMAKE_CURRENT_LIST_DIR}/inc")
set(PROJECT_DOC_DIR	"${CMAKE_CURRENT_LIST_DIR}/doc")

# What external projects will include from this subsystem
# set(PROJECT_EXT_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/ext")

# Common directory with libs, incs and srcs
set(PROJECT_COMMON_DIR		"${CMAKE_CURRENT_LIST_DIR}/../../common")
set(PROJECT_COMMON_DIR_SUBSYS	"${CMAKE_CURRENT_LIST_DIR}/../common")

# Building the project tree
file(MAKE_DIRECTORY ${PROJECT_CONFIG_DIR})
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR})
file(MAKE_DIRECTORY ${PROJECT_INCLUDE_DIR})
file(MAKE_DIRECTORY ${PROJECT_DOC_DIR})

# Set the source files
set(MAIN_FILE ${PROJECT_SOURCE_DIR}/math_debug.c)

set(SRCS
	${PROJECT_SOURCE_DIR}/math_acosf.c
	${PROJECT_SOURCE_DIR}/math_asinf.c
    	${PROJECT_SOURCE_DIR}/math_atan2f.c
    	${PROJECT_SOURCE_DIR}/math_atanf.c
    	${PROJECT_SOURCE_DIR}/math_ceilf.c
    	${PROJECT_SOURCE_DIR}/math_cosf.c
    	${PROJECT_SOURCE_DIR}/math_coshf.c
    	${PROJECT_SOURCE_DIR}/math_expf.c
    	${PROJECT_SOURCE_DIR}/math_fabsf.c
    	${PROJECT_SOURCE_DIR}/math_floorf.c
    	${PROJECT_SOURCE_DIR}/math_fmodf.c
    	${PROJECT_SOURCE_DIR}/math_invsqrtf.c
    	${PROJECT_SOURCE_DIR}/math_ldexpf.c
    	${PROJECT_SOURCE_DIR}/math_log10f.c
    	${PROJECT_SOURCE_DIR}/math_logf.c
    	${PROJECT_SOURCE_DIR}/math_mat2.c
    	${PROJECT_SOURCE_DIR}/math_mat3.c
    	${PROJECT_SOURCE_DIR}/math_mat4.c
    	${PROJECT_SOURCE_DIR}/math_modf.c
    	${PROJECT_SOURCE_DIR}/math_powf.c
    	${PROJECT_SOURCE_DIR}/math_runfast.c
    	${PROJECT_SOURCE_DIR}/math_sincosf.c
    	${PROJECT_SOURCE_DIR}/math_sinf.c
    	${PROJECT_SOURCE_DIR}/math_sinfv.c
    	${PROJECT_SOURCE_DIR}/math_sinhf.c
    	${PROJECT_SOURCE_DIR}/math_sqrtf.c
    	${PROJECT_SOURCE_DIR}/math_sqrtfv.c
    	${PROJECT_SOURCE_DIR}/math_tanf.c
    	${PROJECT_SOURCE_DIR}/math_tanhf.c
    	${PROJECT_SOURCE_DIR}/math_vec2.c
    	${PROJECT_SOURCE_DIR}/math_vec3.c
    	${PROJECT_SOURCE_DIR}/math_vec4.c
)

# CREATING CONFIG HEADER FILE WITH SOME BUILD PARAMS ------------------------
#configure_file (
#  "${PROJECT_CONFIG_DIR}/buildParams.h.in"
#  "${PROJECT_CONFIG_DIR}/buildParams.h"
#)

# CONFIGURATIONS FOR PROJECT  ------------------------------------------------
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "")#"-std=c++0x")
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -Wall -ftime-report")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

# specify the cross compiler
#set(CMAKE_C_COMPILER	/usr/bin/arm-linux-gnueabi-gcc)
#set(CMAKE_CXX_COMPILER	/usr/bin/arm-linux-gnueabi-g++)
set(CMAKE_C_COMPILER	arm-linux-gnueabi-gcc-4.7)
set(CMAKE_CXX_COMPILER	arm-linux-gnueabi-g++-4.7)

#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pg" )
#set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pg" )
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu99 -march=armv7-a -mtune=cortex-a8 -mfloat-abi=softfp -mfpu=neon" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -std=gnu99 -march=armv7-a -mtune=cortex-a8 -mfloat-abi=softfp -mfpu=neon" )

# FINDING PACKAGES -----------------------------------------------------------
find_package(PkgConfig REQUIRED)

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK) #linkar a biblioteca de Logging dinamicamente
set(Boost_INCLUDE_DIRS  "/home/richard/lib/boost_1_58_0/spirit/include")
set(Boost_LIBRARIES   	"/home/richard/lib/boost_1_58_0/stage/lib")

# DEBUG MESSAGES -------------------------------------------------------------
# message(status "** Boost Include: ${Boost_INCLUDE_DIR}")
# message(status "** Boost Libraries: ${Boost_LIBRARIES}")
# message(status "** Boost Dirs: ${Boost_LIBRARY_DIRS}")

# BUILDING IT -----------------------------------------------------------------
set(LIB_POSTFIX bundle)

include_directories(
        ${PROJECT_INCLUDE_DIR}
        #${PROJECT_EXT_INCLUDE}
        ${PROJECT_CONFIG_DIR}

	${PROJECT_COMMON_DIR}/inc	
	${PROJECT_COMMON_DIR_SUBSYS}/inc
	${Boost_INCLUDE_DIRS}
)

link_directories(
        ${Boost_LIBRARIES}
	${PROJECT_SOURCE_DIR}
)


add_library("${PROJECT_NAME}_${LIB_POSTFIX}" ${HDRS} ${SRCS})

add_executable(${PROJECT_NAME} ${MAIN_FILE})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX _d)



target_link_libraries(${PROJECT_NAME}
                        pthread
                        boost_system
                        boost_thread
                        boost_log
                        boost_log_setup
			Math_Neon_bundle m
                     )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BUILD_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BUILD_DIR})


