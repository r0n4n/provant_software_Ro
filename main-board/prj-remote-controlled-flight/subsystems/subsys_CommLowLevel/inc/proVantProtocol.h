/*
 *-----------------------------------------------------------------------------
 *  Filename:    proVantProtocol.h
 *  Header of ProVant protocol Class
 *  Created on: 24/09/2014
 *  Author: Fernando S. Gonçalves
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

#ifndef PROVANTPROTOCOL_H_
#define PROVANTPROTOCOL_H_

#include <stdint.h>   /* Standard types */

#include "uart.h"
#include "vant.h"  

class proVantProtocol {
	uart UARTX;
	vant vantData;
	unsigned char buffer[128];
public:
	proVantProtocol();
	virtual ~proVantProtocol();
	void init(char port[256], int baudRate);
	int  sendMessage(const char* str);
	void sendControlData();
	int updateData();
	void decodeMessage(uint8_t tam, uint8_t msg);
	bool confirmCheckSum(uint8_t tam);
	uint16_t deserialize16(int posInit);
	uint32_t deserialize32(int posInit);
	float decodeFloat(int posInit);
	vant getVantData();
};

#endif /* PROVANTPROTOCOL_H_ */
