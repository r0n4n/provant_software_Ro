/**
 * @file c_datapr_protocolo.h
 * @author Arthur Viana Lara
 * @date 11 Julho 2017
 * @brief Arquivo header com a interface do protocolo de comunicação utilizado no projeto "Plataforma para testes de estratégias de controle para VANTs tilt-rotor via simulação hardware-in-the-loop"
 *
 */


#ifndef C_DATAPR_PROTOCOLO_H_
#define C_DATAPR_PROTOCOLO_H_

#include <stdlib.h>
#include "stm32f4xx_conf.h"
#include "c_common_uart.h"

/* Exported macro ------------------------------------------------------------*/


/**
 * @brief Tipo especial de dados para manipulação de bits de um número de ponto fixo.
 *
 * Durante o algoritmo, o programador consegue acessar um pedaço de memória com 32 bits através de 3 tipos: Inteiro sem sinal, número de ponto flutuante e vetor de caractéres com tamanho 4. Dessa forma o programador consegue manipular 32 bits em 4 pedaços de 8 bits para comunicação serial e reunir quatro pedaços de 8 bits recebidas durante a comunicação serial para formar um pedaço maior de 32 bits.
 */
union u
{
    unsigned int i; /**< acesso a pedaço de mémória de 32 bits através de tipo inteiro sem sinal. */
    float f; /**< acesso a pedaço de mémória de 32 bits através de tipo número de ponto flutuante. */
    char s[4]; /**< acesso a pedaço de mémória de 32 bits pedaços correspondentes a caractéres. */
};


/**
 * @brief Função que permite o envio de um Array de números de ponto flutuantes via protocolo UART.
 *
 * @param array: Dados a serem enviados.
 * @param n: Tamanho do array de dados a ser enviado.
 * @return Não há valores de retorno
 */
void c_datapr_protocolo_send(float* array,int n);
/**
 * @brief Função que permite o recebimento de um Array de números de ponto flutuantes via protocolo UART.
 *
 * @param array: Dados a serem recebidos.
 * @param n: Tamanho do array de dados a ser recebido.
 * @return Não há valores de retorno
 */
void c_datapr_protocolo_receive(int n, float*);
/**
 * @brief Função para sincronização dos nós de comunicação a ser utilizada antes de entrar no loop infinito de uma dada tarefa
 *
 */
void c_datapr_protocolo_synchronization();

#endif /* C_DATAPR_PROTOCOLO_H_ */
