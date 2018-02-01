/**
  ******************************************************************************
  * @file    modules/datapr/protocolo.c
  * @author  Arthur Viana Lara
  * @version V1.0.0
  * @date    Nov 20, 2017
  * @brief   A interface do protocolo de comunicação utilizado no projeto "Plataforma para testes de estratégias de controle para VANTs tilt-rotor via simulação hardware-in-the-loop"
  *****************************************************************************/

#include "protocolo.h"

void SendData(float* data,int n)
{
  // start tag
  union u number;
  for(int i =0;i<n;i++)
  {
    number.f = data[i];
    unsigned int part;
    for(int j = 3 ; j >= 0 ; j--)
    {
    c_common_usart_putchar(USART2, number.s[j]) ;
    }
    while (c_common_usart_available(USART2) ==false){} ;
    c_common_usart_read(USART2);
   }
}

void sincronization()
{
  int nstarts = 0;
  while(1)
  {
    //c_common_usart_putchar(USART2, 0xfd) ; // to remove
    if (c_common_usart_available(USART2) ==true){
      unsigned char j = c_common_usart_read(USART2);

      if(j==0xfe)
      {
        nstarts++;
        if(nstarts==5)
        {
          break;
        }
        c_common_usart_putchar(USART2, 0xfd) ;

      }
      else
      {
        //c_common_usart_putchar(USART2, 0) ;
        nstarts=0;
      }

    }
  }
}



void ReceiveData(int n, float* data)
{
  int nfloats = 0;
  int interator = 4;
  int nstarts = 0;
  int startflag = 0;
  unsigned char j = 0;

  union u number;

  while(nfloats<n)
  {
    j = 0;
    if (c_common_usart_available(USART2) ==true){
      j=c_common_usart_read(USART2);
      interator--;
      number.s[interator] = j;
      if(interator==0)
      {
        (data)[nfloats] = number.f;

        /*if ((data)[nfloats]>100){
          int test = 1 ;
        }*/

        interator = 4;
        c_common_usart_putchar(USART2, j) ; // send back the data received
        //if(nfloats%2==1) //// comentado teste
        nfloats++;
        number.i = 0;
      }
    }
  }
}

