/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_protocolo.c
  * @author  Ronan Blanchard
  * @version V1.0.0
  * @date    Feb 3, 2018
  * @brief   The HIL protocol to interface the board with the ProVant Simulator
  *  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_datapr_protocolo.h"


void c_datapr_protocolo_synchronization()
{
  int nstarts = 0;
  while(1)
  {
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
        nstarts=0;
      }

    }
  }
}


void c_datapr_protocolo_send(float* data,int n)
{
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

void c_datapr_protocolo_receive(int n, float* data)
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
        interator = 4;
        c_common_usart_putchar(USART2, j) ; // send back the data received
        nfloats++;
        number.i = 0;
      }
    }
  }
}

