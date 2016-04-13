/**********************************
Contaions SPI interups funtion


**********************************/

#define PIN 3
#define NUMBER_OF_BYTES 10

#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

//functoins
void SPI_Req_ISR(void);
void SPI_Init(char *SPI_OUT, char *SPI_IN);
