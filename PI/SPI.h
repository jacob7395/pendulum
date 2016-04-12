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

//Global varibles
char 		  SPI_OUT   	[NUMBER_OF_BYTES];
unsigned char SPI_BUFFER	[NUMBER_OF_BYTES];
char		  SPI_IN 		[NUMBER_OF_BYTES];

volatile short Step_Count           = 0;
volatile short Pendelum_Angle_temp  = 0;
volatile float Pendelum_Angle       = 0;

volatile int Packets	 =0;
volatile int BadSOM		 =0;
volatile int PacketFault =0;