/*
 * helloworld.c
 *
 * Copyright 2016  <pi@raspberrypi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */


#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define PIN 3
#define NUMBER_OF_BYTES 10

char SPI_OUT   	[NUMBER_OF_BYTES];
unsigned char SPI_BUFFER	[NUMBER_OF_BYTES];
char SPI_IN 	[NUMBER_OF_BYTES];

volatile short Step_Count           = 0;
volatile short Pendelum_Angle_temp  = 0;
volatile float Pendelum_Angle       = 0;

volatile int i;
volatile unsigned int Angle;
volatile long Position;
volatile unsigned long TimeStamp;
volatile unsigned char chksum;
volatile int ChksumErrorCount;

volatile long TicksNow;

volatile int Packets=0;
volatile int BadSOM=0;
volatile int PacketFault=0;

void SPI_Req_ISR() {

	SPI_OUT[0]=0x55;	// Stat byte
	SPI_OUT[1]=0xe8; // speed lo - 1000 pps
	SPI_OUT[2]=0x03; // speed hi - 1000 pps
	SPI_OUT[3]=0x00; // direction 0 = +ve

	// form cheksum
	for (i=0, SPI_OUT[NUMBER_OF_BYTES-1]=0; i<NUMBER_OF_BYTES-1; ++i)
	{
		SPI_OUT[NUMBER_OF_BYTES-1] += SPI_OUT[i];
    }

	//copy to buffer for tx
	for (i=0; i<NUMBER_OF_BYTES; ++i)
	{
		SPI_BUFFER[i] = SPI_OUT[i];
    }

	// tx 1 byte as a time as SPI master
	for (i=0; i<NUMBER_OF_BYTES; ++i) {
		wiringPiSPIDataRW(0, &SPI_BUFFER[i], 1);
		//wait befor sending next byte
		TicksNow=micros();
		while ((micros()-TicksNow)<50);
	}
	//copy to SPI_IN for SPI_BUFFER(now contains data from arduino)
	for (i=0; i<NUMBER_OF_BYTES; ++i)
	{
		SPI_IN[i] = SPI_BUFFER[i];
    }
	//coung the number of packets recived
	++Packets;
	//check for start byte
	if (SPI_BUFFER[0]==0x55)
	{
		chksum = 0;
		//form the checksum for the resived data
		for (i=0; i<NUMBER_OF_BYTES-1; ++i)
		{
			chksum += SPI_IN[i];
        }
        //if checksum is correct print the spi data
		if (chksum == SPI_BUFFER[NUMBER_OF_BYTES-1])
		{
            printf("Packets recived %02i, Checksum errors %02i\n", Packets, ChksumErrorCount);
            for (i=0; i<NUMBER_OF_BYTES; ++i)
			{
                printf("%02x   ", SPI_OUT[i]);
                printf("%02x\n" , SPI_IN [i]);
            }
            printf(" \n");
            Step_Count = 0;
            Step_Count = Step_Count |  SPI_IN[1];
            Step_Count = Step_Count | (SPI_IN[2]<<8);
            Pendelum_Angle_temp = 0;
            Pendelum_Angle_temp = Pendelum_Angle_temp |  SPI_IN[3];
            Pendelum_Angle_temp = Pendelum_Angle_temp | (SPI_IN[4]<<8);

            Pendelum_Angle = float(Pendelum_Angle_temp)/100;

            printf("%03f\n"  , Pendelum_Angle);
            printf("%02i\n\n", Step_Count);
		}
		//else print data anc a error code
		else
		{
			ChksumErrorCount++;
			for (i=0; i<NUMBER_OF_BYTES; ++i)
			{
                printf("%02x   ", SPI_OUT[i]);
                printf("%02x \n", SPI_IN [i]);
            }
            printf("Checksum error %02x, %02i\n", chksum, ChksumErrorCount);
            printf(" \n");
		}
	}
	//if bad start bit print error
	else
	{
		++BadSOM;
		printf("Bad SOM\n");
	}

	return;

}

int main(int argc, char **argv)
{

	printf("hello wiringPi\n");

	ChksumErrorCount=0;

	wiringPiSetup();

	wiringPiSPISetup(0, 3000000);

	pinMode(0, INPUT);
	pullUpDnControl(0, PUD_UP);
	wiringPiISR(0, INT_EDGE_FALLING, SPI_Req_ISR);

	while(1);

	return 0;
}

