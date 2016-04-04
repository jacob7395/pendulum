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
#define NUMBER_OF_BYTES 5

unsigned char Packet[NUMBER_OF_BYTES];
unsigned char buffer[NUMBER_OF_BYTES];

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

	Packet[0]=0x55;
	Packet[1]=0xe8; // speed lo - 1000 pps
	Packet[2]=0x03; // speed hi - 1000 pps
	Packet[3]=0x00; // direction 0 = +ve

	// form chksum
	for (i=0, Packet[NUMBER_OF_BYTES-1]=0; i<NUMBER_OF_BYTES-1; ++i)
	{
		Packet[NUMBER_OF_BYTES-1] += Packet[i];
    }

	// copy to buffer for tx
	for (i=0; i<NUMBER_OF_BYTES; ++i)
	{
		buffer[i] = Packet[i];
    }

	// tx 1 byte as a time as SPI master
	for (i=0; i<NUMBER_OF_BYTES; ++i) {
		wiringPiSPIDataRW(0, &buffer[i], 1);
		//wait befor sending next byte
		TicksNow=micros();
		while ((micros()-TicksNow)<25);
	}
	++Packets;

	if (buffer[0]==0x55)
	{
		chksum = 0;
		for (i=0; i<NUMBER_OF_BYTES-1; ++i)
		{
			chksum += buffer[i];
        }
		if (chksum==buffer[NUMBER_OF_BYTES])
		{
            printf("%02x\n", buffer[0]);
            printf("%02x\n", buffer[1]);
            printf("%02x\n", buffer[2]);
            printf("%02x\n", buffer[3]);
            printf("%02x\n", buffer[4]);
            printf(" \n", buffer[4]);
		}
		else
		{
			ChksumErrorCount++;
			for (i=0; i<12; ++i)
			{
				printf("Checksum error %02x\n", buffer[i]);
            }
		}
	}
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

	wiringPiSPISetup(0, 2000000);

	pinMode(0, INPUT);
	pullUpDnControl(0, PUD_UP);
	wiringPiISR(0, INT_EDGE_FALLING, SPI_Req_ISR);

	while(1);

	return 0;
}

