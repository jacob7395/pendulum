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


volatile unsigned char Packet[16];
volatile unsigned char buffer[16];

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
	//Packet[1]=0x01; // speed lo
	//Packet[2]=0x00; // speed hi
	//Packet[1]=0x05; // speed lo - 3333 pps
	//Packet[2]=0x0d; // speed hi - 3333 pps
	Packet[1]=0xe8; // speed lo - 1000 pps
	Packet[2]=0x03; // speed hi - 1000 pps
	Packet[3]=0x00; // direction 0 = +ve
	Packet[4]=0x00;

	// form chksum
	for (i=0, Packet[11]=0; i<11; ++i)
		Packet[11] += Packet[i];

	// copy to buffer for tx
	for (i=0; i<12; ++i)
		buffer[i] = Packet[i];

	// tx 1 byte as a time as SPI master
	for (i=0; i<12; ++i) {
		//printf("tx %02x ",buffer[i]);
		wiringPiSPIDataRW(0, &buffer[i], 1);
		//printf("rx %02x\n",buffer[i]);
		TicksNow=micros();
		while ((micros()-TicksNow)<100);
	}
	++Packets;

	if (buffer[0]==0x55) {
		chksum = 0;
		for (i=0; i<11; ++i)
			chksum += buffer[i];
		if (chksum==buffer[11]) {
			Angle = (buffer[2]*256 + buffer[1]);
			Position = (buffer[6]<<24) + (buffer[5]<<16) + (buffer[4]<<8) + buffer[3];
			TimeStamp = (buffer[10]<<24) + (buffer[9]<<16) + (buffer[8]<<8) + buffer[7];
			Angle = (Angle*3600)/1023;
			//printf("Angle..%i ", Angle);
			//printf("Position..%li ", Position);
			//printf("Timestamp..%li ", TimeStamp);
			//printf("Packets..%i ", Packets);
			//printf("Bad Starts..%i ", BadSOM);
			//printf("Checksum Errors..%i \n", ChksumErrorCount);
		} else {
			ChksumErrorCount++;
			for (i=0; i<12; ++i)
				printf("Checksum error %02x\n", buffer[i]);
		}
	} else {
		++BadSOM;
        for (i=0; i<12; ++i)
            printf("%02x\n", buffer[i]);
		printf("Bad SOM count %4i\n", BadSOM);
	}

	return;

}

int main(int argc, char **argv)
{

	printf("hello wiringPi\n");

	ChksumErrorCount=0;

	wiringPiSetup();

	wiringPiSPISetup(0, 500000);

	pinMode(0, INPUT);
	pullUpDnControl(0, PUD_UP);
	wiringPiISR(0, INT_EDGE_FALLING, SPI_Req_ISR);

	while(1);

	return 0;
}

