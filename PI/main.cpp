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

#include "File.h"

#define PIN 3
#define NUMBER_OF_BYTES 10

void SPI_Req_ISR(void);

char SPI_OUT[NUMBER_OF_BYTES];
char SPI_IN [NUMBER_OF_BYTES];

bool Packet_Ready = false;

int main(int argc, char **argv)
{


    char SPI_OUT_TEMP[NUMBER_OF_BYTES];
    char SPI_IN_TEMP [NUMBER_OF_BYTES];

    short Step_Count           = 0;
    short Pendelum_Angle_temp  = 0;
    float Pendelum_Angle       = 0;

    for(int i = 0; i<NUMBER_OF_BYTES; i++)
    {
        SPI_OUT[i];
        SPI_IN [i];
    }

	wiringPiSetup();

	wiringPiSPISetup(0, 3000000);

	pinMode			(0, INPUT);
	pullUpDnControl (0, PUD_UP);
    wiringPiISR		(0, INT_EDGE_FALLING, SPI_Req_ISR);
    //create a file with the current time as a name
    File_Init();
    New_Run();

    printf("hello wiringPi\n");

	while(1)
	{
        //copy the latest SPI_OUT data for transmision to the interupt
        strncpy(SPI_OUT, SPI_OUT_TEMP, NUMBER_OF_BYTES);

        while(!Packet_Ready)
        {
        };

        strncpy(SPI_IN_TEMP , SPI_IN , NUMBER_OF_BYTES);
        strncpy(SPI_OUT_TEMP, SPI_OUT, NUMBER_OF_BYTES);
        Packet_Ready = false;

        Step_Count = 0;
        Step_Count = Step_Count |  SPI_IN_TEMP[1];
        Step_Count = Step_Count | (SPI_IN_TEMP[2]<<8);
        Pendelum_Angle_temp = 0;
        Pendelum_Angle_temp = Pendelum_Angle_temp |  SPI_IN_TEMP[3];
        Pendelum_Angle_temp = Pendelum_Angle_temp | (SPI_IN_TEMP[4]<<8);

        Pendelum_Angle = float(Pendelum_Angle_temp)/100;

        cout << Int_To_String(Pendelum_Angle,2) << '\n';

        // printf("%03f\n"  , Pendelum_Angle);
        // printf("%02i\n\n", Step_Count);
    }

	return 0;
}

void SPI_Req_ISR(void) {

    static int TicksNow = 0;
    static int chksum   = 0;

    volatile int Packets	 = 0;
    volatile int BadSOM		 = 0;
    volatile int PacketFault = 0;

    volatile int ChksumErrorCount = 0;

    unsigned char SPI_BUFFER[NUMBER_OF_BYTES];

	SPI_OUT[0]=0x55;	// Stat byte

	// form cheksum
	SPI_OUT[NUMBER_OF_BYTES-1]=0; //clear old checksum
	for(int i=0; i<NUMBER_OF_BYTES-1; ++i)
	{
		SPI_OUT[NUMBER_OF_BYTES-1] += SPI_OUT[i];
    }

	//copy to buffer for tx
	for (int i=0; i<NUMBER_OF_BYTES; ++i)
	{
		SPI_BUFFER[i] = SPI_OUT[i];
    }

	// tx 1 byte as a time as SPI master
	for (int i=0; i<NUMBER_OF_BYTES; ++i) {
		wiringPiSPIDataRW(0, &SPI_BUFFER[i], 1);
		//wait befor sending next byte
		TicksNow=micros();
		while ((micros()-TicksNow)<50);
	}
	//copy to SPI_IN for SPI_BUFFER(now contains data from arduino)
	for (int i=0; i<NUMBER_OF_BYTES; ++i)
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
		for (int i=0; i<NUMBER_OF_BYTES-1; ++i)
		{
			chksum += SPI_IN[i];
        }
        //if checksum is correct print the spi data
/*		if (chksum == SPI_BUFFER[NUMBER_OF_BYTES-1])
		{
            printf("Packets recived %02i, Checksum errors %02i\n", Packets, ChksumErrorCount);
            for (int i=0; i<NUMBER_OF_BYTES; ++i)
			{
                printf("%02x   ", SPI_OUT[i]);
                printf("%02x\n" , SPI_IN [i]);
            }
            printf(" \n");
		}
		//else print data anc a error code
		else
		{
			ChksumErrorCount++;
			for (int i=0; i<NUMBER_OF_BYTES; ++i)
			{
                printf("%02x   ", SPI_OUT[i]);
                printf("%02x \n", SPI_IN [i]);
            }
            printf("Checksum error %02x, %02i\n", chksum, ChksumErrorCount);
            printf(" \n");
		}
*/	}
	//if bad start bit print error
	else
	{
		++BadSOM;
		printf("Bad SOM\n");
	}

	Packet_Ready = true;

	return;
}

