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
//definces modes of opperation where 0 is the standerd operation mode
int Mode = 0;

int main(int argc, char **argv)
{


    char SPI_OUT_TEMP[NUMBER_OF_BYTES];
    char SPI_IN_TEMP [NUMBER_OF_BYTES];

    short Step_Count           = 0;
    float Current_Motor_Speed  = 0;
    float Pendelum_Angle       = 0;
    float Pendelum_Velocity    = 0;

    for(int i = 0; i<NUMBER_OF_BYTES; i++)
    {
        SPI_OUT[i];
        SPI_IN [i];
    }

	wiringPiSetup();

	wiringPiSPISetup(0, 500000);

	pinMode			(0, INPUT);
	pullUpDnControl (0, PUD_UP);
    wiringPiISR		(0, INT_EDGE_FALLING, SPI_Req_ISR);
    //create a file with the current time as a name
    File_Init();

    printf("hello wiringPi\n");

	while(1)
	{
        switch(Mode)
        {
            //normal operational mode for velocity control
            case 0:
            //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
            strncpy(SPI_IN_TEMP , SPI_IN , NUMBER_OF_BYTES);
            strncpy(SPI_OUT_TEMP, SPI_OUT, NUMBER_OF_BYTES);
            //reset the packet flag
            Packet_Ready = false;
            //merge the income bytes into a single float
            Step_Count = 0;
            Step_Count = SPI_IN_TEMP[1] | (SPI_IN_TEMP[2]<<8);
            //merge the incoming bytes into one float
            Current_Motor_Speed = 0;
            Current_Motor_Speed = float(SPI_IN_TEMP[3] | (SPI_IN_TEMP[4]<<8))/100;
            //merge the incoming bytes into one float
            Pendelum_Angle      = 0;
            Pendelum_Angle      = float(SPI_IN_TEMP[5] | (SPI_IN_TEMP[6]<<8))/100;
            //merge the incoming bytes into one float
            Pendelum_Velocity   = 0;
            Pendelum_Velocity   = float(SPI_IN_TEMP[7] | (SPI_IN_TEMP[8]<<8))/100;
            //record the date from the pie in the oreder the data is resived
            Record_Data(Int_To_String(Step_Count,0) + ' ' + Int_To_String(Current_Motor_Speed,2) + ' ' + Int_To_String(Pendelum_Angle,2) + ' ' + Int_To_String(Pendelum_Velocity,2));

            //copy the latest SPI_OUT data for transmision to the interupt
            strncpy(SPI_OUT, SPI_OUT_TEMP, NUMBER_OF_BYTES);
            //wait for a new packed
            while(!Packet_Ready)
            {};
            break;
            //mode to poroccess error messages
            case 1:
                //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
                strncpy(SPI_IN_TEMP , SPI_IN , NUMBER_OF_BYTES);
                strncpy(SPI_OUT_TEMP, SPI_OUT, NUMBER_OF_BYTES);
                //reset ready flag
                Packet_Ready = FALSE;
                switch(SPI_IN_TEMP[1])
                {
                    //sent on ardino bootup
                    case 0x71:
                    File_Init();
                    break;
                    //sent when arduino colides with a limit switch
                    //0x72 in decimal
                    case 0x72:
                    New_Run();
                    break;
                }
                //copy the latest SPI_OUT data for transmision to the interupt
                strncpy(SPI_OUT, SPI_OUT_TEMP, NUMBER_OF_BYTES);
                while(!Packet_Ready)
                {};
            break;
        }
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

    //normal operation start byte
	SPI_OUT[0] = 0x55;// Stat byte

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
	if (SPI_BUFFER[0]==0x55 || SPI_BUFFER[0]==0x46)
	{
		chksum = 0;
		//form the checksum for the resived data
		for (int i=0; i<NUMBER_OF_BYTES-1; ++i)
		{
			chksum += SPI_IN[i];
        }
        //if checksum is wrong log an error
		if (chksum == SPI_BUFFER[NUMBER_OF_BYTES-1])
		{
			ChksumErrorCount++;
			printf("Checksum Error Count - %i\n", ChksumErrorCount);
			for(int i = 0; i < NUMBER_OF_BYTES; i++)
			{
                printf("%x %x\n", SPI_IN[i], SPI_OUT[i]);
			}
            //may insuret a error reset meaning if chkerror is more then 10 in 1 run reset run and log fail
            Packet_Ready = FALSE;
            return;
		}
        switch(SPI_BUFFER[0])
        {
            //normal message use normal mode
            case 0x55:
                Mode = 0;
            break;
            //error message resived use error mode
            case 0x46:
                Mode = 1;
            break;
        }
	}
	//if bad start bit print error
	else
	{
		++BadSOM;
		printf("Bad SOM\n");
	}

	Packet_Ready = true;

	return;
}

