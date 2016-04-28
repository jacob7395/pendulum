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
#define NUMBER_OF_BYTES 14

void SPI_Req_ISR(void);

volatile char SPI_OUT[NUMBER_OF_BYTES];
volatile char SPI_IN [NUMBER_OF_BYTES];

volatile bool Packet_Ready = false;
//definces modes of opperation where 0 is the standerd operation mode
volatile int Mode = 0;

int main(int argc, char **argv)
{


    char SPI_OUT_TEMP[NUMBER_OF_BYTES];
    char SPI_IN_TEMP [NUMBER_OF_BYTES];

    float Run_Time             = 0;
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

	wiringPiSPISetup(0, 2500000);

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
            while(!Packet_Ready)
			{};
            //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
            for(int i = 0; i < NUMBER_OF_BYTES; i++)
			{
                SPI_IN_TEMP [i] = SPI_IN [i];
                SPI_OUT_TEMP[i] = SPI_OUT[i];
			}
            //reset the packet flag
            Packet_Ready = false;
            //merge the incoming bytes into one float
            static int Run_Time_Short = 0;
            Run_Time = 0;
            Run_Time_Short = SPI_IN_TEMP[1] | (SPI_IN_TEMP[2]<<8) | (SPI_IN_TEMP[3]<<16) | (SPI_IN_TEMP[4]<<24);
            Run_Time = (float)Run_Time_Short / 1000;
            //merge the income bytes into a single float
            Step_Count = 0;
            Step_Count = SPI_IN_TEMP[5] | (SPI_IN_TEMP[6]<<8);
            //merge the incoming bytes into one float
            static short Speed_Temp = 0;
            Current_Motor_Speed = 0;
            Speed_Temp = SPI_IN_TEMP[7] | (SPI_IN_TEMP[8]<<8);
            Current_Motor_Speed = (float)Speed_Temp / 100;
            //merge the incoming bytes into one float
            static short Angle_Short = 0;
            Pendelum_Angle = 0;
            Angle_Short    = SPI_IN_TEMP[9] | (SPI_IN_TEMP[10]<<8);
            Pendelum_Angle = (float)Angle_Short / 100;
            //merge the incoming bytes into one float
            static short Velocity_Short = 0;
            Pendelum_Velocity  = 0;
            Velocity_Short     = SPI_IN_TEMP[11] | (SPI_IN_TEMP[12]<<8);
            Pendelum_Velocity  = (float)Velocity_Short / 100;
            //record the date from the pie in the oreder the data is resived
            Record_Data( Int_To_String(Run_Time,4) + ',' + Int_To_String(Step_Count,1) + ',' + Int_To_String(Current_Motor_Speed,2) + ',' + Int_To_String(Pendelum_Angle,2) + ',' + Int_To_String(Pendelum_Velocity,2));
            //area to calculate the desired speed
            static float Desired_Speed = 0.8;

            if(Step_Count > 1110)
            {
                Desired_Speed = -0.5;
            }
            else if(Step_Count < -1110)
            {
                Desired_Speed =  0.5;
            }

            SPI_OUT_TEMP[1] =  (short)(Desired_Speed * 100)       & 0xff;
            SPI_OUT_TEMP[2] = ((short)(Desired_Speed * 100) >> 8) & 0xff;
            //copy the latest SPI_OUT data for transmision to the interupt
            for(int i = 0; i < NUMBER_OF_BYTES; i++)
			{
                SPI_OUT[i] = SPI_OUT_TEMP[i];
			}
            break;
            //mode to poroccess error messages
            case 1:
                //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
                for(int i = 0; i < NUMBER_OF_BYTES; i++)
                {
                    SPI_IN_TEMP [i] = SPI_IN [i];
                    SPI_OUT_TEMP[i] = SPI_OUT[i];
                }
                //reset ready flag
                Packet_Ready = false;
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
                for(int i = 0; i < NUMBER_OF_BYTES; i++)
                {
                    SPI_OUT[i] = SPI_OUT_TEMP[i];
                }
                //wait for next packet befor switching start
                while(!Packet_Ready)
                {};
            break;
        }
    }

	return 0;
}

void SPI_Req_ISR(void) {

    static int TicksNow = 0;
    static short chksum   = 0;

    volatile int Packets	 = 0;
    volatile int BadSOM		 = 0;
    volatile int PacketFault = 0;

    volatile int ChksumErrorCount = 0;

    unsigned char SPI_BUFFER[NUMBER_OF_BYTES];

    Packet_Ready = true;
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

	//tx 1 byte as a time as SPI master
	for (int i=0; i<NUMBER_OF_BYTES; ++i) {
		wiringPiSPIDataRW(0, &SPI_BUFFER[i], 1);
		//wait befor sending next byte
		TicksNow=micros();
		while ((micros()-TicksNow)<200);
	}
	//copy to SPI_IN for SPI_BUFFER(now contains data from arduino)
	for (int i=0; i<NUMBER_OF_BYTES; ++i)
	{
		SPI_IN[i] = SPI_BUFFER[i];
    }
	//coung the number of packets recived
	++Packets;
	if (SPI_BUFFER[0]==0x55 || SPI_BUFFER[0]==0x46)
	{
		chksum = 0;
		//form the checksum for the resived data
		for (int i=0; i<NUMBER_OF_BYTES-1; ++i)
		{
			chksum += SPI_IN[i];
        }
        chksum = chksum & 0x00FF;
        //if checksum is wrong log an error
		if (chksum != SPI_BUFFER[NUMBER_OF_BYTES-1])
		{
			ChksumErrorCount++;
			printf("Checksum Error Count - %i\n", ChksumErrorCount);
			for(int i = 0; i < NUMBER_OF_BYTES; i++)
			{
                printf("%x %x\n", SPI_IN[i], SPI_OUT[i]);
			}
			printf("%x\n", chksum);
            //may insuret a error reset meaning if chkerror is more then 10 in 1 run reset run and log fail
            Packet_Ready = false;
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

		Packet_Ready = false;
        return;
	}
	Packet_Ready = true;
	return;
}

