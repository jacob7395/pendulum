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
//Liberys used in UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "File.h"
#include "PID_v1.h"

#define PIN 3
#define NUMBER_OF_BYTES 14

using namespace std;

void SPI_Req_ISR(void);

volatile char SPI_OUT[NUMBER_OF_BYTES];
volatile char SPI_IN [NUMBER_OF_BYTES];

volatile bool Packet_Ready = false;
//definces modes of opperation where 0 is the standerd operation mode
volatile int Mode = 0;
// PID varibles
double pidSetpoint, pidInput, pidOutput,Current_Time;

volatile long stepRate;
//difference between real angle and desired angle
float error = 0;
//degrees -179 -> 180
float encoderAngle = 0;

int main(int argc, char **argv)
{
    //initalize UDP connection and send a start packet
    int sockfd;
    struct sockaddr_in destaddr;
    socklen_t len2;
    char mesg[1000];
    int returnv;
    int i;
    int MessageLength;

    // create a UDP socket
    if ((sockfd=socket(AF_INET,SOCK_DGRAM,0))==-1) {
        printf("socket() failed\n");
        exit(0);
    } else;

    bzero(&destaddr,sizeof(destaddr));			// clear destaddr structure
    destaddr.sin_family = AF_INET;				// use IPV4 addresses
    destaddr.sin_port=htons(63243);				// the port on the computer we are sending to

    // convert the IP address of the destination
    if (inet_aton("192.168.168.3", &destaddr.sin_addr)==0) {
	   printf("inet_aton() failed\n");
	   exit(0);
	} else;
    //end of UDP setup
    //varibles usded to represent Arduino PID gains
    //initalized hear and set later
    short P = 0;
    short I = 0;
    short D = 0;

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

<<<<<<< HEAD
	wiringPiSPISetup(0, 2500000);
=======
	wiringPiSPISetup(0, 500000);
>>>>>>> parent of 583e3ba... Pi coms now works, speed can be set by the pi

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
			static int UDP_count = 0;
			UDP_count++;
            //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
            strncpy(SPI_IN_TEMP , SPI_IN , NUMBER_OF_BYTES);
            strncpy(SPI_OUT_TEMP, SPI_OUT, NUMBER_OF_BYTES);
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
            Current_Motor_Speed = 0;
<<<<<<< HEAD
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
            static std::string Data;
            Data =        Int_To_String(Run_Time,4) + ',';
            Data = Data + Int_To_String(Step_Count,1) + ',';
            Data = Data + Int_To_String(Current_Motor_Speed,2) + ',';
            Data = Data + Int_To_String(Pendelum_Angle,2) + ',';
            Data = Data + Int_To_String(Pendelum_Velocity,2) + ',';
            Data = Data + Int_To_String(P,1) + ',';
            Data = Data + Int_To_String(I,1) + ',';
            Data = Data + Int_To_String(D,1);
            //write data to file
            //Record_Data(Data);
            //UDP sectoin
            //only send every 10 packets to provent slow speeds
            if(UDP_count > 10)
            {
                UDP_count = 0;
                char Data_Array[Data.size()];
                for(int i = 0; i <= Data.size(); i ++)
                {
                    Data_Array[i] = Data[i];
                }
                //send tha data over UDP
                len2 = sizeof(destaddr);
                strcpy(mesg, Data_Array);
                MessageLength=strlen(mesg);
                returnv = sendto(sockfd,mesg,MessageLength,0,(struct sockaddr *)&destaddr,len2);
            }
            P = 50;
            I = 0;
            D = 0;
            //Byte 1 tells the arduion what control mode to be in
            //0 == nothing
            //1 == PID
            SPI_OUT_TEMP[1] = (short)(1) & 0xff;
            //byts 2-4 are using for P,I,D Gains
            SPI_OUT_TEMP[2] = (short)(P) & 0xff;
            SPI_OUT_TEMP[3] = (short)(I) & 0xff;
            SPI_OUT_TEMP[4] = (short)(D) & 0xff;
            //copy the latest SPI_OUT data for transmision to the interupt
            for(int i = 0; i < NUMBER_OF_BYTES; i++)
			{
                SPI_OUT[i] = SPI_OUT_TEMP[i];
			}
=======
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
>>>>>>> parent of 583e3ba... Pi coms now works, speed can be set by the pi
            break;
            //mode to poroccess error messages
            case 1:
                //copy SPI_IN to a holder file attempting to stop corrupted files form the interupt
                strncpy(SPI_IN_TEMP , SPI_IN , NUMBER_OF_BYTES);
                strncpy(SPI_OUT_TEMP, SPI_OUT, NUMBER_OF_BYTES);
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
<<<<<<< HEAD
		while ((micros()-TicksNow)<200);
=======
		while ((micros()-TicksNow)<50);
>>>>>>> parent of 583e3ba... Pi coms now works, speed can be set by the pi
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

