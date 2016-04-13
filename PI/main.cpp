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
#include <time.h>
#include <sys/time.h>

#include "SPI.h"

int main(int argc, char **argv)
{
    char SPI_OUT[NUMBER_OF_BYTES];
    char SPI_IN [NUMBER_OF_BYTES];

    char SPI_OUT_TEMP[NUMBER_OF_BYTES];
    char SPI_IN_TEMP [NUMBER_OF_BYTES];

    bool Packet_Ready = false;

    short Step_Count           = 0;
    short Pendelum_Angle_temp  = 0;
    float Pendelum_Angle       = 0;

    char TimeString[128];

    for(int i = 0; i<NUMBER_OF_BYTES; i++)
    {
        SPI_OUT[i];
        SPI_IN [i];
    }

    SPI_Init(SPI_OUT, SPI_IN);

	wiringPiSetup();

	wiringPiSPISetup(0, 3000000);

	pinMode			(0, INPUT);
	pullUpDnControl (0, PUD_UP);
	wiringPiISR		(0, INT_EDGE_FALLING, SPI_Req_ISR);


    //TEST OF FILE MANAGMET WITH TIM
    char file_name[200] = "/root/Desktop/pendulum/PI/Data/";

	timeval curTime;
	tm *my_date_time;
	gettimeofday(&curTime, NULL);
	my_date_time = localtime(&curTime.tv_sec);

    strftime(TimeString, 80, "%Y-%m-%d %H:%M:%S", my_date_time);

    printf("%s \n",TimeString);

    FILE *fp = fopen("/root/Desktop/pendulum/PI/Data/test.txt", "w+");

    printf("hello wiringPi\n");

	while(1)
	{
        //copy the latest SPI_OUT data for transmision to the interupt
        strncpy(SPI_OUT, SPI_OUT_TEMP, NUMBER_OF_BYTES);

        while(!Packet_Ready)
        {};

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

        printf("%03f\n"  , Pendelum_Angle);
        printf("%02i\n\n", Step_Count);
    }

	return 0;
}

