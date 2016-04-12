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

#include <SPI.h>

volatile long TicksNow;

int main(int argc, char **argv)
{

	printf("hello wiringPi\n");

	ChksumErrorCount=0;

	wiringPiSetup();

	wiringPiSPISetup(0, 3000000);

	pinMode			(0, INPUT);
	pullUpDnControl (0, PUD_UP);
	wiringPiISR		(0, INT_EDGE_FALLING, SPI_Req_ISR);

	while(1);

	return 0;
}

