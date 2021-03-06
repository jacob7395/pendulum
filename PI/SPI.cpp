

#include "SPI.h"

//Global varibles
char 		  SPI_OUT[NUMBER_OF_BYTES];
unsigned char  SPI_BUFFER [NUMBER_OF_BYTES];
char		  SPI_IN[NUMBER_OF_BYTES];

void SPI_Req_ISR(void) {

    static int TicksNow = 0;
    static int chksum   = 0;

    volatile int Packets	 = 0;
    volatile int BadSOM		 = 0;
    volatile int PacketFault = 0;

    volatile int ChksumErrorCount = 0;

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

	return;
}
