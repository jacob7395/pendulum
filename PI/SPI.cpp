#include <SPI.h>

void SPI_Req_ISR(void) {

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