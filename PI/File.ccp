

#include "File.h"

void File_Init(void)
{
	//File path for the folder where data will be stored
    char file_name[200] = "/root/Desktop/pendulum/PI/Data/";

	timeval curTime;
	tm *my_date_time;
	gettimeofday(&curTime, NULL);
	my_date_time = localtime(&curTime.tv_sec);

    strftime(TimeString, 80, "%Y-%m-%d %H:%M:%S", my_date_time);

    printf("%s \n",TimeString);

    file_name = file_name + "www"

    FILE *fp = fopen(file_name, "w+");
}
