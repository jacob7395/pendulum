

#include "File.h"

using namespace std;

//these varibles need to be accessed by multiple functions
int iRun_Count = 1;
ofstream outFile;

char TimeString[128];
string Folder_Path ("/root/Desktop/pendulum/PI/Data/");

void File_Init(void)
{
	//File path for the folder where data will be stored
    string File_Path;

	timeval curTime;
	tm *my_date_time;
	gettimeofday(&curTime, NULL);
	my_date_time = localtime(&curTime.tv_sec);

    strftime(TimeString, 80, "%Y-%m-%d", my_date_time);

    File_Path = (Folder_Path + TimeString + "_RunCount-"+Int_To_String(iRun_Count));
    iRun_Count++;

    for(int i = 0; i < File_Path.length(); i++)
    {
        if(File_Path[i] == ' ')
        {
            File_Path[i] = '_';
        }
    }

    cout << File_Path << "\n";

    char File_Path_Array[File_Path.size()];
    for(int i = 0; i <= File_Path.size(); i ++)
    {
        File_Path_Array[i] = File_Path[i];
    }

    outFile.open(File_Path_Array);
}
//create a new file and incroment run count
void New_Run(void)
{
    //File path for the folder where data will be stored
    string File_Path;

    File_Path = (Folder_Path + TimeString + "_RunCount-"+Int_To_String(iRun_Count));
    iRun_Count++;

    for(int i = 0; i < File_Path.length(); i++)
    {
        if(File_Path[i] == ' ')
        {
            File_Path[i] = '_';
        }
    }

    cout << File_Path << "\n";

    char File_Path_Array[File_Path.size()];
    for(int i = 0; i <= File_Path.size(); i ++)
    {
        File_Path_Array[i] = File_Path[i];
    }

    outFile.open(File_Path_Array);
}

string Int_To_String(int num)
{
    string S;

    while(num > 0)
    {
        switch(num % 10)
        {
            case 0:
                S = ('0' + S);
            break;
            case 1:
                S = ('1' + S);
            break;
            case 2:
                S = ('2' + S);
            break;
            case 3:
                S = ('3' + S);
            break;
            case 4:
                S = ('4' + S);
            break;
            case 5:
                S = ('5' + S);
            break;
            case 6:
                S = ('6' + S);
            break;
            case 7:
                S = ('7' + S);
            break;
            case 8:
                S = ('8' + S);
            break;
            case 9:
                S = ('9' + S);
            break;
        }
        num /= 10;
    }
    return S;
}

