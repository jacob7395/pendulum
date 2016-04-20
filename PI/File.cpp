

#include "File.h"

using namespace std;

//these varibles need to be accessed by multiple functions
int iRun_Count = 1;
ofstream outFile;

char TimeString[128];
string Folder_Path ("/root/Desktop/pendulum/PI/Data/");
//File path for the folder where data will be stored
string File_Path;

void File_Init(void)
{
    iRun_Count = 1;
    //close any existing file
    outFile.close();

    File_Path = ' ';

	timeval curTime;
	tm *my_date_time;
	gettimeofday(&curTime, NULL);
	my_date_time = localtime(&curTime.tv_sec);

    strftime(TimeString, 80, "%Y-%m-%d-%H:%M:%S", my_date_time);

    File_Path = (Folder_Path + TimeString);

    for(int i = 0; i < File_Path.length(); i++)
    {
        if(File_Path[i] == ' ')
        {
            File_Path[i] = '_';
        }
    }
    //conert folder path into array
    char File_Path_Array[File_Path.size()];
    for(int i = 0; i <= File_Path.size(); i ++)
    {
        File_Path_Array[i] = File_Path[i];
    }
    //make folder
    mkdir(File_Path_Array, 0700);
    //add runcount to with folder path
    File_Path = File_Path + "/Run-"+Int_To_String(iRun_Count,0)+".csv";
    //convert file path to array
    char File_Path_Array_Two[File_Path.size()];
    for(int i = 0; i <= File_Path.size(); i ++)
    {
        File_Path_Array_Two[i] = File_Path[i];
    }
    //open filepath
    outFile.open(File_Path_Array_Two);
    outFile.close();
    //print file path and incroment run count
    cout << File_Path << "\n";
    iRun_Count++;
}
//create a new file and incroment run count
void New_Run(void)
{
    //close any existing file
    outFile.close();
    //null file path
    File_Path =  "";
    //make new file path
    File_Path = (Folder_Path + TimeString + "/Run-"+Int_To_String(iRun_Count,0)+".csv");
    iRun_Count++;

    for(int i = 0; i < File_Path.length(); i++)
    {
        if(File_Path[i] == ' ')
        {
            File_Path[i] = '_';
        }
    }

    char File_Path_Array[File_Path.size()];
    for(int i = 0; i <= File_Path.size(); i ++)
    {
        File_Path_Array[i] = File_Path[i];
    }

    outFile.open(File_Path_Array);

    cout << File_Path << "\n";
}

void Record_Data(std::string data)
{
    outFile << data << endl;
}

string Int_To_String(float fNum, int Decimal)
{
    string S;
    int num;
    bool Negative = false;

    if(fNum < 0)
    {
        Negative = true;
        fNum *= -1;
    }
    //gets the wanted decimals to above the decimal place
    num = fNum * pow(10, Decimal);
    //cout << (pow(10, Decimal) - 1) << '\n';

    if(Decimal > 0)
    {
        for(int i = 0; i < Decimal; i++)
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
        S = '.' + S;
        if(num == 0)
        {
            S = '0' + S;
        }
    }


    while(num != 0)
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

    if(Negative == true)
    {
        S = '-' + S;
    }

    return S;
}

