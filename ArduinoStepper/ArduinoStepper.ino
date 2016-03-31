//due libery header
#include <DueTimer.h>

/*************************************************************
This program will be used to control a NEMA24 stepper motor for
an inverted pendulem proble. It will be reading a single pedomiter
and 2 switches. Then feeding the required infomatoin to a rasberry
pi that will retrun the next actoin.

Disclaimer i'm dyslexic and there is no spell check
*************************************************************/

//---------------------------------------------------------------//
//function decleratoin
void Set_Directoin(bool Directoin);
void Step(void);
void Set_Motor_Speed(float ms);
void Left_Limit_Hit  (void);
void Right_Limit_Hit (void);
void Hall_One_Hit    (void);
void Hall_Two_Hit    (void);
void Hall_Three_Hit  (void);
void Hall_Four_Hit   (void);
void Hall_Five_Hit   (void);

//---------------------------------------------------------------//
//global and const varible decloratoin
//not good practive but okay in arduino
const int Pulse    = 23;//used in setup functoin and when editing pins
const int Dir      = 25;//don't use digital writes insted directly acces the system
const int EN       = 27;
const int RESET    = 29;
const int SLEEP    = 31;
const int FAULT    = 33;

const int Step_Res = 1;
const int Acceleration_Period_ms = 15;

bool Direction = 1;
int Step_Count = 0;

bool Timer1_State = 0;

volatile float Desired_Motor_Speed = 0;//set by the main the change the speed of the motor using the timer attached to Set_Motor_Speed
volatile float Current_Motor_Speed = 0;//used by set_motor_speed to control the acceleratoin

int Timer2_State = 0;//tracks the status of Timer2 used in a if statment to toggle the timer
//Decleratoin for key system peramiters
//Step Rotatoin is the number of steps per revolution can be found on motor datasheet
//Cart dispasment per step can be found my multiplying the distance between the teeth by the number of teeth on the gear
//finaly Distance per step can be found by deviding cart displasment oer rotation by the steps per rotation
//The 1000 converts it from MM per step to M per step this is a small number around 0.00045 frot he current system
//To avoid floating point arithmatic 1/0.00045 is do giving 2222.22222 the .2222 can be forgot as the loss in accurecy is negligble
const int Steps_Per_Rotation = 200;
const int Cart_Displasment_Per_Rotation = 90;
const int Distance_Per_Step = 2222;
//set of pins used for limet switches and hall effect sensors
//the hall sensors will update the carts current position and correct the step count
//the halls go from right to left where Hall_One is the furthest right and Hall_Five is the furthest right
int Hall_One    = 22;
int Hall_Two    = 24;
int Hall_Three  = 26;
int Hall_Four   = 28;
int Hall_Five   = 30;
//
int Mode = 0;
//when new speed recived plase hear and set mode to 1 for speed change
int New_Speed = 0;

int Current_Frequency = 0; //varible to hold current frequency(pps) of the motor
bool Config_Mode = false; //tell the system that cofig mode is running
//---------------------------------------------------------------//
//random varibls that should be moved or deleted later
float s = 0.1;
int PPS = 0;
//---------------------------------------------------------------//

//***********************************************************************//
void setup() {

  //establish motor direction toggle pins
  pinMode(Pulse  , OUTPUT); //
  pinMode(Dir    , OUTPUT); //
  pinMode(EN     , INPUT ); //
  pinMode(RESET  , HIGH  ); //
  pinMode(SLEEP  , HIGH  ); //
  pinMode(FAULT  , INPUT ); //
  
  //clear pulse at the start of program
  g_APinDescription[Pulse].pPort -> PIO_CODR = g_APinDescription[Pulse].ulPin;
  //set the directoin to right so the program always starts with directoin in the same plase
  Set_Directoin(1);
  //attaches timer1 to the step function that drives the motor PPS
  Timer1.attachInterrupt(Step);
  //attaches timer2 to the speed function that controles motor acceleration/deseleration
  Timer2.attachInterrupt(Set_Motor_Speed).setFrequency(200).start();
  //block to attach interupts to required digital pins with the correct CALLBACK function
  //all pins are set from INPUT
  pinMode(Hall_One    , INPUT_PULLUP);
  pinMode(Hall_Two    , INPUT_PULLUP);
  pinMode(Hall_Three  , INPUT_PULLUP);
  pinMode(Hall_Four   , INPUT_PULLUP);
  pinMode(Hall_Five   , INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(Hall_One)    , Hall_One_Hit    , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Two)    , Hall_Two_Hit    , FALLING );
  //attachInterrupt(digitalPinToInterrupt(Hall_Three)  , Hall_Three_Hit  , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Four)   , Hall_Four_Hit   , FALLING );
  //attachInterrupt(digitalPinToInterrupt(Hall_Five)   , Hall_Five_Hit   , FALLING );


  Serial.begin(115200);
}
//***********************************************************************//
void loop() {
  delay(1000);
 
  Serial.println("Begin");
  Mode = 0;
  
  for( ; ; )
  {
    switch(Mode) {
    
    case 0:
      Config_Mode = false;
      //delay(1);
      Desired_Motor_Speed = -0.3;
    break;
    //this case uses a config opperation
    //it current travels slowly left towards hall 1
    //when hall 1 is hit the step count is reset to 0
    //the cart then slowly travels to the other end stoping when it hits hall 5
    //it then prints the step count to the serial storing the value in memory
    //it also prints the length in meters
    case 1:
      Config_Mode = true;
      delay(1);
      Desired_Motor_Speed = -0.3;

      delay(50); //delay allows time for the movment to begin
      while(Current_Frequency != 0) //waits until the cart stops(when it hits the hall)
      {
        //Serial.println(Current_Frequency);
        delay(1); //short delay while waitting
      }
      Step_Count = 0;//reset the step counts
      Desired_Motor_Speed = 0.3;//begin traveling right towards hall 5

      delay(50); //delay allows time for the movment to begin
      while(Current_Frequency != 0) //waits until the cart stops(when it hits the hall)
      {
        delay(1); //short delay while waitting
      }
      //print results to serial
      Serial.print("Track length in pulses is ");
      Serial.println(Step_Count);
      Serial.print("Track length in meters is ");
      float Track_Length = 0.00045;
      Serial.println(Track_Length);
      //move to standerd opperation mode
      Mode = 0;
    break;
    
    }
  }
  
  //  PPS = (Desired_Motor_Speed*Step_Res) / 0.00045;

  //  Serial.println(s);
  //  Serial.println(PPS);
  //  Serial.println(Step_Count);
  //  Serial.println(Desired_Motor_Speed);
  //  Serial.println(Current_Motor_Speed);
  //  Serial.println(Distance_Per_Step);
}
//***********************************************************************//
//function that sets the frequency of timer1
//the frequenct is set according to the speed given
//this speed translats to the speed at witch the cart is desird to move
//0 will stop the timer
void Set_Motor_Speed(void)
{
  //---------------------------------------------------------------//
  //calculates the frequency needed(pps) to drive the cart at the desired speed
  //the 303 is 1 over the distance each pulse makes the belt trave
  //the 1 over gets rid of the floarting point (1/0.0033 = 3333.33') this loses some accurecy but saves time
  //The required PPS is *2 to get the frequency requred for the interupt so it can pule low and high at the PPS
  //the following if statments manage the changing in directing and speed
  static int Desired_Frequency = 0;  //used to store the result of PPS calculatoins
  static int Delay_Count       = 0;
  Delay_Count++;
  //if the Desired speed(DS) is less then 0 and Current_Frequency(CF) is grater then 0 and the directing == 0 ramp PPS to 0
  //meaning if the cart is in motion but in the wrong direction ramp the spped down to 0
  if(Desired_Motor_Speed < 0 && Current_Frequency > 0 && Direction == 1)
  {
    Desired_Frequency = 0; 
    Delay_Count = 0;
  }
  //if the DS is less than 0 and the CF is = to 0 and the direction is not 0 change the direction
  //if the cart has stopped but the direction is incorrect change directoin
  else if(Desired_Motor_Speed < 0 && Current_Frequency == 0 && Direction == 1 && Delay_Count >= 0)
  {
    Set_Directoin(0);
    Delay_Count = 0;
  }
  //if DS is less than 0 and the CF is less than or equle to 0 and the direction is correct calculate the correct PPS
  //if the is still or moving in the correct direction calculate the wanted fequency
  else if(Desired_Motor_Speed <= 0 && Current_Frequency >= 0 && Direction == 0 && Delay_Count >= 0)
  {
    Desired_Frequency = ((Desired_Motor_Speed * Step_Res * -1) * Distance_Per_Step) * 2;
  }
  //if the DS is greater then 0 and the CF is grater than 0 and the directoin is incorrect ramp PPS to 0
  //if the cart is moving in the wrong directoin ramp it to 0
  else if(Desired_Motor_Speed > 0 && Current_Frequency > 0 && Direction == 0)
  {
    Desired_Frequency = 0; 
    Delay_Count = 0;
  }
  //if DS is grater then 0 and the CF equels 0 and the direction is worn change direction
  //if the cart is not moving but the direction is wrong changedirection
  else if(Desired_Motor_Speed > 0 && Current_Frequency == 0 && Direction == 0 && Delay_Count >= 0)
  {
    Set_Directoin(1);
    Delay_Count = 0;
  }
  //if the DS is grater then 0 and then CF is grater than or equel to 0 and the directoin is correct calculated wanted frequency
  //if the cart is stil or moving in the correct direction calculate the wanted fequency
  else if(Desired_Motor_Speed >= 0 && Current_Frequency >= 0 && Direction == 1 && Delay_Count >= 0)
  {
    Desired_Frequency = ((Desired_Motor_Speed * Step_Res * 1) * Distance_Per_Step) * 2;
  }
  //This section controles the acceleration/dseleratoin of the motor by stepping the PPS up and down
  static int PPS_Step = 333;  //The PPS_Step set the amount of steps that can be incred at one time without staling
  //If the diffrence in frequency is less than or grater than 2*PPS_Step and the DS is larger then the CF and the CF is at 0 incress the PPS by 2* the PPS_Step
  //When the cart is at 0 CF the PPS can be increced by double the PPS_Step and not stall this increces the acceleration rate
  if (((Desired_Frequency - Current_Frequency) > PPS_Step * 2 || (Desired_Frequency - Current_Frequency) < -PPS_Step * 2) && (Desired_Frequency > Current_Frequency) && Current_Frequency == 0)
  {
    Current_Frequency += PPS_Step * 2;
  }
  //if the change in frequency is less than 2* the PPS_Step and CF is 0 jump to the desired frequency
  else if ((Desired_Frequency - Current_Frequency) <= PPS_Step*2 || (Desired_Frequency - Current_Frequency) > -PPS_Step*2 && Current_Frequency == 0)
  {
    Current_Frequency = Desired_Frequency;
  }
  //If the diffrence in frequency is less than or grater than PPS_Step and the DS is larger then the CF and the CF is at 0 incress the PPS by the PPS_Step ramping up the speed
  else if (((Desired_Frequency - Current_Frequency) > PPS_Step || (Desired_Frequency - Current_Frequency) < -PPS_Step) && (Desired_Frequency > Current_Frequency))
  {
    Current_Frequency += PPS_Step;
  }
  //If the diffrence in frequency is less than or grater than PPS_Step and the DS is smaller then the CF and the CF is at 0 incress the PPS by the PPS_Step ramping down the speed
  //after chaning the CF if it drops below 0 set the CF to 0, this provents error in the interup casued by setting the frequenct to a negetive number
  else if (((Desired_Frequency - Current_Frequency) > PPS_Step || (Desired_Frequency - Current_Frequency) < -PPS_Step) && (Desired_Frequency < Current_Frequency))
  {
    Current_Frequency -= PPS_Step;
    if (Current_Frequency < 0)
    {
      Current_Frequency = 0;
    }
  }
  //if the change in frequency is less than the PPS_Step jump to the desired frequency
  else if ((Desired_Frequency - Current_Frequency) <= PPS_Step || (Desired_Frequency - Current_Frequency) > -PPS_Step)
  {
    Current_Frequency = Desired_Frequency;
  }
  //---------------------------------------------------------------//
  static int Applyed_Frequency = 0;
  //sets Timer1 at the currect frequency for the current steps
  //if the CF == the applied frequency don't change Timer1 Frequency
  //this provents unwanted change of the frequency slowing the interupt
  if (Current_Frequency != Applyed_Frequency)
  {
    Timer1.setFrequency(Current_Frequency).start();
    Applyed_Frequency = Current_Frequency;
  }
  //if CF is 0 stop the interupt
  else if (Current_Frequency == 0)
  {
    Timer1.stop();
    //clear the pulse pin when timer is disabled to stop annoying buzzing
    g_APinDescription[Pulse].pPort -> PIO_CODR = g_APinDescription[Pulse].ulPin;
  }
  //---------------------------------------------------------------//
}
//***********************************************************************//
void Step(void) {
  //---------------------------------------------------------------//
  //if the motor direction is forward incroment step count
  //esle decroment step count
  //as this function is called 2* fater the the PPS
  //step count needs to incroment/decroment every 2 calls
  static bool Count = 0;
  Count++;

  if (Direction == 1 && Count >= 1)
  {
    Step_Count++;
    Count = 0;
  }
  else if (Count >= 1)
  {
    Step_Count--;
    Count = 0;
  }
  //---------------------------------------------------------------//
  //toggels the Pulse pin and only the pulse pin
  //pulse pin used is digital 28
  //digital wright is slow so the following lines clear and set the due register
  //with the signal tracking it's state
  static bool state = 0;
  if (state == 0)
  {
    g_APinDescription[Pulse].pPort -> PIO_SODR = g_APinDescription[Pulse].ulPin;
    state = 1;
  }
  else if (state == 1)
  {
    g_APinDescription[Pulse].pPort -> PIO_CODR = g_APinDescription[Pulse].ulPin;
    state = 0;
  }
  //--------------------------------------------------------//
}
//***********************************************************************//
//functoin to set motor directoin
//1 for right 0 for left
void Set_Directoin(bool Directoin) {
  //check the wanter directoin
  //may be able to use a single bit mask
  //I'm not sure so using two with a statment to select
  //both these opperatoins leave the regester unchanged
  if (Directoin == 0)
  {
    //direction pin is attached to PB3
    //so changing the 4th bit in the register will set the directoin
    //this set the directoin low ie left
    //directoin pin used is digital 52
    //this is connected to PB1 on the arduino MEGA
    g_APinDescription[Dir].pPort -> PIO_SODR = g_APinDescription[Dir].ulPin;
    //update the goble varible for directoin
    Direction = 0;
  }
  else if (Directoin == 1)
  {
    //this sets the directoin right if high
    //directoin pin used is digital 52
    //this is connected to PB3 on the arduino MEGA
    g_APinDescription[Dir].pPort -> PIO_CODR = g_APinDescription[Dir].ulPin;
    //update the goble varible for directoin
    Direction = 1;
  }
}
//***********************************************************************//
//The set of callback functions for the limit switches and the hall sensors

void Hall_One_Hit   (void)
{
    if(Config_Mode == false)
    {
      
    }
    else
    {
      Serial.println("Hall One Hit");
    }
}

void Hall_Two_Hit   (void)
{
    if(Config_Mode == false)
    {
      
    }
    else
    {
      Serial.println("Hall Two Hit");
      Desired_Motor_Speed = 0;
    }
}

void Hall_Three_Hit (void)
{
    if(Config_Mode == false)
    {
      
    }
    else
    {
      Serial.println("Hall Three Hit");
    }
}

void Hall_Four_Hit  (void)
{
    if(Config_Mode == false)
    {
      
    }
    else
    {
      Serial.println("Hall Four Hit");
      Desired_Motor_Speed = 0;
    }
}

void Hall_Five_Hit  (void)
{
    if(Config_Mode == false)
    {
      
    }
    else
    {
      Serial.println("Hall Five Hit");
    }
}

/*************************************************************
Jacob Threadgould 2016
*************************************************************/
