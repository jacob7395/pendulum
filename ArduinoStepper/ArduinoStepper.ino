
/*************************************************************
This program will be used to control a NEMA24 stepper motor for
an inverted pendulem proble. It will be reading a single pedomiter
and 2 switches. Then feeding the required infomatoin to a rasberry
pi that will retrun the next actoin.

Disclaimer i'm dyslexic and there is no spell check
*************************************************************/

//https://github.com/ivanseidel/DueTimer
#include <DueTimer.h>
#include <SPI.h>
#include <string.h>
#include <math.h>
#include <PID_v1.h>

//this defines how often (mS) we request SPI transfer from Pi
#define TX_INTERVAL 50
//timeout in mS for "rx data register full" flag to be set
#define RDRF_TIMEOUT 100
//timeout in mS for a full packet (SPI data frame) to be received
#define PACKET_TIMEOUT 100
//the number of bytes being transmited and resived in one SPI message incluing start and chack
#define NUMBER_OF_BYTES 14
//M/sec
#define MAX_SPEED 0.8
//first stage the program will enter after inital setup
#define INITAL_MODE 0

//---------------------------------------------------------------//
//function decleratoin
//motor control
void Set_Directoin(bool Directoin);
void Step(void);
void Set_Motor_Speed(float ms);
//Hall sensors
void Hall_One_Hit    (void);
void Hall_Two_Hit    (void);
void Hall_Three_Hit  (void);
void Hall_Four_Hit   (void);
void Hall_Five_Hit   (void);
//SPI
unsigned char GetByteFromSPI(void);
void SPI_Manager(char opperation);
//pot managment
void Pot_Manager(void);

//---------------------------------------------------------------//
//global and const varible decloratoin
//not good practive but okay in arduino

//Pins associsted withe the motor drives
const int Pulse    = 23;
const int Dir      = 25;
const int EN       = 27;
const int RESET    = 29;
const int SLEEP    = 31;
const int FAULT    = 33;

const int Step_Res = 1;
const int Acceleration_Period_ms = 15;
//Analog pin for the pot using in adc
const int Pot            = 11;
const int Pot_Resolution = 12;
float     Degree_Per_Bit = 0;
float     Pot_Position   = 0;
float     Pot_Velocity   = 0;
float     Pot_Offset     = 62.9;

bool  Direction = 1;
short Step_Count = 0;

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
//represent the position of each hall sensor nomolised around hall three
int Hall_One_Position    = -1378;
int Hall_Two_Position    = -1166;
int Hall_Three_Position  =     0;
int Hall_Four_Position   =  1169;
int Hall_Five_Position   =  1398;
//LED pins
int LED_White   = 35;
int LED_Blue    = 37;
int LED_Green   = 39;
int LED_Yellow  = 41;
int LED_Red     = 43;
//using the the main for the switch statment that drives the arduino
int Mode = 0;
//when new speed recived plase hear and set mode to 1 for speed change
int New_Speed = 0;
//used in mode 5 for motor preformance testing
float speed_test = 0;

int Current_Frequency = 0; //varible to hold current frequency(pps) of the motor
bool Config_Mode = false; //tell the system that cofig mode is running
//---------------------------------------------------------------//
//random varibls that should be moved or deleted later
float s = 0.1;
int PPS = 0;
//---------------------------------------------------------------//
// SPI input and output arrays
unsigned char SPI_IN [NUMBER_OF_BYTES]; // holds incoming data from Pi
unsigned char SPI_OUT[NUMBER_OF_BYTES]; // holds outgoing SPI data 
                                                                     
unsigned int BytesRx;    // count of bytes rx on SPI     

int SensorDataAvailablePin = 49; // tells Pi that sensor data is ready for SPI transfer

bool Ready_For_Data = false;
//the speed taken form SPI coms
float SPI_Speed = 0;

unsigned long timestamp1 = 0;
unsigned long timestamp2 = 0;

short Current_Mode = 0;

int Run_Time = 0;
//---------------------------------------------------------------//
// PID varibles
double pidSetpoint, pidInput, pidOutput;

volatile long stepRate;
//difference between real angle and desired angle
float error = 0;
//degrees -179 -> 180
float encoderAngle = 0;

// create the PID controller
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 0, 0, 0, DIRECT); //tuning
bool PID_Enabled = 0;

short Kp = 1;
short Ki = 100;
short Kd = 1;

short Set_Kp = 0;
short Set_Ki = 0;
short Set_Kd = 0;
//---------------------------------------------------------------//
// Flick varibles
int   Stage                = 0;
int   Max_Angle            = 0;
int   Angle_Reached        = 0;
float Speed                = 0.8;
unsigned long Old_Time_Two = 0;
int   Count                = 0;
int   Pulse_Count          = 0;
int   Old_Angle            = 0;
//***********************************************************************//
void setup() {

  Serial.begin(115200);
  //--------------------------------------------------------------------//
  //SPI setup

  //what is this?
  BytesRx = 0;
  //set SPI_IN/OUT to 0
  for (int j=0; j<NUMBER_OF_BYTES-1; ++j) 
  {
    SPI_IN[j]=0;
    SPI_OUT[j]=0;
  }
  
  // initialise port pin that tells the Pi SPI data available
  // PI is looking for a falling edge so after initalzing the pin set the pin high
  pinMode     (SensorDataAvailablePin, OUTPUT ); 
  digitalWrite(SensorDataAvailablePin, HIGH   );

  // setup SPI as slave
  SPI.begin(10); // is this needed ?

  //Sets up the SPI regesters and enables the arduino as a salve
  //it also preints the content of key registers befor and after setting them
  // //this is not needed to make it work but gives some debug info
  // Serial.print("SPI0_MR "  ); Serial.println(REG_SPI0_MR  , HEX); 
  // Serial.print("SPI0_CSR " ); Serial.println(REG_SPI0_CSR , HEX); 
  // Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);

  REG_SPI0_WPMR = 0x53504900    ; // Write Protection disable
  REG_SPI0_CR   = SPI_CR_SWRST  ; // reset  SPI (0x0080)
  REG_SPI0_CR   = SPI_CR_SPIEN  ; // enable SPI (0x0001)
  REG_SPI0_MR   = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
  REG_SPI0_CSR  = SPI_MODE0     ; // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
  REG_SPI0_IDR  = 0X0000070F    ; // disable all SPI interrupts

  // Serial.print("SPI0_MR ")  ; Serial.println(REG_SPI0_MR  , HEX); 
  // Serial.print("SPI0_CSR " ); Serial.println(REG_SPI0_CSR , HEX); 
  // Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);
  //set up inital SPI_OUT data
  SPI_OUT[0] = 0x55;
  SPI_OUT[1] = 0x01;
  SPI_OUT[2] = 0x02;
  SPI_OUT[3] = 0x03;

  //--------------------------------------------------------------------//
  //Motor and sensor setup
  //ADC pin for the pot and seting analog reselution
  analogReadResolution(Pot_Resolution);
  Degree_Per_Bit = 350/pow(2,Pot_Resolution);
  Timer4.attachInterrupt(Pot_Manager).setFrequency(1000).start();
  //establish motor direction toggle pins
  pinMode(Pulse  , OUTPUT); //
  pinMode(Dir    , OUTPUT); //
  pinMode(EN     , INPUT ); //
  pinMode(RESET  , OUTPUT); //
  pinMode(SLEEP  , OUTPUT); //
  pinMode(FAULT  , INPUT ); //

  digitalWrite(RESET,HIGH);
  digitalWrite(SLEEP,HIGH);
  
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

  attachInterrupt(digitalPinToInterrupt(Hall_One)    , Hall_One_Hit    , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Two)    , Hall_Two_Hit    , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Three)  , Hall_Three_Hit  , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Four)   , Hall_Four_Hit   , FALLING );
  attachInterrupt(digitalPinToInterrupt(Hall_Five)   , Hall_Five_Hit   , FALLING );
  //initilazatoin of LED pius
  pinMode(LED_White   , OUTPUT);
  pinMode(LED_Blue    , OUTPUT);
  pinMode(LED_Green   , OUTPUT);
  pinMode(LED_Yellow  , OUTPUT);
  pinMode(LED_Red     , OUTPUT);
  //--------------------------------------------------------------------//
  //PID setup
  // PID variables
  pidInput    = 0;
  pidOutput   = 0;
  pidSetpoint = 0;
  // turn PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-256.0, 256.0);
  myPID.SetSampleTime(100); // number of calculations per second
  // get rid of initial pid kick
  pidInput = error;
  for (int i=0; i<5; ++i) {
    myPID.Compute();
    delay(100);
  }
  SPI_Manager(1);
}
//***********************************************************************//
void loop() {
  delay(1000); 
  Serial.println(" ");
  Serial.println("Begin");
  Mode = 4;

  for( ; ; )
  {
    switch(Mode) {
    
    case 0: //case used under normal opperation

      Config_Mode = false;
      static unsigned long Old_Time = 0;
      //check if there has been atleast a 10milli delay since the last SPI packet
      while((millis()-Old_Time) < 10)
      {};
      //call the the SPI manager to send a standerd spi packet
      //this will return when the packet has been resived
      //SPI_Manager(0);
      //time in millies last SPI packet was compleate
      Old_Time = millis();
      PID_Enabled = true;
      if(PID_Enabled == true)
      {
        //chack if PID gains have changed
        if(Ki != Set_Ki || Kp != Set_Kp || Kd != Set_Kd)
        {
          myPID.SetTunings(Kp,Ki,Kd);
        }
        //PID Control
        static bool Direction_Latch = 1;
        //get the current encoder angle
        noInterrupts();
        encoderAngle = Pot_Position;
        interrupts();

        if(Step_Count > 10)
        {
          pidSetpoint =    1;
        }
        else if(Step_Count < -10)
        {
          pidSetpoint =    0;
        }
        
        //difference from desiredAngle, doesn't account for desired angles other than 0
        if(Pot_Position > 0)
        {
          error = encoderAngle - 180;
        }
        else if(Pot_Position < 0)
        {
          error = 180 + encoderAngle;
        }
        
        //do the PID stuff
        pidInput = error;
        myPID.Compute();

        // if the pendulum is too far off of vertical to recover, turn off the PID and motor
        if (error > 15 || error < -15) {
          myPID.SetMode(MANUAL);
          pidOutput = 0;
          Desired_Motor_Speed = 0;
          //change to flick mode
          //Mode  = 2;
        } else { //in bounds
          myPID.SetMode(AUTOMATIC);
          Desired_Motor_Speed = (float(pidOutput) * MAX_SPEED) / 256;
          //Serial.println((float(pidOutput) * MAX_SPEED) / 256);
        }
      }
    break;
    //this case is a config opperation
    //it travels slowly left towards hall 1
    //config mode is needed so when hall 1 is hit it doesnt got to case 3 resetting the track
    //when hall 1 is hit the step count is reset to 0
    //the cart then slowly travels to the other end stoping when it hits hall 5
    //it then prints the step count to the serial storing the value in memory
    //it also prints the length in meters
    case 1:
      Config_Mode = true;
      Desired_Motor_Speed = -0.3;
      delay(100); //delay allows time for the movment to begin
      while(Current_Frequency != 0) //waits until the cart stops(when it hits the hall)
      {
        //Serial.println(Current_Frequency);
        delay(1); //short delay while waitting
      }
      Step_Count = 0;//reset the step counts
      Desired_Motor_Speed = 0.3;//begin traveling right towards hall 5
      
      delay(100); //delay allows time for the movment to begin
      while(Current_Frequency != 0) //waits until the cart stops(when it hits the hall)
      {
        delay(1); //short delay while waitting
      }
      //print results to serial
      Serial.print("Track length in pulses is ");
      Serial.println(Step_Count);
      Serial.print("Track length in meters is ");
      Serial.println(0.00045*Step_Count);
      //move to standerd opperation mode
      Mode = 1;
    break;
    //redundant state
    case 2:
      Config_Mode = false;
      
      //Serial.println(Count);
      //check if there has been atleast a 10milli delay since the last SPI packet
      while((millis()-Old_Time_Two) < 10)
      {};
      //call the the SPI manager to send a standerd spi packet
      //this will return when the packet has been resived
      SPI_Manager(0);
      //time in millies last SPI packet was compleate
      Old_Time_Two = millis();

      switch(Stage) {
        //inital stage to move to cart to a set position
        //only used when the case is called
        case 0:
          if(Step_Count < 250)
          {
            Desired_Motor_Speed = 0.4;
          }
          else
          {
            Desired_Motor_Speed = 0;
            Stage = 1;
          }
        break;
        //case to set the speed at witch to movemet form 250 -> -250
        //waits till the pendulm is falling befor seting the speed
        case 1:
          if(Pot_Position >= Max_Angle)
          {
            Pulse_Count++;
          }
          else if(Pot_Position <= (Max_Angle - 5) && Pot_Position > 0)
          {
            //Serial.println(Speed);
            Desired_Motor_Speed = -Speed;
            Stage = 2;
          }
        break;
        //stop and reset the needed varible when the cart reaches -250
        case 2:
          if(Step_Count < -150)
          {
            Pulse_Count = 0;
            Angle_Reached = 0;
            Max_Angle = 0;
            Desired_Motor_Speed = 0;
            Stage = 3;
          }
        break;
        //case to set the motor speed when moving from -250 -> 250
        //waits till the pendulm is falling befor seting the speed
        case 3:
          if(Pot_Position <= Max_Angle)
          {
            Pulse_Count++;
          }
          else if(Pot_Position >= (Max_Angle + 5) && Pot_Position < 0)
          {
            Desired_Motor_Speed =  Speed;
            Stage = 4;
          }
        break;
        //final case befor looping back to stage 1 where the cart moves from -250 -> 250
        case 4:
          if(Step_Count > 150)
          {
            Pulse_Count = 0;
            Angle_Reached = 0;
            Max_Angle = 0;
            Desired_Motor_Speed = 0;
            Stage = 1;
          }
        break;
      }
      //attempts to control spikes coused when the pot moves over the dead zone
      if(Pulse_Count >= 15)
      {
        Pulse_Count = 0;
        Max_Angle = Pot_Position;
        Angle_Reached = Max_Angle;
      }
      //calculated the pendulum speed as it approches the top
      //the speed needs to decrease the colser the pedulm is to the center
      //this makes for the smoothist transition from harmonios motion to PID
      if     (Angle_Reached >=  95 || Angle_Reached <=  -95)
      {
        if(Angle_Reached < 0)
        {
          Angle_Reached *= -1;
        }
        //the formuler used to find the desires speed
        Speed = ((-pow((Angle_Reached - 90),2))/100 + 80)/100;
        if(Speed < 0)
        {
          Speed *= -1;
        }
        //the minumum speed the pendulum can go
        if(Speed < 0.40)
        {
          //Speed = 0.40;
        }
        Serial.println(Speed);
      }
      //default speed when below 95 degrees
      else if(Angle_Reached <=  95 || Angle_Reached >=  -95)
      {
        Speed = 0.8;
      }
      //resets counts when the pendulum passes through 0
      if (Pot_Position < 5 && Pot_Position > -5)
      {
        Pulse_Count = 0;
        Count       = 0;
      }
      //desides when to switch from swing up to PID
      //this attempts to switch just as the pendeulm passes from -180 to 180 be looking at the diffrence between the angles
      //when switchuing to PID key varibles are reset
      if(((Old_Angle -  Pot_Position) > 100 || (Old_Angle -  Pot_Position) < -100) && (Angle_Reached > 150 || Angle_Reached < -150))
      {
        Pulse_Count   = 0;
        Angle_Reached = 0;
        Stage         = 0;
        Mode          = 0;
      }
      Old_Angle = Pot_Position;
    break;
    //case position the cart in the middle of the track and stop
    case 3:
      //suspend spi transfer
      digitalWrite(LED_Green, LOW);
      noInterrupts();
      //if step count is greater than 2 move towards the middle
      if(Step_Count > Hall_Three_Position+2)
      {
        Desired_Motor_Speed = -0.3;
      }
      //if step count if less than 2 move toward the middle
      else if(Step_Count < Hall_Three_Position-2)
      {
        Desired_Motor_Speed =  0.3;
      }
      else if(Step_Count <= Hall_Three_Position+2 && Step_Count >= Hall_Three_Position-2 )
      {
        interrupts();
        Desired_Motor_Speed = 0;
        delay(10);
        SPI_Manager(2); //tell the PI a colition has occured
        digitalWrite(LED_Green, HIGH);
        delay(100);
        Mode = INITAL_MODE;
        //suspend spi transfer
      }
      interrupts();
      delay(10);
    break;
    //Start mode used to find hall 5
    //when hall 5 is hit the mode is switchd to 3 moveing the cart to the middle of the track
    case 4:
      Desired_Motor_Speed = 0.3;
    break;
    //used to test motor preformas with random speed and for random distances
    //simulates the requrements for balancing
    case 5:
      speed_test = random(8, 10);
      Serial.print(" ");
      Serial.println(speed_test);
      noInterrupts();
      Desired_Motor_Speed =  speed_test/10;
      interrupts();
      speed_test = random(10, 500);
      while(Step_Count <  speed_test) 
      {
        delay(1);
      };

      speed_test = random(8, 10);
      Serial.println(-speed_test);
      noInterrupts();
      Desired_Motor_Speed = -speed_test/10;
      interrupts();
      speed_test = random(10, 500);
      while(Step_Count > -speed_test) 
      {
        delay(1); 
      };
    break;
    //Mode for PID Control works between x and y degrees
    case 6:
      static bool Direction_Latch = 1;
      //get the current encoder angle
      noInterrupts();
      encoderAngle = Pot_Position;
      interrupts();

      if(Step_Count > 10)
      {
        pidSetpoint = 1;
      }
      else if(Step_Count < -10)
      {
        pidSetpoint = -0.5;
      }
      
      //difference from desiredAngle, doesn't account for desired angles other than 0
      if(Pot_Position > 0)
      {
        error = encoderAngle - 180 - 0.5;
      }
      else if(Pot_Position < 0)
      {
        error = 180 + encoderAngle + 0.5;
      }
      
      //do the PID stuff
      pidInput = error;
      myPID.Compute();

      // if the pendulum is too far off of vertical to recover, turn off the PID and motor
      if (error > 45 || error < -45) {
        myPID.SetMode(MANUAL);
        pidOutput = 0;
        Desired_Motor_Speed = 0;
      } else { //in bounds
        myPID.SetMode(AUTOMATIC);
        Desired_Motor_Speed = (float(pidOutput) * MAX_SPEED) / 256;
        //Serial.println((float(pidOutput) * MAX_SPEED) / 256);
      }
      delay(10);
    break;
    }
  }
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
  else if(Desired_Motor_Speed < 0 && Current_Frequency == 0 && Direction == 1 && Delay_Count >= 10)
  {
    Set_Directoin(0);
    Delay_Count = 0;
  }
  //if DS is less than 0 and the CF is less than or equle to 0 and the direction is correct calculate the correct PPS
  //if the is still or moving in the correct direction calculate the wanted fequency
  else if(Desired_Motor_Speed <= 0 && Current_Frequency >= 0 && Direction == 0 && Delay_Count >= 15)
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
  else if(Desired_Motor_Speed > 0 && Current_Frequency == 0 && Direction == 0 && Delay_Count >= 10)
  {
    Set_Directoin(1);
    Delay_Count = 0;
  }
  //if the DS is grater then 0 and then CF is grater than or equel to 0 and the directoin is correct calculated wanted frequency
  //if the cart is stil or moving in the correct direction calculate the wanted fequency
  else if(Desired_Motor_Speed >= 0 && Current_Frequency >= 0 && Direction == 1 && Delay_Count >= 15)
  {
    Desired_Frequency = ((Desired_Motor_Speed * Step_Res * 1) * Distance_Per_Step) * 2;
  }
  //This section controles the acceleration/dseleratoin of the motor by stepping the PPS up and down
  //uses a verible speed with the PPS
  //does this through the folowing exponetal function
  float Power = (10 * (float(Current_Frequency)/4444))/2;
  int PPS_Step = -exp(Power)+200;  //The PPS_Step sets the amount of steps that can be incred at one time without staling
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
    digitalWrite(LED_Blue,HIGH);
  }
  //if CF is 0 stop the interupt
  else if (Current_Frequency == 0)
  {
    Timer1.stop();
    //clear the pulse pin when timer is disabled to stop annoying buzzing
    g_APinDescription[Pulse].pPort -> PIO_CODR = g_APinDescription[Pulse].ulPin;
    digitalWrite(LED_Blue,LOW);
  }
  //update the current motor speed using the current frequency
  Current_Motor_Speed = (((((float)Current_Frequency)/2)/Distance_Per_Step)/Step_Res);

  if(!Direction)
    Current_Motor_Speed = -Current_Motor_Speed;
  //---------------------------------------------------------------//
}
//***********************************************************************//
void Step(void) {
  //---------------------------------------------------------------//
  //toggels the Pulse pin and only the pulse pin
  //pulse pin used is digital 28
  //digital wright is slow so the following lines clear and set the due register
  //with the signal tracking it's state

  //if the motor direction is moving forward incroment step count
  //esle decroment step count
  static bool state = 0;
  if (state == 0)
  {
    //set pin high
    g_APinDescription[Pulse].pPort -> PIO_SODR = g_APinDescription[Pulse].ulPin;
    state = 1;
    if (Direction == 1)
    {
      Step_Count++;
    }
    else
    {
      Step_Count--;
    }
  }
  else if (state == 1)
  {
    //set pin low
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
      Step_Count = Hall_One_Position;
      Desired_Motor_Speed = 0; //this will stop the cart while it waits to get to mode 3
      Mode = 3; //mode to return cart to the middle of the track
    }
    else
    {
      Step_Count = 0;
      Serial.println("Hall One Hit");
      Serial.println(Step_Count);
      Desired_Motor_Speed = 0;
    }
}

void Hall_Two_Hit   (void)
{
    if(Config_Mode == false)
    {
      Step_Count = Hall_Two_Position;
    }
    else
    {
      Serial.println("Hall Two Hit");
      Serial.println(Step_Count);
    }
}

void Hall_Three_Hit (void)
{
    if(Config_Mode == false)
    {
      Step_Count = Hall_Three_Position;
    }
    else
    {
      Serial.println("Hall Three Hit");
      Serial.println(Step_Count);
    }
}

void Hall_Four_Hit  (void)
{
    if(Config_Mode == false)
    {
      Step_Count = Hall_Four_Position;
    }
    else
    {
      Serial.println("Hall Four Hit");
      Serial.println(Step_Count);
    }
}

void Hall_Five_Hit  (void)
{
    if(Config_Mode == false)
    {
      Step_Count = Hall_Five_Position;
      Desired_Motor_Speed = 0; //this will stop the cart while it waits to get to mode 3
      Mode = 3; //mode to return cart to the middle of the track
    }
    else
    {
      Serial.println("Hall Five Hit");
      Serial.println(Step_Count);
      Desired_Motor_Speed = 0;
    }
}
//***********************************************************************//
//Sectoin for SPI function

//this function resives the latest message from the SPI register assosiated with SPI0
unsigned char GetByteFromSPI(void) 
{
    //reading REG_SPI0_SR clears the regester conferming a bit is resived
    //the register is a read-only so the bit can't be manuly cleared
    uint32_t s = REG_SPI0_SR;
    //takes a time stamp using millis then waits until the SPI0 flag is set
    unsigned long Byte_Start_Time = millis();
    while ((REG_SPI0_SR & SPI_SR_RDRF) == 0) 
    {
      
      //if the loop takes longer then the RDRF_TIMEOUT as defined the top
      //print a failer message and return 0xff telling the system there was an error
      //it may be possible that 0xff is a valid message if so concider making it invalid or finding a new fault bit
      if((millis()-Byte_Start_Time) > RDRF_TIMEOUT)
      {
        Serial.println("Timeout waiting for RDRF");
        return(0xff);
      }
    };
    //if the while loop is broken a message has been resived so return the message and mask the invalit bits
    //REG_SPI0_RDR is a 32 bit register but the message is only 16 bit (4 byte) that why the 0xFF mask is needed
    return (REG_SPI0_RDR & 0xFF);
}

//
void SPI_Manager(char opperatoin) 
{ 
  noInterrupts();
  switch (opperatoin) 
  {
    //standerd case where normal data is transmited
    case 0:
      //setup the standerd SPI_OUT data in normal conditions
      //the first 2 bytes will be the step count
      //the count needs to be cut in half then recontructed on the other side
      SPI_OUT[0] =  0x55; // standerd opperation start byte
      //run time packet
      static int Time_Stamp = 0;
      Time_Stamp = millis() - Run_Time;
      SPI_OUT[1] =  (Time_Stamp)     & 0xff;
      SPI_OUT[2] = ((Time_Stamp)>>8) & 0xff;
      SPI_OUT[3] = ((Time_Stamp)>>16)& 0xff;
      SPI_OUT[4] = ((Time_Stamp)>>24)& 0xff;

      SPI_OUT[5] =  Step_Count       & 0x00ff;
      SPI_OUT[6] = (Step_Count >> 8) & 0x00ff;

      SPI_OUT[7] =  (int16_t)(Current_Motor_Speed*100)     & 0xff;
      SPI_OUT[8] = ((int16_t)(Current_Motor_Speed*100)>>8) & 0xff;

      SPI_OUT[9] =  (int16_t)(Pot_Position*100)     & 0xff;
      SPI_OUT[10] = ((int16_t)(Pot_Position*100)>>8) & 0xff;

      SPI_OUT[11] =  (int16_t)(Pot_Velocity*100)     & 0xff;
      SPI_OUT[12] = ((int16_t)(Pot_Velocity*100)>>8)& 0xff; 
    break;
    //Run on arduino bootup
    case 1:
      //indicatte 
      SPI_OUT[0] = 0x46;
      //tell the PI arduino has bootup
      SPI_OUT[1] = 0x71;
    break;
    //tell PI a collision witht he limits has been made
    case 2:
      //indicatte 
      SPI_OUT[0] = 0x46;
      //tell the PI arduino has collided
      SPI_OUT[1] = 0x72;
    break;
  }
  interrupts();

  //make the checksum nad clear old SPI_IN data
  SPI_OUT[NUMBER_OF_BYTES - 1] = 0;
  for (int i = 0; i < NUMBER_OF_BYTES - 1; i++) {
    SPI_IN[i] = 0;
    SPI_OUT[NUMBER_OF_BYTES - 1] += SPI_OUT[i];
  } 
  //reading the SPI register clears the flag
  //the register is a read only so this is the only way to clear it
  int DummyRead = REG_SPI0_RDR; 

  //load TDR with the first byte to be ready for tx when Pi trasmits
  BytesRx = 0;
  REG_SPI0_TDR = SPI_OUT[BytesRx] & 0x0ff;; // load outgoing register 
  
  //request SPI transfer from Pi SPI master
  //this way of changing the pins is fater than digitalWrite()
  //there is no delay inbetween the pin high and pin low this may cause problems
  //set pin high
  g_APinDescription[SensorDataAvailablePin].pPort -> PIO_SODR = g_APinDescription[SensorDataAvailablePin].ulPin;
  //set pin low
  g_APinDescription[SensorDataAvailablePin].pPort -> PIO_CODR = g_APinDescription[SensorDataAvailablePin].ulPin;
  
  //each 8-bit SPI transfer should take 16uS with 500000Hz clock
  //the 1st byte from Pi will arrive immediately, then at 50uS intervals
  SPI_IN[BytesRx]=GetByteFromSPI(); // get 1st byte
  //if the fist byte is the start byte as expected procide with the data transfer
  if (SPI_IN[0]==0x55) 
  {
    do
    {
      BytesRx++;

      REG_SPI0_TDR = SPI_OUT[BytesRx] & 0x0ff; // load outgoing register 

      SPI_IN[BytesRx] = GetByteFromSPI(); 
    } while(BytesRx < (NUMBER_OF_BYTES-1));
    //decode complere packet
    if(SPI_IN[0] == 0x55)
    {
      //first byde declares if PID should be enabled
      PID_Enabled = SPI_IN[1];
      Kp          = SPI_IN[2];
      Ki          = SPI_IN[3];
      Kd          = SPI_IN[4];
    } 
  }
  //if start but was not resived in the first byte data is wrong
  //print message to the serial informing user bad transfer then reset the SPI register as in setup
  //hopfully proventing futer bad messages
  else 
  {
    //message to userial
    Serial.println("No start byte resived");
    SPI_IN[1] = 0;
    SPI_IN[2] = 0;
    Mode = 3;
    //reset the SPI registers to recover from fault
    REG_SPI0_WPMR = 0x53504900;   // Write Protection disable
    REG_SPI0_CR  = SPI_CR_SWRST;  // reset SPI (0x0080)
    REG_SPI0_CR  = SPI_CR_SPIEN;  // enable SPI (0x0001)
    REG_SPI0_MR  = SPI_MR_MODFDIS;// slave and no mode fault (0x0010)
    REG_SPI0_CSR = SPI_MODE0;     // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
    REG_SPI0_IDR = 0X0000070F;    // disable all SPI interrupts
  }

  if(SPI_OUT[0] == 0x46)
  {
    Run_Time = millis();
    Serial.println("New Run");
  }
}
//***********************************************************************//
//Sectoin for POT managment

//function to read the pot and calculate the anguler position and speed
void Pot_Manager(void) 
{
  static float Anguler_Count [10];
  static float Velocity_Count[9];
  static float sum   = 0;
  static int   count = 0;
  static int   dif   = 0;
  //calcultes the location of the pot Pot_Offset needs to be configured
  Pot_Position = (analogRead(Pot)*Degree_Per_Bit) - Pot_Offset;
  //used to take a rolling avrage for the position output
  Anguler_Count[count] = Pot_Position;
  count++;
  if(count > 9)
  {
    count = 0;
  }
  //now the lates pot value has been loaded the avrage can be found
  sum = 0;
  for(int i = 0; i < 10; i++)
  {
    sum += Anguler_Count[i];
  }
  Pot_Position = sum/10;
  //if the location is below 0 ie between true 0 and pit offset
  //this statment correct for that
  if(Pot_Position > 180)
  {
    Pot_Position = Pot_Position - 360;
  }
  //velocity is more complecated
  //to take an avrage the change in velocity over the anguler_count needs to be taken
  if(count < 9 && count > 0)
  {
    Velocity_Count[count] = (Anguler_Count[count] - Anguler_Count[count-1])/0.01;
  }
  else
  {
    Velocity_Count[count] = (Anguler_Count[count] - Anguler_Count[9])/0.01;
  }
  sum = 0;
  for (int i = 0; i < 9; i++)
  {
    sum += Velocity_Count[i];
  }
  Pot_Velocity = sum/9;
}
/*************************************************************
Jacob Threadgould 2016
*************************************************************/
