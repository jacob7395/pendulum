/*************************************************************
This program will be used to control a NEMA24 stepper motor for
an inverted pendulem proble. It will be reading a single pedomiter
and 2 switches. Then feeding the required infomatoin to a rasberry
pi that will retrun the next actoin.

Disclaimer i'm dyslexic and there is no spell check
*************************************************************/

#include <DueTimer.h>
#include <SPI.h>
#include <string.h>

//this defines how often (mS) we request SPI transfer from Pi
#define TX_INTERVAL 50
//timeout in mS for "rx data register full" flag to be set
#define RDRF_TIMEOUT 30
//timeout in mS for a full packet (SPI data frame) to be received
#define PACKET_TIMEOUT 100
//the number of bytes being transmited and resived in one SPI message incluing start and chack
#define NUMBER_OF_BYTES 12

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
void SPI_Manager(void);

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
// SPI input and output arrays
unsigned char SPI_in [NUMBER_OF_BYTES]; // holds incoming data from Pi
unsigned char SPI_out[NUMBER_OF_BYTES]; // holds outgoing SPI data 
                                                                     
unsigned int BytesRx;    // count of bytes rx on SPI     

long CurrentPosition;
unsigned long PreviousTime;
unsigned long Timeout;
unsigned long PacketStartTime;
unsigned long PacketsTransferred;
unsigned long ChecksumErrors;

int SetSpeed;   // speed from Pi - decoded from SPI

int SensorDataAvailablePin = 49; // tells Pi that sensor data is ready for SPI transfer
int AngleSensorPin = 0;   // ANALOG A0 - wiper from potentiometer - analog reference is default 5V internal

boolean DataTransferActive; // the sensor data is ready for SPI transfer
unsigned int Angle;

unsigned int k;
unsigned char chksum;
uint32_t OutByte;

// debug
unsigned long timenow;
unsigned long timeout;
unsigned long LastStepTime;
unsigned long LoopDuration;

bool Ready_For_Data = false;

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
    SPI_in[j]=0;
    SPI_out[j]=0;
  }
  //comment?
  PacketsTransferred  =0;
  ChecksumErrors      =0;
  
  // initialise port pin that tells the Pi SPI data available
  // PI is looking for a falling edge so after initalzing the pin set the pin high
  pinMode     (SensorDataAvailablePin, OUTPUT ); 
  digitalWrite(SensorDataAvailablePin, HIGH   );

  // setup SPI as slave
  SPI.begin(10); // is this needed ?

  //Sets up the SPI regesters and enables the arduino as a salve
  //it also preints the content of key registers befor and after setting them
  //this is not needed to make it work but gives some debug info
  Serial.print("SPI0_MR "  ); Serial.println(REG_SPI0_MR  , HEX); 
  Serial.print("SPI0_CSR " ); Serial.println(REG_SPI0_CSR , HEX); 
  Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);

  REG_SPI0_WPMR = 0x53504900    ; // Write Protection disable
  REG_SPI0_CR   = SPI_CR_SWRST  ; // reset  SPI (0x0080)
  REG_SPI0_CR   = SPI_CR_SPIEN  ; // enable SPI (0x0001)
  REG_SPI0_MR   = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
  REG_SPI0_CSR  = SPI_MODE0     ; // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
  REG_SPI0_IDR  = 0X0000070F    ; // disable all SPI interrupts

  Serial.print("SPI0_MR ")  ; Serial.println(REG_SPI0_MR  , HEX); 
  Serial.print("SPI0_CSR " ); Serial.println(REG_SPI0_CSR , HEX); 
  Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);
  //Timer 3 starts and manages the SPI data transfer
  Timer3.attachInterrupt(SPI_Manager).setFrequency(10).start();

  //--------------------------------------------------------------------//
  //Motor and sensor setup

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
  
  //--------------------------------------------------------------------//
}
//***********************************************************************//
void loop() {
  delay(1000);
  Serial.println(" ");
  Serial.println("Begin");
  Mode = 1;
  
  for( ; ; )
  {
    switch(Mode) {
    
    case 0: //case to stop the cart
      if(Ready_For_Data == false)
      {
        Ready_For_Data = true;
      }
      Config_Mode = false;
      Desired_Motor_Speed = 0;
      delay(100);
    break;
    //this case uses a config opperation
    //it current travels slowly left towards hall 1
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
      Mode = 0;
    break;
    //temp state to initiate SPI transfer
    case 2:
      SPI_Manager();
      Ready_For_Data = false;
      Mode = 0;  
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
  else if(Desired_Motor_Speed < 0 && Current_Frequency == 0 && Direction == 1 && Delay_Count >= 5)
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
  else if(Desired_Motor_Speed > 0 && Current_Frequency == 0 && Direction == 0 && Delay_Count >= 5)
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
  //toggels the Pulse pin and only the pulse pin
  //pulse pin used is digital 28
  //digital wright is slow so the following lines clear and set the due register
  //with the signal tracking it's state

  //if the motor direction is moving forward incroment step count
  //esle decroment step count
  static bool state = 0;
  if (state == 0)
  {
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
void SPI_Manager(void) 
{ 
  SPI_out[0] = 0x55; // valid start of message
  SPI_out[1] = 0x01; // Data to be transferd
  SPI_out[2] = 0x02; // Data to be transferd
  SPI_out[3] = 0x03; // Data to be transferd
  SPI_out[4] = 0x04; // Data to be transferd
  SPI_out[5] = 0x05; // Data to be transferd
  SPI_out[6] = 0x06; // Data to be transferd
  SPI_out[7] = 0x07; // Data to be transferd
  SPI_out[8] = 0x08; // Data to be transferd
  SPI_out[9] = 0x09; // Data to be transferd
  SPI_out[10]= 0x0A; // Data to be transferd

  //make the checksum
  SPI_out[NUMBER_OF_BYTES - 1] = 0;
  for (k = 0; k < NUMBER_OF_BYTES - 1; k++) {
    SPI_out[NUMBER_OF_BYTES - 1] += SPI_out[k];
  } 
  //reading the SPI register clears the flag
  //the register is a read only so this is the only way to clear it
  int DummyRead = REG_SPI0_RDR; 

  //load TDR with the first byte to be ready for tx when Pi trasmits
  BytesRx = 0;
  REG_SPI0_TDR = SPI_out[BytesRx] & 0x0ff;; // load outgoing register 
  
  //request SPI transfer from Pi SPI master
  //this way of changing the pins is fater than digitalWrite()
  //there is no delay inbetween the pin high and pin low this may cause problems
  //set pin high
  g_APinDescription[SensorDataAvailablePin].pPort -> PIO_SODR = g_APinDescription[SensorDataAvailablePin].ulPin;
  //set pin low
  g_APinDescription[SensorDataAvailablePin].pPort -> PIO_CODR = g_APinDescription[SensorDataAvailablePin].ulPin;
  
  //each 8-bit SPI transfer should take 16uS with 500000Hz clock
  //the 1st byte from Pi will arrive immediately, then at 50uS intervals
  SPI_in[BytesRx]=GetByteFromSPI(); // get 1st byte
  //if the fist byte is the start byte as expected procide with the data transfer
  if (SPI_in[0]==0x55) 
  {
    do
    {
      BytesRx++;

      REG_SPI0_TDR = SPI_out[BytesRx] & 0x0ff; // load outgoing register 
      
      char data = GetByteFromSPI();
      
    } while(BytesRx < (NUMBER_OF_BYTES-1))  ;  
  }
  //if start but was not resived in the first byte data is wrong
  //print message to the serial informing user bad transfer then reset the SPI register as in setup
  //hopfully proventing futer bad messages
  else 
  {
    //message to userial
    Serial.print("No start byte resived"); Serial.println(PreviousTime);
    //reset the SPI registers to recover from fault
    REG_SPI0_WPMR = 0x53504900;   // Write Protection disable
    REG_SPI0_CR = SPI_CR_SWRST;   // reset SPI (0x0080)
    REG_SPI0_CR = SPI_CR_SPIEN;   // enable SPI (0x0001)
    REG_SPI0_MR = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
    REG_SPI0_CSR = SPI_MODE0;     // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
    REG_SPI0_IDR = 0X0000070F;    // disable all SPI interrupts
  }
}

void SPI_Timer(void)
{
  if(Ready_For_Data == true)
  {
    Ready_For_Data = false;
    Mode = 2;
  }
}
/*************************************************************
Jacob Threadgould 2016
*************************************************************/
