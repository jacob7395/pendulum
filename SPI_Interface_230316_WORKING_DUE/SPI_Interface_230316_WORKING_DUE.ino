/*
 * adc resources
 * http://www.djerickson.com/arduino/
 * 
 * spi resources
 * https://github.com/manitou48/DUEZoo/blob/master/spislave.ino
 * http://forum.arduino.cc/index.php?topic=157203.5;wap2
 * http://asf.atmel.com/docs/3.8.1/sam.drivers.wdt.unit_tests.sam4s_ek/html/group___s_a_m4_s___s_p_i.html
 * http://forum.arduino.cc/index.php?topic=157203.0
 * http://www.jaxcoder.com/Article/SinglePost?postID=1805861259
 * http://neurorobotictech.com/Community/Blog/tabid/184/ID/16/Using-the-Jetson-TK1-SPI--Part-6-Programming-Arduino-Due-as-SPI-Slave.aspx
 * https://www.arduino.cc/en/Reference/SPI
 * http://21stdigitalhome.blogspot.co.uk/2013/02/arduino-due-hardware-spi.html
 * http://forum.arduino.cc/index.php?topic=167992.0
 * http://www.atmel.com/images/atmel-11057-32-bit-cortex-m3-microcontroller-sam3x-sam3a_datasheet.pdf
 * 
 * arduino resources
 * https://www.academia.edu/7114110/Programming_Arduino_Next_Steps_Going_Further_with_Sketches
 * http://www.slideshare.net/venkateshselvaraju/thesis-v0613th-jan
 * http://arduino.stackexchange.com/questions/860/how-can-i-get-the-source-files-for-arduino-libraries
 * 
 * stepper resource
 * http://www.airspayce.com/mikem/arduino/AccelStepper/
 * http://www.instructables.com/id/Playing-with-Accelstepper-Code-HodgePodging-for-a-/
 */
 
#include <SPI.h>
#include <AccelStepper.h>
#include <string.h>

// this defines how often (mS) we request SPI transfer from Pi
#define TX_INTERVAL 50

// timeout in mS for "rx data register full" flag to be set
#define RDRF_TIMEOUT 30

// timeout in mS for a full packet (SPI data frame) to be received
#define PACKET_TIMEOUT 100

// SPI input and output buffers

unsigned char SPI_in[16];  // holds incoming data from Pi
                                   // byte 0 - should be 0x55
                                   // byte 1 - speed lo
                                   // byte 2 - speed hi
                                   // byte 3 - direction 1 = +ve
                                   // 7 padding bytes - no data
                                   // chksum
                                   
unsigned char SPI_out[16]; // holds outgoing SPI data 
                                    // 0x55 - start of message
                                    // byte 1 - angle lo
                                    // byte 2 - angle hi
                                    // byte 3 - position (steps) lo 
                                    // byte 4 - position 
                                    // byte 5 - position 
                                    // byte 6 - position hi
                                    // byte 7 - ticks (mS) lo 
                                    // byte 8 -  
                                    // byte 9 -  
                                    // byte 10 - ticks hi
                                    // chksum
                                                                     
unsigned int BytesRx;    // count of bytes rx on SPI     

long CurrentPosition;
unsigned long PreviousTime;
unsigned long Timeout;
unsigned long ByteStartTime;
unsigned long PacketStartTime;
unsigned long PacketsTransferred;
unsigned long ChecksumErrors;

int SetSpeed;   // speed from Pi - decoded from SPI
unsigned int Direction;  // direction from Pi - decoded from SPI

int SensorDataAvailablePin = 49; // tells Pi that sensor data is ready for SPI transfer
int AngleSensorPin = 0;   // ANALOG A0 - wiper from potentiometer - analog reference is default 5V internal

// StepperAccel uses PWM 2 & 3 
// STEP PULSES PWM 2
// DIRECTION PWM 3

// MOSI SPI 4 - the header nearest TX0
// MISO SPI 1
// SCK  SPI 3
// NSS  PWM 10 (CS from Pi)

boolean DataTransferActive; // the sensor data is ready for SPI transfer
unsigned int Angle;

unsigned int k;
unsigned char chksum;
unsigned long DummyRead;
uint32_t OutByte;

// debug
unsigned long timenow;
unsigned long timeout;
unsigned long LastStepTime;
unsigned long LoopDuration;

AccelStepper stepper(1); // 2 wire interface - pulse and direction

unsigned char GetByteFromSPI() {

    uint32_t d;
    uint32_t s = REG_SPI0_SR;  // clear overrun error

    ByteStartTime=millis();
    while (((REG_SPI0_SR & SPI_SR_RDRF) == 0 && ((millis()-ByteStartTime<RDRF_TIMEOUT)))) ;  // wait till byte rx
    if ((millis()-ByteStartTime)<RDRF_TIMEOUT)
      d = REG_SPI0_RDR; // read incoming byte
      // the Pi SPI master will wait 100us before it sends the next byte
    else {
      Serial.println("Timeout waiting for RDRF");
      return(0xff);
    }
    return d & 0xFF;
}

void setup() {

  unsigned char j;
  
  BytesRx = 0;
  DataTransferActive = false;
  for (j=0; j<12; ++j) {
    SPI_in[j]=0;
    SPI_out[j]=0;
  }

  PacketsTransferred=0;
  ChecksumErrors=0;

  Serial.begin(115200);

  pinMode(4, OUTPUT); // test pin
  
  // initialise port pin that tells Pi we have SPI data available
  // a -ve edge will generate interrupt on Pi
  pinMode(SensorDataAvailablePin, OUTPUT); 
  digitalWrite(SensorDataAvailablePin, HIGH);

  analogReadResolution(10); // resolution of ADC (8, 10, 12)
  // boost adc conversion speed from 39us to 6us !
  //REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;

  delay(2000);
  
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(0); 

  // setup SPI as slave
  SPI.begin(10); // do we need to do this ?

  Serial.print("SPI0_MR ");   Serial.println(REG_SPI0_MR, HEX); 
  Serial.print("SPI0_CSR ");  Serial.println(REG_SPI0_CSR, HEX); 
  Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);

  REG_SPI0_WPMR = 0x53504900;   // Write Protection disable
  REG_SPI0_CR = SPI_CR_SWRST;   // reset SPI (0x0080)
  REG_SPI0_CR = SPI_CR_SPIEN;   // enable SPI (0x0001)
  REG_SPI0_MR = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
  REG_SPI0_CSR = SPI_MODE0;     // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
  REG_SPI0_IDR = 0X0000070F;     // disable all SPI interrupts

  Serial.print("SPI0_MR ");   Serial.println(REG_SPI0_MR, HEX); 
  Serial.print("SPI0_CSR ");  Serial.println(REG_SPI0_CSR, HEX); 
  Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);
  
  PreviousTime = millis();
}

void loop() {  

  stepper.runSpeed(); // this takes 3-23 uS
  
  //g_APinDescription[4].pPort -> PIO_SODR = g_APinDescription[4].ulPin;
        
  if ( ( (millis() - PreviousTime) > TX_INTERVAL ) && !DataTransferActive) {
    // it takes 45us to get to BytesRx=0 from here
    PreviousTime = millis();

    // get data ready to tx on SPI transfer
    Angle = analogRead(AngleSensorPin); // this takes 6uS ?
    CurrentPosition = stepper.currentPosition();

    SPI_out[0] = 0x55; // valid start of message
    SPI_out[1] = Angle & 0x0ff; // bottom 8 bits
    SPI_out[2] = ((Angle & 0xf00) >> 8); // bits 8,9 (0xf00 for 12 bit)

    SPI_out[3] = (CurrentPosition & 0x000000ff);
    SPI_out[4] = (CurrentPosition & 0x0000ff00) >> 8;
    SPI_out[5] = (CurrentPosition & 0x00ff0000) >> 16;
    SPI_out[6] = (CurrentPosition & 0xff000000) >> 24;

    SPI_out[7] = (PreviousTime & 0x000000ff);
    SPI_out[8] = (PreviousTime & 0x0000ff00) >> 8;
    SPI_out[9] = (PreviousTime & 0x00ff0000) >> 16;
    SPI_out[10] = (PreviousTime & 0xff000000) >> 24;

    // form message with checksum
    SPI_out[11] = 0;
    for (k=0; k<11; ++k) {
      SPI_out[11] += SPI_out[k];
    } 
    
    stepper.runSpeed(); // adc took a while so do this ?
     
    //Serial.println("Packet Request"); 
    DummyRead = REG_SPI0_RDR; // FLUSH - read spurious data in buffer

    //g_APinDescription[4].pPort -> PIO_SODR = g_APinDescription[4].ulPin; // test pin high

    // load TDR here as it needs to be ready for tx when Pi (master) sends
    BytesRx = 0;
    OutByte = SPI_out[BytesRx] & 0x0ff;
    //while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
    REG_SPI0_TDR = OutByte; // load outgoing register 
    
    // request SPI transfer from Pi SPI master via -ve edge interrupt
    // use our own inline code as digitalWrite() is slow
    g_APinDescription[SensorDataAvailablePin].pPort -> PIO_SODR = g_APinDescription[SensorDataAvailablePin].ulPin;
    g_APinDescription[SensorDataAvailablePin].pPort -> PIO_CODR = g_APinDescription[SensorDataAvailablePin].ulPin;
    
    // each 8-bit SPI transfer should take 16uS with 500000Hz clock
    // the 1st byte from Pi will arrive immediately, then at 50uS intervals
    // 3333pps on stepper is 300uS period
    // we need to call runSpeed() as often as possible as
    // runSpeed() checks when a step transition is required
    
    SPI_in[BytesRx]=GetByteFromSPI(); // get 1st byte
    if (SPI_in[0]==0x55) {
      BytesRx = 1;
      //while ((REG_SPI0_SR & SPI_SR_TDRE) == 0);
      OutByte = SPI_out[BytesRx] & 0x0ff;
      REG_SPI0_TDR = OutByte; // load outgoing register 

      //Serial.print(BytesRx); Serial.print(SPI_out[BytesRx-1], HEX); Serial.print(" "); Serial.println(SPI_in[BytesRx-1], HEX);

      // we have 50uS to get to the next transfer()
      DataTransferActive = true;  
      PacketStartTime = millis();
      //Timeout = millis() + 100;  
      stepper.runSpeed(); // (>17us?) have we time to do this ?
    } else {
      // not a valid SOM - ignore
      Serial.print("Bad SOM "); Serial.println(PreviousTime);
      PacketStartTime = millis();
      // reset the SPI registers to recover from fault - THIS HAS TO BE DONE !
      REG_SPI0_WPMR = 0x53504900;   // Write Protection disable
      REG_SPI0_CR = SPI_CR_SWRST;   // reset SPI (0x0080)
      REG_SPI0_CR = SPI_CR_SPIEN;   // enable SPI (0x0001)
      REG_SPI0_MR = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
      REG_SPI0_CSR = SPI_MODE0;     // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
      REG_SPI0_IDR = 0X0000070F;     // disable all SPI interrupts
    }
  } else;
  
  if (DataTransferActive) {
  
    if ((millis()-PacketStartTime)<PACKET_TIMEOUT) {
      
      SPI_in[BytesRx]=GetByteFromSPI(); 
      ++BytesRx;
      
      //Serial.print(BytesRx); Serial.print(" ");Serial.print(SPI_out[BytesRx-1], HEX); Serial.print(" "); Serial.println(SPI_in[BytesRx-1], HEX);      

      if (BytesRx==12) {

        BytesRx = 0;
        DataTransferActive = false;
        
        ++PacketsTransferred;
        //Serial.print("Packets Rx "); Serial.println(PacketsTransferred);
        BytesRx = 0;
        DataTransferActive = false;
        //for (k=0; k<12; ++k) 
          //Serial.print("RX ");Serial.println(SPI_in[k], HEX);

        // check the checksum
        chksum = 0;
        for (k=0; k<11; ++k)
          chksum += SPI_in[k];
        if (chksum == SPI_in[11]) {
          SetSpeed = (SPI_in[2]<<8)+SPI_in[1];
          //SetSpeed = (SPI_in[2]*256)+SPI_in[1];
          Direction = SPI_in[3];  
          if (Direction) 
            SetSpeed = -SetSpeed;
          else;
          //Serial.print("Speed "); Serial.println(SetSpeed);
          //Serial.print("Direction ");Serial.println(Direction);
          stepper.setSpeed(SetSpeed);
          stepper.runSpeed();
          //g_APinDescription[4].pPort -> PIO_CODR = g_APinDescription[4].ulPin; // test pin low
        } else {
          ++ChecksumErrors;
          Serial.print("Checksum errors "); Serial.print(ChecksumErrors);  Serial.println(PreviousTime);
          for (k=0; k<12; ++k) {
            Serial.print("RX ");Serial.println(SPI_in[k], HEX);
          }
          // reset the SPI registers to recover from fault - THIS HAS TO BE DONE !
          REG_SPI0_WPMR = 0x53504900;   // Write Protection disable
          REG_SPI0_CR = SPI_CR_SWRST;   // reset SPI (0x0080)
          REG_SPI0_CR = SPI_CR_SPIEN;   // enable SPI (0x0001)
          REG_SPI0_MR = SPI_MR_MODFDIS; // slave and no mode fault (0x0010)
          REG_SPI0_CSR = SPI_MODE0;     // DLYBCT=0, DLYBS=0, SCBR=0, 8 bit transfer (0x0002)
          REG_SPI0_IDR = 0X0000070F;     // disable all SPI interrupts
          //Serial.print("SPI0_MR ");   Serial.println(REG_SPI0_MR, HEX); 
          //Serial.print("SPI0_SR ");  Serial.println(REG_SPI0_SR, HEX); 
          //Serial.print("SPI0_WPMR "); Serial.println(REG_SPI0_WPMR, HEX);
          //Serial.print("SPI0_RDR "); Serial.println(REG_SPI0_RDR);
          if ((REG_SPI0_SR & SPI_SR_RDRF) == 0)
            DummyRead = REG_SPI0_RDR; // read incoming byte
          else;
        }

      } else {
        OutByte = SPI_out[BytesRx] & 0x0ff;
        REG_SPI0_TDR = OutByte; // load outgoing register 
      }
      
    } else {
      // timeout
      Serial.println("SPI packet timeout");            
      DataTransferActive=false;
      BytesRx=0;
    }  
              
  } else; 

}



