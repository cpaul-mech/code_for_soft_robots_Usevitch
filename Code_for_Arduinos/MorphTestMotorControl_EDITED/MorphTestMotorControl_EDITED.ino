
//Radio Communication Setup
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(15,14); // CE, CSN

typedef enum {  RC_Zero, RC_SetDesiredPositionAll, RC_ResetRadio, RC_SetCurrentPosition, RC_ReadCurrentPosition, RC_SetDesiredPosition} RadioCommands_t;
const uint64_t pipes[25] = { 0xF0F0F0F0CDLL, 0xF0F0F0F061LL, 0xF0F0F0F081LL, 0xF0F0F0F0A1LL, 0xF0F0F0F0C1LL,
                              0xF0F0F0F025LL, 0xF0F0F0F065LL, 0xF0F0F0F085LL, 0xF0F0F0F0A5LL, 0xF0F0F0F0C5LL,
                              0xF0F0F0F029LL, 0xF0F0F0F069LL, 0xF0F0F0F089LL, 0xF0F0F0F0A9LL, 0xF0F0F0F0C9LL,
                              0xF0F0F0F02BLL, 0xF0F0F0F06BLL, 0xF0F0F0F08BLL, 0xF0F0F0F0ABLL, 0xF0F0F0F0CBLL,
                              0xF0F0F0F02DLL, 0xF0F0F0F06DLL, 0xF0F0F0F08DLL, 0xF0F0F0F0ADLL, 0xF0F0F0F0CFLL,};
                              
#define NodeID 0
#define RF_Channel 2
#define RadioResetRate_us 2000000

//Motor Control Setup
#include "DCMotorControl.h"
DCMotorControl Motors[] = {
  DCMotorControl(16,17,5,2,3),    // Motor 0 DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin)

};
#define NumberOfMotors (sizeof(Motors)/sizeof(Motors[0]))
#define ControlRate_us 10000
#define DeadbandTicks 100
#define DeadbandDutyCycle 0
#define TicksPerInch ((50*64)/(3.14159265359*0.713))
#define TicksPerRevolution (50*64)
#define HomingSpeedTolerance 0.01
#define MinimumPWM 0
#define Kp 0.01
#define Ki 0.003
#define Kd 0.001
#define DutyCycleStall 25
#define MaxDutyCycleDelta 5

float DutyCycle = 0.0;
float CurrentRPM = 0.0;
int   MotorDrive[NumberOfMotors] = {0};
bool  MotorEnabled = false;
float DesiredPosition[NumberOfMotors] = {0};
int LastTicks = 0;
int CurrentTicks = 0;
int LastTimeMillis = 0;
int CurrentTimeMillis = 0;
uint8_t HomingMotor = 0;
int counter = 0;

// IntervalTimer ControlTimer;
// IntervalTimer RadioResetTimer;
void ControllerISR(void);
//Debug Setup
#define DEBUG 1

void setup() {
  // radio.begin();
  // radio.openReadingPipe(0, pipes[NodeID]);
  // radio.setPALevel(RF24_PA_HIGH);
  // radio.setChannel(RF_Channel);
  // radio.startListening();

  for(uint8_t i = 0; i < NumberOfMotors; i++){
    Motors[i].setParameters(Kp, Ki, Kd, ControlRate_us, DeadbandTicks, DeadbandDutyCycle, TicksPerInch, TicksPerRevolution, MinimumPWM);
    Motors[i].setDutyCycleStall(DutyCycleStall);
    Motors[i].setMaxDutyCycleDelta(MaxDutyCycleDelta);
    
  }

  for(uint8_t i = 0; i < (NumberOfMotors); i++){
    MotorEnabled = true;
    Motors[i].setMotorEnable(MotorEnabled);
    Motors[i].setMode(DC_Automatic);
  }
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  // ControlTimer.begin( ControllerISR , ControlRate_us ); // attach the service routine here
  // RadioResetTimer.begin( RadioResetISR , RadioResetRate_us ); // attach the service routine here
}
void loop() {
  // if (radio.available()) {
  //   RadioResponse();
  // }
  /*
    char inChar = 0;
  if (Serial.available() > 0) {
      // read incoming serial data:
      inChar = Serial.read();
      // clear everything else to prevent flooding (We are polling the keyboard so we dont care about dumping this data)
      while(Serial.available()){Serial.read();}
  }
  switch( inChar){
      case 'p':{
        radio.printDetails();
        
        break;
      }
      case 'r':{
          radio.begin();
          radio.failureDetected = 0;           // Reset the detection value
          radio.setChannel(RF_Channel);
          radio.openReadingPipe(0, pipes[NodeID]);
          radio.setPALevel(RF24_PA_HIGH);
          radio.startListening();
          delay(10);
          Serial.println("ResetRadio");
          break;
      }
      case 'f':{
          radio.flush_rx();
          
          Serial.println("Flush RX");
          break;
      }
      case 's':
      {
        radio.openReadingPipe(0, pipes[NodeID]);
          radio.startListening();
          break;
      }
      default:
        break;        
  }
    Serial.print(NodeID);
        Serial.print(" | Motor 0: ");
        Serial.print(Motors[0].getDesiredPositionTicks());
        Serial.print("\t");
        Serial.print(Motors[0].getCurrentPositionTicks());
        Serial.print("\t");
        Serial.print(Motors[0].getDutyCycle());
        Serial.print("\t");
        Serial.print("| radio debug | ");
        Serial.print(radio.failureDetected);
        Serial.println("");
    //
    delay(100);
    */
   Motors[0].setDesiredPositionTicks(1000);
   Motors[0].run();
     
}

void RadioResponse(void){
    long Ticks[8] = {0};
    //RadioResetTimer.begin( RadioResetISR , RadioResetRate_us ); // attach the service routine here
    Serial.println("Message Received");
    radio.read(&Ticks, sizeof(Ticks));
    Serial.print(Ticks[0]);
    Serial.print(" ");
    Serial.print(Ticks[1]);
    Serial.print(" ");
    Serial.println(Ticks[2]);
    switch(Ticks[0]){
      case RC_SetDesiredPositionAll:
      {
        for(uint8_t i = 0; i < (NumberOfMotors); i++){
          Motors[i].setDesiredPositionTicks(Ticks[i+1]);
        }      
        
      Serial.print(NodeID);
      Serial.print(" | Motor 0: ");
      Serial.print(Motors[0].getDesiredPositionTicks());
      Serial.print("\t");
      Serial.print(Motors[0].getCurrentPositionTicks());
      Serial.print("\t");
      Serial.print(Motors[0].getDutyCycle());
      Serial.print("\t");
      Serial.print(" | Motor 1: ");
      Serial.print(Motors[1].getDesiredPositionTicks());
      Serial.print("\t");
      Serial.print(Motors[1].getCurrentPositionTicks());
      Serial.print("\t");
      Serial.print(Motors[1].getDutyCycle());
      Serial.print("\t");
      Serial.print("| radio debug | ");
      Serial.print(radio.failureDetected);
      Serial.print(" ");
      Serial.print(radio.available());
      Serial.println("");
      
        break;
      }
      case RC_Zero:
      {
        Serial.println("Zero");
        if (0 == Ticks[1])
        {
          for(uint8_t i = 0; i < (NumberOfMotors); i++){
            Motors[i].setCurrentPositionTicks(0);
            Motors[i].setDesiredPositionTicks(0);
          }
        } else{
          Motors[Ticks[1]-1].setCurrentPositionTicks(0);
          Motors[Ticks[1]-1].setDesiredPositionTicks(0);
        }
        break;
      }
      case RC_ResetRadio:
        {
          radio.begin();
          radio.failureDetected = 0;           // Reset the detection value
          radio.openReadingPipe(0, pipes[NodeID]);
          radio.setPALevel(RF24_PA_HIGH);
          radio.startListening();
          delay(10);
          Serial.println("ResetRadio");
          break;
          
        }
      default:
      break;
    }
  }


void ControllerISR(void) {
  // if we are tracking a trajectory, update the setpoint.
  for(uint8_t i = 0; i < (NumberOfMotors); i++){
    Motors[i].run();
  }
}


void RadioResetISR(void) {
  if (radio.available()) {
    RadioResponse();
  }
  radio.begin();
  radio.openReadingPipe(0, pipes[NodeID]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RF_Channel);
  radio.startListening();
}
