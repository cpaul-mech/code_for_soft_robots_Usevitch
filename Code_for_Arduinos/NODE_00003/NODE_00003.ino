/*
 * Preliminary code for the Middle Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for ME 497r at BYU. November 2023.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 *            TimerInterrupt, https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
 */

//--------------Timer Library setup--------------------//
// The link to the repository is as follows: https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
#define USE_TIMER_1     true
#define USE_TIMER_2     false
#warning Using Timer1, Timer2
#include "TimerInterrupt.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#define LED_PIN_RED 19   // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18 // green LED, use to indicate transmitting.
#define MOTOR_ERROR 6 // pin for reading error state from the half bridge motor driver. If it goes low, something is wrong.
#define MOTOR_SLEEP 4 // pin for controlling the sleep state of the half bridge motor driver. LOW is sleep, HIGH is awake.
#define FAST_BLINK 100  // miliseconds
#define SLOW_BLINK 1000 // miliseconds
RF24 radio(7, 8);       // CE, CSN
//-----------Motor Control Setup----------------//
#include "DCMotorControl.h"
DCMotorControl Motors[] = { //There is only one motor that each will be controlling. There is no need for multiple motors.
  //DCMotorControl(9, 10, 5, 2, 3), //DCMotorControl::DCMotorControl( uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor first!!
  DCMotorControl(10, 5, 3, 2) //DCMotorControl::DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor second to illustrate the point.
};
#define NumberOfMotors 1
#define ControlRate_ms 10
#define ControlRate_us 10000
#define TIMER_INTERVAL_MS 10L // 10ms, or 10,000us as specfified by the ControlRate_us variable in the DCMotorControl.h file.
#define DeadbandTicks 100
#define DeadbandDutyCycle 0
#define TicksPerInch ((50 * 64) / (3.14159265359 * 0.713))
#define TicksPerRevolution (50 * 64)
#define HomingSpeedTolerance 0.01
#define MinimumPWM 0
#define Kp 0.01
#define Ki 0.003
#define Kd 0.001
#define DutyCycleStall 25
#define MaxDutyCycleDelta 5
float DutyCycle = 0.0;
float CurrentRPM = 0.0;
int MotorDrive[NumberOfMotors] = {0};
bool MotorEnabled = false;
float DesiredPosition[NumberOfMotors] = {0};
int LastTicks = 0;
int CurrentTicks = 0;
int LastTimeMillis = 0;
int CurrentTimeMillis = 0;
uint8_t HomingMotor = 0;
int counter = 0;
bool motor_running = false;
//---------------VARIABLES-------------------//
enum child_state
{
  RECEIVING_1,
  RECEIVING_2,
  TRANSMITTING,
  OFF,
  COMPLETED
} child_state;

int global_received_data[8] = {0, 10, 0, 0, 0, 0, 0, 0};
int transmit_count = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
//this node is 00003, receives data from 00002, and is the end node.
const byte addresses[][6] = {"00001", "00002", "00003", "00004","00005"};
const byte* self = addresses[3];
const byte* parent = addresses[1];
int num_children = 0;
// -------------------- FUNCTIONS ------------------- //
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
    if (led_ON)
    {
      digitalWrite(LED_PIN_RED, LOW);
      led_ON = false;
    }
    else
    {
      digitalWrite(LED_PIN_RED, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}
void child_RX_1(void)
{
  blink_led_unblocking(SLOW_BLINK);
  if (radio.available())
  {
    radio.read(&global_received_data, sizeof(global_received_data));
    // iterate through values and print data
    for (int x = 0; x < 8; x++)
    {
      Serial.print(global_received_data[x]);
      Serial.print(" ");
    }
    Serial.println();
    if (num_children > 0)
    {
      //WE LEAVE CHILD STATE UNAFFECTED.
    }
    digitalWrite(LED_PIN_RED, HIGH);
    digitalWrite(LED_PIN_GREEN, HIGH);
    motor_startup();
  }
}
void Child_TX_function()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count = 0;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  // const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
  radio.write(&global_received_data, sizeof(global_received_data));
  child_state = RECEIVING_2;
  Serial.println("TRANSMITTING DATA");
  transmit_count++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}
void ControllerISR(void)
{
  // Serial.println("ControllerISR");
  // if we are tracking a trajectory, update the setpoint.
  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    Motors[i].run();
    // Serial.println("Motor Ran");
  }
}
void Child_RX_2()
{
  Serial.println("RECEIVING_2");
  // the radio should already be in listening mode.
  static unsigned long past_time_r = millis();
  blink_led_unblocking(SLOW_BLINK);
  // two paths out of receiving:
  //  1. We receive the data back.
  //  2. 1000 ms have passed so we go back to transmitting.
  if (millis() - past_time_r > 1000)
  {
    child_state = TRANSMITTING;
    past_time_r = millis();
  }
  else if (radio.available())
  {
    int received_data2[4][2];
    radio.read(&received_data2, sizeof(received_data2));
    bool data_correct = true;
    // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
    // Serial.println(recieved_data);
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < 2; y++)
      {
        Serial.print(received_data2[x][y]);
        Serial.print(" ");
        if (received_data2[x][y] != 1)
        {
          data_correct = false;
        }
      }
      Serial.println();
    }
    if (data_correct)
    {
      Serial.println("MATCHES");
      child_state = COMPLETED;
      digitalWrite(LED_PIN_RED, HIGH);
      digitalWrite(LED_PIN_GREEN, HIGH);
    }
    else
    {
      Serial.println("DOES NOT MATCH");
      child_state = TRANSMITTING; // something seems off here.
    }
  }
}
void print_motor_position(void)
{
  static unsigned int print_count = 0;
  if (print_count > 5000)
  {
    Serial.println(Motors[0].getCurrentPositionInches());
    print_count = 0;
  }
  else
  {
    print_count++;
  }
}
void position_reached_checker()
{
  // this should only be run every 2 seconds.
  static unsigned long past_time = millis();
  // checks if the motor has reached its desired position. Then it puts the motor driver to sleep and stops the timer.
  if (millis() - past_time > 2000)
  {
    past_time = millis();
    if (Motors[0].getCurrentPositionInches() >= (Motors[0].getDesiredPositionInches() - 0.1) && Motors[0].getCurrentPositionInches() <= (Motors[0].getDesiredPositionInches() + 0.1))
    {
      // the motor has reached its desired position.
      Serial.println("Motor has reached desired position.");
      digitalWrite(MOTOR_SLEEP, LOW); // put the motor to sleep.
      motor_running = false;
      ITimer1.pauseTimer(); // stop the timer for now.
    }
  }
}
void motor_startup()
{
  // This function will be used to start the motor, wake the motor driver, and start the timer.
  digitalWrite(MOTOR_SLEEP, HIGH); // wake the motor driver.
  Serial.println("Beginning motor control.");
  auto new_motor_position = global_received_data[1]; // make sure to get the data that corresponds to this roller.
  Serial.print("New Motor Position: ");
  Serial.println(new_motor_position);
  Motors[0].setDesiredPositionInches(new_motor_position);
  motor_running = true;
  ITimer1.resumeTimer();
}

void setup()
{
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(parent);    // 00002 the address of the middle node.
  radio.openReadingPipe(0, self); // 00003 the address of the end node. (THIS MODULE)
  radio.setPALevel(RF24_PA_MIN);          // This sets the power level at which the module will transmit.
                                 // The level is super low now because the two modules are very close to each other.
  child_state = RECEIVING_1;
  radio.startListening();
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
  pinMode(MOTOR_ERROR, INPUT);
  pinMode(MOTOR_SLEEP, OUTPUT);
  digitalWrite(MOTOR_SLEEP, HIGH); // set motor to awake.
#if USE_TIMER_1
  ITimer1.init();
  if (ITimer1.attachInterruptInterval(ControlRate_ms, ControllerISR))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
    ITimer1.pauseTimer();
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
#endif
#if USE_TIMER_2
  ITimer2.init();
  if (ITimer2.attachInterruptInterval(6000, FLIP_Direction))
  {
    Serial.print(F("Starting  ITimer2 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer2. Select another freq. or timer"));
#endif
  for (uint8_t i = 0; i < NumberOfMotors; i++)
  {
    Motors[i].setParameters(Kp, Ki, Kd, ControlRate_us, DeadbandTicks, DeadbandDutyCycle, TicksPerInch, TicksPerRevolution, MinimumPWM);
    Motors[i].setDutyCycleStall(DutyCycleStall);
    Motors[i].setMaxDutyCycleDelta(MaxDutyCycleDelta);
  }

  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    MotorEnabled = true;
    Motors[i].setMotorEnable(MotorEnabled);
    Motors[i].setMode(DC_Automatic);
  }
  motor_startup(); // I want it to run the motor as soon as it is set up, then listen for further instructions.
  Serial.println("Setup Complete");
}

void loop()
{
  if (motor_running)
  {
    position_reached_checker();
    print_motor_position();
  }
  switch (child_state)
  {
  case RECEIVING_1: // this one will be run muliple timees.
    child_RX_1();
    break;
  case RECEIVING_2: // this is the one waiting for if the sent data was correct.
    // if the data was correctly received, the tx will send back an array of all ones.
    // Serial.println("RECEIVING_2");
    Child_RX_2();

    break;

  case TRANSMITTING: // stay here for a few times at least before i implement the second bounce back.
    Child_TX_function();

    break;

  case OFF:
    Serial.println("OFF");
    // blink_led(500);
    // transmitter_state = TRANSMITTING;
    /// radio.stopListening();

    break;

  case COMPLETED:
    // link_led_unblocking(5000);
    
    Serial.println("COMPLETED");
    
    break;

  default:
    Serial.println("ERROR: transmitter_state is in an unknown state");
    break;
  }
}
