/*
 * Preliminary code for the Middle Node module of the soft Robotics Project
 * Soft Robotics For the Moon!
 * Run by doctor Nathan Usevitch, Assitant Professor at Brigham Young University.
 * Code Written by Christopher Paul for ME 497r at BYU. November 2023.
 * Libraries: TMRh20/RF24, https://github.com/tmrh20/RF24/
 *            TimerInterrupt, https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
 *            Search for each of these libraries in the Arduino Library Manager to install them.
 */
//--------------Timer Library setup--------------------//
// The link to the repository is as follows: https://github.com/khoih-prog/TimerInterrupt?tab=readme-ov-file#important-notes-about-isr
#define USE_TIMER_1 true
#define USE_TIMER_2 false
#warning Using Timer1, Timer2
#include "TimerInterrupt.h"
// This code will include an embedded state machine with a state for parent and child statemachines.
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
#define LED_PIN_RED 19   // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18 // green LED, use to indicate transmitting.
#define MOTOR_ERROR 6    // pin for reading error state from the half bridge motor driver. If it goes low, something is wrong.
#define MOTOR_SLEEP 4    // pin for controlling the sleep state of the half bridge motor driver. LOW is sleep, HIGH is awake.
#define FAST_BLINK 100   // miliseconds
#define SLOW_BLINK 1000  // miliseconds
RF24 radio(7, 8);        // CE, CSN
//-----------Motor Control Setup----------------//
#include "DCMotorControl.h"
DCMotorControl Motors[] = {
    // There is only one motor that each will be controlling. There is no need for multiple motors.
    // DCMotorControl(9, 10, 5, 2, 3), //DCMotorControl::DCMotorControl( uint8_t DirectionPinA, uint8_t DirectionPinB, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor first!!
    DCMotorControl(10, 5, 3, 2) // DCMotorControl::DCMotorControl( uint8_t DirectionPin, uint8_t DrivePin, uint8_t Encoder1Pin, uint8_t Encoder2Pin) //uses this constructor second to illustrate the point.
};
#define NumberOfMotors 1
#define ControlRate_ms 10
#define ControlRate_us 10000
#define TIMER_INTERVAL_MS 10L // 10ms, or 10,000us as specfified by the ControlRate_us variable in the DCMotorControl.h file.
#define DeadbandTicks 100
#define DeadbandDutyCycle 5
#define TicksPerRevolution (6678.624)
#define TicksPerInch (24.5)*(6678.624/43.9822) // = 3270.283 for 60 rpm motor.
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
// -------------------- VARIABLES ------------------- //
enum overall_state
{
  PARENT,
  CHILD
} overall_state;
enum parent_state
{
  RECEIVING,
  TRANSMITTING_1,
  TRANSMITTING_2,
  OFF_P,
  COMPLETED_1
} parent_state;
enum child_state
{
  RECEIVING_1,
  RECEIVING_2,
  TRANSMITTING,
  OFF,
  TRANS_TO_PARENT
} child_state;

// data to compare the received data back against to see if it matches.
// const int transmit_data[4][2] = {{-7, -7}, {14, 0}, {-14, 0}, {7, 7}};
int global_received_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00002, receives from 00001, sends array to 00003 and 00004.
const byte addresses[][6] = {"00001", "00002", "00003", "00004", "00005"};
const byte *self = addresses[1];
const byte *parent = addresses[0];
const byte *child1 = addresses[2];
const byte *child2 = addresses[3];
const byte child_array[][6] = {*child1, *child2};
const unsigned short num_children = 2; // the number of children rollers that this roller will send data to.
// -------------------- FUNCTIONS ------------------- //

// checks for whether the delay_time has passed and sets the LED on or off.
void blink_led_unblocking(int delay_time)
{
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if (millis() - past_time > delay_time)
  {
    // Serial.println("blinking");
    // the time to blink has come.
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
void Parent_TX_1_function_unblocking()
{
  digitalWrite(LED_PIN_GREEN, HIGH);
  // static int transmit_count = 0;
  // static int child_count = num_children - 2; // currently we're starting with writing to the last child in the array.
  static bool successful;
  // static unsigned long past_time2 = millis();
  // blink_led_unblocking(FAST_BLINK);
  successful = radio.write(&global_received_data, sizeof(global_received_data));
  if (successful)
  {
    // if (child_count >= 0)
    // {
    //   radio.openWritingPipe(child_array[child_count]);
    //   child_count--;
    // }
    // else
    // {
      parent_state = COMPLETED_1; // finishes transmission
      overall_state = CHILD;
      child_state = RECEIVING_1;
      radio.startListening();
    // }
  }
  // Serial.println("TRANSMITTING DATA");
  // Serial.println(successful);
  // // transmit_count++;
  digitalWrite(LED_PIN_GREEN, LOW);
}
void Parent_TX_2_COMPLETED()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count_2 = 0;
  static int child_count2 = num_children - 2; // currently we're starting with writing to the last child in the array.
  // make the array all 1's
  const int complete_data_array[4][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};
  radio.write(&complete_data_array, sizeof(complete_data_array));
  // Serial.println("TRANSMITTING COMPLETED DATA");
  transmit_count_2++;
  digitalWrite(LED_PIN_RED, HIGH);
  digitalWrite(LED_PIN_GREEN, HIGH);

  // need to check for if there are more children to send data to.
  if (num_children > 1)
  {
    radio.stopListening();
    radio.setPALevel(RF24_PA_MIN);
    radio.openWritingPipe(child2);
    parent_state = TRANSMITTING_1;
    child_count2--;
  }
  else
  {
    parent_state = COMPLETED_1; // finishes transmission
  }
}
void Parent_RX_func()
{
  // Serial.println("RECEIVING");
  // the radio should already be in listening mode.
  static unsigned long past_time_r = millis();
  blink_led_unblocking(SLOW_BLINK);
  // two paths out of receiving:
  //  1. We receive the data back.
  //  2. 1000 ms have passed so we go back to transmitting.
  if (millis() - past_time_r > 1000)
  {
    parent_state = TRANSMITTING_1;
    past_time_r = millis();
  }
  else if (radio.available())
  {
    int received_data[8];
    radio.read(&received_data, sizeof(received_data));
    bool matches_data = true;
    // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
    for (int y = 0; y < 8; y++)
    {
      Serial.print(received_data[y]);
      Serial.print(" ");
      if (received_data[y] != global_received_data[y])
      {
        matches_data = false;
      }

      Serial.println();
    }
    if (matches_data)
    {
      // Serial.println("MATCHES");
      parent_state = TRANSMITTING_2;
    }
    else
    {
      // Serial.println("DOES NOT MATCH");
      parent_state = TRANSMITTING_1; // something seems off here.
    }
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
      child_state = TRANS_TO_PARENT;
    }
  }
}
void Child_TX_function()
{
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count_3 = 0;
  radio.write(&global_received_data, sizeof(global_received_data));
  child_state = RECEIVING_2;
  // Serial.println("TRANSMITTING DATA");
  transmit_count_3++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}
// checks for array of all ones.
void Child_RX_2()
{
  // Serial.println("RECEIVING_2");
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
        // Serial.print(received_data2[x][y]);
        // Serial.print(" ");
        if (received_data2[x][y] != 1)
        {
          data_correct = false;
        }
      }
      // Serial.println();
    }
    if (data_correct)
    {
      // Serial.println("MATCHES");
      child_state = TRANS_TO_PARENT;
      digitalWrite(LED_PIN_RED, HIGH);
      digitalWrite(LED_PIN_GREEN, HIGH);
    }
    else
    {
      // Serial.println("DOES NOT MATCH");
      child_state = TRANSMITTING; // something seems off here.
    }
  }
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
void FLIP_Direction(void)
{
  Serial.println("FLIP_Direction");
  for (uint8_t i = 0; i < (NumberOfMotors); i++)
  {
    Motors[i].setDesiredPositionInches(-Motors[i].getDesiredPositionInches());
  }
}
void print_motor_position(void)
{
  static unsigned int print_count = 0;
  if (print_count > 2500)
  {
    // Serial.println(Motors[0].getCurrentPositionInches());
    // Serial.print("Duty_Cycle: ");
    // Serial.println(Motors[0].getDutyCycle()); //I want to investigate the PWM output and graph it.
    // Serial.print("Current Position: ");
    // Serial.println(Motors[0].getCurrentPositionInches());
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
      // Serial.println("Motor has reached desired position.");
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
  // Serial.println("Beginning motor control.");
  float new_motor_position = global_received_data[0]; // make sure to get the data that corresponds to this roller.
  // Serial.print("New Motor Position: ");
  // Serial.println(new_motor_position);
  Motors[0].setDesiredPositionInches(new_motor_position);
  motor_running = true;
  ITimer1.resumeTimer();
}

void setup()
{
  Serial.begin(9600);
  // Serial.println("STARTING");
  radio.begin();
  radio.openReadingPipe(1, self); // 00002 the address of node 2, or the middle node. (THIS MODULE)
  radio.setPALevel(RF24_PA_MAX);  // This sets the power level at which the module will transmit.
                                  // The level is super low now because the two modules are very close to each other.
  overall_state = CHILD;          // start here!!
  child_state = RECEIVING_1;      // start here!!
  radio.startListening();   // comment just to change something
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
    // Serial.print(F("Starting  ITimer1 OK, millis() = "));
    // Serial.println(millis());
    ITimer1.pauseTimer();
  }
  else
    // Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
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
  Serial.print("Control gains: ");
  Serial.print("Kp: ");
  Serial.print(Motors[0].getKp(), 4);
  Serial.print(" Ki: ");
  Serial.print(Motors[0].getKi(), 4);
  Serial.print(" Kd: ");
  Serial.println(Motors[0].getKd(), 4);
  motor_startup(); // I want it to run the motor as soon as it is set up, then listen for further instructions.
  // Serial.println("Setup Complete");
}

void loop()
{
  // if (motor_running)
  // {
  //   position_reached_checker();
  print_motor_position();
  // }
  // position_reached_checker(); //I need to make it so that this can be flagged to only be run when the motor is running.
  switch (overall_state)
  {
  case PARENT:
    switch (parent_state)
    {
    case RECEIVING: // this one will be run muliple times.
      Parent_RX_func();
      break;

    case TRANSMITTING_1:
      Parent_TX_1_function_unblocking();
      break;

    case TRANSMITTING_2:
      // Serial.println("TRANSMITTING_2");
      Parent_TX_2_COMPLETED();
      break;

    case OFF_P:
      // Serial.println("OFF");
      break;

    case COMPLETED_1:
      // Serial.println("COMPLETED");
      break;
    }
    break;

  case CHILD:
    switch (child_state)
    {
    case RECEIVING_1: // this one will be run muliple times.
      // Serial.println("RECEIVING_1");
      child_RX_1();
      break;
    case RECEIVING_2: // this is the one waiting for if the sent data was correct.
      // if the data was correctly received, the tx will send back an array of all ones.
      // Serial.println("RECEIVING_2");
      Child_RX_2();

      break;

    case TRANSMITTING:
      Child_TX_function();

      break;

    case OFF:
      // Serial.println("CHILD STATE_OFF");
      break;

    case TRANS_TO_PARENT:
      // Blink_led_unblocking(5000);

      // Serial.println("TRANSITIONING TO PARENT MODE");
      motor_startup();
      // overall_state = PARENT;
      // parent_state = TRANSMITTING_1;
      child_state = RECEIVING_1;
      digitalWrite(LED_PIN_RED, LOW);
      // child_state = OFF;
      // radio.setPALevel(RF24_PA_MIN);
      // radio.openWritingPipe(child1); // we start with the last child in the array.
      // radio.stopListening();
      break;

      break;
    }
  }
}
