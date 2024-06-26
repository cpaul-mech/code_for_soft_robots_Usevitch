/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* Adapted and modified by: Chris Paul for ME 497r at BYU.
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>
//#include <Time.h>
#define LED_PIN 2
#define FAST_BLINK 100 //miliseconds
#define SLOW_BLINK 1000 //miliseconds
RF24 radio(7, 8); // CE, CSN
// -------------------- VARIABLES ------------------- //
enum radio_state
  {
    RECEIVING,
    TRANSMITTING,
    OFF,
    COMPLETED
  }transmitter_state;
const int compare_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};

int transmit_count = 0;
/*Next we need to create a byte array which will 
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to 
which receiver we will talk, so in our case we will have the same address at both the receiver 
and the transmitter.*/
const byte addresses[][6] = {"00001", "00002"}; 
// -------------------- FUNCTIONS ------------------- //
// checks for whether the delay_time has passed and sets the LED on or off.
void blink_led_unblocking(int delay_time){
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if(millis() - past_time > delay_time ){
    //Serial.println("blinking");
    //the time to blink has come.
    if(led_ON){
      digitalWrite(LED_PIN_RED, LOW);
      led_ON = false;
    }
    else{
      digitalWrite(LED_PIN_RED, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}
void transmitter_function_unblocking(){
  static int transmit_count = 0;
  static unsigned long past_time2 = millis();
  blink_led_unblocking(FAST_BLINK);
  if(transmit_count < 3 ){
    if(millis() - past_time2 > 1000){
      Serial.println("TRANSMITTING DATA");
      //const char transmit_text[] = "Hello World";
      /*-7    -7
        14     0
       -14     0
         7     7*/
      const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
      radio.write(&transmit_data, sizeof(transmit_data));
      transmit_count++;
      past_time2 = millis();
    }
  }
  else{
    transmitter_state = OFF;
    Serial.println("To Off");
    //Serial.println(transmitter_state);
    transmit_count = 0;
    digitalWrite(LED_PIN_RED, LOW);
    //led_ON = false;
    //past_time = millis();
  }
}
void setup() {
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002 the address of the receiver.
  radio.openReadingPipe(1, addresses[0]); // 00001 the address of the transmitter (THIS MODULE)
  radio.setPALevel(RF24_PA_MIN); //This sets the power level at which the module will transmit. 
                                //The level is super low now because the two modules are very close to each other.
  transmitter_state = TRANSMITTING;
  radio.stopListening();
  pinMode(LED_PIN_RED, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW); //LED is off.
  // const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
  // unsigned long size = sizeof(transmit_data);
  // Serial.println(size);
}

void loop() {
  switch (transmitter_state)
  {
    case RECEIVING: //this one will be run muliple times.
    Serial.println("RECEIVING");
    blink_led_unblocking(SLOW_BLINK);
    if (radio.available()){
      int received_data[4][2];
      radio.read(&received_data, sizeof(received_data));
      bool matches_data = true;
      // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
      //Serial.println(recieved_data);
      for(int x = 0; x < 4; x++){
        for(int y = 0; y < 2; y++){
          Serial.print(received_data[x][y]);
          Serial.print(" ");
          if(received_data[x][y] != compare_data[x][y]){
            matches_data = false;
          }
        }
        Serial.println();
      }
      if(matches_data){
        Serial.println("MATCHES");
        transmitter_state = COMPLETED;
      }
      else{
        Serial.println("DOES NOT MATCH");
        transmitter_state = TRANSMITTING;
      }
    }
    break;

    case TRANSMITTING:
    //Serial.println(transmit_count);
    transmitter_function_unblocking();
    break;

    case OFF:
    delay(1000);
    Serial.println("OFF");
    //blink_led(500);
    transmitter_state = RECEIVING;
    //past_time = millis();
    radio.startListening();
    break;

    case COMPLETED:
    blink_led_unblocking(5000);
    Serial.println("COMPLETED");
    break;

    // default:
    // Serial.println("ERROR: transmitter_state is in an unknown state");
    // break;
  }
  //print something here to catch infinite unhandled states.
  // Serial.println("Reaching Bottom of switch.");
  // Serial.print("Current Transmitter_state: ");
  // Serial.println(transmitter_state);
}
