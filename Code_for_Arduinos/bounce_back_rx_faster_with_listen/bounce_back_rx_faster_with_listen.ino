
/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define LED_PIN_WHITE 2 //White LED, use to indicate RECEIVING_1.
#define LED_PIN_GREEN 3 //Green LED, use to indicate transmitting.
#define FAST_BLINK 100 //miliseconds
#define SLOW_BLINK 1000 //miliseconds
RF24 radio(7, 8); // CE, CSN
enum radio_state
  {
    RECEIVING_1,
    RECEIVING_2,
    TRANSMITTING,
    OFF,
    COMPLETED
  }transmitter_state;
int received_data[4][2] = {{0, 0},{0, 0},{0, 0},{0, 0}};
int transmit_count = 0;
/*Next we need to create a byte array which will 
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to 
which receiver we will talk, so in our case we will have the same address at both the receiver 
and the transmitter.*/
const byte addresses[][6] = {"00001", "00002"}; 
// -------------------- FUNCTIONS ------------------- //
void blink_led_unblocking(int delay_time){
  static unsigned long past_time = millis();
  static bool led_ON = false;
  if(millis() - past_time > delay_time ){
    if(led_ON){
      digitalWrite(LED_PIN_WHITE, LOW);
      led_ON = false;
    }
    else{
      digitalWrite(LED_PIN_WHITE, HIGH);
      led_ON = true;
    }
    past_time = millis();
  }
}

void transmitter_function_unblocking(){
  radio.stopListening();
  digitalWrite(LED_PIN_GREEN, HIGH);
  static int transmit_count = 0;
  //static unsigned long past_time2 = millis();
  //blink_led_unblocking(FAST_BLINK);
  //const int transmit_data[4][2] = {{-7, -7},{14, 0},{-14, 0},{7, 7}};
  radio.write(&received_data, sizeof(received_data));
  transmitter_state = RECEIVING_2;
  Serial.println("TRANSMITTING DATA");
  transmit_count++;
  radio.startListening();
  digitalWrite(LED_PIN_GREEN, LOW);
}

void receiving_function_checking(){
  Serial.println("RECEIVING_2");
  //the radio should already be in listening mode.
  static unsigned long past_time_r = millis();
  blink_led_unblocking(SLOW_BLINK);
  //two paths out of receiving:
  // 1. We receive the data back.
  // 2. 2000 ms have passed so we go back to transmitting.
  if(millis() - past_time_r > 2000){
    transmitter_state = TRANSMITTING;
    past_time_r = millis();
  }
  else if(radio.available()){ 
    int received_data2[4][2];
    radio.read(&received_data2, sizeof(received_data2));
    bool data_correct = true;
    // NEED TO WRITE A PRINTING FUNCTION that can also compare the arrays.
    //Serial.println(recieved_data);
    for(int x = 0; x < 4; x++){
      for(int y = 0; y < 2; y++){
        Serial.print(received_data2[x][y]);
        Serial.print(" ");
        if(received_data2[x][y] != 1){
          data_correct = false;
        }
      }
      Serial.println();
    }
    if(data_correct){
      Serial.println("MATCHES");
      transmitter_state = COMPLETED;
      digitalWrite(LED_PIN_WHITE, HIGH);
      digitalWrite(LED_PIN_GREEN, HIGH);
    }
    else{
      Serial.println("DOES NOT MATCH");
      transmitter_state = TRANSMITTING; //something seems off here. 
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING");
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001 the address of the receiver. (THIS MODULE)
  radio.openReadingPipe(0, addresses[1]); // 00001 the address of the transmitter 
  radio.setPALevel(RF24_PA_MIN); //This sets the power level at which the module will transmit. 
                                //The level is super low now because the two modules are very close to each other.
  transmitter_state = RECEIVING_1;
  radio.startListening();
  pinMode(LED_PIN_WHITE, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_WHITE, LOW);
  digitalWrite(LED_PIN_GREEN, LOW);
}

void loop() {
    switch (transmitter_state)
    {
      case RECEIVING_1: //this one will be run muliple timees.
      Serial.println("RECEIVING_1");
      blink_led_unblocking(SLOW_BLINK);
      if (radio.available()){
        radio.read(&received_data, sizeof(received_data));
        //iterate through values and print data
        for(int x = 0; x < 4; x++){
        for(int y = 0; y < 2; y++){
          Serial.print(received_data[x][y]);
          Serial.print(" ");
        }
        Serial.println();
      }
        transmitter_state = TRANSMITTING;
        //radio.stopListening();
      }
      break;
      case RECEIVING_2: //this is the one waiting for if the sent data was correct. 
      //if the data was correctly received, the tx will send back an array of all ones.
      //Serial.println("RECEIVING_2");
      receiving_function_checking();

      break;

      case TRANSMITTING: //stay here for a few times at least before i implement the second bounce back.
      transmitter_function_unblocking();
      
      break;

      case OFF:
      Serial.println("OFF");
      //blink_led(500);
      //transmitter_state = TRANSMITTING;
      ///radio.stopListening();
      
      break;

      case COMPLETED:
      //link_led_unblocking(5000);
      Serial.println("COMPLETED");
      break;

      default:
      Serial.println("ERROR: transmitter_state is in an unknown state");
      break;
    }
}
