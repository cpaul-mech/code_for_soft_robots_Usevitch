#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <time.h>
#define LED_PIN_RED 19          // red LED, use to indicate receiving.
#define LED_PIN_GREEN 18        // green LED, use to indicate transmitting.
#define FAST_BLINK 100          // miliseconds
#define SLOW_BLINK 1000         // miliseconds
#define MAX_SERIAL_DATA_NUM 14  // Roller 0 sends data for us to 14 active rollers
RF24 radio(7, 8);               // CE, CSN

// -------------------- VARIABLES ------------------- //
enum Parent_state {
    SERIAL_RECEIVE,
    TRANSMITTING,
    RADIO_RECIEVE
} parent_state;
// data to compare the received data back against to see if it matches.
int16_t tx_data[16] = {};
uint8_t tx_data_index = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this node is 00001, the master node or start node, has long range atennae. This one will start off the communication.
const byte addresses[][6] = {"00000", "00001", "00002"};
const byte *self = addresses[0];
const byte *child1 = addresses[1];
const byte *child2 = addresses[2];

void init_radio();
void init_LEDs();
void serial_receive();
void radio_transmit();

void setup() {
    Serial.begin(9600);

    init_radio();

    // Set parent state to wait for data from serial
    parent_state = SERIAL_RECEIVE;

    // Set tx_data to 0s
    memset(tx_data, 0, sizeof(tx_data));

    // Set LED pins high then low to show power on
    init_LEDs();
}

void loop() {
    switch (parent_state) {
        case SERIAL_RECEIVE:  // this case will be run until all serial data is received.
            // read in the number from the serial port and set the first element in transmit data to that number.
            serial_receive();
            break;
        case TRANSMITTING:

            radio_transmit();
            break;
    }
}

void init_radio() {
    radio.begin();

    // This sets the radio frequency to 2476 MHz. This is the default value
    radio.setChannel(76);

    // Open reading pipes to both children
    radio.openReadingPipe(1, child1);
    radio.openReadingPipe(2, child2);

    // Set power level to max for long distance communication
    radio.setPALevel(RF24_PA_MAX);

    // Open first writing pipe by default (to child1)
    radio.openWritingPipe(child1);

    // Set the timeout and number of tries for the child to sent back an auto ack
    radio.setRetries(15, 15);

    // Roller 0 should be prepared to transmit by default
    radio.stopListening();
}

void init_LEDs() {
    pinMode(LED_PIN_RED, OUTPUT);
    pinMode(LED_PIN_GREEN, OUTPUT);
    digitalWrite(LED_PIN_RED, HIGH);    // LED is ON.
    digitalWrite(LED_PIN_GREEN, HIGH);  // LED is ON
    delay(1000);
    digitalWrite(LED_PIN_RED, LOW);    // LED is OFF.
    digitalWrite(LED_PIN_GREEN, LOW);  // LED is OFF
}

void serial_receive() {
    if (Serial.available()) {
        // reset tx_data array
        memset(tx_data, 0, sizeof(tx_data));
        tx_data_index = 0;

        // Turn on LEDs
        digitalWrite(LED_PIN_RED, HIGH);
        digitalWrite(LED_PIN_GREEN, HIGH);

        // Get data from Serial
        String incoming_data_str = Serial.readStringUntil('\n');  // Read until newline character (the resultant string does not include \n)
        incoming_data_str.trim();                                 // Remove any leading or trailing whitespace

        int start_index = 0;
        int end_index = incoming_data_str.indexOf(',');

        // Iterate through each substring in between ',' characters until end of string
        while (end_index > 0) {
            String number_substring = incoming_data_str.substring(start_index, end_index);  // grabs the substring at start_index inclusive and ends at end_index exclusive
            int16_t number = number_substring.toInt();

            tx_data[tx_data_index++] = number;

            start_index = end_index + 1;
            end_index = incoming_data_str.indexOf(',', start_index);

            // If more data was sent in string than allowed for, break from the loop
            if (tx_data_index == MAX_SERIAL_DATA_NUM) {
                break;
            }
        }

        //If maximum data has not been reached, then collect the last number from incoming_data_str
        if (tx_data_index < MAX_SERIAL_DATA_NUM && start_index < incoming_data_str.length()) {
            String number_substring = incoming_data_str.substring(start_index);  // grabs the substring at start_index inclusive till end of string
            int16_t number = number_substring.toInt();

            tx_data[tx_data_index++] = number;
        }

        //If data was correctly received, then transition to Transmitting
        if (tx_data_index > 0) {
            parent_state = TRANSMITTING;
            Serial.println();
        }

        // Turn LEDs off
        digitalWrite(LED_PIN_RED, LOW);
        digitalWrite(LED_PIN_GREEN, LOW);
    }
}

void radio_transmit() {
    int16_t checksum = 0;

    for (int i = 0; i < tx_data_index; i++) {
        checksum += tx_data[i];
        Serial.print(i + ": " + tx_data[i]);

        if (i < tx_data_index - 1) {
            Serial.print(", ");
        }
    }

    Serial.println();
    Serial.println("Checksum: " + checksum);
    Serial.println();

    parent_state = SERIAL_RECEIVE;
}
