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
enum State {
    TRANSMITTING,
    RECEIVING,
    CALC_CHECKSUM
};

typedef struct Roller {
    State state;
    bool return_to_transmitting;
} Roller;

typedef struct Parent {
    const uint8_t *address;
    uint8_t reading_pipe_num;
} Parent;

typedef struct Child {
    const uint8_t *address;
    bool transmission_received;
    uint8_t reading_pipe_num;
} Child;
// data to compare the received data back against to see if it matches.
int16_t tx_data[MAX_SERIAL_DATA_NUM + 1] = {};
uint8_t tx_data_index = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this roller is 00000, the master node or start node, has long range atennae. This one will start off the communication.
// For addresses, the name convention is that the last digit is the number of the node in hex and the previous digit is the number of the parent in hex
Roller self = {RECEIVING, false};
Child child1 = {"00001", false, 1};
Child children[] = {child1};
const uint8_t NUM_CHILDREN = 1;

void init_radio();
void init_LEDs();
int16_t calculate_checksum(int16_t *data, uint8_t len);
void serial_receive();
void radio_receive();
void radio_transmit();
void blink_red_led(int delay_time);

void setup() {
    Serial.begin(115200);

    init_radio();

    // Set parent state to wait for data from serial
    self.state = RECEIVING;

    // Set tx_data to 0s
    memset(tx_data, 0, sizeof(tx_data));

    // Set LED pins high then low to show power on
    init_LEDs();
}

void loop() {
    switch (self.state) {
        case RECEIVING:  // this case will be run until all serial data is received, up to 14 numbers
            blink_red_led(SLOW_BLINK);
            serial_receive();
            radio_receive();
            break;
        case CALC_CHECKSUM:
            tx_data[MAX_SERIAL_DATA_NUM] = calculate_checksum(tx_data, MAX_SERIAL_DATA_NUM);

            for (int i = 0; i < MAX_SERIAL_DATA_NUM + 1; i++) {
                Serial.print(tx_data[i]);
                Serial.print(", ");
            }

            Serial.println();
            self.state = TRANSMITTING;
        case TRANSMITTING:
            radio_transmit();
            break;
    }
}

void init_radio() {
    radio.begin();

    // This sets the radio frequency to 2476 MHz. This is the default value
    radio.setChannel(76);

    // Sets data transfer rate to 1 megabits per sec
    radio.setDataRate(RF24_1MBPS);

    // Open reading pipes to all children
    for (int i = 0; i < NUM_CHILDREN; i++) {
        radio.openReadingPipe(children[i].reading_pipe_num, children[i].address);
    }

    // Set power level to max for long distance communication
    radio.setPALevel(RF24_PA_MAX);

    // Open first writing pipe by default (to child1)
    if (NUM_CHILDREN > 0) {
        radio.openWritingPipe(children[0].address);
    }

    // Set the timeout and number of tries for the child to sent back an auto ack
    radio.setRetries(15, 15);

    // All rollers are listening by default. Once the TRANSMITTING state is reached, it is set to stop listening
    radio.startListening();
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

        // If maximum data has not been reached, then collect the last number from incoming_data_str
        if (tx_data_index < MAX_SERIAL_DATA_NUM && start_index < incoming_data_str.length()) {
            String number_substring = incoming_data_str.substring(start_index);  // grabs the substring at start_index inclusive till end of string
            int16_t number = number_substring.toInt();

            tx_data[tx_data_index++] = number;
        }

        // If data was correctly received, then transition to CALC_CHECKSUM
        if (tx_data_index > 0) {
            self.state = CALC_CHECKSUM;
            self.return_to_transmitting = false;  // This prevents the state machine from trying to submit old data
        }

        // Turn LEDs off
        digitalWrite(LED_PIN_RED, LOW);
        digitalWrite(LED_PIN_GREEN, LOW);
    }
}

void radio_receive() {
    if (radio.available()) {
        // Code
    }
    if (self.return_to_transmitting == true) {
        self.return_to_transmitting = false;
        self.state = TRANSMITTING;
    }
}

int16_t calculate_checksum(int16_t *data, uint8_t len) {
    int16_t checksum = 0;

    for (int i = 0; i < len; i++) {
        checksum += (i % 3 + 1) * data[i];
        Serial.print(i);
        Serial.print(": ");
        Serial.print(data[i]);

        Serial.print(",");
    }

    Serial.println();

    return checksum;
}

void radio_transmit() {
    if (!radio.available()) {
        digitalWrite(LED_PIN_GREEN, HIGH);
        radio.stopListening();
        uint8_t attempt = 0;
        bool transmission_complete = false;

        while (!transmission_complete && attempt < 3) {
            transmission_complete = true;
            for (int i = 0; i < NUM_CHILDREN; i++) {
                if (children[i].transmission_received == false) {
                    radio.openWritingPipe(children[i].address);
                    if (radio.write(tx_data, sizeof(tx_data)) == false) {
                        transmission_complete = false;
                    } else {
                        children[i].transmission_received = true;
                    }
                }
            }
            attempt++;
        }

        radio.startListening();

        if (!transmission_complete) {
            self.return_to_transmitting = true;
        } else {
            reset_children_flags();
        }

        digitalWrite(LED_PIN_GREEN, LOW);

    } else {  // if radio is available, then switch to receiving, process that data, then transmit
        self.return_to_transmitting = true;
    }
    self.state = RECEIVING;
}

void blink_red_led(int delay_time) {
    static unsigned long past_time = millis();
    static uint8_t led_state = LOW;
    if (millis() - past_time > delay_time) {
        led_state = ~led_state;
        digitalWrite(LED_PIN_RED, led_state);
        past_time = millis();
    }
}

void reset_children_flags() {
    for (int j = 0; j < NUM_CHILDREN; j++) {
        children[j].transmission_received = false;
    }
}
