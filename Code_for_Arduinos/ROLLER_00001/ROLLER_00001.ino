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
int16_t rx_data[MAX_SERIAL_DATA_NUM + 1] = {};
//uint8_t tx_data_index = 0;
/*Next we need to create a byte array which will
represent the address, or the so called pipe through which the two modules will communicate.
We can change the value of this address to any 5 letter string and this enables to choose to
which receiver we will talk, so in our case we will have the same address at both the receiver
and the transmitter.*/
// this roller is 00000, the master node or start node, has long range atennae. This one will start off the communication.
// For addresses, the name convention is that the last digit is the number of the node in hex and the previous digit is the number of the parent in hex
Roller self = {RECEIVING, false};
Parent parent = {"00001", 0};
//Child child1 = {"00001", false, 1};
Child children[] = {};
const uint8_t NUM_CHILDREN = 0;

void init_radio();
void init_LEDs();
//void calculate_checksum();
void radio_receive();
//void radio_transmit();
void blink_red_led(int delay_time);

void setup() {
    Serial.begin(115200);

    init_radio();

    // Set parent state to wait for data from serial
    self.state = RECEIVING;

    // Set tx_data to 0s
    memset(tx_data, 0, sizeof(tx_data));
    memset(rx_data, 0, sizeof(rx_data));

    // Set LED pins high then low to show power on
    init_LEDs();
}

void loop() {
    switch (self.state) {
        case RECEIVING:  // this case will be run until all serial data is received, up to 14 numbers
            //blink_red_led(SLOW_BLINK);
            radio_receive();
            break;
            /*
        case CALC_CHECKSUM:
            calculate_checksum();
        case TRANSMITTING:
            radio_transmit();
            break;
            */
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

    //Initialize parent
    if (parent.address != nullptr) {
        radio.openReadingPipe(parent.reading_pipe_num, parent.address);
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

void radio_receive() {
    if (radio.available()) {
        digitalWrite(LED_PIN_GREEN, HIGH);
        radio.read(rx_data, sizeof(rx_data));
        for (int i = 0; i < MAX_SERIAL_DATA_NUM + 1; i++) {
            /*
            if (rx_data[i] == 0) {
                break;
            }
            */
            Serial.print(rx_data[i]);
            Serial.print(", ");
        }

        Serial.println();
        digitalWrite(LED_PIN_GREEN, LOW);
    }
    if (self.return_to_transmitting == true) {
        self.return_to_transmitting = false;
        self.state = TRANSMITTING;
    }
}

/*
void calculate_checksum() {
    int16_t checksum = 0;

    for (int i = 0; i < tx_data_index; i++) {
        checksum += tx_data[i];
        Serial.print(i);
        Serial.print(": ");
        Serial.print(tx_data[i]);

        if (i < tx_data_index - 1) {
            Serial.print(", ");
        } else {
            Serial.println();
        }
    }

    tx_data[tx_data_index++] = checksum;

    for (int i = 0; i < tx_data_index; i++) {
        Serial.print(tx_data[i]);
        Serial.print(", ");
    }

    Serial.println();

    self.state = TRANSMITTING;
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

        if (!transmission_complete) {
            self.return_to_transmitting = true;
        }
        radio.startListening();

        digitalWrite(LED_PIN_GREEN, LOW);

    } else { //if radio is available, then switch to receiving, process that data, then transmit
        self.return_to_transmitting = true;
    }
    self.state = RECEIVING;
}
*/
void blink_red_led(int delay_time) {
    static unsigned long past_time = millis();
    static uint8_t led_state = LOW;
    if (millis() - past_time > delay_time) {
        led_state = ~led_state;
        digitalWrite(LED_PIN_RED, led_state);
        past_time = millis();
    }
}
