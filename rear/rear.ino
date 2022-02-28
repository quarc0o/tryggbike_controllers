#include "RF24.h"
#include "SPI.h"
#include "nRF24L01.h"

#define LEFT_INDICATOR 2
#define RIGHT_INDICATOR 3

int ReceivedMessage[1] = {0}; // Used to store value received by the NRF24L01

RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO

 // Identifier 
const uint64_t pipe = 0xE6E6E6E6E6E5;

void setup(void) {
  Serial.begin(9600);
  pinMode(LEFT_INDICATOR, OUTPUT);
  pinMode(RIGHT_INDICATOR, OUTPUT);

  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
}

void loop(void) {

  while (radio.available()) {
    radio.read(ReceivedMessage, 1);
    if (ReceivedMessage[0] == 0b10) // Turn on left indicator
    {
      digitalWrite(LEFT_INDICATOR, HIGH);
      digitalWrite(RIGHT_INDICATOR, LOW);
    } else if (ReceivedMessage[0] == 0b01) // Turn on right indicator
    {
      digitalWrite(RIGHT_INDICATOR, HIGH);
      digitalWrite(LEFT_INDICATOR, LOW);
    } else {
      digitalWrite(LEFT_INDICATOR, LOW);
      digitalWrite(RIGHT_INDICATOR, LOW);
    }
    Serial.println(ReceivedMessage[0]);
    delay(10);
  }
}
