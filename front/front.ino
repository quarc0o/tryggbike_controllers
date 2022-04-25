#include <stdint.h>

// Libraries from the Arduino IDE 1.8.15's "Library manager"
// "RF24" version 1.4.2

// Radio
#include "RF24.h"
#include "SPI.h"
#include "nRF24L01.h"

#define LEFT_INDICATOR_PIN 3
#define RIGHT_INDICATOR_PIN 2
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5
#define FRONT_OUTER_LIGHTS 7
#define FRONT_CENTER_LIGHTS 8

enum Indicator {
  INDICATOR_LEFT = 0,
  INDICATOR_RIGHT = 1,
  INDICATOR_MSG_LENGTH
};

uint8_t SentMessage[INDICATOR_MSG_LENGTH] = {LOW, LOW};

RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO

const uint64_t pipe =
    0xE6E6A43BE6E5; // Needs to be the same for communicating between 2 NRF24L01

void setup(void) {
  // Serial.begin(9600);

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_INDICATOR_PIN, OUTPUT);
  pinMode(RIGHT_INDICATOR_PIN, OUTPUT);
  
  pinMode(FRONT_OUTER_LIGHTS, OUTPUT);
  pinMode(FRONT_CENTER_LIGHTS, OUTPUT);
  digitalWrite(FRONT_OUTER_LIGHTS, LOW);
  digitalWrite(FRONT_CENTER_LIGHTS, LOW);

  if (!radio.begin()) { // Start the NRF24L01
    // Serial.println("Radio hardware not responding!");
  }
  radio.openWritingPipe(pipe); // Get NRF24L01 ready to transmit
}

void loop(void) {
  static unsigned int counter{0};
  static bool left_turn_indicator_activated{false};
  static bool right_turn_indicator_activated{false};
  static bool light_high{false};

  // Delay per iteration
  static constexpr unsigned int delay_time{10};   // [ms]
  static constexpr unsigned int cycle_length{60}; // iterations
  counter = (counter + 1) % cycle_length;
  delay(delay_time);

  if (digitalRead(LEFT_BUTTON) == LOW) {
    if (!left_turn_indicator_activated) {
      counter = 0;
    }
    left_turn_indicator_activated = true;
    right_turn_indicator_activated = false;
  } else if (digitalRead(RIGHT_BUTTON) == LOW) {
    if (!right_turn_indicator_activated) {
      counter = 0;
    }
    left_turn_indicator_activated = false;
    right_turn_indicator_activated = true;
  } else {
    left_turn_indicator_activated = false;
    right_turn_indicator_activated = false;
  }

  light_high = counter < static_cast<unsigned int>(cycle_length / 2.0);

  if (light_high) {
    SentMessage[INDICATOR_LEFT] = left_turn_indicator_activated ? HIGH : LOW;
    SentMessage[INDICATOR_RIGHT] = right_turn_indicator_activated ? HIGH : LOW;

  } else {
    SentMessage[INDICATOR_LEFT] = LOW;
    SentMessage[INDICATOR_RIGHT] = LOW;
  }
  digitalWrite(LEFT_INDICATOR_PIN, SentMessage[INDICATOR_LEFT]);
  digitalWrite(RIGHT_INDICATOR_PIN, SentMessage[INDICATOR_RIGHT]);
  radio.write(SentMessage, INDICATOR_MSG_LENGTH);
  //Serial.println("Sent message with payload " + String(SentMessage[0]) + ", " +
  //               String(SentMessage[1]));
}
