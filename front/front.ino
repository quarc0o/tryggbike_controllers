#include "RF24.h"
#include "SPI.h"
#include "nRF24L01.h"

#define LEFT_INDICATOR 2
#define RIGHT_INDICATOR 3
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 5

#define LEFT_INDICATOR_ON 0b10
#define RIGHT_INDICATOR_ON 0b01
#define BOTH_INDICATORS_OFF 0b00

int SentMessage[1] = {0b00};

RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO

const uint64_t pipe =
    0xE6E6E6E6E6E5; // Needs to be the same for communicating between 2 NRF24L01

void setup(void) {
  Serial.begin(9600);

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_INDICATOR, OUTPUT);
  pinMode(RIGHT_INDICATOR, OUTPUT);

  radio.begin();               // Start the NRF24L01
  radio.openWritingPipe(pipe); // Get NRF24L01 ready to transmit
}

void loop(void) {
  static unsigned int counter{0};
  static bool left_turn_indicator_activated{false};
  static bool right_turn_indicator_activated{false};
  static bool light_high{false};

  // Delay per iteration
  static constexpr unsigned int delay_time{10};   // [ms]
  static constexpr unsigned int cycle_length{40}; // iterations
  counter = (counter + 1) % cycle_length;
  delay(delay_time);

  if (digitalRead(LEFT_BUTTON) == HIGH) {
    if (!left_turn_indicator_activated) {
      counter = 0;
    }
    left_turn_indicator_activated = true;
    right_turn_indicator_activated = false;
  } else if (digitalRead(RIGHT_BUTTON) == HIGH) {
    if (!right_turn_indicator_activated) {
      counter = 0;
    }
    left_turn_indicator_activated = false;
    right_turn_indicator_activated = true;
  } else {
    left_turn_indicator_activated = false;
    right_turn_indicator_activated = false;
  }

  light_high = counter < (cycle_length / 2);

  if (light_high) {
    if (left_turn_indicator_activated) {
      digitalWrite(LEFT_INDICATOR, HIGH);
      digitalWrite(RIGHT_INDICATOR, LOW);
      SentMessage[0] = LEFT_INDICATOR_ON;

    } else if (right_turn_indicator_activated) {
      digitalWrite(RIGHT_INDICATOR, HIGH);
      digitalWrite(LEFT_INDICATOR, LOW);
      SentMessage[0] = RIGHT_INDICATOR_ON;
    }
  } else {
    digitalWrite(LEFT_INDICATOR, LOW);
    digitalWrite(RIGHT_INDICATOR, LOW);
    SentMessage[0] = BOTH_INDICATORS_OFF;
  }

  radio.write(SentMessage, 1);
}
