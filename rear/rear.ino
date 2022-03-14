#include <stdint.h>

// Interrupt
#include <SAMDTimerInterrupt.h>
#include <SAMDTimerInterrupt.hpp>
#include <SAMD_ISR_Timer.h>
#include <SAMD_ISR_Timer.hpp>

// Radio
#include "RF24.h"
#include "SPI.h"
#include "nRF24L01.h"

// Accelerometer
#include <Arduino_LSM6DS3.h>

static constexpr float kBrakeLightActivateThreshold{0.3};   // [g]
static constexpr float kBrakeLightDeactivateThreshold{0.1}; // [g]

#define LEFT_INDICATOR_PIN 2
#define RIGHT_INDICATOR_PIN 3
#define BRAKE_LIGHT_PIN 7
#define REAR_LIGHT_PIN 8

enum Indicator {
  INDICATOR_LEFT = 0,
  INDICATOR_RIGHT = 1,
  INDICATOR_MSG_LENGTH
};

uint8_t ReceivedMessage[INDICATOR_MSG_LENGTH] = {LOW, LOW};
RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO
// Identifier
const uint64_t pipe = 0x6BAAD04ADD;

static long kHardwareTimerMs{1000L};
SAMDTimer timer{TIMER_TC3};
static long kTimerMs{static_cast<long>(1000.0 / kHardwareTimerMs)};
SAMD_ISR_Timer isr_timer;

void timerHandler(void) { isr_timer.run(); }

void radioInterrupt(void);
void imuInterrupt(void);

void setup(void) {
  // Serial.begin(9600);
  pinMode(LEFT_INDICATOR_PIN, OUTPUT);
  pinMode(RIGHT_INDICATOR_PIN, OUTPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  IMU.begin();
  bool timer_success{
      timer.attachInterruptInterval(kHardwareTimerMs * 1, timerHandler)};
  if (!timer_success) {
    Serial.println("Unable to start timer!");
  }
  isr_timer.setInterval(kTimerMs * 10, radioInterrupt);
  isr_timer.setInterval(kTimerMs * 20, imuInterrupt);
}

void loop(void) {}

void radioInterrupt(void) {
  // Serial.println("Entering radio interrupt!");
  if (radio.available()) {
    radio.read(ReceivedMessage, INDICATOR_MSG_LENGTH);
    if (ReceivedMessage[INDICATOR_LEFT] == HIGH) {
      digitalWrite(LEFT_INDICATOR_PIN, HIGH);
      digitalWrite(RIGHT_INDICATOR_PIN, LOW);
    } else if (ReceivedMessage[INDICATOR_RIGHT] == HIGH) {
      digitalWrite(RIGHT_INDICATOR_PIN, HIGH);
      digitalWrite(LEFT_INDICATOR_PIN, LOW);
    } else {
      digitalWrite(LEFT_INDICATOR_PIN, LOW);
      digitalWrite(RIGHT_INDICATOR_PIN, LOW);
    }
    // Serial.println("Received message with payload " +
    // String(ReceivedMessage[0]) + ", " + String(ReceivedMessage[1]));
  }
}

void imuInterrupt(void) {
  static float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
  // Serial.println("x: " + String(x) + ", y: " + String(y) + ", z: " + String(z));
  if (-z > kBrakeLightActivateThreshold) {
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
  } else if (-z < kBrakeLightDeactivateThreshold) {

    digitalWrite(BRAKE_LIGHT_PIN, LOW);
  }
}
