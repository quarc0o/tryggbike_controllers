#include <stdint.h>

// Arduino libraries from the Arduino IDE 1.8.15's "Library manager"
// "RF24" version 1.4.2
// "Arduino_LSM6DS3" version 1.0.0
// "FIR filter" version 0.1.1

// Radio
#include "RF24.h"
#include "SPI.h"
#include "nRF24L01.h"

// Accelerometer/IMU
#include <Arduino_LSM6DS3.h>

// Filter
#include <FIR.h>
  
static constexpr double kDegToRad{3.1416/180};

static constexpr double kBrakeLightActivateThreshold{-0.15}; // [g]
static constexpr double kBrakeLightDeactivateThreshold{-0.09};  // [g]
static constexpr int kImuFilterWindowTime{100};  // [ms]
static constexpr int kBrakeLightExtendTime{200}; // [ms]
static constexpr double kMountingAngle{17.75*kDegToRad}; // [deg]

#define LEFT_INDICATOR_PIN 2
#define RIGHT_INDICATOR_PIN 3
#define REAR_LIGHT_PIN 7
#define BRAKE_LIGHT_PIN 8

enum Indicator {
  INDICATOR_LEFT = 0,
  INDICATOR_RIGHT = 1,
  INDICATOR_MSG_LENGTH
};

uint8_t ReceivedMessage[INDICATOR_MSG_LENGTH] = {LOW, LOW};
RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO
// Identifier
const uint64_t pipe = 0xE6E6A43BE6E5;

static constexpr long kHardwareTimerMs{1000L};
static constexpr long kTimerMs{static_cast<long>(1000.0 / kHardwareTimerMs)};

static constexpr long kRadioInterruptPeriod{10};  // [ms]
static constexpr long kImuInterruptPeriod{10};    // [ms]


static constexpr size_t kFilterWindow{kImuFilterWindowTime / kImuInterruptPeriod};
static FIR<float, kFilterWindow> filter;

void radioInterrupt(void);
void imuInterrupt(void);

void setup(void) {
  //Serial.begin(9600);
  pinMode(LEFT_INDICATOR_PIN, OUTPUT);
  pinMode(RIGHT_INDICATOR_PIN, OUTPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(REAR_LIGHT_PIN, OUTPUT);
  digitalWrite(REAR_LIGHT_PIN, HIGH);

  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  
  float coeffs[kFilterWindow];
  memset(coeffs, 1.f, sizeof(coeffs));
  filter.setFilterCoeffs(coeffs);;

  IMU.begin();
}

void loop(void) {
  imuInterrupt();
  radioInterrupt();
  delay(10);
  }

void radioInterrupt(void) {
  //Serial.println("Entering radio interrupt!");
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
    //Serial.println("Received message with payload " +
    //String(ReceivedMessage[0]) + ", " + String(ReceivedMessage[1]));
  }
}

void imuInterrupt(void) {
  static unsigned int light_extend_countdown{0};
  
  static float raw_x, raw_y, raw_z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(raw_x, raw_y, raw_z);
  }
  static float flipped_x, flipped_y, flipped_z;
  flipped_x = raw_z;
  flipped_y = -raw_x;
  flipped_z = -raw_y;
  //Serial.println("flipped x: " + String(flipped_x) + ", flipped y: " + String(flipped_y) + ", flipped z: " + String(flipped_z) + ", countdown: " + String(light_extend_countdown));

  // Orient measurements into bicycle frame
  static float x, y, z;
  x = flipped_x * cos(kMountingAngle) - flipped_z * sin(kMountingAngle);
  y = flipped_y;
  z = flipped_x * sin(kMountingAngle) + flipped_z * cos(kMountingAngle);

  x = filter.processReading(x);
  
  //Serial.println("x: " + String(x) + ", y: " + String(y) + ", z: " + String(z) + ", countdown: " + String(light_extend_countdown));
  
  if (x < kBrakeLightActivateThreshold) {
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    light_extend_countdown = kBrakeLightExtendTime/kImuInterruptPeriod;
  } else if (x > kBrakeLightDeactivateThreshold) {

    if (light_extend_countdown == 0) {
      digitalWrite(BRAKE_LIGHT_PIN, LOW);
      //Serial.println("Not braking");
    }
    else {
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      light_extend_countdown--;
      //Serial.println("Braking");
    }
  }
}
