
//Include needed Libraries at beginning
#include "nRF24L01.h" // NRF24L01 library created by TMRh20 https://github.com/TMRh20/RF24
#include "RF24.h"
#include "SPI.h"

int ReceivedMessage[1] = {000}; // Used to store value received by the NRF24L01

RF24 radio(9,10); // NRF24L01 used SPI pins + Pin 9 and 10 on the UNO

const uint64_t pipe = 0xE6E6E6E6E6E6; // Needs to be the same for communicating between 2 NRF24L01 


void setup(void){
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  radio.begin(); // Start the NRF24L01
  radio.openReadingPipe(1,pipe); // Get NRF24L01 ready to receive
  radio.startListening(); // Listen to see if information received
}

void loop(void){
  
  while (radio.available())
  {
  radio.read(ReceivedMessage, 1); // Read information from the NRF24L01
  if (ReceivedMessage[0] == 111) // Indicates switch is pressed
    {
      Serial.println("Pin high");
      digitalWrite(2, HIGH); 
    }
    else
    
    {
      Serial.println("Pin low");
      digitalWrite(2, LOW); 
   }
  delay(10);
  }
}
