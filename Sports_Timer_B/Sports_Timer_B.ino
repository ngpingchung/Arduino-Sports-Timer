/*   Created by Ng Ping Chung
  Version 0.1
  16 Nov 2017

  For Arduino Nano
  SPI:
  10 (SS) or Chip Select,
  11 (MOSI),
  12 (MISO),
  13 (SCK).
  Therefore, connections will be:
  Pin 9 - CE
  Pin 10 - CS(N)
  Pin 11 - MOSI
  Pin 12 - MISO
  Pin 13 - SCK
  3.3v - VCC
  GND - GND
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int sensorPin = 2;
const int redLEDPin = 4;                              // need to attach 470Ω or 1KΩ resistor?
const int greenLEDPin = 5;                            // need to attach 470Ω or 1KΩ resistor?
const int buzzerPin = 6;

const int redToGreen = 1;
const int greenToRed = 2;

unsigned long bPowerUpTime = 0;
unsigned long finishTime = 0;

unsigned long msg[1];

boolean startupOK = false;
boolean pendingToSend = false;
boolean recReqForFinishTime = false;
/*

boolean waitForFinishTime = false;
*/

RF24 radio(9, 10);
byte addresses[][6] = {"1Node", "2Node"};             // nRF24L01+ chip is capable to listen for up to 6 addresses. We use "1Node" for A and "2Node" for B
unsigned long gotUnsignedLong[1];                     // Initialize a variable for the incoming response

void ChangeLED(int mode) {
  if (mode == redToGreen) {
      digitalWrite(redLEDPin, LOW);
      digitalWrite(greenLEDPin, HIGH);
  } else if (mode == greenToRed) {
      digitalWrite(greenLEDPin, LOW);  
      digitalWrite(redLEDPin, HIGH);
  }
}

void soundBuzzer(unsigned long duration) {
  digitalWrite(buzzerPin, LOW);               // Assume buzzer trigger on low;
  delay(duration);
  digitalWrite(buzzerPin, HIGH);  
}

void sensorISR() {
  if (startupOK && !pendingToSend){
    finishTime = millis();
    pendingToSend = true;    
  }
}

void setup()
{
  bPowerUpTime = millis();
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, FALLING);  //set up interrupt on pin "sensorPin"
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(buzzerPin, HIGH);              // Assume buzzer trigger on low;
    
  // Listen to A for command;
  radio.begin();
  radio.openWritingPipe(addresses[1]);        // Open a writing pipe on address 1, pipe 1
  radio.openReadingPipe(1, addresses[0]);     // Open a reading pipe on address 0, pipe 1
}

void loop()
{
  if (radio.available()){
    radio.read( &gotUnsignedLong, 4 );
    if (gotUnsignedLong == 0x1LL) {                   // Received ping message for bPowerUpTime
      radio.stopListening();
      msg[0] = bPowerUpTime;
      radio.write(msg, 4);
      radio.startListening();
      ChangeLED(redToGreen);
      soundBuzzer(500);
      startupOK = true;
    } else if (gotUnsignedLong == 0x2LL) {            // Received request for finishTime
      recReqForFinishTime = true;
    }
  }

  if (pendingToSend) {
    if (recReqForFinishTime) {
      radio.stopListening();
      msg[0] = finishTime;
      radio.write(msg, 4);
      radio.startListening();      
      ChangeLED(redToGreen);
      soundBuzzer(500);
      finishTime = 0;
      recReqForFinishTime = false;
      pendingToSend = false;      
    } else {
      ChangeLED(greenToRed);
      soundBuzzer(500);
    }
  }
}

//The End


