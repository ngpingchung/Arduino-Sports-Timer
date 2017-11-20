/*   Created by Ng Ping Chung
  Version 0.1
  16 Nov 2017

  Pseudocode:

  In setup() {
    aPowerUpTime = millis();
    ping B to get bPowerUpTime;
    if (aPowerUpTime > bPowerUpTime) { //No good, either B didn't response or have started A first.
      set LCD to read ("Restart B then A")
      change LED from Green to Red;
      beep buzzer;
      startupOK = false;
      } else {
      timeDifference = bPowerUpTime - aPowerUpTime;
      set up to receive from B;
      set LCD to read "Ready";
      change LED from Red to Green;
      beep buzzer;
      startupOK = true;
    }
  }

  In loop() {
    if startupOK {
      if crossedStartLine {
        change LED from Green to Red;
        sound buzzer;
        change LED from Red to Green;
        if receive from B {
          FinishTime = bSentTime - timeDifference;
          result = endTime - startTime;
          write result to LCD;
        }
      }
    }

    if Reset Button Is Pressed {
      startTime = 0;
      FinishTime = 0;
      set LCD to read "Ready";
      change LED from Red to Green;
      beep buzzer;
    }

  }

  ISR() {
    startTime = millis();
    clear receive buffer // discard anything sent before crossedStartLine
    crossedStartLine = true;
  }

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

//#include <stdio.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int sensorPin = 2;
const int redLEDPin = 4;                              // need to attach 470Ω or 1KΩ resistor?
const int greenLEDPin = 5;                            // need to attach 470Ω or 1KΩ resistor?
const int buzzerPin = 6;
const int resetPin = 7;

const int redToGreen = 1;
const int greenToRed = 2;

unsigned long aPowerUpTime = 0;
unsigned long bPowerUpTime = 0;
unsigned long timeDifference = 0;
unsigned long startTime = 0;
unsigned long finishTime = 0;
unsigned long result = 0;
int* pHH;
int* pMM; 
int* pSS;
int* pMS;

unsigned long msg[1];
boolean waitForPing = false;
boolean startupOK = false;
boolean crossedStartLine = false;
boolean waitForFinishTime = false;

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

void millisToTime(unsigned long input, int* pHour, int* pMinute, int* pSecond, int* pMilliSec) {    // input, output, output, output, output
  unsigned long H;
  unsigned long M;
  unsigned long S;
  unsigned long MS;
  unsigned long whatsLeft;

  H = input / (1000 * 60 * 60);
  whatsLeft = input % (1000 * 60 * 60);
  M = whatsLeft / (1000 * 60);
  whatsLeft = whatsLeft % (1000 * 60);
  S= whatsLeft / (1000);
  MS = whatsLeft % (1000);

  *pHour = (int) H;
  *pMinute = (int) M;
  *pSecond = (int) S;
  *pMilliSec = (int) MS;
}

void sensorISR() {
  if (startupOK && !crossedStartLine){
    startTime = millis();
    crossedStartLine = true;    
  }
}

void setup()
{
  aPowerUpTime = millis();
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorPin), sensorISR, FALLING);  //set up interrupt on pin "sensorPin"
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(buzzerPin, HIGH);              // Assume buzzer trigger on low;
    
  // Ping B to get bPowerUpTime;
  radio.begin();
  radio.openWritingPipe(addresses[1]);        // Open a writing pipe on address 1, pipe 1
  radio.openReadingPipe(1, addresses[0]);     // Open a reading pipe on address 0, pipe 1

  radio.stopListening();
  msg[0] = 0x1LL;   // 0x1 = 1 in decimal = ping message
  radio.write(msg, 4);
  radio.startListening();
  waitForPing = true;
}

void loop()
{
  if (waitForPing) {
    if (radio.available()) {
      radio.read( &gotUnsignedLong, 4 );
      bPowerUpTime = gotUnsignedLong;
      waitForPing = false;
    }

    if (aPowerUpTime > bPowerUpTime) { //No good, either B didn't response or have started A first.
        Serial.println(F("Restart B then A"));
        ChangeLED(greenToRed);
        soundBuzzer(500);                                 // beep 500 ms
        startupOK = false;
      } else {
        timeDifference = bPowerUpTime - aPowerUpTime;
        Serial.print(F("Ready"));
        ChangeLED(redToGreen);
        soundBuzzer(500);
        startupOK = true;
      }
  }
  
  // Normally waiting for input
  if (startupOK && crossedStartLine) {
    radio.stopListening();
    msg[0] = 0x2LL;   // 0x2 = 2 in decimal = request for finish time
    radio.write(msg, 4);
    radio.startListening();
    waitForFinishTime = true;
  }

  if (startupOK && waitForFinishTime) {
    if (radio.available()) {
      radio.read( &gotUnsignedLong, 4 );
      finishTime = gotUnsignedLong - timeDifference;
      result = finishTime - startTime;
      millisToTime(result, pHH, pMM, pSS, pMS);
//      Serial.println(*pHH + ":" + *pMM + ":" + *pSS + ":" + *pMS);
      Serial.print(*pHH);
      Serial.print(":");
      Serial.print(*pMM);
      Serial.print(":");
      Serial.print(*pSS);
      Serial.print(":");
      Serial.println(*pMS);
      waitForFinishTime = false;
    }
  }

  if (startupOK && !crossedStartLine && !waitForFinishTime) {
    if (digitalRead(resetPin) == HIGH) {                        //pressed reset switch
      startTime = 0;
      finishTime = 0;
      result = 0;
      crossedStartLine = false;
      Serial.print(F("Ready"));
      ChangeLED(redToGreen);
      soundBuzzer(500);
    }
  }
}

//The End

