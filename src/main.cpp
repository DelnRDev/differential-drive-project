#include <Arduino.h>

int encoderPin = 2;
int motorPin1 = 4;
int motorPin2 = 5;
int motorPwmPin = 8;

unsigned long prevTime;
unsigned long prevEncoderPulseTime = 0;

unsigned long encoderPulseInterval = 500;
unsigned long encoderUpdatePeriod = 50000; //50ms

int pulsePerRev = 40;
long totalPulseCount = 0;
long prevPulseCount = 0;

double rpm = 0.0;

void encoderISR(){

  unsigned long currentTime = micros();

  if(currentTime - prevEncoderPulseTime >= encoderPulseInterval){

    totalPulseCount++;

  }


}

void setup(){

  Serial.begin(115200);

  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPwmPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, FALLING);

}

void loop(){
  

  unsigned long currentTime = micros();

  if(currentTime - prevTime >= encoderUpdatePeriod){

    double dt = (currentTime - prevTime) / 1000000.0;

    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();

    long deltaPulseCount = currentPulseCount - prevPulseCount;

    rpm = (deltaPulseCount / pulsePerRev) * (60.0 / dt);

    Serial.print("rpm: ");
    Serial.print(rpm);

    prevPulseCount = currentPulseCount;
    prevTime = currentTime;

  }

  /*
  


  */

}