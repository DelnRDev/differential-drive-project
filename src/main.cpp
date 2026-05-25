#include <Arduino.h>

int motorPin1 = 3;
int motorPin2 = 4;
int motorPwmPin = 10;

int encoderPin = 2;

int pulsePerRev = 40;
long totalPulseCount = 0;
unsigned prevEncoderUpdateTime = 0;
unsigned encoderUpdateInterval = 500;

double averageRpm = 0.0; //sample average over 

unsigned long motorSettlingPeriod = 5000; //5 sec
unsigned long samplingPeriod = 5000; //5 sec
unsigned long nextMeasurementWaitPeriod = 5000;

int testPwm = 255;


void encoderISR(){

  unsigned long currentTime = micros();

  if(currentTime - prevEncoderUpdateTime){

    totalPulseCount++;

  }

}

void setMotorPWM(int pwm){

  if(pwm > 0){

    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorPwmPin, pwm);

  }else if(pwm < 0){

    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPwmPin, -pwm);

  }else{

    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorPwmPin, 0);

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

    /*
    start motor
    wait 5 sec
    start count
    wait 5 sec
    stop count
    print result
    */

    setMotorPWM(testPwm);

    delay(motorSettlingPeriod);

    noInterrupts();
    totalPulseCount = 0;
    interrupts();

    delay(samplingPeriod);

    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();

    averageRpm = (currentPulseCount / pulsePerRev) * (60.0/(samplingPeriod / 1000));

    Serial.print("pwm: ");
    Serial.print(testPwm);

    Serial.print("sample period: ");
    Serial.print(samplingPeriod);

    Serial.print("pulse count: ");
    Serial.print(currentPulseCount);

    Serial.print("average RPM: ");
    Serial.print(averageRpm);

    Serial.print("waiting for next test");
    delay(nextMeasurementWaitPeriod);


}

