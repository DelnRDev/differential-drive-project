#include <Arduino.h>


class Encoder{
public:
  int encoderPin;
  volatile long totalPulseCount = 0;

  unsigned long prevPulseCheckTime = 0;
  unsigned long incrementDebounceTime = 500;

  Encoder(int pin):encoderPin(pin){

  }

  void begin(){

    pinMode(encoderPin, INPUT_PULLUP);

  }

  void pulseIncrement(){

    unsigned long currentTime = micros();

    if(currentTime - prevPulseCheckTime > incrementDebounceTime){

      totalPulseCount++;

      prevPulseCheckTime = currentTime;
    }

  }

};

class Motor{
public:

  int pin1;
  int pin2;
  int pwmPin;

  int direction = 1;

  Motor(int a, int b, int pwm):pin1(a), pin2(b), pwmPin(pwm){

  }

  void begin(){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }

  void setMotorByPWM(int pwm){

    if(pwm > 0){
      direction = 1;
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, pwm);
    }else if (pwm < 0){
      direction = -1;
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pwmPin, -pwm);
    }else{
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, 0);
    }

  }

};


class Drivetrain{
public:

  Encoder* lEdr;
  Encoder* rEdr;
  Motor* lMtr;
  Motor* rMtr;

  Drivetrain(Encoder* leftEdr, Encoder* rightEdr, Motor* leftMtr, Motor* rightMtr):lEdr(leftEdr), rEdr(rightEdr), lMtr(leftMtr), rMtr(rightMtr){

  }

  void begin(){

    lEdr->begin();
    rEdr->begin();
    lMtr->begin();
    rMtr->begin();

  }

  void setDriveByPWM(int leftPWM, int rightPWM){

    lMtr->setMotorByPWM(leftPWM);
    rMtr->setMotorByPWM(rightPWM);

  }

};


int LEncoderPin;
int REncoderPin;

int AIN1;
int AIN2;
int PWMA;

int BIN1;
int BIN2;
int PWMB;

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(AIN1, AIN2, PWMA);
Motor rightMotor(BIN1, BIN2, PWMB);

Drivetrain drivetrain(&leftEncoder, &rightEncoder, &leftMotor, &rightMotor);


void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}


void setup() {

  Serial.begin(9600);

  drivetrain.begin();
  

  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);
}

void loop() {
 


}

