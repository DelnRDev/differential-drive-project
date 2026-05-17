#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

double clampVal(double lowerBound, double upperBound, double val){

  if(val < lowerBound){
    return lowerBound;
  }else if(val > upperBound){
    return upperBound;
  }

  return val;

}

double wrapAngleDeg(double angle){

  while(angle < -180){
    angle += 360;
  }

  while (angle >= 180){
    angle -= 360;
  }

  return angle;

}

double wrapAngleRad(double angle){

  while(angle < -PI){
    angle += 2 * PI;
  }

  while (angle >= PI){
    angle -= 2 * PI;
  }

  return angle;

}



class Encoder{
public:
  int encoderPin;
  int pulsePerRev = 40;
  volatile long totalPulseCount = 0;

  //ISR
  unsigned long prevPulseCheckTime = 0;
  unsigned long incrementDebounceTime = 500;

  //RPM
  long prevPulseCount;
  double currentRPM = 0.0;
  //double prevRPM = 0.0;

  int direction = 1;

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

  void updateRPM(double dt){

    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();

    long deltaPulseCount = currentPulseCount - prevPulseCount;

    currentRPM = (deltaPulseCount / pulsePerRev) * (60.0 / dt);

  }

  double getRPM(){

    return currentRPM;
    
  }

};

class Motor{
public:

  Encoder* encoder;

  int pin1;
  int pin2;
  int pwmPin;

  int direction = 1;

  //setMotorByRPM
  int minRPM = 30;
  int maxRPM = 255;
  double prevRPM = 0.0;
  double errorIntegral = 0.0;

  Motor(int a, int b, int pwm,  Encoder* edr):pin1(a), pin2(b), pwmPin(pwm), encoder(edr){

  }

  void begin(){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }

  void setMotorByPWM(int pwm){

    if(pwm > 0){
      direction = 1;
      encoder->direction = 1;
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pwmPin, pwm);
    }else if (pwm < 0){
      direction = -1;
      encoder->direction = -1;
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, -pwm);
    }else{
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, 0);
    }

  }
  
  void setMotorByRPM(int targetRPM, double dt){
    //test this function

    double currentRPM = encoder->getRPM();

    double Kp = 2.0;
    double Ki = 0.1;

    double rpmError = currentRPM - prevRPM;

    double maxIntegralRPMContribution = 40;
    
    errorIntegral += rpmError * dt;

    //prevent intgral windup
    if(errorIntegral > 0 && Ki * errorIntegral > maxIntegralRPMContribution){
      errorIntegral = maxIntegralRPMContribution / Ki;
    }else if(errorIntegral < 0 && Ki * Ki * errorIntegral < -maxIntegralRPMContribution ){
      errorIntegral = -maxIntegralRPMContribution / Ki;
    }

    double u = Kp * rpmError + Ki * errorIntegral;

    u = clampVal(-maxRPM, maxRPM, u);

    if(u > 0 && u < minRPM){
      u = minRPM;
    }else if(u < 0 && u > -minRPM){
      u = -minRPM;
    }

    setMotorByPWM(u);




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


int LEncoderPin = 2;
int REncoderPin = 3;

int AIN1 = 4;
int AIN2 = 9;
int PWMA = 10;

int BIN1 = 6;
int BIN2 = 7;
int PWMB = 5;

Adafruit_MPU6050 imu;

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(AIN1, AIN2, PWMA, &leftEncoder);
Motor rightMotor(BIN2, BIN1, PWMB, &rightEncoder);

Drivetrain drivetrain(&leftEncoder, &rightEncoder, &leftMotor, &rightMotor);


void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}

unsigned long prevTime;

double encoderPeriod = 10000;

void setup() {

  prevTime = micros();

  Serial.begin(9600);

  drivetrain.begin();

  imu.begin();
   
  

  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);
}

void loop() {

  unsigned long currentTime = micros();

 if(currentTime - prevTime >= encoderPeriod){

  double dt = (currentTime - prevTime) / 1000000.0;

  leftEncoder.updateRPM(dt);
  rightEncoder.updateRPM(dt);

 }

}

