#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>




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
  unsigned long incrementDebounceTime = 0;

  //RPM
  long prevPulseCount = 0;
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
    //Serial.println(currentPulseCount);
    long deltaPulseCount = currentPulseCount - prevPulseCount;

    currentRPM = (deltaPulseCount / (double)pulsePerRev) * (60.0 / dt);
     Serial.println(currentRPM);

    prevPulseCount = currentPulseCount;
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

    double rpmError = targetRPM - currentRPM;

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
    Serial.println(currentRPM);
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


int LEncoderPin = 3;
int REncoderPin = 2;

int AIN1 = 8;
int AIN2 = 7;
int PWMA = 5;

int BIN1 = 9;
int BIN2 = 10;
int PWMB = 6;

Adafruit_MPU6050 mpu;

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(BIN1, BIN2, PWMB, &leftEncoder);
Motor rightMotor(AIN2, AIN1, PWMA, &rightEncoder);

Drivetrain drivetrain(&leftEncoder, &rightEncoder, &leftMotor, &rightMotor);


void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}

unsigned long prevEncoderUpdateTime;

double encoderPeriod = 10000;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {

  prevEncoderUpdateTime = micros();

  Serial.begin(115200);
  

  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);
}

unsigned long updatePIDPeriod = 10000;
unsigned long updateEncoderPeriod = 50000;

void loop() {
  /*
  unsigned long currentTime = micros();
  //Serial.println(currentTime - prevEncoderUpdateTime);
  if(currentTime - prevEncoderUpdateTime >= updateEncoderPeriod){
    double dt = (currentTime - prevEncoderUpdateTime)/ 1000000.0;
    leftEncoder.updateRPM(dt);

    //Serial.println(leftEncoder.getRPM());
    leftMotor.setMotorByRPM(200,dt);
    prevEncoderUpdateTime = currentTime;
  }
  */
  Serial.println(leftEncoder.totalPulseCount);
 
}

