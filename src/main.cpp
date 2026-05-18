#include <Arduino.h>

double clampVal(double lowerBound, double upperBound, double val){

  if(val < lowerBound){
    return lowerBound;
  }else if(val > upperBound){
    return upperBound;
  }

  return val;
  
}

class Encoder{
private:
  volatile long totalPulseCount = 0;

  int direction = 1;
  long prevPulseCount = 0;
  double rpm = 0.0;

public:

  int encoderPin;

  //pulse interrupt
  unsigned long prevPulseTime = 0;
  long debonceTime = 500;

  const int pulsePerRev = 40; //checkByHand



  Encoder(int pin):encoderPin(pin){

  }

  void begin(){
    pinMode(encoderPin, INPUT_PULLUP);
  }

  void pulseIncrement(){
    
    unsigned long currentTime = micros();

    if(currentTime - prevPulseTime >= debonceTime){
      totalPulseCount++;
      prevPulseTime = currentTime;
    }

  }

  void updateRPM(double dt){
    
    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();

    long deltaPulseCount = currentPulseCount - prevPulseCount;
    
    rpm = direction * (deltaPulseCount / (double)pulsePerRev) * (60.0 / dt);

    prevPulseCount = currentPulseCount;
  }

  double getRPM(){
    return rpm;
  }

  void setDirection(int dir){
    direction = dir; //1 = forward, -1 = backward
  }
};

class Motor{
private:
  
public:

  int pin1;
  int pin2;
  int pwmPin;

  Encoder* encoder;

  //Speed control
  const int minRPM = 100;
  const int minPWM = 40;
  const int maxPWM = 255;
  const int maxIntegral = 500;

  double errorIntegral = 0.0;
  double prevRPMError = 0.0;




  Motor(int a, int b, int pwm, Encoder* edr):pin1(a), pin2(b), pwmPin(pwm), encoder(edr){

  }

  void begin(){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }

  void setMotorByPWM(int pwm){
    //comeback verify direction experimentally
    if(pwm > 0){
      encoder->setDirection(1);
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, pwm);
    }else if(pwm < 0){
      encoder->setDirection(-1);
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

    if(abs(targetRPM) < minRPM){
      setMotorByPWM(0);
      return;
    }

    double Kp = 1.0;
    double Ki = 0.2;
    double Kd = 0.1;
    double Kv = 0.1;

    double currentRPM = encoder->getRPM();
    double rpmError = targetRPM - currentRPM;

    errorIntegral += rpmError * dt;
    errorIntegral = clampVal(-maxIntegral, maxIntegral, errorIntegral);

    double errorDerivative = (rpmError - prevRPMError) / dt;

    double u = Kp * rpmError + Ki * errorIntegral + Kd * errorDerivative + Kv * targetRPM;

    u = clampVal(-maxPWM, maxPWM, u);

    if(u > 0 && u < minPWM){
      u = minPWM;
    }else if(u < 0 && u < -minPWM){
      u = -minPWM;
    }

    setMotorByPWM(u);

    prevRPMError = rpmError;
  }

};


//======================================
int LEncoderPin;
int REncoderPin;
int AIN1;
int AIN2;
int PWMA;
int BIN1;
int BIN2;
int PWMB;
//======================================
Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(BIN1, BIN2, PWMB, &leftEncoder);
Motor rightMotor(AIN1, AIN2, PWMA, &rightEncoder);

//======================================

void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}

//======================================
unsigned long prevRPMTime;
unsigned long prevControlTime;

unsigned long rpmUpdatePeriod = 100000;
unsigned long controlUpdatePeriod = 100000;
//======================================


void setup(){
  
  Serial.begin(115200);
  delay(1000);

  
  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);
  
  unsigned long currentTime = micros();

  prevRPMTime = currentTime;
  
}

void loop(){

  unsigned long currentTime = micros();
  
  
  if(currentTime - prevRPMTime >= rpmUpdatePeriod){
    double dt = (currentTime - prevRPMTime) / 1000000.0;
    leftEncoder.updateRPM(dt);
    prevRPMTime = currentTime;
  }

  if(currentTime - prevControlTime >= controlUpdatePeriod){
    double dt = (currentTime - prevControlTime) / 1000000.0;
    leftMotor.setMotorByRPM(200, dt);
    prevControlTime = currentTime;
  }

}