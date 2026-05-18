#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>

double wrapAngleDeg(double angle){

  while(angle < -180.0){
    angle += 360.0;
  }

  while(angle >= 180.0){
    angle -= 360.0;
  }

  return angle;

}

double clampVal(double lowerBound, double upperBound, double val){

  if(val < lowerBound){
    return lowerBound;
  }else if(val > upperBound){
    return upperBound;
  }
  
  return val;
  
}

double wrapAngleRad(double angle){

  while(angle < -PI){
    angle += 2 * PI;
  }

  while(angle >= PI){
    angle -= 2 * PI;
  }

  return angle;

}

class Encoder{
public:

  int encoderPin;
  volatile long totalPulseCount = 0;

  //ISR
  unsigned long debounceTime = 500;
  unsigned long prevPulseUpdateTime;

  

  Encoder(int pin):encoderPin(pin){

  }

  void begin(){
    pinMode(encoderPin, INPUT_PULLUP);
  }

  void pulseIncrement(){
    
    unsigned long currentTime = micros();

    if(currentTime - prevPulseUpdateTime >= debounceTime){
      totalPulseCount++;
      prevPulseUpdateTime = currentTime;
    }

  }

};

class Motor{
public:

  int pin1;
  int pin2; 
  int pwmPin;

  Motor(int a, int b, int pwm):pin1(a), pin2(b), pwmPin(pwm){
     
  }

  void begin(){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }

  void setMotorByPWM(int pwm){
    if(pwm > 0){
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pwmPin, pwm);
    }else if(pwm < 0){
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

class IMUHeading{
private:
  double theta = 0.0;
  double omegaZBias = 0.0;
public:

  Adafruit_MPU6050 mpu;

  //calibration
  int numberOfSample = 1000;

  //heading
  double alpha = 0.3;
  double prevFilteredOmega = 0.0;
  double prevTheta = 0.0;

  IMUHeading(){

  }

  void begin(){

    if(!mpu.begin(0x68)){
      Serial.println("MPU6050 init failed");
      while(1);
    }

    Serial.println("MPU6050 connected");
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  void calibrate(){

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    double rawOmegaZ = gyro.gyro.z;

    double sampleSum = 0.0;

    for(int i = 0; i < numberOfSample; i++){
      sampleSum += rawOmegaZ;
    }

    omegaZBias = sampleSum / numberOfSample;

  }

  double getCalibratedOmegaZ(){
    
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    double rawOmegaZ = gyro.gyro.z;

    return rawOmegaZ - omegaZBias;

  }

  

  void updateHeading(double dt){

    double calibratedOmegaZ = getCalibratedOmegaZ();

    double filteredOmegaZ = alpha * calibratedOmegaZ + (1 - alpha) * prevFilteredOmega;

    double theta = prevTheta + filteredOmegaZ * dt;

    prevFilteredOmega = filteredOmegaZ;
    prevTheta = theta;

  }

  double getTheta(){
    return theta;
  }

};

class Drivetrain{
public:

  Encoder* lEdr;
  Encoder* rEdr;
  Motor* lMtr;
  Motor* rMtr;

  IMUHeading* imuhding;

  //turnPID
  int maxPWM = 255;
  int minPWM = 40;
  int maxIntegral = 200;

  double errorIntegral = 0.0;
  double prevThetaError = 0.0;

  Drivetrain(Encoder* leftEdr, Encoder* rightEdr, Motor* leftMtr, Motor* rightMtr, IMUHeading* imuhd):lEdr(leftEdr), rEdr(rightEdr), lMtr(leftMtr), rMtr(rightMtr), imuhding(imuhd){

  }

  void begin(){
    lEdr->begin();
    rEdr->begin();
    lMtr->begin();
    rMtr->begin();
    imuhding->begin();
  }

  void setDriveByPWM(int leftPWM, int rightPWM){
    lMtr->setMotorByPWM(leftPWM);
    rMtr->setMotorByPWM(rightPWM);
  }

  void turnToDeg(double targetTheta, double dt){//directly using PWM, unit:degree, this is relative to the initial theta

    targetTheta = wrapAngleDeg(targetTheta);

  

    double Kp;
    double Kd;
    double Ki;

    double currentTheta = imuhding->getTheta();
    double thetaError = targetTheta - currentTheta;

    double tolerance = 1.0;
    if(abs(thetaError) < tolerance){
      return;
    }

    errorIntegral += thetaError * dt;
    errorIntegral = clampVal(-maxIntegral, maxIntegral, errorIntegral);

    double errorDerivative = (thetaError - prevThetaError) / dt;

    double u = Kp * thetaError + Ki * errorIntegral + Kd * errorDerivative;

    u = clampVal(-maxPWM, maxPWM, u);

    if(u > 0 && u < minPWM){
      u = minPWM;
    }else if(u < 0 && u > -minPWM){
      u = -minPWM;
    }

    setDriveByPWM(-u, u);


  }

};



//==================================

int LEncoderPin;
int REncoderPin;
int AIN1;
int AIN2;
int PWMA;
int BIN1;
int BIN2;
int PWMB;

//==================================
unsigned long prevIMUUpdateTime;
unsigned long prevControlUpdateTime;

unsigned long imuUpdatePeriod = 500;
unsigned long controlUpdatePeriod = 10000;


//==================================
int LEncoderPin;
int REncoderPin;
int AIN1;
int AIN2;
int PWMA;
int BIN1;
int BIN2;
int PWMB;

//==================================

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(BIN1, BIN2, PWMB);
Motor rightMotor(AIN1, AIN2, PWMA);

IMUHeading imuHeading;
Drivetrain drivetrain(&leftEncoder, &rightEncoder, &leftMotor, &rightMotor, &imuHeading);


//==================================


void setup(){

  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  delay(100);
  drivetrain.begin();

  unsigned long currentTime = micros();

  prevControlUpdateTime = currentTime;
  prevIMUUpdateTime = currentTime;
}

void loop(){

  unsigned long currentTime = micros();

  //update sensor
  //set actuator

  if(currentTime - prevIMUUpdateTime >= imuUpdatePeriod){
    
    double dt = (currentTime - prevIMUUpdateTime) / 1000000.0;

    imuHeading.updateHeading(dt);
  }

  if(currentTime - prevControlUpdateTime >= controlUpdatePeriod){


    double dt = (currentTime - prevControlUpdateTime) / 1000000.0;

    drivetrain.turnToDeg(90.0, dt);

  }

}