#include <Arduino.h>

class Encoder{
public:

   int encoderPin;
   volatile long totalPulseCount = 0;

   const int pulsePerRev = 20;

   //ISR
    unsigned long debounceTime = 500;
    unsigned long prevPulseUpdateTime = 0;

    //RPM
    double rpm = 0.0;
    long prevPulseCountRPM = 0;

    //Odom
    double wheelRadius = 1.0;
    long prevPulseCountOdom = 0;


   Encoder(int pin):encoderPin(pin){
    
   }

   void pulseIncrement(){

    unsigned long currentTime = micros();

    if(currentTime - prevPulseUpdateTime >= debounceTime){
      totalPulseCount++;
      prevPulseUpdateTime = currentTime;
    }

   }

   void updateRPM(double dt){

    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();
    
    long deltaPulseCount = currentPulseCount - prevPulseCountRPM;

    rpm = (deltaPulseCount / (double)pulsePerRev) * ((60.0) / dt);

   }
   
   double getDeltaS(double dt){//only for odom, maybe we can put into odom class
    
    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();
    
    long deltaPulseCount = currentPulseCount - prevPulseCountOdom;

    return (deltaPulseCount / (double)pulsePerRev) * 2 * PI * wheelRadius;

   }

};

class Motor{
public:



};

class Odometry{
public:
  
  Encoder* lEdr;
  Encoder* rEdr;

  const double trackWidth = 1.0;

  Odometry(Encoder* leftEdr, Encoder* rightEdr): lEdr(leftEdr), rEdr(rightEdr){

  }

struct{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
}currentPose;

  //purely from encoder
  void updatePose(double dt){
    
    double deltaSL = lEdr->getDeltaS(dt);
    double deltaSR = rEdr->getDeltaS(dt);

    double deltaSCM = (deltaSL + deltaSR) / 2;
    double deltaTheta = (deltaSR - deltaSL) / trackWidth;

    currentPose.x += deltaSCM * cos(currentPose.theta);
    currentPose.y += deltaSCM * sin(currentPose.theta);
    currentPose.theta += deltaTheta;

  }

};

class Drivetrain{
public:

};
//==========================================
int LEncoderPin;
int REncoderPin;
//==========================================

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);

Odometry odom(&leftEncoder, &rightEncoder);
//==========================================

void leftEncoderISR(){

  leftEncoder.pulseIncrement();

}

void rightEncoderISR(){

  rightEncoder.pulseIncrement();

}

//==========================================

unsigned long prevTime;
//==========================================

void setup(){
  
  Serial.begin(115200);
  delay(1000);


  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);
  
  unsigned long currentTime = micros();
  prevTime = currentTime;
}


void loop(){

  unsigned long currentTime = micros();

  double dt = (currentTime - prevTime);
  //updateRPM

  //updateOdm
  odom.updatePose(dt);

  //updateControl

  prevTime = currentTime;

}