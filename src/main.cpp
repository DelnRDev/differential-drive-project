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
  

  Encoder(int pin):encoderPin(pin){

  }

  void begin(){

    pinMode(encoderPin, INPUT_PULLUP);

  }

  void pulseIncrement(){

    unsigned long currentTime = micros();

    if(currentTime - prevPulseUpdateTime >= incrementDebounceTime){

      totalPulseCount++;

      prevPulseUpdateTime = currentTime;
    }

  }

  void updateRPM(double dt){

    noInterrupts();
    long currentPulseCount = totalPulseCount;
    interrupts();
   
    long deltaPulseCount = currentPulseCount - prevPulseCount;

    currentRPM = (deltaPulseCount / (double)pulsePerRev) * (60.0 / dt);
   
    prevPulseCount = currentPulseCount;
  }

  double getCurrentRPM(){

    return currentRPM;
    
  }

  private:
    int encoderPin;
    int pulsePerRev = 40;
    volatile long totalPulseCount = 0;

    //ISR
    unsigned long prevPulseUpdateTime = 0;
    unsigned long incrementDebounceTime = 500;

    //RPM
    long prevPulseCount = 0;
    double currentRPM = 0.0;

};

class Motor{
public:

  
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
      direction = 1;
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pwmPin, pwm);
    }else if (pwm < 0){
      direction = -1;
      direction = -1;
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

    double currentRPM = encoder->getCurrentRPM();

    double Kp = 2.0;
    double Ki = 0.1;
    double Kd = 0.0;
    double Kv = 0.0;

    double uff = Kv * targetRPM + minRPM; // feed forward pwm

    double rpmError = targetRPM - currentRPM;

    double maxIntegralRPMContribution = 40;
    
    errorIntegral += rpmError * dt;
    errorIntegral = clampVal(-maxIntegral, maxIntegral, errorIntegral);
  
    double errorDerivative = (rpmError - prevRPMError) / dt;

    double u = Kp * rpmError + Ki * errorIntegral +Kd * errorDerivative + uff;

    u = clampVal(-maxRPM, maxRPM, u);

    if(u > 0 && u < minRPM){
      u = minRPM;
    }else if(u < 0 && u > -minRPM){
      u = -minRPM;
    }

    Serial.println(currentRPM);
    setMotorByPWM(u);

  }
  
  private:
    Encoder* encoder;

    int pin1;
    int pin2;
    int pwmPin;

    int direction = 1;

    //motor controller
    int minRPM = 30;
    int maxRPM = 255;
    double maxIntegral = 50.0 ;
    double prevRPMError = 0.0;
    double errorIntegral = 0.0;



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

Adafruit_SSD1306 display(128, 64, &Wire, -1);


void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}

//------------------------------------------------

unsigned long prevEncoderUpdateTime;
unsigned long prevAutonUpdateTime;


unsigned long encoderUpdatePeriod = 10000;
unsigned long autonUpdatePeriod = 10000;
//-------------------------------------------------
enum AutoMode{
  AUTO_IDLE,
  AUTO_DRIVE1,
  AUTO_TURN1,
  AUTO_DRIVE2,
  AUTO_TURN2,
  DONE
};

AutoMode autoMode = AUTO_IDLE;



void runAutonomous(double dt){
  
  switch(autoMode){

    case(AUTO_DRIVE1):
      bool completed; //motion cmd
      if(completed){
        autoMode = AUTO_TURN1;
      }
      break;
    
    case(AUTO_TURN1):
      bool completed; //motion cmd
      if(completed){
        //setInitialPose to current state
        autoMode = AUTO_DRIVE2;
      }
      break;
      
    case(AUTO_DRIVE2):
      bool completed; //motion cmd
      if(completed){
        autoMode = AUTO_TURN2;
      }
      break;

    case(AUTO_TURN2):
      bool completed; //motion cmd
      if(completed){
        autoMode = DONE;
      }
      break;
    
  }

}

//

void setup() {

  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  drivetrain.begin();
  

  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);

  unsigned long currentTime = micros();
  prevEncoderUpdateTime = currentTime;
  prevAutonUpdateTime = currentTime;
}


void loop() {
 
  unsigned long currentTime = micros();

  //sense -> think -> act
  //sensorInput -> process and calculate and predict -> set the actuator

  if(currentTime - prevEncoderUpdateTime >= encoderUpdatePeriod){
    
    unsigned long dt = (currentTime - prevAutonUpdateTime) / 1000000.0;

    leftEncoder.updateRPM(dt);
    rightEncoder.updateRPM(dt);

  }

  //updateOdom here

  if(currentTime - prevAutonUpdateTime >= autonUpdatePeriod){

    unsigned long dt = (currentTime - prevAutonUpdateTime) / 1000000.0;

    runAutonomous(dt);

  }
 
}

