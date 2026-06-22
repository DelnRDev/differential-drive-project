#include <Arduino.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>




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

    double measuredRPM = (deltaPulseCount / (double)pulsePerRev) * (60.0 / dt);
    currentRPM = (rpmFilterAlpha * measuredRPM) + ((1.0 - rpmFilterAlpha) * currentRPM);
    lastDeltaPulseCount = deltaPulseCount;
   
    prevPulseCount = currentPulseCount;
  }

  double getCurrentRPM(){

    return currentRPM;
    
  }

  long consumeDeltaPulses(){

    long deltaPulses = lastDeltaPulseCount;
    lastDeltaPulseCount = 0;
    return deltaPulses;

  }

  int getPulsesPerRev(){

    return pulsePerRev;

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
    long lastDeltaPulseCount = 0;
    double currentRPM = 0.0;
    double rpmFilterAlpha = 0.25;

  

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
    int pwmCommand = (int)clampVal(-maxPWM, maxPWM, pwm);
    previousPWMCommand = pwmCommand;

    if(pwmCommand > 0){
      direction = 1;
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, pwmCommand);
    }else if (pwmCommand < 0){
      direction = -1;
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pwmPin, -pwmCommand);
    }else{
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      analogWrite(pwmPin, 0);
    }

  }

  void stop(){

    errorIntegral = 0.0;
    prevRPMError = 0.0;
    previousTargetRPM = 0.0;
    rampedTargetRPM = 0.0;
    setMotorByPWM(0);

  }

  void setTelemetryEnabled(bool enabled){

    telemetryEnabled = enabled;

  }

  int getDirection(){

    return direction;

  }
  
  void setMotorByRPM(double targetRPM, double dt){
    if(dt <= 0.0){
      return;
    }

    if(fabs(targetRPM) < rpmEpsilon){
      errorIntegral = 0.0;
      prevRPMError = 0.0;
      previousTargetRPM = 0.0;
      setMotorByPWM(0);
      return;
    }

    if((targetRPM > 0.0 && previousTargetRPM < 0.0) || (targetRPM < 0.0 && previousTargetRPM > 0.0)){
      errorIntegral = 0.0;
      prevRPMError = 0.0;
      rampedTargetRPM = 0.0;
    }
    previousTargetRPM = targetRPM;

    double controllerTargetRPM = rampTargetRPM(targetRPM, dt);
    double currentRPM = encoder->getCurrentRPM() * direction;
    double targetSign = controllerTargetRPM >= 0.0 ? 1.0 : -1.0;
    double feedForward = targetSign * ((Kv * fabs(controllerTargetRPM)) + minPWM);
    double rpmError = controllerTargetRPM - currentRPM;

    errorIntegral += rpmError * dt;
    errorIntegral = clampVal(-maxIntegral, maxIntegral, errorIntegral);
  
    double errorDerivative = (rpmError - prevRPMError) / dt;

    double u = Kp * rpmError + Ki * errorIntegral + Kd * errorDerivative + feedForward;

    if(targetSign > 0.0){
      u = clampVal(0.0, maxPWM, u);
    }else{
      u = clampVal(-maxPWM, 0.0, u);
    }

    bool needsMoreSpeed = targetSign * rpmError > rpmDeadband;
    if(needsMoreSpeed && fabs(u) > 0.0 && fabs(u) < minPWM){
      u = targetSign * minPWM;
    }

    prevRPMError = rpmError;

    int pwmCommand = rampPWM((int)round(u), dt);
    printTelemetry(currentRPM, rpmError, pwmCommand);

    setMotorByPWM(pwmCommand);

  }
  
  private:
    
    int pin1;
    int pin2;
    int pwmPin;

    Encoder* encoder;


    int direction = 1;

    //motor controller
    double Kp = 0.75;
    double Ki = 0.35;
    double Kd = 0.0;
    double Kv = 0.55;

    int minPWM = 35;
    int maxPWM = 255;
    double rpmEpsilon = 1.0;
    double rpmDeadband = 5.0;
    double maxIntegral = 80.0 ;
    double prevRPMError = 0.0;
    double errorIntegral = 0.0;
    double previousTargetRPM = 0.0;
    double rampedTargetRPM = 0.0;
    int previousPWMCommand = 0;
    double maxRPMChangePerSecond = 300.0;
    double maxPWMChangePerSecond = 450.0;
    unsigned long prevDebugPrintTime = 0;
    unsigned long debugPrintPeriod = 100000;
    bool telemetryEnabled = true;

    double rampTargetRPM(double requestedRPM, double dt){
      double maxStep = maxRPMChangePerSecond * dt;
      rampedTargetRPM = clampVal(rampedTargetRPM - maxStep, rampedTargetRPM + maxStep, requestedRPM);
      return rampedTargetRPM;
    }

    int rampPWM(int requestedPWM, double dt){
      int maxStep = max(1, (int)round(maxPWMChangePerSecond * dt));
      int rampedPWM = (int)clampVal(previousPWMCommand - maxStep, previousPWMCommand + maxStep, requestedPWM);
      return (int)clampVal(-maxPWM, maxPWM, rampedPWM);
    }

    void printTelemetry(double currentRPM, double rpmError, int pwmCommand){
      if(!telemetryEnabled){
        return;
      }

      unsigned long currentTime = micros();
      if(currentTime - prevDebugPrintTime < debugPrintPeriod){
        return;
      }
      prevDebugPrintTime = currentTime;

      Serial.print(">");

      Serial.print("currentRPM: ");
      Serial.print(currentRPM);
      Serial.print(",");

      Serial.print("rpmError: ");
      Serial.print(rpmError);
      Serial.print(",");

      Serial.print("pwm: ");
      Serial.println(pwmCommand);
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
    delay(5000);

    lMtr->begin();
    rMtr->begin();

  }

  void setDriveByPWM(int leftPWM, int rightPWM){

    lMtr->setMotorByPWM(leftPWM);
    rMtr->setMotorByPWM(rightPWM);

  }

  void setDriveByRPM(double leftRPM, double rightRPM, double dt){

    lMtr->setMotorByRPM(leftRPM, dt);
    rMtr->setMotorByRPM(rightRPM, dt);

  }

  void stop(){

    lMtr->stop();
    rMtr->stop();

  }

  double getLeftRPM(){

    return lEdr->getCurrentRPM() * lMtr->getDirection();

  }

  double getRightRPM(){

    return rEdr->getCurrentRPM() * rMtr->getDirection();

  }

};

struct Pose2D{
  double x;
  double y;
  double theta;
};

class Odometry{
public:

  void begin(double wheelDiameterMeters, double trackWidthMeters, int encoderPulsesPerRev){

    metersPerPulse = (PI * wheelDiameterMeters) / (double)encoderPulsesPerRev;
    trackWidth = trackWidthMeters;
    reset(0.0, 0.0, 0.0);

  }

  void reset(double x, double y, double theta){

    pose.x = x;
    pose.y = y;
    pose.theta = wrapAngleRad(theta);
    leftDistanceTotal = 0.0;
    rightDistanceTotal = 0.0;

  }

  void update(long leftPulses, long rightPulses, int leftDirection, int rightDirection){

    double leftDistance = leftPulses * leftDirection * metersPerPulse;
    double rightDistance = rightPulses * rightDirection * metersPerPulse;
    double centerDistance = (leftDistance + rightDistance) * 0.5;
    double deltaTheta = (rightDistance - leftDistance) / trackWidth;
    double midTheta = pose.theta + (deltaTheta * 0.5);

    leftDistanceTotal += leftDistance;
    rightDistanceTotal += rightDistance;
    pose.x += centerDistance * cos(midTheta);
    pose.y += centerDistance * sin(midTheta);
    pose.theta = wrapAngleRad(pose.theta + deltaTheta);

  }

  void setHeadingRad(double theta){

    pose.theta = wrapAngleRad(theta);

  }

  Pose2D getPose(){

    return pose;

  }

  double getHeadingRad(){

    return pose.theta;

  }

  double getLeftDistanceM(){

    return leftDistanceTotal;

  }

  double getRightDistanceM(){

    return rightDistanceTotal;

  }

private:

  Pose2D pose = {0.0, 0.0, 0.0};
  double leftDistanceTotal = 0.0;
  double rightDistanceTotal = 0.0;
  double metersPerPulse = 0.0;
  double trackWidth = 0.15;

};

class YawKalmanFilter{
public:

  void reset(double headingRad){

    heading = wrapAngleRad(headingRad);
    variance = 0.1;

  }

  void predict(double gyroRateRadPerSecond, double dt){

    if(dt <= 0.0){
      return;
    }

    heading = wrapAngleRad(heading + gyroRateRadPerSecond * dt);
    variance += processNoise * dt;

  }

  void correct(double measuredHeadingRad){

    double innovation = wrapAngleRad(measuredHeadingRad - heading);
    double kalmanGain = variance / (variance + measurementNoise);

    heading = wrapAngleRad(heading + kalmanGain * innovation);
    variance = (1.0 - kalmanGain) * variance;

  }

  double getHeadingRad(){

    return heading;

  }

private:

  double heading = 0.0;
  double variance = 0.1;
  double processNoise = 0.006;
  double measurementNoise = 0.20;

};

class PeriodicTask{
public:

  void begin(unsigned long periodMicros, unsigned long startTime){

    period = periodMicros;
    prevRunTime = startTime;

  }

  bool ready(unsigned long currentTime){

    return currentTime - prevRunTime >= period;

  }

  double consumeDt(unsigned long currentTime){

    double dt = (currentTime - prevRunTime) / 1000000.0;
    prevRunTime = currentTime;
    return dt;

  }

private:

  unsigned long period = 20000;
  unsigned long prevRunTime = 0;

};

enum AutoActionType{
  ACTION_DRIVE,
  ACTION_TURN,
  ACTION_WAIT,
  ACTION_DONE
};

struct AutoAction{
  AutoActionType type;
  double value;
  double maxRPM;
};

class AutonomousRunner{
public:

  void begin(const AutoAction* programPtr, int actionCount){

    program = programPtr;
    programLength = actionCount;

  }

  void reset(Pose2D currentPose){

    currentActionIndex = 0;
    actionStarted = false;
    actionElapsed = 0.0;
    pauseElapsed = 0.0;
    timedOut = false;
    finished = false;
    startPose = currentPose;

  }

  void update(double dt, Pose2D currentPose, Drivetrain* drivetrain, Odometry* odometry){

    if(program == NULL || programLength <= 0 || currentActionIndex >= programLength){
      drivetrain->stop();
      finished = true;
      return;
    }

    if(finished){
      drivetrain->stop();
      return;
    }

    if(pauseElapsed < interActionPause){
      pauseElapsed += dt;
      drivetrain->stop();
      return;
    }

    if(!actionStarted){
      startAction(currentPose, odometry);
    }

    actionElapsed += dt;

    AutoAction action = program[currentActionIndex];

    switch(action.type){
      case ACTION_DRIVE:
        if(actionElapsed >= driveTimeoutSeconds(action)){
          failStop(drivetrain);
          return;
        }
        runDriveAction(action, currentPose, drivetrain, odometry, dt);
        break;
      case ACTION_TURN:
        if(actionElapsed >= turnTimeoutSeconds(action)){
          failStop(drivetrain);
          return;
        }
        runTurnAction(action, currentPose, drivetrain, dt);
        break;
      case ACTION_WAIT:
        runWaitAction(action, drivetrain);
        break;
      case ACTION_DONE:
        drivetrain->stop();
        finished = true;
        break;
    }

  }

  int getActionIndex(){

    return currentActionIndex;

  }

  double getTurnTargetDeg(){

    return lastTurnTargetDeg;

  }

  double getTurnErrorDeg(){

    return lastTurnErrorDeg;

  }

  bool isFinished(){

    return finished;

  }

  bool didTimeout(){

    return timedOut;

  }

private:

  const AutoAction* program = NULL;
  int programLength = 0;
  int currentActionIndex = 0;
  bool actionStarted = false;
  bool finished = false;
  bool timedOut = false;
  double actionElapsed = 0.0;
  double pauseElapsed = 0.0;
  Pose2D startPose = {0.0, 0.0, 0.0};

  double headingKpRPMPerRad = 180.0;
  double wheelBalanceKp = 0.10;
  double wheelDistanceKpRPMPerMeter = 650.0;
  double turnKpRPMPerRad = 70.0;
  double minTurnRPM = 28.0;
  double turnSlowZoneRad = 12.0 * DEG_TO_RAD;
  double headingCorrectionMaxRPM = 80.0;
  double wheelBalanceMaxRPM = 35.0;
  double minDriveRPM = 65.0;
  double driveAccelTime = 0.45;
  double driveBrakeDistanceM = 0.10;
  double turnAccelTime = 0.35;
  double turnBrakeZoneRad = 25.0 * DEG_TO_RAD;
  double driveToleranceM = 0.015;
  double turnToleranceRad = 2.0 * DEG_TO_RAD;
  double turnSettleTime = 0.12;
  double interActionPause = 0.18;
  double turnSettledTime = 0.0;
  double lastTurnTargetDeg = 0.0;
  double lastTurnErrorDeg = 0.0;
  double startLeftDistanceM = 0.0;
  double startRightDistanceM = 0.0;

  void startAction(Pose2D currentPose, Odometry* odometry){

    startPose = currentPose;
    actionElapsed = 0.0;
    pauseElapsed = interActionPause;
    turnSettledTime = 0.0;
    if(odometry != NULL){
      startLeftDistanceM = odometry->getLeftDistanceM();
      startRightDistanceM = odometry->getRightDistanceM();
    }
    actionStarted = true;

  }

  void advanceAction(Drivetrain* drivetrain){

    drivetrain->stop();
    currentActionIndex++;
    actionStarted = false;
    pauseElapsed = 0.0;

  }

  void failStop(Drivetrain* drivetrain){

    drivetrain->stop();
    timedOut = true;
    finished = true;

  }

  double driveTimeoutSeconds(AutoAction action){

    return 1.5 + fabs(action.value) * 5.0;

  }

  double turnTimeoutSeconds(AutoAction action){

    return 2.0 + fabs(action.value) * 0.035;

  }

  double profiledDriveRPM(AutoAction action, double distanceError){

    double maxRPM = fabs(action.maxRPM);
    double distanceRemaining = fabs(distanceError);
    double sign = distanceError >= 0.0 ? 1.0 : -1.0;
    double accelScale = clampVal(0.0, 1.0, actionElapsed / driveAccelTime);
    double brakeScale = clampVal(0.0, 1.0, sqrt(distanceRemaining / driveBrakeDistanceM));
    double profileScale = min(accelScale, brakeScale);

    if(distanceRemaining > driveToleranceM * 2.5){
      profileScale = max(profileScale, minDriveRPM / maxRPM);
    }

    return sign * maxRPM * profileScale;

  }

  double profiledTurnRPM(AutoAction action, double headingError){

    double maxRPM = fabs(action.maxRPM);
    double sign = headingError >= 0.0 ? 1.0 : -1.0;
    double accelScale = clampVal(0.0, 1.0, actionElapsed / turnAccelTime);
    double brakeScale = clampVal(0.0, 1.0, sqrt(fabs(headingError) / turnBrakeZoneRad));
    double profileScale = min(accelScale, brakeScale);
    double turnRPM = maxRPM * profileScale;

    if(fabs(headingError) > turnSlowZoneRad){
      turnRPM = max(turnRPM, minTurnRPM);
    }

    return sign * turnRPM;

  }

  void runDriveAction(AutoAction action, Pose2D currentPose, Drivetrain* drivetrain, Odometry* odometry, double dt){

    double dx = currentPose.x - startPose.x;
    double dy = currentPose.y - startPose.y;
    double distanceAlongStart = dx * cos(startPose.theta) + dy * sin(startPose.theta);
    double distanceError = action.value - distanceAlongStart;

    if(fabs(distanceError) <= driveToleranceM){
      advanceAction(drivetrain);
      return;
    }

    double forwardRPM = profiledDriveRPM(action, distanceError);
    double headingError = wrapAngleRad(startPose.theta - currentPose.theta);
    double headingCorrectionRPM = clampVal(-headingCorrectionMaxRPM, headingCorrectionMaxRPM, headingKpRPMPerRad * headingError);
    double rpmBalanceError = drivetrain->getRightRPM() - drivetrain->getLeftRPM();
    double wheelDistanceError = 0.0;

    if(odometry != NULL){
      double leftTravel = odometry->getLeftDistanceM() - startLeftDistanceM;
      double rightTravel = odometry->getRightDistanceM() - startRightDistanceM;
      wheelDistanceError = rightTravel - leftTravel;
    }

    double wheelBalanceRPM = (wheelBalanceKp * rpmBalanceError) + (wheelDistanceKpRPMPerMeter * wheelDistanceError);
    wheelBalanceRPM = clampVal(-wheelBalanceMaxRPM, wheelBalanceMaxRPM, wheelBalanceRPM);

    drivetrain->setDriveByRPM(forwardRPM - headingCorrectionRPM + wheelBalanceRPM, forwardRPM + headingCorrectionRPM - wheelBalanceRPM, dt);

  }

  void runTurnAction(AutoAction action, Pose2D currentPose, Drivetrain* drivetrain, double dt){

    double targetHeading = wrapAngleRad(startPose.theta + action.value * DEG_TO_RAD);
    double headingError = wrapAngleRad(targetHeading - currentPose.theta);
    lastTurnTargetDeg = targetHeading * RAD_TO_DEG;
    lastTurnErrorDeg = headingError * RAD_TO_DEG;

    if(fabs(headingError) <= turnToleranceRad){
      turnSettledTime += dt;
      drivetrain->stop();
      if(turnSettledTime >= turnSettleTime){
        advanceAction(drivetrain);
      }
      return;
    }
    turnSettledTime = 0.0;

    double pTurnRPM = turnKpRPMPerRad * headingError;
    double turnRPM = profiledTurnRPM(action, headingError);
    if(fabs(pTurnRPM) < fabs(turnRPM)){
      turnRPM = pTurnRPM;
    }

    drivetrain->setDriveByRPM(-turnRPM, turnRPM, dt);

  }

  void runWaitAction(AutoAction action, Drivetrain* drivetrain){

    drivetrain->stop();
    if(actionElapsed >= action.value){
      advanceAction(drivetrain);
    }

  }

};

//-------------------------------------------------
int LEncoderPin = 2;
int REncoderPin = 3;

int AIN1 = 8;
int AIN2 = 7;
int PWMA = 5;

int BIN1 = 9;
int BIN2 = 10;
int PWMB = 6;

const double wheelDiameterM = 0.0666;
const double effectiveTrackWidthM = 0.1550;

Adafruit_MPU6050 mpu;

Encoder leftEncoder(LEncoderPin);
Encoder rightEncoder(REncoderPin);
Motor leftMotor(AIN1, AIN2, PWMA, &leftEncoder);
Motor rightMotor(BIN2, BIN1, PWMB, &rightEncoder);

Drivetrain drivetrain(&leftEncoder, &rightEncoder, &leftMotor, &rightMotor);

Odometry odometry;
YawKalmanFilter headingFilter;
PeriodicTask sensorTask;
PeriodicTask controlTask;
PeriodicTask telemetryTask;
PeriodicTask oledTask;
AutonomousRunner autoRunner;

const AutoAction autoProgram[] = {
  {ACTION_DRIVE, 0.25, 110.0},
  {ACTION_WAIT, 0.50, 0.0},
  {ACTION_DONE, 0.0, 0.0}
};

const int autoProgramLength = sizeof(autoProgram) / sizeof(autoProgram[0]);

bool imuReady = false;
double gyroZBias = 0.0;
bool oledReady = false;
bool autonomousHardStopped = false;
bool autonomousStarted = false;
double autonomousElapsed = 0.0;
double autonomousStartDelayElapsed = 0.0;
const double autonomousStartDelaySeconds = 2.0;
const double maxAutonomousRuntimeSeconds = 3.0;

uint8_t oledAddress = 0x3C;

const char debugFontChars[] PROGMEM = " 0123456789.-:XYHDGSTEPLRIMUONKA";
const uint8_t debugFont[][5] PROGMEM = {
  {0x00, 0x00, 0x00, 0x00, 0x00}, // space
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
  {0x00, 0x60, 0x60, 0x00, 0x00}, // .
  {0x08, 0x08, 0x08, 0x08, 0x08}, // -
  {0x00, 0x36, 0x36, 0x00, 0x00}, // :
  {0x63, 0x14, 0x08, 0x14, 0x63}, // X
  {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
  {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
  {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
  {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
  {0x46, 0x49, 0x49, 0x49, 0x31}, // S
  {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
  {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
  {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
  {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
  {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
  {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
  {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
  {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
  {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
  {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
  {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
  {0x7E, 0x09, 0x09, 0x09, 0x7E}  // A
};

const uint8_t oledInitCommands[] PROGMEM = {
  0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
  0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
  0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
  0xAF
};

//-------------------------------------------------
void leftEncoderISR(){
  leftEncoder.pulseIncrement();
}

void rightEncoderISR(){
  rightEncoder.pulseIncrement();
}

//------------------------------------------------

unsigned long sensorUpdatePeriod = 50000;    // 50ms
unsigned long controlUpdatePeriod = 20000;   // 20ms
unsigned long telemetryUpdatePeriod = 100000; // 100ms
unsigned long oledUpdatePeriod = 250000;     // 250ms

bool i2cAddressResponds(uint8_t address){

  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;

}

void printI2CScan(){

  Serial.print(F("I2C devices:"));

  bool foundAny = false;
  for(uint8_t address = 1; address < 127; address++){
    if(i2cAddressResponds(address)){
      Serial.print(F(" 0x"));
      if(address < 16){
        Serial.print(F("0"));
      }
      Serial.print(address, HEX);
      foundAny = true;
    }
  }

  if(!foundAny){
    Serial.print(F(" none"));
  }

  Serial.println();

}

void oledCommand(uint8_t command){

  Wire.beginTransmission(oledAddress);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();

}

void oledSetCursor(uint8_t page, uint8_t column){

  oledCommand(0xB0 | (page & 0x07));
  oledCommand(0x00 | (column & 0x0F));
  oledCommand(0x10 | (column >> 4));

}

uint8_t getDebugGlyphByte(char character, uint8_t column){

  if(character >= 'a' && character <= 'z'){
    character -= 32;
  }

  for(uint8_t i = 0; i < sizeof(debugFontChars) - 1; i++){
    if((char)pgm_read_byte(&debugFontChars[i]) == character){
      return pgm_read_byte(&debugFont[i][column]);
    }
  }

  return 0x00;

}

void oledWriteChar(char character){

  Wire.beginTransmission(oledAddress);
  Wire.write(0x40);

  for(uint8_t col = 0; col < 5; col++){
    Wire.write(getDebugGlyphByte(character, col));
  }

  Wire.write(0x00);
  Wire.endTransmission();

}

void oledPrint(const char* text){

  while(*text != '\0'){
    oledWriteChar(*text);
    text++;
  }

}

void oledPrintInt(int value){

  char buffer[8];
  itoa(value, buffer, 10);
  oledPrint(buffer);

}

void oledPrintDouble(double value, uint8_t decimals){

  char buffer[12];
  dtostrf(value, 0, decimals, buffer);
  oledPrint(buffer);

}

void oledClearLine(uint8_t page){

  oledSetCursor(page, 0);

  for(uint8_t chunk = 0; chunk < 8; chunk++){
    Wire.beginTransmission(oledAddress);
    Wire.write(0x40);
    for(uint8_t i = 0; i < 16; i++){
      Wire.write(0x00);
    }
    Wire.endTransmission();
  }

}

void oledClear(){

  for(uint8_t page = 0; page < 8; page++){
    oledClearLine(page);
  }

}

bool initOled(){

  if(i2cAddressResponds(0x3C)){
    oledAddress = 0x3C;
  }else if(i2cAddressResponds(0x3D)){
    oledAddress = 0x3D;
  }else{
    return false;
  }

  for(uint8_t i = 0; i < sizeof(oledInitCommands); i++){
    oledCommand(pgm_read_byte(&oledInitCommands[i]));
  }

  oledCommand(0xA5);
  delay(500);
  oledCommand(0xA4);
  oledClear();
  return true;

}

void oledPrintLine(uint8_t page, const char* label){

  oledClearLine(page);
  oledSetCursor(page, 0);
  oledPrint(label);

}

void updateOledDebug(){

  if(!oledReady){
    return;
  }

  Pose2D pose = odometry.getPose();

  oledPrintLine(0, "STEP:");
  oledPrintInt(autoRunner.getActionIndex());
  if(autoRunner.didTimeout() || autonomousHardStopped){
    oledPrint(" TO");
  }else if(!autonomousStarted){
    oledPrint(" HOLD");
  }else{
    oledPrint(" IMU:");
    oledPrint(imuReady ? "OK" : "NO");
  }

  oledPrintLine(1, "X:");
  oledPrintDouble(pose.x * 100.0, 1);
  oledPrint(" Y:");
  oledPrintDouble(pose.y * 100.0, 1);

  oledPrintLine(2, "HDG:");
  oledPrintDouble(pose.theta * RAD_TO_DEG, 1);

  oledPrintLine(3, "ERR:");
  oledPrintDouble(autoRunner.getTurnErrorDeg(), 1);
  oledPrint(" TGT:");
  oledPrintDouble(autoRunner.getTurnTargetDeg(), 0);

  oledPrintLine(4, "L:");
  oledPrintDouble(leftEncoder.getCurrentRPM() * leftMotor.getDirection(), 0);
  oledPrint(" R:");
  oledPrintDouble(rightEncoder.getCurrentRPM() * rightMotor.getDirection(), 0);

}

void calibrateGyroZ(){

  if(!imuReady){
    return;
  }

  const int sampleCount = 100;
  double gyroSum = 0.0;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  for(int i = 0; i < sampleCount; i++){
    mpu.getEvent(&accel, &gyro, &temp);
    gyroSum += gyro.gyro.z;
    delay(5);
  }

  gyroZBias = gyroSum / sampleCount;

}

void updateHeadingFilter(double dt){

  Pose2D pose = odometry.getPose();

  if(imuReady){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);
    headingFilter.predict(gyro.gyro.z - gyroZBias, dt);
    headingFilter.correct(pose.theta);
    odometry.setHeadingRad(headingFilter.getHeadingRad());
  }else{
    headingFilter.predict(0.0, dt);
    headingFilter.correct(pose.theta);
  }

}

void updateSensors(double dt){

  leftEncoder.updateRPM(dt);
  rightEncoder.updateRPM(dt);

  long leftDeltaPulses = leftEncoder.consumeDeltaPulses();
  long rightDeltaPulses = rightEncoder.consumeDeltaPulses();

  odometry.update(leftDeltaPulses, rightDeltaPulses, leftMotor.getDirection(), rightMotor.getDirection());
  updateHeadingFilter(dt);

}

void printRobotTelemetry(){

  Pose2D pose = odometry.getPose();

  Serial.print(F(">xCm: "));
  Serial.print(pose.x * 100.0);
  Serial.print(F(","));

  Serial.print(F("yCm: "));
  Serial.print(pose.y * 100.0);
  Serial.print(F(","));

  Serial.print(F("headingDeg: "));
  Serial.print(pose.theta * RAD_TO_DEG);
  Serial.print(F(","));

  Serial.print(F("leftRPM: "));
  Serial.print(leftEncoder.getCurrentRPM() * leftMotor.getDirection());
  Serial.print(F(","));

  Serial.print(F("rightRPM: "));
  Serial.print(rightEncoder.getCurrentRPM() * rightMotor.getDirection());
  Serial.print(F(","));

  Serial.print(F("autoStep: "));
  Serial.print(autoRunner.getActionIndex());
  Serial.print(F(","));

  Serial.print(F("autoTimeout: "));
  Serial.print((autoRunner.didTimeout() || autonomousHardStopped) ? 1 : 0);
  Serial.print(F(","));

  Serial.print(F("autoStarted: "));
  Serial.print(autonomousStarted ? 1 : 0);
  Serial.print(F(","));

  Serial.print(F("turnTargetDeg: "));
  Serial.print(autoRunner.getTurnTargetDeg());
  Serial.print(F(","));

  Serial.print(F("turnErrorDeg: "));
  Serial.println(autoRunner.getTurnErrorDeg());

}

void runAutonomous(double dt){

  if(autonomousHardStopped){
    drivetrain.stop();
    return;
  }

  if(!autonomousStarted){
    drivetrain.stop();
    autonomousStartDelayElapsed += dt;

    if(autonomousStartDelayElapsed < autonomousStartDelaySeconds){
      return;
    }

    odometry.reset(0.0, 0.0, 0.0);
    headingFilter.reset(0.0);
    autoRunner.reset(odometry.getPose());
    autonomousElapsed = 0.0;
    autonomousStarted = true;
  }

  autonomousElapsed += dt;
  if(autonomousElapsed >= maxAutonomousRuntimeSeconds){
    autonomousHardStopped = true;
    drivetrain.stop();
    return;
  }

  autoRunner.update(dt, odometry.getPose(), &drivetrain, &odometry);

}
//-------------------------------------------------

void runTest(double dt){
  /*
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 255);
  */

  //rightMotor.setMotorByPWM(50);
  //rightMotor.setMotorByPWM(255);

 // drivetrain.setDriveByPWM(-51 ,51); 

  //Serial.println(dt);
  //Serial.print(">");
  //Serial.print("currentRPM: ");
  //Serial.println(rightEncoder.getCurrentRPM());

  drivetrain.setDriveByRPM(200, 200, dt);
}


//-------------------------------------------------
void runManual(double dt){

  drivetrain.stop();

}

//-------------------------------------------------
enum ControlMode{
  TEST,
  MANUAL,
  AUTONOMOUS
};

ControlMode controlMode = AUTONOMOUS;
//-------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);
  printI2CScan();

  oledReady = initOled();
  Serial.print(F("OLED: "));
  Serial.print(oledReady ? F("OK 0x") : F("NOT FOUND 0x"));
  Serial.println(oledAddress, HEX);
  if(oledReady){
    oledPrintLine(0, "OLED OK");
  }

  imuReady = mpu.begin();
  if(imuReady){
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyroZ();
  }

  drivetrain.begin();
  leftMotor.setTelemetryEnabled(false);
  rightMotor.setTelemetryEnabled(false);

  odometry.begin(wheelDiameterM, effectiveTrackWidthM, leftEncoder.getPulsesPerRev());
  headingFilter.reset(0.0);
  

  attachInterrupt(digitalPinToInterrupt(LEncoderPin), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REncoderPin), rightEncoderISR, FALLING);

  unsigned long currentTime = micros();
  sensorTask.begin(sensorUpdatePeriod, currentTime);
  controlTask.begin(controlUpdatePeriod, currentTime);
  telemetryTask.begin(telemetryUpdatePeriod, currentTime);
  oledTask.begin(oledUpdatePeriod, currentTime);

  autoRunner.begin(autoProgram, autoProgramLength);
  autoRunner.reset(odometry.getPose());
  autonomousElapsed = 0.0;
  autonomousStartDelayElapsed = 0.0;
  autonomousStarted = false;
  autonomousHardStopped = false;
}


void loop() {
 
  unsigned long currentTime = micros();

  //sense -> think -> act -> display
  //sensorInput -> process and calculate and predict -> set the actuator

  if(sensorTask.ready(currentTime)){
    
    double dt = sensorTask.consumeDt(currentTime);
    updateSensors(dt);
  }

  if(controlTask.ready(currentTime)){

    double dt = controlTask.consumeDt(currentTime);
    switch(controlMode){

      case TEST:
        runTest(dt);
        break;
      case MANUAL:
        runManual(dt);
        break;
      case AUTONOMOUS:
        runAutonomous(dt);
        break;

    }
  }

  if(telemetryTask.ready(currentTime)){

    telemetryTask.consumeDt(currentTime);
    printRobotTelemetry();

  }

  if(oledTask.ready(currentTime)){

    oledTask.consumeDt(currentTime);
    updateOledDebug();

  }
 
}
