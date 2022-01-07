//IMPORTS - ensure libraries are installed
#include <PID_v1.h>
#include <Wire.h>

//GLOBAL DATA 

//motors
const int RIN1 = 5;  // R ACW
const int RIN2 = 10; // R CW
const int LIN1 = 6;  // L CW
const int LIN2 = 9;  // L ACW

//encoders
const int REN1 = 4; // R
const int REN2 = 2; // R
volatile int prevCR = 0, countR;
unsigned long period = 100, nextTR = period;
const int LEN1 = 3; // L
const int LEN2 = 7; // L
volatile int prevCL = 0, countL;
unsigned long nextTL = period;
int ECLinear = 0; //ticks/100ms  .1m
int ECAngL = 0; //ticks/100ms .1rad
int ECAngR = 0; //ticks/100ms .1rad

//linear measurements
const int ticksPerWR = 852;
const double wheelCirc = .207;
const double angDiameter = .17;
const float Pi = 3.14159;
const double circleCirc = angDiameter*Pi;
const double maxEncCount = 216;

//pid - variables related to pid
double leftMotorSetPoint, leftMotorInput, leftMotorOutput;
double rightMotorSetPoint, rightMotorInput, rightMotorOutput;
const int Kp=5, Ki=2, Kd=3;
PID leftMotorPID(&leftMotorSetPoint, &leftMotorInput, &leftMotorOutput, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&rightMotorSetPoint, &rightMotorInput, &rightMotorOutput, Kp, Ki, Kd, DIRECT);

//SETUP
void setup() {
  Serial.begin(9600);
  pinMode(REN1, INPUT_PULLUP);
  pinMode(REN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  //pid
  rightMotorPID.SetTunings(Kp, Ki, Kd);
  leftMotorPID.SetTunings(Kp, Ki, Kd);
  rightMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetMode(AUTOMATIC);
  //interrups
  attachInterrupt(0, interruptL, CHANGE);
  attachInterrupt(1, interruptR, CHANGE); 
}

//LOOP
void loop() {
  moveTurtle(.5, 0);                     
}

// Move Turtle
void moveTurtle(double linear_x, double angular_z) {
  setAngular(linear_x, angular_z);
  setLinear(linear_x); 
}

//Angular direction & velocity
void setAngular(double lx, double az) {
  if (az > 0) {
    if (lx >= 0){
      ECAngL = az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; 
      ECAngR = 0;
    } else {
      ECAngL = 0;
      ECAngR = az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; 
    }
  } else if (az < 0) {
    if (lx >= 0) {
      ECAngR = -az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; 
      ECAngL = 0;
    } else {
      ECAngR = 0;
      ECAngL = -az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; 
    }
  } else {
    ECAngL = 0;
    ECAngR = 0;
  }
}
//Linear direction & velocity
void setLinear(double lx){
    ECLinear = lx*(ticksPerWR/wheelCirc)/10; // ticks/100ms
  if (ECLinear >= 0) {
    moveForward();
  } else if (ECLinear < 0){
    moveBackward();
  } else {
    stopTurtle();
  }
}

//Motors aggregation
void moveForward(){
  leftMotor('L');
  rightMotor('R');
}
void moveBackward(){
  leftMotor('R');
  rightMotor('L');
}
void stopTurtle(){
  leftMotor('stop');
  rightMotor('stop');
}

//Left Motor
 void leftMotor(char d){
  leftMotorSetPoint = ECLinear; //  + ECAngL/2 - ECAngR/2
  leftMotorInput = leftMotorSetPoint - getEncoderSpeedLeftMotor();
  leftMotorPID.Compute();
  if (d=='L'){
    analogWrite(LIN1, 0);
    analogWrite(LIN2, &leftMotorOutput);
  } else if (d=='R') {
    analogWrite(LIN1, &leftMotorOutput);
    analogWrite(LIN2, 0);
  } else {
    analogWrite(LIN1, 0);
    analogWrite(LIN2, 0);
  }
}

//Right Motor
void rightMotor(char d){
  rightMotorSetPoint = ECLinear; //  - ECAngL/2 + ECAngR/2
  rightMotorInput = rightMotorSetPoint - getEncoderSpeedRightMotor();
  rightMotorPID.Compute();
  if (d=='R'){
    analogWrite(RIN1, 0);
    analogWrite(RIN2, &rightMotorOutput);
  } else if (d=='L') {
    analogWrite(RIN1, &rightMotorOutput);
    analogWrite(RIN2, 0);
  } else {
    analogWrite(RIN1, 0);
    analogWrite(RIN2, 0);
  }
}

//Interrupts
//  read pulse and update count
void interruptR() { 
      countR++;
}
void interruptL() { 
      countL++;
}

//ENCODER COUNT
// return ecoder speed ticks/100ms
int getEncoderSpeedLeftMotor() {
  unsigned long currT = millis();
  if (currT >= nextTL){
      noInterrupts();
      int currentCount = countL;
      interrupts();
      int encoderSpeed = currentCount - prevCL; // ticks/100ms
      prevCL = currentCount;
      nextTL = currT + period;  
      return encoderSpeed;
  } else {
    return 0;
  } 
}

int getEncoderSpeedRightMotor() {
  unsigned long currT = millis();
  if (currT >= nextTR){
      noInterrupts();
      int currentCount = countR;
      interrupts();
      int encoderSpeed = currentCount - prevCR; // ticks/100ms
      prevCR = currentCount;
      nextTR = currT + period;  
      Serial.print(rightMotorSetPoint);
      Serial.print(" ");
      Serial.println(encoderSpeed);        
      return encoderSpeed;
  } else {
    return 0;
  } 
}
