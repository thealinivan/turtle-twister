//IMPORTS - ensure libraries are installed
#include <PID_v1.h>

//GLOBAL DATA - variables that can be used in any method

//twist message data
double linearX = .2;
double angularZ = -2;

//motors
const int RIN1 = 5;  // R ACW
const int RIN2 = 10; // R CW
const int LIN1 = 6;  // L CW
const int LIN2 = 9;  // L ACW

//linear measurements
const int ticksPerWheelRotation = 850;
const double wheelCircumference = .207;
const double angRadius = .17;
const float Pi = 3.14159;
const double circleCircumference = angRadius*Pi;
const int maxEnC = 216;

//encoders
const int REN1 = 3; // R
const int REN2 = 7; // R
const int LEN1 = 4; // L
const int LEN2 = 2; // L
volatile int prevC = 0, count;
unsigned long period = 100, nextT = period;
int linearSpeedEncoderCount = 0; //ticks/100ms  .1m
int leftMotorAngularSpeedEncoderCount = 0; //ticks/100ms .1rad
int rightMotorAngularSpeedEncoderCount = 0; 

//pid - variables related to pid
double leftMotorSetPoint, leftMotorInput, leftMotorOutput;
double rightMotorSetPoint, rightMotorInput, rightMotorOutput;
const int Kp=5, Ki=3, Kd=3;
PID leftMotorPID(&leftMotorSetPoint, &leftMotorInput, &leftMotorOutput, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&rightMotorSetPoint, &rightMotorInput, &rightMotorOutput, Kp, Ki, Kd, DIRECT);

//SETUP
void setup() {
  if (angularZ > 0) {
    leftMotorAngularSpeedEncoderCount = angularZ*(ticksPerWheelRotation*(circleCircumference/wheelCircumference)/(2*Pi))/10;
  } else if (angularZ < 0) {
    rightMotorAngularSpeedEncoderCount = -angularZ*(ticksPerWheelRotation*(circleCircumference/wheelCircumference)/(2*Pi))/10; 
  } else {
    leftMotorAngularSpeedEncoderCount = 0;
    rightMotorAngularSpeedEncoderCount = 0;
  }
  //print
  Serial.begin(9600);
  //pins
  pinMode(REN1, INPUT_PULLUP);
  pinMode(REN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  //pid - setup related to pid
  rightMotorSetPoint = linearSpeedEncoderCount;
  leftMotorSetPoint = linearSpeedEncoderCount;
  rightMotorPID.SetTunings(Kp, Ki, Kd);
  leftMotorPID.SetTunings(Kp, Ki, Kd);
  rightMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetMode(AUTOMATIC);
  
  //encoder counts
  //interrups
  attachInterrupt(0, interruptR, CHANGE);
  attachInterrupt(1, interruptL, CHANGE); 
}

//LOOP
void loop() {
  if (linearX > 0) {
    linearSpeedEncoderCount = linearX*(ticksPerWheelRotation/wheelCircumference)/10; // ticks/100ms
    moveForward();
  } else if (linearX < 0) {
      linearSpeedEncoderCount = -linearX*(ticksPerWheelRotation/wheelCircumference)/10; // ticks/100ms 
      moveBackward(); 
  } else {
    stopTurtle();
  }
  getEncoderSpeed();
}

//INTERRUPTS
// read pulse and update count
void interruptR() { 
      if (digitalRead(REN1)==digitalRead(REN2)) { --count; } 
      else { ++count;}
}
void interruptL() { 
      if (digitalRead(REN1)==digitalRead(REN2)) { ++count; } 
      else { --count;}
}

//ENCODER COUNT
// return ecoder speed ticks/100ms
int getEncoderSpeed() {
  unsigned long currT = millis();
  if (currT >= nextT){
      noInterrupts();
      int currentCount = count;
      interrupts();
      int encoderSpeed = currentCount - prevC;  // ticks/100ms
      prevC = currentCount;
      nextT = currT + period;
      Serial.println(encoderSpeed);
      Serial.print(" "); 
      return encoderSpeed;
  } else {
    return 0;
  } 
}

//RUN LEFT MOTOR
 void leftMotor(char d){
//  leftMotorInput = leftMotorSetPoint - getEncoderSpeed();
//  leftMotorPID.Compute();
  int ms = map(linearSpeedEncoderCount + leftMotorAngularSpeedEncoderCount/2 - rightMotorAngularSpeedEncoderCount/2, 0, maxEnC, 0, 255);
  if (d=='L'){
    analogWrite(LIN1, 0);
    analogWrite(LIN2, ms);
  } else if (d=='R') {
    analogWrite(LIN1, ms);
    analogWrite(LIN2, 0);
  } else {
    analogWrite(LIN1, 0);
    analogWrite(LIN2, 0);
  }
 
}

//RUN RIGHT MOTOR
void rightMotor(char d){
  int ms = map(linearSpeedEncoderCount - leftMotorAngularSpeedEncoderCount/2 + rightMotorAngularSpeedEncoderCount/2, 0, maxEnC, 0, 255);
  if (d=='R'){
    analogWrite(RIN1, 0);
    analogWrite(RIN2, ms);
  } else if (d=='L') {
    analogWrite(RIN1, ms);
    analogWrite(RIN2, 0);
  } else {
    analogWrite(RIN1, 0);
    analogWrite(RIN2, 0);
  }

}

//RUN BOTH MOTORS
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
