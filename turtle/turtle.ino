//IMPORTS - ensure libraries are installed
#include <PID_v1.h>
#include <Wire.h>
//GLOBAL DATA 

//motors
const int LIN1 = 6;  // L CW
const int LIN2 = 9;  // L ACW
const int RIN1 = 5;  // R ACW
const int RIN2 = 10; // R CW

//encoders
const int LEN1 = 3; // L
const int LEN2 = 7; // L
volatile int prevCL = 0, countL = 0;
unsigned long period = 100, nextTL = period;

const int REN1 = 4; // R
const int REN2 = 2; // R
volatile int prevCR = 0, countR = 0;
unsigned long nextTR = period;

int ECLinear = 0; //ticks/100ms  .1m
int ECAngL = 0; //ticks/100ms .1rad
int ECAngR = 0; //ticks/100ms .1rad

//linear measurements
const int ticksPerWR = 852;
const double wheelCirc = .2;
const double angDiameter = .165;
const float Pi = 3.14159265259;
const double circleCirc = angDiameter*Pi;
const double minENcCount = 0;
const double maxEncCount = 213;

//pid - variables related to pid
double Setpoint, Input, Output;
PID leftPID(&Input, &Output, &Setpoint, .3, .6, .3, DIRECT);
PID rightPID(&Input, &Output, &Setpoint, .3, .6, .3, DIRECT);

//SETUP
void setup()
{
  Serial.begin(9600);
  pinMode(REN1, INPUT_PULLUP);
  pinMode(REN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  attachInterrupt(0, interruptL, CHANGE);
  attachInterrupt(1, interruptR, CHANGE); 
}

//LOOP
void loop()
{
  moveTurtle(.3, 0); 
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
//Linear direction &  velocity
void setLinear(double lx){
    ECLinear = lx*(ticksPerWR/wheelCirc)/10; // ticks/100ms
  if (ECLinear >= 0) {
    leftMotor('L');
    rightMotor('R');
  } else if (ECLinear < 0){
    leftMotor('R');
    rightMotor('L');
  } else {
     leftMotor('stop');
     rightMotor('stop');
  }
}

//Left Motor
 void leftMotor(char d){
  Setpoint = ECLinear + ECAngL/2 - ECAngR/2;
  Input = getEncoderSpeedLeftMotor();
  leftPID.Compute();
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(Output);
  if (d=='L'){
    analogWrite(LIN1, 0);
    analogWrite(LIN2, Output);
  } else if (d=='R') {
    analogWrite(LIN1, Output);
    analogWrite(LIN2, 0);
  } else {
    analogWrite(LIN1, 0);
    analogWrite(LIN2, 0);
  }
}

//Right Motor
void rightMotor(char d){
  Setpoint = ECLinear - ECAngL/2 + ECAngR/2;
  Input = getEncoderSpeedRightMotor();
  rightPID.Compute();
  if (d=='R'){
    analogWrite(RIN1, 0);
    analogWrite(RIN2, Output);
  } else if (d=='L') {
    analogWrite(RIN1, Output);
    analogWrite(RIN2, 0);
  } else {
    analogWrite(RIN1, 0);
    analogWrite(RIN2, 0);
  }
}


//Interrupts
//  read pulse and update count
void interruptR() { countR++; }
void interruptL() { countL++; }

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
      return encoderSpeed;
  } else {
    return 0;
  } 
}
