//IMPORTS - ensure libraries are installed
#include <PID_v1.h>

//GLOBAL DATA 

//twist message variables;
double linearX = 4, angularZ = .8;

//speed limits
double x_val_limit = .5, z_val_limit = 5;

//motors
const int RIN1 = 5;  // R ACW
const int RIN2 = 10; // R CW
const int LIN1 = 6;  // L CW
const int LIN2 = 9;  // L ACW

//linear measurements
const int ticksPerWR = 850;
const double wheelCirc = .207;
const double angRadius = .17;
const float Pi = 3.14159;
const double circleCirc = angRadius*Pi;
const int maxEnC = 216;

//encoders
const int REN1 = 3; // R
const int REN2 = 7; // R
const int LEN1 = 4; // L
const int LEN2 = 2; // L
volatile int prevC = 0, count;
unsigned long period = 100, nextT = period;
int ECLinear = 0; //ticks/100ms  .1m
int ECAngL = 0; //ticks/100ms .1rad
int ECAngR = 0; 

//pid - variables related to pid
double leftMotorSetPoint, leftMotorInput, leftMotorOutput;
double rightMotorSetPoint, rightMotorInput, rightMotorOutput;
const int Kp=5, Ki=3, Kd=3;
PID leftMotorPID(&leftMotorSetPoint, &leftMotorInput, &leftMotorOutput, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&rightMotorSetPoint, &rightMotorInput, &rightMotorOutput, Kp, Ki, Kd, DIRECT);

//wobble
unsigned long wobbleInterval = 500, nextWobbleTime = wobbleInterval;

//SETUP
void setup() {
  //print
  Serial.begin(9600);
  //pins
  pinMode(REN1, INPUT_PULLUP);
  pinMode(REN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  pinMode(LEN2, INPUT_PULLUP);
  //pid - setup related to pid
  rightMotorSetPoint = ECLinear;
  leftMotorSetPoint = ECLinear;
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
  //moveTurtle(linearX, angularZ);                   
  wobbleTurtle(linearX, angularZ, wobbleInterval);
}

//Wobble turtle left and right args| wi: positive ms 
void wobbleTurtle(double linear_x, double angular_z, int wobble_interval) {
  linearX = linear_x;
  angularZ = angular_z;
  unsigned long currentTime = millis();
  if (currentTime >= nextWobbleTime){
    angularZ *= -1;
    nextWobbleTime = currentTime + wobble_interval;
    Serial.println(angularZ);
  } 
  moveTurtle(linearX, angularZ);
}

// Move Turtle args| x: -.5 to .5; y: -5 to 5
void moveTurtle(double linear_x, double angular_z) {
  setVelocityLimits(x_val_limit, z_val_limit);
  setAngularDirection(linear_x, angular_z);
  setLinearDirection(linear_x); 
}

//Speed Limits
void setVelocityLimits(double linear_limit, double angular_limit){
  if (linearX > linear_limit) {linearX = linear_limit;}
  else if (linearX < -linear_limit) {linearX = -linear_limit;}
  if (angularZ > angular_limit) {angularZ = angular_limit;}
  else if (angularZ < -angular_limit) {angularZ = -angular_limit;}
}
//Angular direction
void setAngularDirection(double lx, double az) {
  if (az > 0) {
    if (lx >= 0){
      ECAngL = az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; // rad/100ms
      ECAngR = 0;
    } else {
      ECAngL = 0;
      ECAngR = az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; // rad/100ms;
    }
  } else if (az < 0) {
    if (lx >= 0) {
      ECAngR = -az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; // rad/100ms
      ECAngL = 0;
    } else {
      ECAngR = 0;
      ECAngL = -az*(ticksPerWR*(circleCirc/wheelCirc)/(2*Pi))/10; // rad/100ms
    }
  } else {
    ECAngL = 0;
    ECAngR = 0;
  }
}
//Linear direction
void setLinearDirection(double lx){
  if (lx > 0) {
    ECLinear = lx*(ticksPerWR/wheelCirc)/10; // ticks/100ms
    moveForward();
  } else if (lx < 0) {
      ECLinear = -lx*(ticksPerWR/wheelCirc)/10; // ticks/100ms 
      moveBackward(); 
  } else {
    stopTurtle();
  }
}


//MOTORS

//Both Motors
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
//  leftMotorInput = leftMotorSetPoint - getEncoderSpeed();
//  leftMotorPID.Compute();
  int ms = map(ECLinear + ECAngL/2 - ECAngR/2, 0, maxEnC, 0, 255);
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

//Right Motor
void rightMotor(char d){
  int ms = map(ECLinear - ECAngL/2 + ECAngR/2, 0, maxEnC, 0, 255);
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
