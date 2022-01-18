// Imports - ensure libraries are installed
#include <PID_v1.h>
#include <Wire.h>

// Motors
const int LIN1 = 6;  // L CW
const int LIN2 = 9;  // L ACW
const int RIN1 = 5;  // R ACW
const int RIN2 = 10; // R CW

// Encoders
const int LEN1 = 3; // L
const int LEN2 = 7; // L
volatile int prevCL = 0, countL = 0;
unsigned long period = 100, nextTL = period;

const int REN1 = 4; // R
const int REN2 = 2; // R
volatile int prevCR = 0, countR = 0;
unsigned long nextTR = period;

int currEncL = 0, currEncR = 0;

int ECLinear = 0; // target ticks/100ms linear speed
int ECAngL = 0; // target ticks/100ms angular speed left motor
int ECAngR = 0; // target ticks/100ms angular speed right motor
int ECAngBoth = 0; // target ticks/100ms angular speed common for both motors

// Linear measurements
const int ticksPerWR = 852;
const double wheelCirc = .2;
const double angDiameter = .165;
const float Pi = 3.14159265259;
const double circleCirc = angDiameter*Pi;
const double minEncCount = 0;   // ticks/100 milliseconds
const double maxEncCount = 216; // ticks/100 milliseconds


// PID
double Kp=.3, Ki=.6, Kd=.3;
double SetpointL, InputL, OutputL, SetpointR, InputR, OutputR;
PID leftPID(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID rightPID(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

// SETUP
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

// LOOP
void loop()
{
  moveTurtle(.4, 0); 
}

// Limit encoder counts
void getEC(int ec){
  if (ec > maxEncCount) { return maxEncCount; } 
  else { return ec; }
}

// Move Turtle
void moveTurtle(double linear_x, double angular_z) {
  setAngular(linear_x, angular_z);
  setLinear(linear_x, angular_z); 
}

// Angular direction & velocity
void setAngular(double lx, double az) {
  int ECAng = az*((ticksPerWR*(circleCirc/wheelCirc)/(2*Pi)))/10; // ticks/100ms for az velocity
  if (az > 0) {
    if (lx >= 0){
      ECAngL = ECAng;
      ECAngR = 0;
      ECAngBoth = 0;
    } else if (lx < 0) {
      ECAngL = 0;
      ECAngR = ECAng;
      ECAngBoth = 0;
    } else if (lx == 0){
      ECAngL = 0;
      ECAngR = 0;
      ECAngBoth = ECAng/2;
    }
  } else if (az < 0) {
    if (lx > 0) {
      ECAngL = 0;
      ECAngR = ECAng;
      ECAngBoth = 0;
    } else if (lx < 0) {
      ECAngL = ECAng;
      ECAngR = 0;
      ECAngBoth = 0;
    } else if (lx == 0){
      ECAngL = 0;
      ECAngR = 0;
      ECAngBoth = ECAng/2;
    }
  } else {
    ECAngL = 0;
    ECAngR = 0;
  }
}

// Linear direction &  velocity
void setLinear(double lx, double az){
  ECLinear = lx*(ticksPerWR/wheelCirc)/10; // ticks/100ms for lx velocity
  if (ECLinear > 0) {
    leftMotor('L');
    rightMotor('R');
  } else if (ECLinear < 0){
    leftMotor('R');
    rightMotor('L');
  } else if (ECLinear == 0) {
    if (az > 0){
      leftMotor('L');
      rightMotor('L');
    } else if (az < 0) {
      leftMotor('R');
      rightMotor('R');
    } else {
      leftMotor("stop");
      rightMotor("stop");
    }
  }
}

// Left Motor
 void leftMotor(char d){ 
  SetpointL = ECLinear + ECAngL/2 - ECAngR/2 + ECAngBoth;
  InputL = getEncoderSpeed('L'); // 
  leftPID.Compute();
  //Serial.print(SetpointL); // the target speed (enconder counts / 100 milisenconds)
  //Serial.print(" ");
  //Serial.println(InputL); // current speed (enconder counts / 100 milisenconds)
  //Serial.print(" ");
  if (d=='L'){
    analogWrite(LIN1, 0);
    analogWrite(LIN2, OutputL);
  } else if (d=='R') {
    analogWrite(LIN1, OutputL);
    analogWrite(LIN2, 0);
  } else {
    analogWrite(LIN1, 0);
    analogWrite(LIN2, 0);
  }
}

// Right Motor
void rightMotor(char d){
  SetpointR = ECLinear - ECAngL/2 + ECAngR/2 + ECAngBoth;
  InputR = getEncoderSpeed('R');
  rightPID.Compute();
  Serial.print(SetpointR);
  Serial.print(" ");
  Serial.println(OutputR);
  if (d=='R'){
    analogWrite(RIN1, 0);
    analogWrite(RIN2, OutputR);
  } else if (d=='L') {
    analogWrite(RIN1, OutputR);
    analogWrite(RIN2, 0);
  } else {
    analogWrite(RIN1, 0);
    analogWrite(RIN2, 0);
  }
}

// Interrupts - read pulse and update count
void interruptR() { countR++; }
void interruptL() { countL++; }

// Ecoder speed - return ticks/100ms
int getEncoderSpeed(char motor) {
  unsigned long currT = millis();
  if (motor=='L') {
    if (currT >= nextTL){
      noInterrupts();
      int currentCount = countL;
      interrupts();
      currEncL = currentCount - prevCL; // ticks/100ms
      prevCL = currentCount;
      nextTL = currT + period; 
      return currEncL;
    } else { return currEncL; } 
  } else if (motor=='R'){
     if (currT >= nextTR){
      noInterrupts();
      int currentCount = countR;
      interrupts();
      currEncR = currentCount - prevCR; // ticks/100ms
      prevCR = currentCount;
      nextTR = currT + period;      
      return currEncR;
     } else { currEncR; } 
  } else { return 0; }  
}
