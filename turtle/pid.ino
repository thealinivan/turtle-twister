//IMPORTS - ensure libraries are installed
#include <PID_v1.h>
#include <Wire.h>
//GLOBAL DATA 

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,.5,.5,.5, DIRECT);

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

void setup()
{
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Setpoint = 100;
  Input = getEncoderSpeedRightMotor();
  myPID.Compute();
  analogWrite(10,Output);
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(Output);
}

//Interrupts
//  read pulse and update count
void interruptR() { countR++; }

int getEncoderSpeedRightMotor() {
  unsigned long currT = millis();
  if (currT >= nextTR){
      noInterrupts();
      int currentCount = countR;
      interrupts();
      int encoderSpeed = currentCount - prevCR; // ticks/100ms
      prevCR = currentCount;
      nextTR = currT + period;  
      //Serial.print(rightMotorSetPoint);
      //Serial.print(" ");
      //Serial.println(encoderSpeed);        
      return encoderSpeed;
  } else {
    return 0;
  } 
}
