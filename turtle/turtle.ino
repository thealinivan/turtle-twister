//IMPORTS - ensure libraries are installed
#include <PID_v1.h>

//GLOBAL DATA - variables that can be used in any method

//twist message - access method (velocity.linear.x / velocity.angular.z)
struct velocity {
  struct linear {float x = 0.05; float y = 0; float z = 0;};
  struct angular {float x = 0; float y = 0; float z = 3;};
};
const double linearX = 0.5;
const double angularZ = 3;

//motors
//right
const int AIN1 = 5;  // acw
const int AIN2 = 10; // cw
//left
const int BIN1 = 6;  // cw
const int BIN2 = 9;  // acw

//pid - variables related to pid
double leftMotorSetPoint, leftMotorInput, leftMotorOutput;
double rightMotorSetPoint, rightMotorInput, rightMotorOutput;
const int Kp=1, Ki=1, Kd=1;
PID leftMotorPID(&leftMotorSetPoint, &leftMotorInput, &leftMotorOutput, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&rightMotorSetPoint, &rightMotorInput, &rightMotorOutput, Kp, Ki, Kd, DIRECT);

//interrupts


//encoder counts


//motor rotation direction

//SETUP
void setup() {
  //print
  Serial.begin(9600);
  //pid - setup related to pid
  
  //encoder counts
  
  //motor rotation direction
  
}

//LOOP
void loop() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

//ENCODER CONTS
// return motor encoder counts to reach speed in m/s
double getMotorSpeedInEncoderCounts(double linear_x, double angular_z){

}

//MOTOR ROTATION DIRECTION
// return PWMA pin that sets direction (cw or acw)
int getMotorRotationDirection (double linear_x, double angluar_z) {
  
}

//PID - run a specific motor
void runMotor(int motor_encoder_counts, int motor_rotation_direction, PID motor_PID){
  
}

//MAP DATA FOR LINEAR MOVEMENT
//MAP DATA FOR ANGULAR MOVEMENT
//RUN LEFT MOTOR
//RUN RIGHT MOTOR
