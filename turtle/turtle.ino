//IMPORTS - ensure libraries are installed
#include <PID_v1.h>

//GLOBAL DATA - variables that can be used in any method

//twist message - access method (velocity.linear.x / velocity.angular.z)
struct velocity {
  struct linear {float x = 0.05; float y = 0; float z = 0;};
  struct angular {float x = 0; float y = 0; float z = 3;};
};
double linearX = 0.5;
double angularZ = 3;

//pid - variables related to pid
double leftMotorSetPoint, leftMotorInput, leftMotorOutput;
double rightMotorSetPoint, rightMotorInput, rightMotorOutput;
int Kp=.5, Ki=.5, Kd=.5;
PID leftMotorPID(leftMotorSetPoint, leftMotorInput, leftMotorOutput, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(rightMotorSetPoint, rightMotorInput, rightMotorOutput, Kp, Ki, Kd, DIRECT);

//interrupts


//encoder counts


//motor rotation direction


void setup() {
  //print
  Serial.begin(9600);
  //pid - setup related to pid
  
  //encoder counts
  
  //motor rotation direction
  
}

void loop() {
}

//ENCODER CONTS - return encoder counts


//MOTOR ROTATION DIRECTION - return rotation direction


//PID - run a specific motor
void runPID(int motor_encoder_counts, int motor_rotation_direction, PID motor_PID){
  
}


//MAP DATA FOR LINEAR MOVEMENT
//MAP DATA FOR ANGULAR MOVEMENT
//RUN LEFT MOTOR
//RUN RIGHT MOTOR
