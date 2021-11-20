
//Left motor
const int D10 = 10;
const int D5 = 5;
//Right motor
const int D6 = 6;
const int D9 = 9;
//PWM on opposites Q3 and Q5
const int Pmw_LM = 5;
const int Pmw_RM = 6;

void setup() {
  // setting up the pins , serial port and prompting user for input to drive the bot 
Serial.begin(9600);
pinMode(D10,OUTPUT);
pinMode(D5,OUTPUT);
pinMode(D6,OUTPUT);
pinMode(D9,OUTPUT);
Serial.println(" Option 1: Enter a value between 0 and 9 ");
Serial.println(" Option 2: Enter a + value to set the direction of the bot to a Clock Wise rotation");
Serial.println(" Option 3: Enter a - value to set the direction of the bot to a Counter Clock Wise rotation");
}

void loop() {
  // reading input from the serial terminal
if (Serial.available()){  
  char M_request = Serial.read();
 if (isDigit(M_request))
  { 
      //if the response is a value from 0 to 1 the bot drives forward in linear motion for 5 second at the given speed.
    int speed = map(M_request,'0','9',0,255); // scalling respoce value between 0 and  (need to be improved )
    //applying PMW on the left motor
    analogWrite(Pmw_LM,speed);
     //applying PMW on the right motor
    analogWrite(Pmw_RM,speed);
    Serial.println(speed);
    //delay(5000);
    }
  else if (M_request =='-'){
    //if the response is a value - the bot rotate towards left 
    Serial.println("Counter Clock Wise");
  digitalWrite(D10,LOW) ;
  digitalWrite(D5,HIGH) ;
  digitalWrite(D6,LOW) ;
  digitalWrite(D9,HIGH) ;
  delay(1000);
  }
  else if (M_request=='+'){
     //if the response is a value + the bot rotate towards right 
    Serial.println(" Clock Wise");
    digitalWrite(D10,HIGH) ;
    digitalWrite(D5,LOW) ;
    digitalWrite(D6,HIGH) ;
    digitalWrite(D9,LOW) ;
     delay(1000);
    }
  else{
     //if the response is any other keyboard value - the motors stop rotating
    Serial.println("Stoped Motors");
    digitalWrite(D10,LOW) ;
    digitalWrite(D5,LOW) ;
    digitalWrite(D6,LOW) ;
    digitalWrite(D9,LOW) ;
  }
  }
}
