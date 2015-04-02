// Project1Week3.ino
// BDK:2015-02-11
//
// declare constants to identify which pins
// we will use for inputs and outputs
// declarations outside of a function have global scope
// (usually best to avoid using global variables)
//
#include <stdio.h>
const int JoystickPin = A0; // Joystick Input
const int bbOrangePin = A5; // Black Box Orange Wire (Output From BB)
const int bbIndigoPin = 3;  // Black Box Indigo Wire (Input to BB) (Must be PWM Capable Pin)
float counter = 0;
static unsigned long microsLast = 0; // static = keep value next time function is called
unsigned long time ; 
unsigned long time_prev =0;
int JoystickRaw;
float JoystickV;
int bbOrangeRaw;
float bbOrangeV;
int bbIndigoRaw;
float bbIndigoV;
float deltaT = 0.005; // time between digital updates
float Kp = .1;//1.5;
float Kd = 0.0; //Kp*0.02; //Kd = 0;//
float Ki = 0.0;//0.01;
float errorV;
static float errorVlast = 0.0;
float errorVdot = 0.0;
float errorVint = 0.0;
float offset = 0.5;
unsigned long dt; 

//
// This is everything that has to happen once to get system ready
//
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200); 
  pinMode(bbOrangePin,OUTPUT);
  pinMode(5,OUTPUT);
}

//
// This is stuff that happens over and over again!
//
void loop() {
//  static unsigned long microsLast = 0; // static = keep value next time function is called
//  int JoystickRaw;
//  float JoystickV;
//  int bbOrangeRaw;
//  float bbOrangeV;
//  int bbIndigoRaw;
//  float bbIndigoV;
//  float deltaT = 0.02; // time between digital updates
//  float Kp = 1.5;
//  float Kd = Kp*0.02; //Kd = 0;//
//  float Ki = 0.01;
//  float errorV;
//  static float errorVlast = 0.0;
//  float errorVdot = 0.0;
//  float errorVint = 0.0;
//  float offset = 2.0;

  
//
// Read the Joystick Value & Convert to Voltage
// DON'T EDIT THIS
//
  JoystickRaw = analogRead(JoystickPin);
  JoystickV = (5.0*JoystickRaw)/1023.0;
//

//
// Read the bbOrange Value & Convert to Voltage
// DON'T EDIT THIS
//
  bbOrangeRaw = analogRead(bbOrangePin);
  bbOrangeV = (5.0*bbOrangeRaw)/1023.0;
 
  
/////////////////////////////////////////////////////////////////
// Compensator Goes Here -- EDIT THIS
/////////////////////////////////////////////////////////////////
  time_prev = time; 
  time = millis();
  dt = time - time_prev;
  
  errorV = (JoystickV-(bbOrangeV));
  errorVdot = (errorV-errorVlast)/deltaT; 
  errorVint = (errorV+errorVlast)*deltaT;
  bbIndigoV = offset + Kp*errorV + Kd*errorVdot + Ki*errorVint;  // PID control
  errorVlast = errorV;

 
  counter ++; 
  // print the value to the serial window
// you can add commands like this for debugging
// too much printing slows down execution
// 

Serial.print("In = ");
Serial.print(JoystickV, DEC);
Serial.print(" Out = ");
Serial.print(bbOrangeV, DEC);
Serial.print(" Time : "); 
Serial.print(dt,DEC); 
Serial.print(" InRaw : "); 
Serial.print(bbIndigoRaw,DEC); 
Serial.print('\n'); 

/////////////////////////////////////////////////////////////////
// End of Compensator Code
/////////////////////////////////////////////////////////////////

//
// Write the bbIndigo Value After Converting to Raw
// DON'T EDIT THIS
//
  bbIndigoRaw = constrain(255*bbIndigoV/5,0,255);
analogWrite(3, bbIndigoRaw);
//analogWrite(3,127);
analogWrite(5,67);

  while (micros() < microsLast + deltaT*1000000);  // wait for deltaT since last time through
  microsLast = micros(); // save value for next time through
 /////////////////////////////////////////////////////////////////////////
}
