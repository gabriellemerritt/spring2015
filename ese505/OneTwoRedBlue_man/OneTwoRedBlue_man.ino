
// Headers contain device drivers
#include <stdio.h> 
#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>

// Types
// Pixy is an object type defined in Pixy header file
// Servo is an object type defined in Servo header file
Pixy PingPongPixy;
Servo PingPongServo;

//==================
// Program constants
//==================
// Arduino pins used
const int PingPongServoPin = 3;
const int JoystickPin = A0;
const int ServoDisabledPin = 6;
const int bound = 120; 
const double true_zero = 0.05; 
const double RBzero = true_zero; 
const double RBone = -0.25; 
const double RBtwo =  0.25; 
const double RBRed = -0.45; 
const double RBBlue = 0.45; 
// Servo constants
const int ServoCenter_us =  1347; //higher gets closer to red, lower closer to blue
const int ServoScale_us = 750;
const unsigned long deltaT = 20; // time between digital updates (20ms = PIXY & Servo update rates)
//==========================================
// Controller variables, gains and constants
//==========================================
// Gains
float Tu = (2.6); // period in seconds
float Ku = 0.03; //0.04; .
float Kp = 0.05;//0.17;//.15//0.3; //0.04;//.6*Ku; //.1
float Ki = 0.0011; //.5*Tu; 
float Kd = 5;//7;//10; //7//.1*Tu; This is most liekly too high  //5
//float Kp = 0.2; //0.3; //0.04;//.6*Ku;
//float Ki = .02; //.5*Tu; 
//float Kd = .03;//.1*Tu;
// Controller variables
static float error;
static float error_prev = 0; 
float deadband = 0;//0.01;
float deadband_kd = 0;//0.0005;
static float error_kp = 0.0;

static float error_dot; 
static float error_dot_prev = 0; 
static float error_dot_kd = 0.0;

static float error_int; 
static float xBall_des; 
static float xBall_old; 
int SPt = 0; //initial set point = 0
int SPt_prev = 0; //initial set point = 0

//====================
// Filtering variables
//====================
float beta1 = 0.9; //error filter
float beta2 = 0.92; //error_dot filter

//====================
// Program variables
//====================
int counter = 0; 
unsigned long t_switch; 
unsigned long t_ms = 0;
unsigned long t_ms_prev = 0; 
unsigned long dt; 
unsigned long cycles = 0;
unsigned long t_full_cycle_ms = 30000;

// Initalization routine
void setup()
{
  // Serial baud rate
  Serial.begin(115200);
  
  // Servo attached to Arduino digital pin 3
  PingPongServo.attach(3);
  PingPongServo.writeMicroseconds(ServoCenter_us);
  
  // initialize the Pixy camera
  PingPongPixy.init();
  
  // Monitor Servo status as digital input
  pinMode(ServoDisabledPin,INPUT);
  
}

// Main program loop
void loop()
{ 
  counter++; 
   
  static unsigned long microsLast = 0; // elapsed time at end of last call to loop
  word JoystickRaw = 512;
  double JoystickOne;
  static int xBallRawLast = 2;  // start with max left position as indication of failed PIXY read
  int xBallRaw;
  double xBallOne = 0.0;
  double ServoOne = 0.0;
  double t_us;
  double Joystick_Rate; 
  boolean ServoDisabled;
   
  // Get the joystick position, Convert to +/- 1.0 relative to center
  JoystickRaw = analogRead(JoystickPin);
  JoystickOne = -1.0 + (2*JoystickRaw)/1023.0;
  
  // Get the ball position, Convert to +/- 1.0
  // negative value indicates no ball found
  xBallRaw = PingPongCheck();
  if (xBallRaw < 0){
    xBallRaw = xBallRawLast;
  } else {
    xBallRawLast = xBallRaw;
  }
  xBallOne = -1.0 + (2.0*xBallRaw)/319.0;  // 319 = max pixel value in PIXY camera

  // Check if the servo is disabled, may use this logic in control law
  ServoDisabled = digitalRead(ServoDisabledPin);
  
  //////////////////////////////////////////////////////////////////
  // Controller
  //////////////////////////////////////////////////////////////////
  //Choose the new set point based on time.
  SPt_prev = SPt;
  t_switch = millis()-cycles*t_full_cycle_ms;
  if ( t_switch <= t_full_cycle_ms/5) { 
    SPt = 0; 
    if (SPt_prev != SPt){
      error_int = 0;
    }
  }  else if ( t_switch <= 2*t_full_cycle_ms/5) { //Move to Set Point 2 after 4 seconds
    SPt = 1; 
    if (SPt_prev != SPt){
      error_int = 0;
    }
  }  else if (t_switch <= 3*t_full_cycle_ms/5) { //Move to Set Point 3 after 8 seconds
    SPt = 2; 
    if (SPt_prev != SPt){
      error_int = 0;
    }
  }  else if (t_switch <= 4*t_full_cycle_ms/5) { //Move to Set Point 4 after 12 seconds
    SPt = 3;
    if (SPt_prev != SPt){
      error_int = 0;
    }
  }  else if (t_switch <= t_full_cycle_ms) { //Move to Set Point 0 after 12 seconds
    SPt = 4; 
    if (SPt_prev != SPt){
      error_int = 0;
    }
  }  else { //Move to Set Point 0 after 12 seconds
    SPt = 0; 
    if (SPt_prev != SPt){
      error_int = 0;
    }
    cycles++; //Move to back to Set Point 0 after 12 seconds
  } 
  
  // Given a set point, determine the Set Position relative to center of rack
  // Center = 0, ends = +/- 1
  switch(SPt) {
    case 0: 
      xBall_des = RBzero;
      break;
    case 1: 
      xBall_des = RBone;
      break; 
    case 2: 
      xBall_des = RBtwo; 
      break; 
    case 3:
      xBall_des = RBRed; 
      break;
    case 4: 
      xBall_des = RBBlue; 
      break;
  }
 
  
  //read error in position, position_error = desired_pos - measured_pos
  //filter the sensor data stream
//  error = error*beta1 + (1-beta1)*(xBall_des - xBallOne); //filtered measured error 
  error = error*beta1 + (1-beta1)*(JoystickOne - xBallOne); //filtered measured error 
  
  //put a deadband on the response of the proportional control
  if (abs(error)<=deadband){
    error_kp = 0;
  } else if (error > 0) {
    error_kp = error-deadband;
  } else {
    error_kp = error+deadband;
  }
  
  //calculate the integral of the error
  error_int = error_int + error;
  
  //calculate the velocity of the error and filter
  error_dot = (xBall_old-xBallOne)*(1-beta2)+beta2*error_dot;
  
  //put a deadband on the response of the derivative control
  if (abs(error_dot)<=deadband_kd){
    error_dot_kd = 0;
  } else if (error_dot > 0) {
    error_dot_kd = error_dot-deadband_kd;
  } else {
    error_dot_kd = error_dot+deadband_kd;
  }

  //anti integral windup, reset periodically
  if(abs(error_int) >=100){
    error_int = 0;
  }
  
  //control equation
  ServoOne = Kp*error_kp+ Ki*error_int + Kd*error_dot_kd;  
  
  //record values for next loop
  xBall_old = xBallOne;
  error_prev = error; 
  
  //////////////////////////////////////////////////////////////////
  // Control Law Ends Here
  //////////////////////////////////////////////////////////////////
  
  // convert ServoOne to MicroSeconds, then send the command to the servo
  // smaller values of t_us tilt right (positive sense of ServoOne)
  // DON'T CHANGE LIMITS IN CONSTRAIN FUNCTION -- CAN DAMAGE SERVO
  if (ServoDisabled){
    t_us = constrain(ServoCenter_us, ServoCenter_us-bound, ServoCenter_us + bound);
  } else {
    t_us = constrain(ServoCenter_us - ServoScale_us * ServoOne, ServoCenter_us-bound, ServoCenter_us+bound);
  }
  PingPongServo.writeMicroseconds(t_us);

  // Send serial output 
  Serial.print("err: "); 
  Serial.print(error,4); 
  Serial.print(", "); 

//  Serial.print("errd: "); 
//  Serial.print(error_dot,4); 
//  Serial.print(", "); 
//  
//  Serial.print("erri:"); 
//  Serial.print(error_int,4);
//  Serial.print(",");  
  
  Serial.print("JS:"); 
  Serial.print(JoystickOne);
  Serial.print(",");  
  
  
 
  Serial.println(".");  
  
  // Force a constant loop time
  t_ms = millis(); 
  dt = t_ms - t_ms_prev; 
  while(dt < deltaT) // wait for deltaT since last time through
  {  
    t_ms = millis(); // save value for next time through
    dt = t_ms - t_ms_prev; 
  }
  t_ms_prev = t_ms; 
  
}
  
word PingPongCheck(){
  int xpos=-1;
  uint16_t nblocks;

  nblocks = PingPongPixy.getBlocks();  
  if (nblocks)
  {
    xpos = PingPongPixy.blocks[0].x;
  }
  return xpos;
}

void SimPlot(int data1, int data2, int data3, int data4){
  int pktSize;
  int buffer[20];  
  buffer[0] = 0xCDAB;             //SimPlot packet header. Indicates start of data packet
  buffer[1] = 4*sizeof(int);      //Size of data in bytes. Does not include the header and size fields
  buffer[2] = data1;
  buffer[3] = data2;
  buffer[4] = data3;
  buffer[5] = data4;
  pktSize = 2 + 2 + (4*sizeof(int)); //Header bytes + size field bytes + data
  Serial.write((uint8_t * )buffer, pktSize);
}
