//
// Headers contain device drivers
//
#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>
#include <stdio.h> 

//
// Pixy is an object type defined in Pixy header file
// Servo is an object type defined in Servo header file
//
Pixy PingPongPixy;
Servo PingPongServo;

//
// here are the pins used
//
const int PingPongServoPin = 3;
const int JoystickPin = A0;
const int ServoDisabledPin = 6;
const double RBone = -.35; 
const double RBtwo = .21; 
const double RBRed = -.59; 
const double RBBlue = .5; 
int rb_flag =1 ; 
//
// servo constants -- YOU MAY CHANGE THE CENTER POSITION
//
const int ServoCenter_us =  1347; //higher gets closer to red, lower closer to blue
const int ServoScale_us = 750;
unsigned long t_switch; 
unsigned long time; 
unsigned long time_prev; 
float Kp = .08; 
float Kd = .165*Kp; 
float Ki =0.3*Kp; 
float errorXball;
float errorXballdot; 
float errorXballint; 
float xBall_des; 
unsigned long dt; 
unsigned long base_time = 0; 

void setup()
{
//
// Serial baud rate -- faster is usually better
//
  Serial.begin(57600);
//
// Servo attached to Arduino digital pin 3
//
  PingPongServo.attach(3);
  PingPongServo.writeMicroseconds(ServoCenter_us);
//
// initialize the Pixy camera
//
  PingPongPixy.init();
//
// Monitor Servo status as digital input
//
pinMode(ServoDisabledPin,INPUT);
}

void loop()
{ 
  t_switch = millis(); 
  static unsigned long microsLast = 0; // elapsed time at end of last call to loop
  double deltaT = 0.02; // time between digital updates (20ms = PIXY & Servo update rates)

  word JoystickRaw = 512;
  double JoystickOne;
  double JoystickOne_old;
  double JoystickRate;
  static int xBallRawLast = 2;  // start with max left position as indication of failed PIXY read
  int xBallRaw;
  double xBallOne = 0.0;
  double ServoOne = 0.0;
  double t_us;
  double Joystick_Rate; 
  boolean ServoDisabled;
  double xBallOld; 

//
// get the joystick position
// Convert to +/- 1.0 relative to center
//
  JoystickRaw = analogRead(JoystickPin);
  JoystickRate = -1.0 + (2*JoystickRaw)/1023.0;
  
//
// get the ball position
// negative value indicates no ball found
// convert to +/- 1.0 relative to camera fi  eld of view
//
  xBallRaw = PingPongCheck();
  if (xBallRaw < 0)
  {
    xBallRaw = xBallRawLast;
  }
  else
  {
    xBallRawLast = xBallRaw;
  }
  xBallOne = -1.0 + (2.0*xBallRaw)/319.0;  // 319 = max pixel value in PIXY camera

//
// check if the servo is disabled
// May use this logic in control laws
// for example, suspend integral feedback if disabled
//
  ServoDisabled = digitalRead(ServoDisabledPin);
  
//////////////////////////////////////////////////////////////////
// Control Law Starts Here
//////////////////////////////////////////////////////////////////
rb_flag =1; 
if ( t_switch > 3000){ 
  rb_flag = 2; 
} 
else if ( t_switch > 6000){ 
  rb_flag = 3; 
} 
else if (t_switch > 9000){ 
  rb_flag =4; 
} 

//  Joystick_Rate =   (JoystickRate - .03) ;
//  JoystickOne = Joystick_Rate; //* .1 + JoystickOne_old; 
  
//  ServoOne = 0.5*JoystickOne;  // pass Joystick command 
//  JoystickOne_old = JoystickOne;
switch(rb_flag){
  case 1: 
    xBall_des = RBone;
    break; 
  case 2: 
   xBall_des = RBone; 
    break; 
  case 3:
    xBall_des = RBone; 
    break;
  case 4: 
    xBall_des = RBone; 
    break;
}
 errorXball = ( xBall_des - xBallOne); 
 errorXballdot = (xBall_des - xBallOne)/dt; 
 errorXballint = (xBall_des + xBallOne)*dt;
ServoOne = Ki*errorXballint + Kd*errorXballdot + Kp*errorXball; 

//////////////////////////////////////////////////////////////////
// Control Law Ends Here
//////////////////////////////////////////////////////////////////

//
// convert ServoOne to MicroSeconds
// then send the command to the servo
// smaller values of t_us tilt right (positive sense of ServoOne)
// DON'T CHANGE LIMITS IN CONSTRAIN FUNCTION -- CAN DAMAGE SERVO
//
  if (ServoDisabled)
  {
    t_us = constrain(ServoCenter_us, 1000, 1800);
  }
  else
  {
    t_us = constrain(ServoCenter_us - ServoScale_us * ServoOne, 1000, 1800);
  }
 
  PingPongServo.writeMicroseconds(t_us);

//
// now send serial information
//
  time_prev = time; 
  time = millis(); 
  dt = time - time_prev; 
//  Serial.print(JoystickOne,2);
//  Serial.print(',');  
//  Serial.print(xBallOne,2);
//  Serial.print(',');
//  Serial.println(ServoOne,2);
Serial.print(t_switch,DEC); 
Serial.print(','); 
Serial.print(dt,DEC); 
Serial.print(',');  
Serial.println (errorXball,DEC); 



//
// force constant frame rate
//
  while (micros() < microsLast + deltaT*10000);  // wait for deltaT since last time through
  microsLast = micros(); // save value for next time through
}

//
// modified from online library
// assumes only 1 block is reported
// that block must be the Ping Pong Ball
//
word PingPongCheck()
{
  int xpos=-1;
  uint16_t nblocks;

  nblocks = PingPongPixy.getBlocks();  
  if (nblocks)
  {
    xpos = PingPongPixy.blocks[0].x;
  }
  
  return xpos;
}

void SimPlot(int data1, int data2, int data3, int data4)
{
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
