#include<math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse width min/max for servos
// Bernard was here


// Direction constants
#define XDIR 0;
#define YDIR 1;
#define ZDIR 2;

// Pin inputs and outputs
const int analogInPin1 = A0;  	// Analog input pin that EMG-1 is attached to
const int analogInPin2 = A1;	// Analog input pin that EMG-2 is attached to
const int analogInPin3 = A2;	// Analog input pin that EMG-3 is attached to

const int dirButton = 2;	// Digital input pin indicating positive or negative direction
const int modeButton = 4;       // Digital input pin to toggle mode (xyz direction or wrist movement)

// Wrist outputs
int wristAngPort = 0;
int wristRotPort = 1;
int clawPosPort = 2;

// Remaining servo outputs
int base_Rotation = 3
int shoulder_Servo = 4
int elbow_Servo = 5

// XYZ velocity increments
const int incX = 5;
const int incY = 5;
const int incZ = 5;

// Sensor values
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

// Button values
int dirValue = 0;
int modeValue = 0;

// Thresholds
int xThresh = 500;
int yThresh = 500;
int zThresh = 500;
int threshhold = 250;  // for wrist

// Current XYZ position
int currentX = 0;
int currentY = 0;
int currentZ = 0;

// Current XYZ velocity
int velX = 0;
int velY = 0;
int velZ = 0;

// Move in direction
boolean moveX = false;
boolean moveY = false;
boolean moveZ = false;

// wrist servo min
int wristAngMin = 100;
int wristRotMin = 200; //guess
int clawPosMin = 150;

// wrist servo max
int wristAngMax = 585;
int wristRotMax = 400; //guess
int clawPosMax = 600;

// current servo angles
int wristAng = 350;
int wristRot = 300;
int clawPos = 370;

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // max frequency is 1000 Hz
  
  //Setting all to middle, can do starting value if we want
  pwm.setPWM(wristAngPort, 0, wristAng);
  pwm.setPWM(wristRotPort, 0, wristRot);
  pwm.setPWM(clawPosPort, 0, clawPos);
}

void loop()
{
  // Link EMG voltage to velocity
	
  // get EMG sensor values
  sensorValue1 = analogRead(analogInPin1);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3);
	
  // get direction of movement
  // 0: neg. direction
  // 1: pos. direction
  dirValue = digitalRead(dirButton);
  
  // get mode
  // 0: xyz motion
  // 1: wrist motion
  modeValue = digitalRead(modeButton);
	
  // compare sensor values with threshold and take corresponding action
  if (modeValue)
  {
    // move wrist servos
    wristAng = moveWrist(sensorValue1, dirValue, modeValue, wristAngPort, wristAng, wristAngMin, wristAngMax);
    wristRot = moveWrist(sensorValue2, dirValue, modeValue, wristRotPort, wristRot, wristRotMin, wristRotMax);
    clawPos = moveWrist(sensorValue3, dirValue, modeValue, clawPosPort, clawPos, clawPosMin, clawPosMax);
  }
  else
  {
    // move xyz servos
    // check thresholds
    moveX = checkThresh(sensorValue1, xThresh);
    moveY = checkThresh(sensorValue2, yThresh);
    moveZ = checkThresh(sensorValue3, zThresh);
    // update velocities
    updateVelocities();
    if (dirValue) moveToPosition(currentX + velX, currentY + velY, currentZ + velZ);
    else moveToPosition(currentX - velX, currentY - velY, currentZ - velZ);
  }
}

/*
This function returns True if the sensor value is greater than the threshold value.
*/
boolean checkThresh(int sensor, int thresh)
{
  return sensor > thresh; 
}

void updateVelocities()
{
  if (moveX) velX = incX;
  else velX = 0;
  if (moveY) velY = incY;
  else velY = 0;
  if (moveZ) velZ = incZ;
  else velZ = 0;
}

int moveWrist(int value, int posNeg, int wristTrue, int portnum, int pos, int thismin, int thismax)
{
  if (value > threshhold){
    if (wristTrue){
      if (posNeg){
        pos += 5;
        if (pos > thismax){
          pos-= 5;
        }
        else{
          pwm.setPWM(portnum, 2000, pos+2000);
        }
      }
      else{
        pos-=5;
        if(pos < thismin){
          pos+= 5;
        }
        else{
          pwm.setPWM(portnum, 2000, pos+2000);
        }
      }
    }
  }
  delay(50);
  return pos;
}
  
// checking
//work pls
// "lion king noises"
/*this?*/
// haha this is great, Daniel. it's Daniel, right? otherwise, hi, Bernard.

/*
Daniel's Inverse Kinematics math
*/
// FYI, Daniel, Arduino doesn't recognize '^' as an exponent. If you want to calculate the square of 3, you would use 'pow(3.0, 2.0)'
// http://arduino.cc/en/Reference/Pow
void moveToPosition(int x, int y, int z) {
  const float pi = 3.14159265259;
//  int x = 2; // x we give
//  int y = 2; // y we give
//  int z = 2; // z we give
  float phi = 2; // wrist angle we give, between end effector and line of forearm
  float length1 = 1; // height from base to shoulder
  float length2 = 1; // length from shoulder to elbow
  float length3 = 1; // length from elbow to wrist
  float ro = 1; //length from wrist to end effector
  float r = 0; // distance to end effector from base on base plane
  float theta1 = 0;  // base angle
  float theta2 = 0; // angle of the servo at the shoulder, from horizontal
  float theta3 = 0; // angle of the servo at the elbow, between wrist and shoulder
  r = sqrt(pow(x, 2.0) +pow(y, 2.0)); // pythagoras
  theta1 = atan2(y,x); // trig
//  float length5 = sqrt(length3^2 + ro^2 - 2*length3*ro*cos(180-phi)); // law of cosines to find the length from the wrist to the end effector
//  float phi2 = acos((-ro^2 - length5^2 + length3^2)/(-2*length5*length3)); //angle from wrist-elbow-end effector
//  float length4 = sqrt(r^2 + (z- length1)^2); // length from shoulder to end effector
//  theta3 = acos((length4^2 - length5^2 - length2^2)/(-2*length5*length2) + phi2; // elbow angle using law of cosines, correcting for the fact that the end effector placement angle is not the same as the elbow angle due to phiand ro being nonzero.
//  float theta4 = atan2(z-length1, r); // angle from horizontal to end effector
//  float theta5 = acos((length5^2 - length2^2 - length4^2)/(-2*length2*length4); // angle from theta4 to humerus
//  theta2 =theta4 +theta5; // adding to get theta 2
  // I am Africa
}
