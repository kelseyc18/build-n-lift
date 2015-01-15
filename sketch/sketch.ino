#include<math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse width min/max for servos



// Direction constants
#define XDIR 0;
#define YDIR 1;
#define ZDIR 2;

// Pin inputs and outputs
const int analogInPin1 = A0;  	// Analog input pin that EMG-1 is attached to
const int analogInPin2 = A1;	// Analog input pin that EMG-2 is attached to
const int analogInPin3 = A2;	// Analog input pin that EMG-3 is attached to

const int buttonX = 2;	// Digital input pin for X direction
const int buttonY = 4;	// Digital input pin for Y direction
const int buttonZ = 7;	// Digital input pin for Z direction

// Sensor values
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

// Button values
int xDir = 0;
int yDir = 0;
int zDir = 0;

// Thresholds
int xThresh = 500;
int yThresh = 500;
int zThresh = 500;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // Link EMG voltage to velocity
	
  // get EMG sensor values
  sensorValue1 = analogRead(analogInPin1);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3);
	
  // get direction of XYZ movement
  // 0: neg. direction
  // 1: pos. direction
  xDir = digitalRead(buttonX);
  yDir = digitalRead(buttonY);
  zDir = digitalRead(buttonZ);
	
  // compare sensor values with threshold and take corresponding action
  checkThresh(sensorValue1, xThresh, XDIR, xDir);
  checkThresh(sensorValue2, yThresh, YDIR, yDir);
  checkThresh(sensorValue3, zThresh, ZDIR, zDir);
}

/*
This function compares a given value to the threshold value. If the value is
larger than the threshold, the robot arm will move in the indicated direction.
Otherwise, the robot arm will stop moving in the indicated direction.
*/
void checkThresh(int value, int thresh, int direction, int posNeg)
{
  if (value > thresh) {
    if (posNeg) {
      // move at certain velocity in positive indicated direction 
    }
    else {
      // move at certain velocity in negative indicated direction
    }
  }
  else {
    // stop moving in indicated direction
    stop(direction);
  }
}

/*
This function stops the robot arm from moving in the indicated direction.
*/
void stop(int direction)
{
  // stop moving in indicated direction
}

/*
Daniel's Inverse Kinematics math
*/
const float pi = 3.14159265259
int x = 2; // x we give
int y = 2; // y we give
int z = 2; // z we give
float phi = 2; // wrist angle we give, between end effector and line of forearm
float length1 = 1; // height from base to shoulder
float length2 = 1; // length from shoulder to elbow
float length3 = 1; // length from elbow to wrist
float ro = 1; //length from wrist to end effector
float r = 0; // distance to end effector from base on base plane
float theta1 = 0;  // base angle
float theta2 = 0; // angle of the servo at the shoulder, from horizontal
float theta3 = 0; // angle of the servo at the elbow, between wrist and shoulder
r = sqrt(x^2 +y^2); // pythagoras
theta1 = atan2(y,x); // trig
float length5 = sqrt(length3^2 + ro^2 - 2*length3*ro*cos(180-phi)); // law of cosines to find the length from the wrist to the end effector
float phi2 = acos((-ro^2 - length5^2 + length3^2)/(-2*length5*length3)); //angle from wrist-elbow-end effector
float length4 = sqrt(r^2 + (z- length1)^2); // length from shoulder to end effector
theta3 = acos((length4^2 - length5^2 - length2^2)/(-2*length5*length2) + phi2; // elbow angle using law of cosines, correcting for the fact that the end effector placement angle is not the same as the elbow angle due to phiand ro being nonzero.
float theta4 = atan2(z-length1, r); // angle from horizontal to end effector
float theta5 = acos((length5^2 - length2^2 - length4^2)/(-2*length2*length4); // angle from theta4 to humerus
theta2 =theta4 +theta5; // adding to get theta 2

