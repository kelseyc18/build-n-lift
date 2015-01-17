#include<math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Pulse Ranges for Joint servos
#define const int BASE_ROTATION_MIN = 150;
#define const int BASE_ROTATION_MAX = 600;

#define const int WRIST_ANGLE_MIN = 100;
#define const int WRIST_ANGLE_MAX = 585;

#define const int WRIST_GRIPPER_MIN = 150;
#define const int WRIST_GRIPPER_MAX = 600;

//#define const int ELBOW_MIN
//#define const int ELBOW_MAX
//#define const int SHOULDER_MIN
//#define const int SHOULDER_MAX

// temp
int servo_test_port = 0;

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

// wrist increments
const int incWrist = 5;

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
int WRIST_ANGLE_MIN = 100;
int WRIST_ROT_MIN = 200; //guess
int WRIST_GRIPPER_MIN = 150;

// wrist servo max
int WRIST_ANGLE_MAX = 585;
int WRIST_ROT_MAX = 400; //guess
int WRIST_GRIPPER_MAX = 600;

// current servo value
int wristAng = 350;
int wristRot = 300;
int clawPos = 370;

// pulse constants
int pulseOn = 2000;

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // max frequency is 1000 Hz
  
  // Setting all to middle, can do starting value if we want
  pwm.setPWM(wristAngPort, 0, degreesToPulse(90, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX));
  pwm.setPWM(wristRotPort, 0, degreesToPulse(90, WRIST_ROT_MIN, WRIST_ROT_MAX));
  pwm.setPWM(clawPosPort, 0, degreesToPulse(90, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX));
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
    wristAng = moveWrist(sensorValue1, dirValue, wristAngPort, wristAng, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
    wristRot = moveWrist(sensorValue2, dirValue, wristRotPort, wristRot, WRIST_ROT_MIN, WRIST_ROT_MAX);
    clawPos = moveWrist(sensorValue3, dirValue, clawPosPort, clawPos, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX);
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

int moveWrist(int value, int posNeg, int portnum, int pos, int thismin, int thismax)
{
  if (value > threshhold){
    if (posNeg){
      pos += incWrist;
      if (pos > thismax){
        pos-= incWrist;
      }
      else{
        pwm.setPWM(portnum, pulseOn, pos + pulseOn);
      }
    }
    else{
      pos -= incWrist;
      if(pos < thismin){
        pos += incWrist;
      }
      else{
        pwm.setPWM(portnum, pulseOn, pos + pulseOn);
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
  int[3] desiredDegrees = [0, 0, 0];
  calculateDegrees(x, y, z, desiredDegrees);
  setPWM()
}

void calculateDegrees (int x, int y, int z, int[] desiredDegrees) {
  desiredDegrees[0] = 0;
  desiredDegrees[1] = 0;
  desiredDegrees[2] = 0;
}
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
  //float length5 = sqrt(pow(length3,2.0) + ro^2 - 2*length3*ro*cos(180-phi)); // law of cosines to find the length from the wrist to the end effector
  //float phi2 = acos((pow(-ro,2.0) - pow(length5,2.0) + pow(length3, 2.0))/(-2*length5*length3)); //angle from wrist-elbow-end effector
  //float length4 = sqrt(pow(r,2.0) + pow(z-length1,2.0); // length from shoulder to end effector
  //theta3 = acos((pow(length4,2.0) - pow(legnth5,2.0) - pow(length2,2.0))/(-2*length5*length2) + phi2; // elbow angle using law of cosines, correcting for the fact that the end effector placement angle is not the same as the elbow angle due to phiand ro being nonzero.
  //float theta4 = atan2(z-length1, r); // angle from horizontal to end effector
  //float theta5 = acos((pow(length5,2.0) - pow(length2,2.0) - pow(length4,2.0)/(-2*length2*length4); // angle from theta4 to humerus
  //theta2 =theta4 +theta5; // adding to get theta 2
  // I am Africa
}

int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length;
  if(angle_Degree > 180){
      pulse_length = map(angle_Degree, 0, 360, pulseMin, pulseMax);
   }
  else if (angle_Degree < 180){
      pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
    }
  
  return pulse_length;
}
