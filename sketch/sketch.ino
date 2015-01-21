#include<math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Arm dimensions (mm)
const float BASE_HEIGHT = 67.31; // height from base to shoulder
const float HUMERUS = 146.05; // length from shoulder to elbow
const float ULNA = 187.325; // length from elbow to wrist
const float GRIPPER = 100.00; //length from wrist to end effector

//Pulse Ranges for Joint servos
// min values correspond to 0 degrees
// max values correspond to 180 degrees
const int BASE_ROTATION_MIN = 600;
const int BASE_ROTATION_MAX = 150;

const int WRIST_ANGLE_MIN = 100;
const int WRIST_ANGLE_MAX = 585;

const int WRIST_GRIPPER_MIN = 150;
const int WRIST_GRIPPER_MAX = 600;

// 360 degree servo
const int WRIST_ROT_MIN = 200; //guess
const int WRIST_ROT_MAX = 400; // guess

const int ELBOW_MIN = 650; // corrected
const int ELBOW_MAX = 100; 

const int SHOULDER_MIN = 100; // guess
const int SHOULDER_MAX = 500; // guess

// Pin inputs
const int analogInPin1 = A0;  	// Analog input pin that EMG-1 is attached to
const int analogInPin2 = A1;	// Analog input pin that EMG-2 is attached to
const int analogInPin3 = A2;	// Analog input pin that EMG-3 is attached to
const int dirButton = 2;	// Digital input pin indicating positive or negative direction
const int modeButton = 4;       // Digital input pin to toggle mode (xyz direction or wrist movement)

// servo outputs
int wristAngPort = 0;
int wristRotPort = 1;
int wristGripperPort = 2;
int basePort = 3;
int shoulderPort = 4;
int elbowPort = 5;

// increments
const int incX = 5;
const int incY = 5;
const int incZ = 5;
const int incWrist = 5;

// Sensor values
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
// Button values
int dirValue = 0;
int modeValue = 0;

// Thresholds
int xThresh = 300;
int yThresh = 300;
int zThresh = 300;
int threshold = 300;  // for wrist

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

// current servo value
int wristAng = 350;
int wristRot = 300;
int wristGripper = 370;

// desired angles for inverse kinematics
float desiredDegrees[] = {0, 0, 0};

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // max frequency is 1000 Hz
  
  // initialize the EMG input pins
  pinMode(analogInPin1, INPUT);
  pinMode(analogInPin2, INPUT);
  pinMode(analogInPin3, INPUT);
  // initialize the buttons
  pinMode(dirButton, INPUT);
  pinMode(modeButton, INPUT);
  
  // initialize servo output pins
  pinMode(wristAngPort, OUTPUT);
  pinMode(wristRotPort, OUTPUT);
  pinMode(wristGripperPort, OUTPUT);
  pinMode(basePort, OUTPUT);
  pinMode(shoulderPort, OUTPUT);
  pinMode(elbowPort, OUTPUT);
  
  // Setting all to middle, can do starting value if we want
  pwm.setPWM(wristAngPort, 0, degreesToPulse(90, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX));
  pwm.setPWM(wristRotPort, 0, degreesToPulse(90, WRIST_ROT_MIN, WRIST_ROT_MAX));
  pwm.setPWM(wristGripperPort, 0, degreesToPulse(90, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX));
}

void loop()
{ 
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
  if (modeValue == HIGH)
  {
    // move wrist servos
    wristAng = moveWrist(sensorValue1, dirValue, wristAngPort, wristAng, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
    wristRot = moveWrist(sensorValue2, dirValue, wristRotPort, wristRot, WRIST_ROT_MIN, WRIST_ROT_MAX);
    wristGripper = moveWrist(sensorValue3, dirValue, wristGripperPort, wristGripper, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX);
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
    if (dirValue == HIGH) moveToPosition(currentX + velX, currentY + velY, currentZ + velZ);
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
  
/* 
Daniel, I think it might help to use the degreesToPulse() method to find 
out how much you need to increase the pulse length by in order to change the angle.
*/
int moveWrist(int value, int posNeg, int portnum, int pos, int thismin, int thismax)
{
  if (value > threshold){
    if (posNeg == HIGH){
      pos += incWrist;
      if (pos > thismax){
        pos-= incWrist;
      }
      else{
        pwm.setPWM(portnum, 0, pos);
      }
    }
    else{
      pos -= incWrist;
      if(pos < thismin){
        pos += incWrist;
      }
      else{
        pwm.setPWM(portnum, 0, pos);
      }
    }
  }
  delay(50);
  return pos;
}
  
/*
This method sets the servo angles so that the end effector is at the given position (in xyz coordinates).
*/
void moveToPosition(int x, int y, int z) {
  currentX = x;
  currentY = y;
  currentZ = z;
  calculateDegrees(x, y, z); // updates desiredDegrees
  pwm.setPWM(basePort, 0, degreesToPulse(desiredDegrees[0], BASE_ROTATION_MIN, BASE_ROTATION_MAX));
  pwm.setPWM(shoulderPort, 0, degreesToPulse(desiredDegrees[1], SHOULDER_MIN, SHOULDER_MAX));
  pwm.setPWM(elbowPort, 0, degreesToPulse(desiredDegrees[2], ELBOW_MIN, ELBOW_MAX));
}

/*
This method uses inverse kinematics to determine the desired angles of the servos given
a desired xyz coordinate position.
*/
// FYI, Arduino uses RADIANS, not degrees.
void calculateDegrees (int x, int y, int z) {
  const float pi = 3.14159265259;
  float phi = map(wristAng,WRIST_ANGLE_MIN,WRIST_ANGLE_MAX,1,179); // wrist angle we give, between end effector and line of forearm NEED TO KNOW WHAT 100 AND 585 LOOK LIKE ON THE WRIST!! UNSURE IF CODE WORKS THIS WAY
  phi = phi*pi/180;
  float r; // distance to end effector from base on base plane
  float theta1;  // base angle
  float theta2; // angle of the servo at the shoulder, from horizontal
  float theta3; // angle of the servo at the elbow, between wrist and shoulder
  r = sqrt(pow(x, 2.0) +pow(y, 2.0)); // pythagoras
  theta1 = atan2(y,x); // trig
  float length5 = sqrt(pow(ULNA,2.0) + pow(GRIPPER,2.0) - 2*ULNA*GRIPPER*cos(pi-phi)); // law of cosines to find the length from the wrist to the end effector
  float phi2 = acos((pow(GRIPPER,2.0) - pow(length5,2.0) - pow(ULNA, 2.0))/(-2*length5*ULNA)); //angle from wrist-elbow-end effector
  float length4 = sqrt(pow(r,2.0) + pow(z-BASE_HEIGHT,2.0)); // length from shoulder to end effector
  theta3 = acos((pow(length4,2.0) - pow(length5,2.0) - pow(HUMERUS,2.0))/(-2*length5*HUMERUS)) - phi2; // elbow angle using law of cosines, correcting for the fact that the end effector placement angle is not the same as the elbow angle due to phiand ro being nonzero.
  float theta4 = atan2(z-BASE_HEIGHT, r); // angle from horizontal to end effector
  float theta5 = acos((pow(length5,2.0) - pow(HUMERUS,2.0) - pow(length4,2.0))/(-2*HUMERUS*length4)); // angle from theta4 to humerus
  theta2 =theta4 +theta5; // adding to get theta 2
  if (theta1 == NAN || theta2 == NAN || theta3 == NAN){
    return;
  }
  desiredDegrees[0] = theta1; // desired base angle
  desiredDegrees[1] = theta2; // desired shoulder angle
  desiredDegrees[2] = theta3; // desired elbow angle
}

/*
This method converts a desired angle in degrees to the corresponding desired pulse length.
*/
int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length;
  if(angle_Degree > 180){
      pulse_length = map(angle_Degree, 0, 360, pulseMin, pulseMax);
  }
  else{ // (angle_Degree <= 180) 
      pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
  }
  return pulse_length;
}
