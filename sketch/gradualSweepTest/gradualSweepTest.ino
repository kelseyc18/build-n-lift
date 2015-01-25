#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Arm dimensions (mm)
const float BASE_HEIGHT = 67.31; // height from base to shoulder
const float HUMERUS = 146.05; // length from shoulder to elbow
const float ULNA = 187.325; // length from elbow to wrist
const float GRIPPER = 100.00; //length from wrist to end effector

const int BASE_ROTATION_MIN = 600;
const int BASE_ROTATION_MAX = 150;

const int WRIST_ANGLE_MIN = 180; // 90 degrees below arm
const int WRIST_ANGLE_MAX = 630; // - 90 degrees below arm level

const int WRIST_GRIPPER_MIN = 150; 
const int WRIST_GRIPPER_MAX = 600;

// 360 degree servo
const int WRIST_ROT_MIN = 200; //guess
const int WRIST_ROT_MAX = 400; // guess

const int ELBOW_MIN = 500; // corrected Old Min 100
const int ELBOW_MAX = 100; // Old Number 500

const int SHOULDER_MIN = 150; // Old 150
const int SHOULDER_MAX = 600; // Old 600

// CHANGE THESE
const int velX = 5;
const int velY = 5;
const int velZ = 5;
const int incWristAng = 2;
const int incWristRot = 2;
const int incWristGripper = 5;

// AND THESE
int xThresh = 700;
int yThresh = 300;
int zThresh = 300;
int wristAngThresh = 300;  // for wrist
int wristRotThresh = 300;
int wristGripperThresh = 300;

// Initialize output pins
const int basePort = 0;
const int shoulderPort = 1;
const int elbowPort = 2;
const int wristAngPort = 3;
const int wristRotPort = 4;
const int wristGripperPort = 5;

const int analogInPin1 = A0;  	// Analog input pin that EMG-1 is attached to
const int analogInPin2 = A1;	// Analog input pin that EMG-2 is attached to
const int analogInPin3 = A2;	// Analog input pin that EMG-3 is attached to
const int dirPort = 12;         // Digital input pin to toggle direction (positive or negative)
const int modePort = 13;        // Digital input pin to toggle mode (xyz direction or wrist movement)

// Wrist pulse lengths
int wristAng = 0;
int wristAngBelowHorizontal = 90; //when wristAng = 0 and ulna is horizontal, this equals 90
int wristRot = 90;
int wristGripper = 90;

// Current XYZ position
int currentX = 0;
int currentY = 150;
int currentZ = 60;

int dirValue;
int modeValue;

int sensorValue1;
int sensorValue2;
int sensorValue3;

float desiredDegrees[] = {0, 0, 0};

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  
  pinMode(basePort, OUTPUT);
  pinMode(shoulderPort, OUTPUT);
  pinMode(elbowPort, OUTPUT);
  pinMode(wristAngPort, OUTPUT);
  pinMode(wristRotPort, OUTPUT);
  pinMode(wristGripperPort, OUTPUT);
  
  pinMode(analogInPin1, INPUT);
  pinMode(analogInPin2, INPUT);
  pinMode(analogInPin3, INPUT);
  pinMode(dirPort, INPUT);
  pinMode(modePort, INPUT);
  
  pwm.setPWM(wristAngPort, 0, degreesToPulse(90, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX));
  pwm.setPWM(wristRotPort, 0, degreesToPulse(90, WRIST_ROT_MIN, WRIST_ROT_MAX));
  pwm.setPWM(wristGripperPort, 0, degreesToPulse(90, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX));
  moveToPosition(currentX, currentY, currentZ);
}

void loop()
{
  sensorValue1 = analogRead(analogInPin1);//Right Bicep
  Serial.print("Sensor Value1: ");
  Serial.println(sensorValue1);
  sensorValue2 = analogRead(analogInPin2);//Left Bicep
  sensorValue3 = analogRead(analogInPin3);//Forearm
  dirValue = digitalRead(dirPort);
  Serial.print("dirValue: ");
  Serial.println(dirValue);
  modeValue = digitalRead(modePort);
  
  // insert test method here
  oneSensorMoveWrist();
  delay(1000);
}

void oneSensorMoveWrist() {
  if(sensorValue1 > wristGripperThresh){
    if (dirValue == HIGH && wristGripper < 180) wristGripper += incWristGripper;
  }
    else {
      if(wristGripper >= incWristGripper) wristGripper -= incWristGripper;
    }
    pwm.setPWM(wristGripperPort, 0, degreesToPulse(wristGripper, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX));// Open hand

}

void updateXYZ() {
  if(sensorValue1 > xThresh) {
    if (dirValue == HIGH) currentX += velX;
    else currentX -= velX;
  }
  if(sensorValue2 > yThresh) {
    if (dirValue == HIGH) currentY += velY;
    else currentY -= velY;
  }
  if(sensorValue3 > zThresh) {
    if (dirValue == HIGH) currentZ += velZ;
    else currentZ -= velZ;
  }
  moveToPosition(currentX, currentY, currentZ);
}

void updateWrist() {
  if (sensorValue1 > wristAngThresh) {
    if (dirValue == HIGH && wristAng <= 180-incWristAng){
      wristAng += incWristAng;
      wristAngBelowHorizontal -=incWristAng;
    }
    else if (wristAng >= incWristAng){
      wristAng -= incWristAng;
      wristAngBelowHorizontal += incWristAng;
    }
    pwm.setPWM(wristAngPort, 0, degreesToPulse(wristAng, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX));
  }   
  if (sensorValue2 > wristGripperThresh) {
    if (dirValue == HIGH && wristGripper < 180-incWristGripper) wristGripper += incWristGripper;
    else if (wristGripper > incWristGripper) wristGripper -= incWristGripper;
    pwm.setPWM(wristGripperPort, 0, degreesToPulse(wristGripper, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX));
  }
  if (sensorValue3 > wristRotThresh) {
    if (dirValue == HIGH) pwm.setPWM(wristRotPort, 0, 500);
    else pwm.setPWM(wristRotPort, 0, 850);
  }
  else pwm.setPWM(wristRotPort, 0, 0);
} 

void wholeShebang() {
  if (modeValue == HIGH) updateWrist();
  else updateXYZ();
}

void threesixtyServo(int sensorValue, int thresh) {
  if (sensorValue > thresh) {
    if (dirValue == HIGH) pwm.setPWM(wristRotPort, 0, 500);
    else pwm.setPWM(wristRotPort, 0, 850);
  }
  else pwm.setPWM(wristRotPort, 0, 0);
}

void oneSensorMove(int sensorValue, int thresh) {
  if (sensorValue > thresh) {
    currentX += velX;
  }
  moveToPosition(currentX, currentY, currentZ);

  delay(300);
}

void moveButton() {
  if (dirValue == HIGH) currentX += velX;
  Serial.println(dirValue);
  moveToPosition(currentX, currentY, currentZ);
}

void sweep() {
  for (int deg = 0; deg <= 180; deg += 45)
  {
    pwm.setPWM(elbowPort, 0, degreesToPulse(deg, ELBOW_MIN, ELBOW_MAX));
    Serial.println(deg);
    Serial.println(degreesToPulse(deg, ELBOW_MIN, ELBOW_MAX));
    Serial.println();
    delay(1000);
  }

}

void moveToPosition(int x, int y, int z) {
  calculateDegrees(x, y, z); // updates desiredDegrees
  pwm.setPWM(basePort, 0, degreesToPulse(desiredDegrees[0], BASE_ROTATION_MIN, BASE_ROTATION_MAX));
  pwm.setPWM(shoulderPort, 0, degreesToPulse(desiredDegrees[1], SHOULDER_MIN, SHOULDER_MAX));
  pwm.setPWM(elbowPort, 0, degreesToPulse(desiredDegrees[2], ELBOW_MIN, ELBOW_MAX));
  pwm.setPWM(wristAngPort,0,degreesToPulse(wristAng,WRIST_ANGLE_MIN,WRIST_ANGLE_MAX));
}

int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
  return pulse_length;
}

void calculateDegrees (int x, int y, int z) {
  const float pi = 3.14159265259;
  float alpha = wristAngBelowHorizontal * pi/180;
  float r = sqrt(pow(x, 2.0) +pow(y, 2.0));
  float theta1 = atan2(y,x); // trig
  float length4 = sqrt(pow(r-GRIPPER*cos(alpha),2.0) + pow(z-BASE_HEIGHT-GRIPPER*cos(alpha),2.0)); // length from shoulder to wrist
  float theta4 = atan2(z-BASE_HEIGHT-GRIPPER*sin(alpha), r-GRIPPER*cos(alpha)); // angle from horizontal to wrist
  float theta5 = acos((pow(ULNA,2.0)-pow(HUMERUS,2.0) - pow(length4,2.0))/(-2*HUMERUS*length4));
  float theta3 = acos((pow(length4,2.0) - pow(HUMERUS,2.0) - pow(HUMERUS,2.0))/(-2*HUMERUS*ULNA));
  float theta6 = pi- theta5 - theta3;
  float theta2 = theta5+ theta4;
  float wristAngleFromUlna = theta6 + pi/2 -theta4 + pi/2 - alpha;
  wristAngleFromUlna = wristAngleFromUlna*180/pi;  
  if (theta1 == NAN || theta2 == NAN || theta3 == NAN || wristAngleFromUlna > 270 || WristAngleFromUlna < 90){
    return;
  }
  float theta1Deg = theta1/pi*180;
  float theta2Deg = theta2/pi*180;
  float theta3Deg = theta3/pi*180;
  if (theta3Deg < 5) {
    theta3Deg = 5;
  }
  desiredDegrees[0] = theta1Deg; // desired base angle
  desiredDegrees[1] = theta2Deg; // desired shoulder angle
  desiredDegrees[2] = theta3Deg; // desired elbow angle
  wristAng = wristAngleFromUlna -90;
}
