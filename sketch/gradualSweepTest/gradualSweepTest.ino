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

const int basePort = 0;
const int shoulderPort = 1;
const int elbowPort = 2;

int wristAng = 150;
int wristRot = 300;
int wristGripper = 370;

float desiredDegrees[] = {0, 0, 0};

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  
  pinMode(basePort, OUTPUT);
  pinMode(shoulderPort, OUTPUT);
  pinMode(elbowPort, OUTPUT);
}

void loop()
{
//  for (int deg = 0; deg <= 180; deg += 45)
//  {
//    pwm.setPWM(elbowPort, 0, degreesToPulse(deg, ELBOW_MIN, ELBOW_MAX));
//    Serial.println(deg);
//    Serial.println(degreesToPulse(deg, ELBOW_MIN, ELBOW_MAX));
//    Serial.println();
//    delay(1000);
//  }
  moveToPosition(0, 150, 60);
}

void moveToPosition(int x, int y, int z) {
  calculateDegrees(x, y, z); // updates desiredDegrees
  pwm.setPWM(basePort, 0, degreesToPulse(desiredDegrees[0], BASE_ROTATION_MIN, BASE_ROTATION_MAX));
  pwm.setPWM(shoulderPort, 0, degreesToPulse(desiredDegrees[1], SHOULDER_MIN, SHOULDER_MAX));
  pwm.setPWM(elbowPort, 0, degreesToPulse(desiredDegrees[2], ELBOW_MIN, ELBOW_MAX));
}

int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
  return pulse_length;
}

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
  float theta1Deg = theta1/pi*180;
  float theta2Deg = theta2/pi*180;
  float theta3Deg = theta3/pi*180;
  if (theta3Deg < 45) {
    theta3Deg = 45;
  }
  desiredDegrees[0] = theta1Deg; // desired base angle
  desiredDegrees[1] = theta2Deg; // desired shoulder angle
  desiredDegrees[2] = theta3Deg; // desired elbow angle
}
