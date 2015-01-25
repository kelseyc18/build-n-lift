#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
//    pwm.setPWM(elbowPort, 0, degreesToPulse(deg, ELBOW_MAX, ELBOW_MIN));
//    Serial.println(deg);
//    Serial.println(degreesToPulse(deg, ELBOW_MAX, ELBOW_MIN));
//    Serial.println();
//    delay(1000);
//  }
  pwm.setPWM(basePort, 0, degreesToPulse(135,BASE_ROTATION_MIN,BASE_ROTATION_MAX));
  delay(1000);
  pwm.setPWM(shoulderPort, 0, degreesToPulse(122,SHOULDER_MIN, SHOULDER_MAX));
  delay(1000);
  pwm.setPWM(elbowPort, 0, degreesToPulse(45,ELBOW_MIN,ELBOW_MAX));
  delay(1000);
}

int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length;
  /*
  if(angle_Degree > 180){// Does not have any affect
      pulse_length = map(angle_Degree, 0, 360, pulseMin, pulseMax);
  }
  */
  //else// (angle_Degree <= 180) 
      pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
      Serial.println(pulse_length);
  //}
  return pulse_length;
}
