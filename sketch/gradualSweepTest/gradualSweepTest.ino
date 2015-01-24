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

const int ELBOW_MIN = 650; // corrected
const int ELBOW_MAX = 100; 

const int SHOULDER_MIN = 100; // guess
const int SHOULDER_MAX = 500; // guess

const int servoPort = 0;

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  
  pinMode(servoPort, OUTPUT);

  pwm.setPWM(servoPort, 0, degreesToPulse(0, SERVO_MIN, SERVO_MAX));
}

void loop()
{
  for (int deg = 0; deg <= 180; deg += 10)
  {
    pwm.setPWM(servoPort, 0, degreesToPulse(deg, SERVO_MIN, SERVO_MAX));
    Serial.println(deg);
    Serial.println(degreesToPulse(deg, SERVO_MIN, SERVO_MAX));
    Serial.println();
    delay(1000);
  }
}
