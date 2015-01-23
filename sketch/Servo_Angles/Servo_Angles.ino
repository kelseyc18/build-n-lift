#include <Adafruit_PWMServoDriver.h>

/*

Program that Outputs a pulselength for a desired angle
The pulselength is based off the Arduino map() function

*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Pulse Ranges for Joint servos
const int BASE_ROTATION_MIN = 150;
const int BASE_ROTATION_MAX = 600;

const int WRIST_ANGLE_MIN = 100;
const int WRIST_ANGLE_MAX = 585;

const int WRIST_GRIPPER_MIN = 150;
const int WRIST_GRIPPER_MAX = 600;

const int ELBOW_SERVO_MIN = 200;
const int ELBOW_SERVO_MAX = 500;

const int SHOULDER_MIN = 175;
const int SHOULDER_MAX = 600;
//#define const int ELBOW_MIN
//#define const int ELBOW_MAX
//#define const int SHOULDER_MIN
//#define const int SHOULDER_MAX

int servo_test_port = 0;


// called this way, it uses the default address 0x40


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

int degreesToPulse(int angle_Degree, int pulseMin, int pulseMax){
  int pulse_length;
      pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
   
  
  Serial.println(pulse_length);
  return pulse_length;
 }


//int sweep_pulselength = degreesToPulse(180, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX);
int half_sweep_pulselength = degreesToPulse(90, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
int full_sweep_pulselength = degreesToPulse(180, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);


void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}



void loop(){
  
  int dummy_variable = 300;
  // Testing Base Rotation
  int sweep_pulselength = degreesToPulse(70, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX);
  pwm.setPWM(servo_test_port, 0, sweep_pulselength); // 0 degrees
  
 
  }
  
  
 
  





