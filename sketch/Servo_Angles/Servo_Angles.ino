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
  if(angle_Degree > 180){
      pulse_length = map(angle_Degree, 0, 360, pulseMin, pulseMax);
   }
  else if (angle_Degree < 180){
      pulse_length = map(angle_Degree, 0, 180, pulseMin, pulseMax);
    }
  
  return pulse_length;
 }


int no_sweep_pulselength = degreesToPulse(0, BASE_ROTATION_MIN, BASE_ROTATION_MAX);
int half_sweep_pulselength = degreesToPulse(90, BASE_ROTATION_MIN, BASE_ROTATION_MAX);
int full_sweep_pulselength = degreesToPulse(180, BASE_ROTATION_MIN, BASE_ROTATION_MAX);


void setup(){
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop(){
  // Testing Base Rotation
  pwm.setPWM(servo_test_port, 0, no_sweep_pulselength); // 0 degrees
  delay(500);
  pwm.setPWM(servo_test_port, 0, half_sweep_pulselength); // 90 degrees
  delay(500);
  pwm.setPWM(servo_test_port, 0, full_sweep_pulselength); // 180 degrees
}




