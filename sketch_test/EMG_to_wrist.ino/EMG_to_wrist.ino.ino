/*
This program tests the code that reads EMG values and
outputs the desired pwm pulse lengths.
*/

const int WRIST_ANGLE_MIN = 100;
const int WRIST_ANGLE_MAX = 585;

const int WRIST_GRIPPER_MIN = 150;
const int WRIST_GRIPPER_MAX = 600;

const int WRIST_ROT_MIN = 200; //guess
const int WRIST_ROT_MAX = 400; // guess

// Wrist outputs
int wristAngPort = 0;
int wristRotPort = 1;
int wristGripperPort = 2;

// current servo value
int wristAng = 350;
int wristRot = 300;
int wristGripper = 370;

// wrist increments
const int incWrist = 5;

const int threshold = 500;

int sensorValue1;
int sensorValue2;
int sensorValue3;
int dirValue;

int counter = 0;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  while (counter < 1) {
    for (int i = 400; i <= 600; i += 100) {
        for (int j = 400; j <= 600; j += 100) {
          for (int k = 400; k <= 600; k += 100) {
            for (int l = 0; l <= 1; l++) {
              sensorValue1 = i;
              sensorValue2 = j;
              sensorValue3 = k;
              dirValue = l;
              
              Serial.println(i);
              Serial.println(j);
              Serial.println(k);
              Serial.println(l);
    
              wristAng = moveWrist(sensorValue1, dirValue, wristAngPort, wristAng, WRIST_ANGLE_MIN, WRIST_ANGLE_MAX);
              wristRot = moveWrist(sensorValue2, dirValue, wristRotPort, wristRot, WRIST_ROT_MIN, WRIST_ROT_MAX);
              wristGripper = moveWrist(sensorValue3, dirValue, wristGripperPort, wristGripper, WRIST_GRIPPER_MIN, WRIST_GRIPPER_MAX);
              Serial.print(wristAng);
              Serial.print(", ");
              Serial.print(wristRot);
              Serial.print(", ");
              Serial.println(wristGripper);
            }
            Serial.println();
          }
        }
      }
  counter++;
  }
}

int moveWrist(int value, int posNeg, int portnum, int pos, int thismin, int thismax)
{
  if (value > threshold){
    if (posNeg){
      pos += incWrist;
      if (pos > thismax){
        pos-= incWrist;
      }
      else{
        // pwm.setPWM(portnum, 0, pos);
      }
    }
    else{
      pos -= incWrist;
      if(pos < thismin){
        pos += incWrist;
      }
      else{
        // pwm.setPWM(portnum, 0, pos);
      }
    }
  }
  delay(50);
  return pos;
}
