//#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
//#include <Math.h>
//
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//
//const int analogInPin1 = A0;
//const int analogInPin2 = A1;
//const int analogInPin3 = A2;
//
//const int buttonDir = 2;
//const int buttonWrist = 4;
//
//int threshhold = 250;
//
//int sensorValue1 = 0;
//int sensorValue2 = 0;
//int sensorValue3 = 0;
//
//int wristAngMin = 100;
//int wristRotMin = 200; //guess
//int clawPosMin = 150;
//
//int wristAngMax = 585;
//int wristRotMax = 400; //guess
//int clawPosMax = 600;
//
//int wristAng = 350;
//int wristRot = 300;
//int clawPos = 370;
//int negPos;
//int wrist;
//int wristAngPort = 0;
//int wristRotPort = 1;
//int clawPosPort = 2;
//void setup()
//{
//  Serial.begin(9600);
//  pwm.begin();
//  pwm.setPWMFreq(1600);
//  
//  if( buttonWrist){ //Setting all to middle, can do starting value if we want
//    pwm.setPWM(wristAngPort, 0, wristAng);
//    pwm.setPWM(wristRotPort, 0, wristRot);
//    pwm.setPWM(clawPosPort, 0, clawPos);
//  }
//}
//    
//
//void loop()
//{
//  // Link EMG voltage to velocity
//	
//  // get EMG sensor values
//  sensorValue1 = analogRead(analogInPin1);
//  sensorValue2 = analogRead(analogInPin2);
//  sensorValue3 = analogRead(analogInPin3);
//  
//  negPos = digitalRead(buttonDir);
//  wrist = digitalRead(buttonWrist);
//  
//  wristAng = checkThresh(sensorValue1,negPos, wrist, wristAngPort, wristAng, wristAngMin,wristAngMax);
//  wristRot = checkThresh(sensorValue1,negPos, wrist, wristRotPort, wristRot, wristRotMin,wristRotMax);
//  clawPos = checkThresh(sensorValue1,negPos, wrist, clawPosPort, clawPos, clawPosMin,clawPosMax);
//  
//}
//
//int checkThresh (int value, int posNeg, int wristTrue, int portnum, int pos, int thismin, int thismax ){
//  if (value > threshhold){
//    if (wristTrue){
//    if(posNeg){
//      pos += 5;
//      if (pos > thismax){
//        pos-= 5;
//      }
//       else{
//         pwm.setPWM(portnum, 2000, pos+2000);
//       }
//    }
//    else{
//      pos-=5;
//      if(pos <thismin){
//        pos+= 5;
//      }
//      else{
//        pwm.setPWM(portnum, 2000, pos+2000);
//      }
//    }
//  }
//  }
//  delay(50);
//  return pos;
//  }
//    
//    
