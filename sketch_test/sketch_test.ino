/*
This program tests the code that reads EMG values and
outputs the desired XYZ location accordingly.
*/

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

// Thresholds
int xThresh = 500;
int yThresh = 500;
int zThresh = 500;

// XYZ velocity increments
const int incX = 5;
const int incY = 5;
const int incZ = 5;

int sensorValue1 = 0; 
int sensorValue2 = 0;
int sensorValue3 = 0;
int dirValue = 0;

int counter = 0;

void setup()
{
  Serial.begin(115200);
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.print(", ");
  Serial.println(currentZ);
}

void loop()
{
  if (counter == 0) {
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

            moveX = checkThresh(sensorValue1, xThresh);
            moveY = checkThresh(sensorValue2, yThresh);
            moveZ = checkThresh(sensorValue3, zThresh);
            updateVelocities();
            if (dirValue) moveToPosition(currentX + velX, currentY + velY, currentZ + velZ);
            else moveToPosition(currentX - velX, currentY - velY, currentZ - velZ);
          }
          Serial.println();
        }
      }
    }
  }
  counter++;
}

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

void moveToPosition(int x, int y, int z) {
  currentX = x;
  currentY = y;
  currentZ = z;
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.print(", ");
  Serial.println(currentZ);
}
