// wall_following.ino

// A - Forward Direction 
// B - Backward Direction 
// LMotorA - Left Motor Forward Direction

#include "wall_following_functions.h"

void setup()
{

  Serial.begin(9600);

  // Initialize Ultrasonic sensors
  initializeUltrasonicSensors();

  // Initialize Motors
  initializeMotors();
  
  // Give signals after defining pinModes
  shock_forward(); 
}

void loop()
{
  // ---------------- Sasika Changes -----------------------
  delay(500); // I added some delay
  // shock();  I commented this 

  // Move forward while keeping the desired distance from the right side
  wpid_foward();
}







