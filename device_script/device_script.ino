#include "TinyGPS++.h"
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 21

// Serial pins for GPS module
#define RXD2 16
#define TXD2 17

// Define classes
HardwareSerial neogps(1);

// GPS instance
TinyGPSPlus gps;

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// Use float for lat/lng (for decimal precision)
float longitude = NAN;
float latitude = NAN;
float target_lng = NAN;
float target_lat = NAN;

// Angle in degrees
unsigned long angle = 0;

void setup() {
  // set the speed at 5 rpm
  myStepper.setSpeed(10);

  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  delay(2000); // Wait for GPS module to boot
  Serial.println("Starting GPS...");
}

void loop() {
  bool newData = false;
  unsigned int wait_time = 1000;

  // Read for wait_time milliseconds
  for (unsigned long start = millis(); millis() - start < wait_time;) {
    while (neogps.available()) {
      char c = neogps.read();
      //Serial.write(c); // Print raw NMEA for debugging

      if (gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    newData = false;
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    set_long_lat();
  }

  getAngle();
}

void set_long_lat() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("Latitude: ");
    Serial.println(latitude, 8);
    Serial.print("Longitude: ");
    Serial.println(longitude, 8);

    // Set target position only once
    if (isnan(target_lat) || isnan(target_lng)) {
      target_lat = latitude;
      target_lng = longitude;
      Serial.println("Target location set.");
    }
  } else {
    Serial.println("Location invalid.");
  }
}

void getAngle() {
  if (!isnan(latitude) && !isnan(longitude) && !isnan(target_lat) && !isnan(target_lng)) {
    float x = target_lng - longitude;
    float y = target_lat - latitude;

    unsigned long newAngle = radiansToDeg(atan2(x, y));
    
    // Set motor direction HERE
    setMotorDirection((newAngle - angle));

    angle = radiansToDeg(atan2(x, y));

    Serial.print("Angle: ");
    Serial.println(angle);
  } else {
    Serial.println("Angle not calculated — missing GPS fix.");
  }
}

unsigned long radiansToDeg(float radians) {
  return ((unsigned long)(radians * 180.0 / PI) % 360);
}

// Rotate the motor accordingly, assume the angle is always between 0 and 360 DEGREES
void setMotorDirection(unsigned long deltaTheta) {
  unsigned long plus = deltaTheta + 360;
  unsigned long minus = deltaTheta + 360;
  if(abs((int)deltaTheta) > abs((int)plus)) {
    deltaTheta += 360;
  } else if(abs((int)deltaTheta) > abs((int)minus)) {
    deltaTheta -= 360;
  }

  float conversionFactor = 2048 / 360.0;

  // Convert the deltaTheta to steps
  int steps = (int)(conversionFactor * deltaTheta);

  // Step the motor
  myStepper.step(steps);

  delay(1000);
}
