#include <TinyGPS++.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

/*
  10, 11, 12, 13
*/

// ULN2003 Motor Driver Pins
#define IN1 4 // 19 for ESP 32, 4 for ESP 32 S3
#define IN2 5 // 18 for ESP 32, 5 for ESP 32 S3
#define IN3 6 // 5 for ESP 32, 6 for ESP 32 S3
#define IN4 7 // 21 for ESP 32, 7 for ESP 32 S3

// Serial pins for GPS module
#define RXD2 16 // 16 for ESP 32, 44 for ESP 32 S3
#define TXD2 17 // 17 for ESP 32, 43 for ESP 32 S3

// Define classes
HardwareSerial neogps(1);

// GPS instance
TinyGPSPlus gps;

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN2, IN3, IN4);

// Use float for lat/lng (for decimal precision)
float longitude = 74.0060; // -117.837425;
float latitude = 40.7128; // 33.646526;
float target_lng = 99.1332; // -117.83924;
float target_lat = 19.4326; // 33.646526

// Angle in degrees
long angle = 0;

void setup() {
  #if CONFIG_USB_CDC_ENABLED
  Serial.setTxTimeoutMs(0);  // Optional: prevents USB CDC blocking
  #endif

  //Serial.begin(115200);

  // set the speed at 5 rpm
  myStepper.setSpeed(10);

  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  delay(5000); // Wait for GPS module to boot
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
    Serial.println(gps.location.lat());
    set_long_lat();
  }

  getAngle();
}

void set_long_lat() {
  if (gps.location.isValid()) {
    Serial.println("GPS LOCATION VALID");

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
    // float x = target_lng - longitude;
    // float y = target_lat - latitude;

    float newAngle = getBearingToTarget(latitude, longitude, target_lat, target_lng);

    Serial.print("New Angle: ");
    Serial.println(newAngle);
    
    // Set motor direction HERE
    setMotorDirection((newAngle - angle));

    angle = newAngle;

    Serial.print("Angle: ");
    Serial.println(angle);
  } else {
    Serial.println("Angle not calculated — missing GPS fix.");
  }
}

float getBearingToTarget(float latitude, float longitude, float target_lat, float target_lng) {
  // Convert degrees to radians
  float lat1 = radians(latitude);
  float lat2 = radians(target_lat);
  float deltaLon = radians(target_lng - longitude);

  float x = sin(deltaLon) * cos(lat2);
  float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);

  float bearingRad = atan2(x, y);
  float bearingDeg = degrees(bearingRad);

  // Normalize to [-180, 180]
  if (bearingDeg > 180.0) bearingDeg -= 360.0;
  if (bearingDeg < -180.0) bearingDeg += 360.0;

  return bearingDeg;
}

long radiansToDeg(float radians) {
  return ((long)(radians * 180.0 / PI));
}

// Rotate the motor accordingly, assume the angle is always between 0 and 360 DEGREES
void setMotorDirection(float deltaTheta) {
  float plus = deltaTheta + 360;
  float minus = deltaTheta - 360;
  if(abs((int)deltaTheta) > abs((int)plus)) {
    deltaTheta += 360;
  } else if(abs((int)deltaTheta) > abs((int)minus)) {
    deltaTheta -= 360;
  }

  float conversionFactor = 2048 / 360.0;

  // Convert the deltaTheta to steps
  int steps = (int)(conversionFactor * deltaTheta);

  Serial.print("Steps: ");
  Serial.println(steps);

  // Step the motor
  myStepper.step(steps);

  delay(1000);
}
