#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Preferences.h>
#include "TinyGPS++.h"
#include <Adafruit_LSM6DSOX.h>
#include <Stepper.h>

using namespace std;


#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

#define RXD2 16
#define TXD2 17

// For SPI mode, we need a CS pin
#define LSM_CS 10   // CS pin
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 12  // SCL pin
#define LSM_MISO 13 // DO pin
#define LSM_MOSI 11 // SDA pin

const int LCD_SDA = 8;
const int LCD_SCL = 42;
const int POT_PIN = 3;
const int BUTTON = 41;
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

Preferences prefs;
LiquidCrystal_I2C lcd(0x27, 16, 2);
HardwareSerial neogps(1);
TinyGPSPlus gps;
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);


/*
  state is selection between choosing checkpoint or new checkpoint
  scrolls through different Lodestone options and able to select them
  they all being as "unsaved" then saves current location after button press
  and then becomes "saved". If you ever press button on a saved one, it becomes 
  the new tracking target. The state will then be "tracking". The needle will point
  to this new target.
*/
// 0 = unsaved 1 = saved 2 = tracking
int lodestate1 = 0; //First lodestone state 
int lodestate2 = 0; //Second loadstone state
int lodestate3 = 0; //Third loadstone state

int potVal = 0;

long angle = 0;

double targetLat = 33.645939, targetLong = -117.846511;  

// Use float for lat/lng (for decimal precision)
float longitude = -117.842723;
float latitude = 33.649478;

// IDK wht this is, the tutorial has it...
Adafruit_LSM6DSOX sox;

// varible to store yawAngle
static double yawAngle = 0;

// Get angle offset values
double xDegOffset;
double yDegOffset;
double zDegOffset;

// Setting up gyro sensor config
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;


bool buttonPress(){
  delay(100);
  return digitalRead(BUTTON) == 0;
}


void printLodestate(int loadstate){
  if(loadstate == 0){
    lcd.setCursor(0,1);
    lcd.print("Missing Coords     ");
  }
  if(loadstate == 1){
    lcd.setCursor(0,1);
    lcd.print("Saved Coords       ");
  }
  if(loadstate == 2){
    lcd.setCursor(0,1);
    lcd.print("Tracking Coords    ");
  }
}

void calibrate(sensors_event_t gyroSensor){
  Serial.println("Currently Calibrating...");
  // Basically getting the average values, after timePasses seconds of reading

  unsigned long timePassed = 10 * 1000; // calibration time
  unsigned long timeStart = millis();

  // Acumulators for the average readings
  //double xTotal = 0;
  //double yTotal = 0;
  double zTotal = 0;

  // Counter for how many readings occured
  int counter = 0;

  while (millis() - timeStart < timePassed)
  {
    //xTotal += gyroSensor.gyro.x;
    //yTotal += gyroSensor.gyro.y;
    zTotal += gyroSensor.gyro.z;

    counter++;
  }

  // Comput averages, store to pointers
  //xDegOffset = xTotal / counter;
  //yDegOffset = yTotal / counter;
  zDegOffset = zTotal / counter;

  //Serial.println(zDegOffset);

  Serial.println("Finish Calibrating.");
}

// NOTE, prevAngle is a global variable...
// For yaw angle, only focus on z-axis.
double updateYawAngle(sensors_event_t gyroSensor)
{
  // Constant to multiply the angle measurement
  double scaleFactor = 9.2;
  
  // Get the theta, because I have spent 10 hours
  // on configuring the sensor, and I really don't
  // have the mental capacity to explain how it works anymore
  double theta = gyroSensor.gyro.z - zDegOffset;

  yawAngle += (scaleFactor * theta);

  // Serial.println(scaleFactor * theta);
  // Serial.println(yawAngle);

  // Angle can only be between (-180, 180), where
  // - (0, 180) if rotating clockwise
  // - (-180, 0) if rotating counterclockwise
  if (yawAngle > 180)
  {
    // Update angle to negative
    yawAngle = -180 + (yawAngle - 180);
  }
  if (yawAngle < -180)
  {
    // update to positive
    yawAngle = 180 + (yawAngle + 180);
  }

  // Negat the yawAngle, to match the sign with the servo...
  return -1 * yawAngle;
}

void adjustNeedle(){
  sox.getEvent(&accel, &gyro, &temp);
  /* Display the results (rotation is measured in rad/s) */
  unsigned int wait_time = 1000;
  getAngle();
  Serial.println(updateYawAngle(gyro));
}


int getTarget(){
  if(lodestate1 == 2){
    return 1;
  }
  if(lodestate2 == 2){
    return 2;
  }
  if(lodestate3 == 2){
    return 3;
  }
  return 0;
}

void getAngle() {
  if (!isnan(latitude) && !isnan(longitude) && !isnan(targetLat) && !isnan(targetLong)) {
    // float x = target_lng - longitude;
    // float y = target_lat - latitude;

    float newAngle = getBearingToTarget(latitude, longitude, targetLat, targetLong);

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

  sox.getEvent(&accel, &gyro, &temp); //needs to be tested
  // Convert the deltaTheta to steps
  int steps = (int)(conversionFactor * deltaTheta);


  // Step the motor
  myStepper.step(steps);

}

void updateCoords(int num){
  if (gps.location.isValid()) {
    Serial.println("GPS LOCATION VALID");
    longitude = gps.location.lng(); // lng = longitude
    latitude = gps.location.lat();  // lat = latitude
  } else {
    Serial.println("Location invalid.");
  }

  String latKey = "lat" + String(num);
  String longKey = "long" + String(num);

  double existingLat = prefs.getDouble(latKey.c_str(), -999.0);
  double existingLong = prefs.getDouble(longKey.c_str(), -999.0);

  // Only store if they haven't been set before
  if (existingLat == -999.0 && existingLong == -999.0) {
    prefs.putDouble(latKey.c_str(), latitude);
    prefs.putDouble(longKey.c_str(), longitude);
    Serial.println("New coordinates stored.");
  } else {
    Serial.println("Coordinates already exist. Skipping write.");
  }

  prefs.end();
}

void updateTarget(int num){
  String latKey = "lat" + String(num);
  String longKey = "long" + String(num);

  /*if(prefs.getDouble(latKey.c_str(), -999.0) != -999.0 && prefs.getDouble(longKey.c_str(), -999.0) != -999.0){
    targetLat = prefs.getDouble(latKey.c_str(), -999.0);
    targetLong = prefs.getDouble(longKey.c_str(), -999.0);
  }*/
}

int updateLodestate(int& loadstate){
    if(loadstate != 2){
      ++loadstate;
    }
    if(loadstate == 2){
      if(lodestate1){
        lodestate1 = 1;
      }
      if(lodestate2){
        lodestate2 = 1;
      }
      if(lodestate3){
        lodestate3 = 1;
      }
      loadstate = 2;
      updateTarget(getTarget());
    }
  return loadstate;
}

void menu(){
  potVal = analogRead(POT_PIN);
  if(potVal <= 30){
      lcd.setCursor(0,0);
      lcd.print("Use Dial to     ");
      lcd.setCursor(0,1);
      lcd.print("Select Lodestone   ");
  }
  else if(potVal <= 1200){
    lcd.setCursor(0,0);
    lcd.print("Lodestone 1      ");
    printLodestate(lodestate1);
    if(buttonPress()){
        updateLodestate(lodestate1);
        printLodestate(lodestate1);
        updateCoords(1);
      }
  }
  else if(potVal <= 2400){
    lcd.setCursor(0,0);
    lcd.print("Lodestone 2      ");
    printLodestate(lodestate2);
    if(buttonPress()){
        updateLodestate(lodestate2);
        printLodestate(lodestate2);
        updateCoords(2);
      }
  }
  else if(potVal <= 3600){
    lcd.setCursor(0,0);
    lcd.print("Lodestone 3       ");
    printLodestate(lodestate3);
    if(buttonPress()){
        updateLodestate(lodestate3);
        printLodestate(lodestate3);
        updateCoords(3);
      }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  myStepper.setSpeed(10);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(500);  // Give time for Serial to initialize

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C()) {
    if (!sox.begin_SPI(LSM_CS)) {
      if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
        Serial.println("Failed to find LSM6DSOX chip");
        Serial.print("MOSI: ");
        Serial.println(MOSI);
        Serial.print("SCK: ");
        Serial.println(SCK);
        Serial.print("MISO: ");
        Serial.println(MISO);

        while (1) {
          delay(10);
        }
      }
    }
  }

  Serial.println("LSM6DSOX Found!");

  // Set the offset values as zero
  xDegOffset = 0;
  yDegOffset = 0;
  zDegOffset = 0;

  // Something from the Adafruit's tutorial???
  sox.getEvent(&accel, &gyro, &temp);

  calibrate(gyro);
}


void loop() {
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  menu();
  adjustNeedle();
  Serial.print("Potval: ");  
  Serial.println(potVal);
}
