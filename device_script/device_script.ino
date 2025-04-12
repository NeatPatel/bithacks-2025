#include <TinyGPS++.h>

// Define a general purpose undefined value
#define UNDEFINED_VALUE 0xFFFFFFFF

// Might need to swap the 44 and 43
#define RXD2 44
#define TXD2 43
HardwareSerial neogps(1);

// Create GPS instance
TinyGPSPlus gps;

unsigned int longitude = UNDEFINED_VALUE;
unsigned int latitude = UNDEFINED_VALUE;
unsigned int target_lng = UNDEFINED_VALUE;
unsigned int target_lat = UNDEFINED_VALUE;

// Angle in degrees
unsigned int angle = UNDEFINED_VALUE;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // begin the GPS serial communication
  // SERIAL_8N1 basically is the bit protocol definer, (i.e. 8 bits, no parity bits, one stop bit, etc.)
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD1);

  // Set default value of angle to zero
  angle = 0;

  // Wait 2 seconds, possibly optional
  delay(2000);
}

void loop() {
  boolean newData = false;
  unsigned int wait_time = 1000;

  // Check every "wait_time" milliseconds
  for(unsigned long start = millis(); millis() - start < wait_time;) {
    // Make sure the GPS is present
    while(neogps.available()) {
      // Check if there was a change in the data
      if(gps.encode(neogps.read())) {
        newData = true;
      }
    }
  }

  // if the newData is true
  if(newData == true) {
    // Reset the newData for future
    newData = false;
    // Print GPS satellite value to console
    Serial.println(gps.satellites.value());
    // Call print_data function (below)
    set_long_lat();
  }

  // Note for latitude/longitude program user:
  /*
    The latitude and longitude variables are defined initially as UNDEFINED_VALUE.
    This means that you cannot utilize them as though they are always initialized (for example to 0)
    When they are undefined, you may assume compass must point North, and connection is pending.
  */
  getAngle();
}

void set_long_lat() {
  // Check if the location is valid, if so, set the longitude and latitude variables
  if(gps.location.isValid() == 1) {
    longitude = gps.location.lng();
    latitude = gps.location.lat();

    // Temporary target location initialization, use button instead
    if(target_lng == UNDEFINED_VALUE) {
      target_lng = longitude;
      target_lat = latitude;
    }
  }
}

// Get the angle in degrees if latitude and longitude are defined
void getAngle() {
  if(latitude != UNDEFINED_VALUE && latitude != UNDEFINED_VALUE) {
    unsigned long x = target_lng - longitude;
    unsigned long y = target_lat - latitude;

    // Math formula from calculations of angle based on reference of North
    if(y == 0) {
      if(target_lat > latitude) {
        angle = 90;
      } else if(target_lat < latitude) {
        angle = -90;
      } else {
        angle = 0;
      }
    } else {
      angle = radiansToDeg(atan2(x/y));
    }
  }
}

unsigned long radiansToDeg(unsigned long radians) {
  // Convert to float for accurate computation
  float deg = (float)radians * (180.0 / PI);
  return (unsigned long)deg;
}

