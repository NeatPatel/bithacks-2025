// Basic demo for accelerometer & gyro readings from Adafruit
// LSM6DSOX sensor

#include <Adafruit_LSM6DSOX.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 12
#define LSM_MISO 13
#define LSM_MOSI 11

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

// Define functions
double updateYawAngle(sensors_event_t gyroSensor);

void calibrate(sensors_event_t gyroSensor);

double formatYawAngle(void);


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // Set the offset values as zero
  xDegOffset = 0;
  yDegOffset = 0;
  zDegOffset = 0;

  // Something from the Adafruit's tutorial???
  sox.getEvent(&accel, &gyro, &temp);

  calibrate(gyro);
}

void loop() {
  sox.getEvent(&accel, &gyro, &temp);

  /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.println(updateYawAngle(gyro));
}

void calibrate(sensors_event_t gyroSensor)
{
  Serial.println("Currently Calibrating...");
  // Basically getting the average values, after timePasses seconds of reading

  unsigned long timePassed = 10 * 1000; // calibration time
  unsigned long timeStart = millis();

  // Acumulators for the average readings
  double xTotal = 0;
  double yTotal = 0;
  double zTotal = 0;

  // Counter for how many readings occured
  int counter = 0;

  while (millis() - timeStart < timePassed)
  {
    xTotal += gyroSensor.gyro.x;
    yTotal += gyroSensor.gyro.y;
    zTotal += gyroSensor.gyro.z;

    counter++;
  }

  // Comput averages, store to pointers
  xDegOffset = xTotal / counter;
  yDegOffset = yTotal / counter;
  zDegOffset = zTotal / counter;

  Serial.println(zDegOffset);

  Serial.println("Finish Calibrating.");
}

// NOTE, prevAngle is a global variable...
double updateYawAngle(sensors_event_t gyroSensor)
{
  // For yaw angle, only focus on z-axis.
  //  - angle used trapezoidal apporox, where
  //.   b1: first angle reading
  //.   b2: second angle reading
  //.    h: sampleRate

  // Constant to format angle
  // 33: so every 90 degrees, the value is 1.
  unsigned float sensorConst = 33;
  float sampleRate = 0.00001 * sensorConst;

  // Read the z value
  float base1 = gyroSensor.gyro.z;
  // Serial.print("1 ");
  // Serial.println(base1);

  // delay the same amout as sampling rate
  delay(sampleRate * 1000);

  // Read z value again
  float base2 = gyroSensor.gyro.z;
  // Serial.print("2 ");
  // Serial.println(base2);

  // Find area uder curve (area of trapezoid):
  double area = (base1 + base2 + zDegOffset) * (0.5);
  //Serial.print("dTheta: ");
  //Serial.println(area);
  area *= sampleRate;

  //Serial.println(area);

  // update prevAngle
  yawAngle += area;

  // Reset value if yawAngle > 4, or yawAngle < -4
  if (yawAngle > 4)
  {
    yawAngle -= 4;
  }
  if (yawAngle < -4)
  {
    yawAngle += 4;
  }

  //Serial.println(yawAngle);

  double output = formatYawAngle();

  return output;
}

double formatYawAngle(void)
{
  // yawAngle is 1 when 90 degrees, so the range should be
  // from (-2, 2). 

  // Get the true angle, where
  // left side of circle is positive
  // right side of circle is negative
  // both side range from 0-180 deg
  

  double formatAngle = yawAngle;

  // format if negative
  if (formatAngle > 2)
  {
    // Update angle to negative
    formatAngle = -2 + (formatAngle - 2);
  }
  if (formatAngle < -2)
  {
    // update to positive
    formatAngle = 2 + (formatAngle + 2);
  }

  // Multiply 90 to correct format
  formatAngle *= 90;

  return formatAngle;
}
