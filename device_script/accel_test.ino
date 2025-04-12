#include <SPI.h>
#include <Arduino_LSM6DSOX.h>

constexpr int PIN_SCK 36;
constexpr int PIN_MISO 37;
constexpr int PIN_MOSI 35;
constexpr int PIN_CS 7;

Arduino_LSM6DSOX IMU;
SPIClass LSM_SPI(FSPI);


bool setupIMU() {
    return IMU.begin_SPI(PIN_CS, LSM_SPI);
}

void setup(){
    Serial.begin(115200);
    LSM_SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);  // Deselect LSM6DSOX
    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");

        while (1);
    }

    if(!setupIMU()){
        Serial.println("Failed to initialize SPI!");
    }
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Gyroscope in degrees/second");
    Serial.println("X\tY\tZ");
    // Initialize sensor library here
}

void loop(){
    float x, y, z;

    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
    }
}