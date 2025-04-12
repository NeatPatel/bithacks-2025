#include <LiquidCrystal_I2C.h>
#include <Wire.h>

const int POT_PIN = 4;  // Use an ADC-capable pin like 34
const int LCD_SDA = 8;
const int LCD_SCL = 7;

LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup() {
  Serial.begin(115200);
  lcd.init();
  pinMode(POT_PIN, INPUT);
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.backlight();
  delay(1000);  // Give time for Serial to initialize
}

void loop() {
  int potValue = analogRead(POT_PIN);  // Read raw ADC value (0-4095)
  lcd.clear();                 // clear display
	lcd.setCursor(0, 0);         // move cursor to   (0, 0)
	lcd.print("Hello");          // print message at (0, 0)
	lcd.setCursor(2, 1);         // move cursor to   (2, 1)
	lcd.print("World"); 
  delay(1000);   
  // Optionally, convert to voltage (ESP32 ADC = 12-bit, 3.3V range)
  Serial.print("Raw: ");
  Serial.println(potValue);

  delay(200);
}
