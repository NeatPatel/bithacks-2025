#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// Button pin definitions
#define BTN1 13
#define BTN2 12

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void setup() {
  // set the speed at 5 rpm
  myStepper.setSpeed(10);
  // initialize the serial port
  Serial.begin(115200);

  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
}

void loop() {
  int buttonState = digitalRead(BTN1);

  // If left button is on, rotate clockwise
  if(buttonState == HIGH) {
    rotate_c();
  } else {
    buttonState = digitalRead(BTN2);

    // If right button is on, rotate counterclockwise
    if(buttonState == HIGH) {
      rotate_cc();
    }
  }
}

void rotate_c() {
  // step once in clockwise direction:
  Serial.println("clockwise");

  for(int i = 0; i < 360; ++i) {
    myStepper.step();
  }

  delay(1000);
}

void rotate_cc() {
  // step once in the counterclockwise direction:
  Serial.println("counterclockwise");
  myStepper.step(-512);
  delay(1000);
}

