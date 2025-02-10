#include "Keyboard.h"

const int buttonPin = 4;         // input pin for pushbutton
int previousButtonState = HIGH;  // for checking the state of a pushButton

void setup() {
  // make the pushButton pin an input:
  pinMode(buttonPin, INPUT);
  // initialize control over the keyboard:
  Keyboard.begin();
}

void loop() {
  // read the pushbutton:
  int buttonState = digitalRead(buttonPin);
  // if the button state has changed,
  if ((buttonState != previousButtonState)
      // and it's currently pressed:
      && (buttonState == HIGH)) {

    // type out a message
    Keyboard.press(KEY_RIGHT_ALT);
    delay(500);
    Keyboard.press('R');
    delay(200);
    Keyboard.release('R');
    delay(100);
    Keyboard.releaseAll();
  }
  // save the current button state for comparison next time:
  previousButtonState = buttonState;
}