/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground through 220 ohm resistor
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 42;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin
const uint8_t button_frequency = 4;
const uint8_t button_press_threshold = 3;

// variables will change:
// int buttonState = 0;  // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  // pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  // read the state of the pushbutton value:
  // int buttonState = digitalRead(buttonPin);
  uint8_t counter = 0;
  int p = 0;
  // Serial.println(p);
  while (digitalRead(buttonPin)) {
    counter++; 
    Serial.println(counter);
    delay(1000 / button_frequency);
    if (counter == button_press_threshold * button_frequency) {
    // Serial.println(p);
    // Serial.println(counter);
    Serial.println("IF condition");
    }
    else {
    Serial.println("Else condition");
    }
  }
  
  
  // Serial.println(buttonState);

  // // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  // if (buttonState == HIGH) {
  //   // turn LED on:

  //   // digitalWrite(ledPin, HIGH);
    
  // } else {
  //   // turn LED off:
  //   // digitalWrite(ledPin, LOW);
    
  // }
}
