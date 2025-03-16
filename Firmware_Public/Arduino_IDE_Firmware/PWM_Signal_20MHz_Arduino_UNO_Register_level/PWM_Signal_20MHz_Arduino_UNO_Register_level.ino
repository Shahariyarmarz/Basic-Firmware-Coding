#include "mypwm.h"

mypwm pwmController; // Create an instance of MyPWM

void setup() {
    Serial.begin(9600);
    pwmController.setFrequency(20000);  // Set PWM frequency to 20 kHz
    Serial.println("PWM initialized");
}

void loop() {
    pwmController.setDutyCycle(50.0, OCR1A); // Set 50% duty cycle
    delay(1000);
}