// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 02;

// variable for storing the potentiometer value

void setup() {
  Serial.begin(115200);
  //delay(10);
}

void loop() {
  // Reading potentiometer value
  int potValue = analogRead(potPin);
  Serial.println(potValue);
  delay(10);
}
