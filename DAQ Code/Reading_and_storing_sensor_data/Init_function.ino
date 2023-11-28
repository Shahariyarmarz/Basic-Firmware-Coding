void set_pinMode(void){
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(led_red, OUTPUT);
  pinMode(PIN_VBAT, INPUT);
  pinMode(led_yellow, OUTPUT);
}


void blink_LED_RED(void) {
  for(uint8_t i = 1; i < 4; i++){
    digitalWrite(led_red, HIGH);
    delay(250);
    digitalWrite(led_red, LOW);
    delay(250);
  }
}


void blink_LED_Yellow (void) {
  for (uint8_t i = 1; i < 4; i++) { // loop for blinking the LED 3 times
    digitalWrite(led_yellow, HIGH);
    delay (250); // delays of 0.5 to see the blinking
    digitalWrite(led_yellow, LOW);
    delay (250); // delays of 0.5 to see the blinking
  }
}



void battery_check (void) {
  float batteryLevel = (analogRead(PIN_VBAT) * 3.3f / 4095.0f) / (1.2f / (1.2f + 0.33f));
  if (batteryLevel > battery_threshold) { // battery_threshold is a global variable
    digitalWrite(led_yellow, HIGH); // Yellow LED is turned on
    low_battery_flag = 0; // clears the low battery flag
  }
  else {
    digitalWrite(led_yellow, LOW); // Yellow LED is turned off
    low_battery_flag = 1; // Raises the low_battery_flag, which indicates that the battery level is below the thershold
  }
}




void set_SD_card(void){

  pinMode(14, INPUT_PULLUP);	//manuell switch to enable SD CARD
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, chipSelect);
  // const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  // char fileName[13] = FILE_BASE_NAME "00.csv";

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. We have to try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }
  Serial.println("card initialized.");

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {   // check for the unity position of the file number
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {   // check for the decimal position of the file number
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  blink_LED_RED();  // blinks the Red LED 3 times
}