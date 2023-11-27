/*--------------------------------------------------------- Function to check long button press to start the device -------------------------------------------------
   This function checks if the button is pressed for more than a threshold duration to start the device.
   This is done by reading the button pin and checking if it is continuously high for more than that period.
   LED will blink when the threshold time will pass and then turn off
   When the button is released the LED will turn on again and the data acquisition will start
*/
void DAQ_start (void) {
  uint8_t counter = 0;
  while (digitalRead(buttonPin)) {
    counter++; // increments the counter
    delay(1000 / button_frequency); // delay (ms) before reading the button_pin again if the button was pressed during the previous reading
    if (counter == button_press_threshold * button_frequency) {
      /* Following function will check the battery level
          a. If it is below a threshold value, the program will be stucked here and wait for the battery level to increase.
             When that happens, it will wait for the button press to start the DAQ.
          b. If battery level is fine, the LED will blink 3 times and then turned off and the program will move to the next line
      */
      battery_check ();

      if (low_battery_flag == 0) {
        blink_LED_RED();// this function blinks the red LED 3 times and then turns it off
        while (digitalRead(buttonPin)); // the program won't move until the button is unpressed
        digitalWrite(led_red, HIGH); // turn on the red LED
        start_flag = 0; // clears the start flag
        stop_flag = 0; // clears the stop flag
        //we have to start a timer for ESP32
        // start_timer(timer3_freq); // starts the timer3 for data acquisition after the button is released
      }
    }
  }
  
  if (low_battery_flag == 1) {
    while (analogRead(PIN_VBAT) <= battery_threshold); // Program will wait here until the battery level is above the threshold value
    blink_LED_Yellow (); // this function blinks the yellow LED 3 times and then turns it off
    digitalWrite(led_yellow, HIGH); // Yellow LED is turned on
    low_battery_flag == 0; // clears the low battery flag
  }
}

/*-------------------------------------------------------------- Function to read the analog sensors ------------------------------------------------------------------
   This function reads the analog sesors and stores them in a buffer
   If any data from the buffer is copied in the loop() function, those data are removed from the buffer and buffer index is updated
*/
void read_analog_sensors(void) {

  // Loop for deleting the elements being copied in the temporary ADC_copy_buf and shifing the rest of the data
  if (ADC_copy_flag == 1) {
    /* If flag is set, the copying operation in the loop() function has been performed
      Therefore, it is necessary to delete those data and shift the uncopied data to the beginning of the buffer
      To do this, the index value is reduced by button_copy_size so that if the previous value of buf_index was 512, 0-511 elements will be deleted and 512 index element will be 0 index element
    */
    ADC_buf_indx = ADC_buf_indx - ADC_copy_size;
    for (uint16_t j = ADC_copy_size; j < ADC_buf_indx + ADC_copy_size; j++ ) {
      ADC_buf [j - ADC_copy_size] = ADC_buf [j];
    }

    ADC_copy_flag = 0; // Resetting the copy_flag
  }


  // Reading the analog pins (01-08 and 11,12)
  // For 1024 Hz sampling speed
  ADC_buf[ADC_buf_indx++] = (analogRead(A0) + analogRead(A0) + analogRead(A0)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A1) + analogRead(A1) + analogRead(A1)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A2) + analogRead(A2) + analogRead(A2)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A3) + analogRead(A3) + analogRead(A3)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A4) + analogRead(A4) + analogRead(A4)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A5) + analogRead(A5) + analogRead(A5)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A6) + analogRead(A6) + analogRead(A6)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A7) + analogRead(A7) + analogRead(A7)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A10) + analogRead(A10) + analogRead(A10)) / 3;
  ADC_buf[ADC_buf_indx++] = (analogRead(A11) + analogRead(A11) + analogRead(A11)) / 3;

  // For 512 Hz sampling speed
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A0) + analogRead(A0) + analogRead(A0) + analogRead(A0) + analogRead(A0) + analogRead(A0)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A1) + analogRead(A1) + analogRead(A1) + analogRead(A1) + analogRead(A1) + analogRead(A1)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A2) + analogRead(A2) + analogRead(A2) + analogRead(A2) + analogRead(A2) + analogRead(A2)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A3) + analogRead(A3) + analogRead(A3) + analogRead(A3) + analogRead(A3) + analogRead(A3)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A4) + analogRead(A4) + analogRead(A4) + analogRead(A4) + analogRead(A4) + analogRead(A4)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A5) + analogRead(A5) + analogRead(A5) + analogRead(A5) + analogRead(A5) + analogRead(A5)) / 6;
  //  ADC_buf[ADC_buf_indx++] = (analogRead(A6) + analogRead(A6) + analogRead(A6) + analogRead(A6) + analogRead(A6) + analogRead(A6)) / 6;


  // Reading the button press data
  uint16_t button_value = digitalRead(buttonPin);

  // Combining button_vlaue with the ADC_count
  uint16_t combo = (button_value << 8) | ADC_count;
  // 8 MSB of combo will hold button value and 8 LSB of combo will hold ADC_count
  // ADC_count will be used to check tha data drop

  // Storing the combo in the ADC_buf for writing to SD card
  ADC_buf[ADC_buf_indx++] = combo;

  // resetting the ADC count
  ADC_count = ADC_count + 1;
  if (ADC_count > 255) {
    ADC_count = 0;
  }

//    Serial.println(micros() - start_time);

  //  Check for a long button press
  if (button_value == 1) { // check if the current value is 1
    if (button_value_buf == 1) { // check if the previous value is 1
      long_button_press_count ++; // increments the counter

      if (long_button_press_count == button_press_threshold * ADC_frequency) { // check if the button is pressed for more than 4s
        stop_flag = 1; // set the stop flag to stop the data recording
        //We have include timer stop
        // zt3.enable(false); // disable the timer
      }
    }

    button_value_buf = 1;
  }

  else {
    long_button_press_count = 0; // resets the counter
    button_value_buf = 0;
  }
}
















