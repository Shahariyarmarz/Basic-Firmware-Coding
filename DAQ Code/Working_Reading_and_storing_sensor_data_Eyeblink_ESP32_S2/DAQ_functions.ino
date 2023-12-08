void read_analog_sensors(void){
  if (DATA_READ_ENABLE) {
		ADC_BUFFER[0][ADC_BUF_END_INDX] = (analogRead(A0) + analogRead(A0) + analogRead(A0)) / 3;
		// ADC_BUFFER[1][ADC_BUF_END_INDX] = (analogRead(A1) + analogRead(A1) + analogRead(A1)) / 3;
		// ADC_BUFFER[2][ADC_BUF_END_INDX] = (analogRead(A2) + analogRead(A2) + analogRead(A2)) / 3;
		ADC_BUF_END_INDX = (ADC_BUF_END_INDX + 1) % ADC_BUF_SIZE;
		if (ADC_BUF_END_INDX == ADC_BUF_START_INDX) {
			Serial.println("Buffer Overflow!!!!!");
		}
	}
}

void logData(uint16_t ADC_buf_copy[1][ADC_COPY_SIZE], uint16_t size) {
  // uint16_t data[ANALOG_COUNT];
  if (file) {
        for (size_t i = 0; i < ADC_COPY_SIZE; i++) {
            file.print(String(ADC_buf_copy[0][i]));
            file.println(",");
            // file.print(String(ADC_buf_copy[1][i]));
            // file.print(",");
            // file.println(String(ADC_buf_copy[2][i]));
        }
        file.close();
    } else {
        Serial.println("error opening datalog.dat");
    }

}