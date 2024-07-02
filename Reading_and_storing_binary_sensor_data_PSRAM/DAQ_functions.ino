void read_analog_sensors(void){
  if (DATA_READ_ENABLE) {
		ADC_BUFFER[0][ADC_BUF_END_INDX] = (analogRead(A0) + analogRead(A0)) / 2;
		ADC_BUFFER[1][ADC_BUF_END_INDX] = (analogRead(A1) + analogRead(A1)) / 2;
		ADC_BUFFER[2][ADC_BUF_END_INDX] = (analogRead(A2) + analogRead(A2)) / 2;
    ADC_BUFFER[3][ADC_BUF_END_INDX] = (analogRead(A3) + analogRead(A3)) / 2;
    ADC_BUFFER[4][ADC_BUF_END_INDX] = (analogRead(A4) + analogRead(A4)) / 2;
		ADC_BUF_END_INDX = (ADC_BUF_END_INDX + 1) % ADC_BUF_SIZE;
		if (ADC_BUF_END_INDX == ADC_BUF_START_INDX) {
			Serial.println("Buffer Overflow!!!!!");
		}
	}
}

void logData(uint16_t ADC_buf_copy[Sensor_size][ADC_COPY_SIZE], uint16_t size) {
  // uint16_t data[ANALOG_COUNT];
  if (file) {
        for (size_t i = 0; i < size; i++) {
            file.write((const uint8_t *)&ADC_buf_copy[0][i], sizeof(ADC_buf_copy[0][i]));
            file.write((const uint8_t *)&ADC_buf_copy[1][i], sizeof(ADC_buf_copy[1][i]));
            file.write((const uint8_t *)&ADC_buf_copy[2][i], sizeof(ADC_buf_copy[2][i]));
            file.write((const uint8_t *)&ADC_buf_copy[3][i], sizeof(ADC_buf_copy[3][i]));
            file.write((const uint8_t *)&ADC_buf_copy[4][i], sizeof(ADC_buf_copy[4][i]));
            // file.print(String(ADC_buf_copy[0][i]));
            // file.write((const uint8_t *)&ADC_buf_copy, sizeof(ADC_buf_copy));
            // file.println(",");
            // file.print(String(ADC_buf_copy[1][i]));
            // file.print(",");
            // file.println(String(ADC_buf_copy[2][i]));
        }
        file.close();
    } else {
        Serial.println("error opening datalog.dat");
    }

}