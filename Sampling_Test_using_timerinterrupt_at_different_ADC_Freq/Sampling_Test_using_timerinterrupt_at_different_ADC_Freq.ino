uint16_t A0_pin = A0;
uint16_t A1_pin = A1;
uint16_t A2_pin = A2;
uint16_t A3_pin = A3;
uint16_t A4_pin = A4;
// long start;
const uint16_t ADC_BUF_SIZE = 1024; 
uint16_t Sensor_Array[16][ADC_BUF_SIZE];
volatile uint32_t ADC_BUF_START_INDX = 0;   // Start index of ADC Buffer
volatile uint32_t ADC_BUF_END_INDX = 1; 
const uint16_t TIMER_PRESCALER = 50;  // CPU APB CLock Frequency*(1/prescaler)
const uint16_t TIMER_TICK = 6250;    // TimerTick = CPU_CLK_FREQ / (TIMER_PRESCALER * ADC_FREQ) Hard coded because u_16_integer size limit
// Timer Tick = 1953 and Prescaler =80 so sampling frequency is 512. Timer Tick =1000 so sampling frequency is 1000 Hz.
// Timer Tick = 3125 and Prescaler =50 so sampling frequency is 512.
// Timer Tick = 1250 and Prescaler =125 so sampling frequency is 512. 
hw_timer_t* Timer0_Cfg = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int count=0;
float Sampling_Freq;

void IRAM_ATTR Timer0_ISR() {
    // start = millis();
    portENTER_CRITICAL_ISR(&timerMux);
    Sensor_Array[0][ADC_BUF_END_INDX] = analogRead(A0)+analogRead(A0) ;
    Sensor_Array[1][ADC_BUF_END_INDX] = analogRead(A1)+analogRead(A1) ;
    Sensor_Array[2][ADC_BUF_END_INDX] = analogRead(A2)+analogRead(A2) ;
    // Serial.println("timerinterrupt");
    Sensor_Array[3][ADC_BUF_END_INDX] = analogRead(A3)+analogRead(A3) ;
    Sensor_Array[4][ADC_BUF_END_INDX] = analogRead(A4)+analogRead(A4) ;
    Sensor_Array[5][ADC_BUF_END_INDX] = analogRead(A5) ;
    Sensor_Array[6][ADC_BUF_END_INDX] = analogRead(A6) ;
    Sensor_Array[7][ADC_BUF_END_INDX] = analogRead(A7) ;
    Sensor_Array[8][ADC_BUF_END_INDX] = analogRead(A8) ;
    Sensor_Array[9][ADC_BUF_END_INDX] = analogRead(A9) ;
    Sensor_Array[10][ADC_BUF_END_INDX] = analogRead(A10) ;
    Sensor_Array[11][ADC_BUF_END_INDX] = analogRead(A11) ;
    Sensor_Array[12][ADC_BUF_END_INDX] = analogRead(A12) ;
    Sensor_Array[13][ADC_BUF_END_INDX] = analogRead(A13) ;
    Sensor_Array[14][ADC_BUF_END_INDX] = analogRead(A14) ;
    Sensor_Array[15][ADC_BUF_END_INDX] = analogRead(A15) ;
    // Sensor_Array[9][ADC_BUF_END_INDX] = analogRead(A14) ;

    ADC_BUF_END_INDX = (ADC_BUF_END_INDX + 1) % ADC_BUF_SIZE;
    portEXIT_CRITICAL_ISR(&timerMux);
    // count++;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(13);
  int CloCk = getCpuFrequencyMhz();
  int apb = getApbFrequency();
  Serial.println((String) "Clock Frequrncy: " + (CloCk) + (String) " MHz");
  Serial.println((String) "APB Frequrncy: " + (apb / 1000000) + (String) " MHz");

  Timer0_Cfg = timerBegin(0, TIMER_PRESCALER, true);    // timerBegin(timer, prescaler, counUp)
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);  // True = Edge Triggered Timer & False = Level Triggered Timer
  timerAlarmWrite(Timer0_Cfg, TIMER_TICK, true);        // timerAlarmWrite(timer, TimerTick, autoReload)
  timerAlarmEnable(Timer0_Cfg);
}

void loop() {
  float start = millis();
  int Sensor_Copy[5][512];
  for (int i = 0; i < 512; i++) {
    // start = millis();
    Sensor_Copy[0][i]=Sensor_Array[0][ADC_BUF_START_INDX];
    Sensor_Copy[1][i]=Sensor_Array[1][ADC_BUF_START_INDX];
    Sensor_Copy[2][i]=Sensor_Array[2][ADC_BUF_START_INDX];
    Sensor_Copy[3][i]=Sensor_Array[3][ADC_BUF_START_INDX];
    Sensor_Copy[4][i]=Sensor_Array[4][ADC_BUF_START_INDX];
    Sensor_Copy[5][i]=Sensor_Array[5][ADC_BUF_START_INDX];
    Sensor_Copy[6][i]=Sensor_Array[6][ADC_BUF_START_INDX];
    Sensor_Copy[7][i]=Sensor_Array[7][ADC_BUF_START_INDX];
    Sensor_Copy[8][i]=Sensor_Array[8][ADC_BUF_START_INDX];
    Sensor_Copy[9][i]=Sensor_Array[9][ADC_BUF_START_INDX];
    Sensor_Copy[10][i]=Sensor_Array[10][ADC_BUF_START_INDX];
    Sensor_Copy[11][i]=Sensor_Array[11][ADC_BUF_START_INDX];
    Sensor_Copy[12][i]=Sensor_Array[12][ADC_BUF_START_INDX];
    Sensor_Copy[13][i]=Sensor_Array[13][ADC_BUF_START_INDX];
    Sensor_Copy[14][i]=Sensor_Array[14][ADC_BUF_START_INDX];
    Sensor_Copy[15][i]=Sensor_Array[15][ADC_BUF_START_INDX];
    ADC_BUF_START_INDX = (ADC_BUF_START_INDX + 1) % ADC_BUF_SIZE;
    // if (ADC_BUF_START_INDX==1023) {
    // Serial.println((String) "Sampling Interval: " + (millis() - start));
    // }
    
    // Sampling_Freq = (10000 * 1000 / (millis() - start));  //millis()-start = time in millisecond. So convert it to second we have to multiply with 1000.
    // Serial.println((String) "Sampling Frequency: " + Sampling_Freq);
  }
  // count++;
  // Serial.println(count);
  //count=0;
  // Sampling_Freq = (10000 * 1000 / (millis() - start));  //millis()-start = time in millisecond. So convert it to second we have to multiply with 1000.
  // Serial.println((String) "Sampling Frequency: " + Sampling_Freq);
  Serial.println((String) "Sampling Interval: " + (millis() - start));
}
