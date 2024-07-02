uint16_t A0_pin = A0;
uint16_t A1_pin = A1;
uint16_t A2_pin = A2;
uint16_t A3_pin = A3;
uint16_t A4_pin = A4;
const uint16_t ADC_BUF_SIZE = 1024; 
uint16_t Sensor_Array[5][ADC_BUF_SIZE];
volatile uint32_t ADC_BUF_START_INDX = 0;   // Start index of ADC Buffer
volatile uint32_t ADC_BUF_END_INDX = 1; 
const uint16_t TIMER_PRESCALER = 80;  // CPU CLock Frequency
const uint16_t TIMER_TICK = 1953;    // TimerTick = CPU_CLK_FREQ / (TIMER_PRESCALER * ADC_FREQ) Hard coded because u_16_integer size limit
hw_timer_t* Timer0_Cfg = NULL;

float Sampling_Freq;

void IRAM_ATTR Timer0_ISR() {
    Sensor_Array[0][ADC_BUF_END_INDX] = (analogRead(A0) + analogRead(A0) + analogRead(A0)) / 3;
    Sensor_Array[1][ADC_BUF_END_INDX] = (analogRead(A1) + analogRead(A1) + analogRead(A1)) / 3;
    Sensor_Array[2][ADC_BUF_END_INDX] = (analogRead(A2) + analogRead(A2) + analogRead(A2)) / 3;
    Sensor_Array[3][ADC_BUF_END_INDX] = (analogRead(A3) + analogRead(A3) + analogRead(A3)) / 3;
    Sensor_Array[4][ADC_BUF_END_INDX] = (analogRead(A4) + analogRead(A4) + analogRead(A4)) / 3;
    ADC_BUF_END_INDX = (ADC_BUF_END_INDX + 1) % ADC_BUF_SIZE;
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
  long start = millis();
  int Sensor_Copy[5][512];
  for (int i = 0; i < 512; i++) {
    Sensor_Copy[0][i]=Sensor_Array[0][ADC_BUF_START_INDX];
    Sensor_Copy[1][i]=Sensor_Array[1][ADC_BUF_START_INDX];
    Sensor_Copy[2][i]=Sensor_Array[2][ADC_BUF_START_INDX];
    Sensor_Copy[3][i]=Sensor_Array[3][ADC_BUF_START_INDX];
    Sensor_Copy[4][i]=Sensor_Array[4][ADC_BUF_START_INDX];
    ADC_BUF_START_INDX = (ADC_BUF_START_INDX + 1) % ADC_BUF_SIZE;
  }
  // Sampling_Freq = (10000 * 1000 / (millis() - start));  //millis()-start = time in millisecond. So convert it to second we have to multiply with 1000.
  // Serial.println((String) "Sampling Frequency: " + Sampling_Freq);
  Serial.println((String) "Sampling Interval: " + (millis() - start));
}
