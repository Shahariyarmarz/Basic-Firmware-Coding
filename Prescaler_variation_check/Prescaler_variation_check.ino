uint16_t A0_pin = A0;
uint16_t A1_pin = A1;
uint16_t A2_pin = A2;
uint16_t A3_pin = A3;
uint16_t A4_pin = A4;
// long start;
uint16_t sensorvalue;
const uint16_t ADC_BUF_SIZE = 1024; 
uint16_t Sensor_Array[16][ADC_BUF_SIZE];
volatile uint32_t ADC_BUF_START_INDX = 0;   // Start index of ADC Buffer
volatile uint32_t ADC_BUF_END_INDX = 1; 
const uint16_t TIMER_PRESCALER = 50;  // CPU APB CLock Frequency*(1/prescaler)
const uint16_t TIMER_TICK = 3125;    // TimerTick = CPU_CLK_FREQ / (TIMER_PRESCALER * ADC_FREQ) Hard coded because u_16_integer size limit
hw_timer_t* Timer0_Cfg = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int count=0;
float Sampling_Freq;

void IRAM_ATTR Timer0_ISR() {
    // start = millis();
    portENTER_CRITICAL_ISR(&timerMux); //Use for data integrity 
    sensorvalue=analogRead(A0);
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
  Serial.println(sensorvalue);
}
