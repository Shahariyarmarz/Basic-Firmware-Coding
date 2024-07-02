
//--------------------------------------ADC Frequency Test-----------------------------------------------------
// ESP-32-S2-ADC can sample at 14KHz means 14K samples per second so it can sample 12-14 sample per millisecond.
//This code provide the justification of ADC sampling frequency
int i;
long samplingtime;
float Sampling_Freq_per_millisecond;
volatile int sensorvalue;

void setup() {
  Serial.begin(115200);

}
void loop() {
  long start = millis();
  //Read 1000 samples and calculate the approximate time required for 1000 samples.
  for (i=0; i<1000; i++) {
  sensorvalue = analogRead(A0);
  if (i==999) {
  samplingtime=millis()-start;  //total approximate sampling time in millisecond unit for 1000 samples.
  }
  }
  Sampling_Freq_per_millisecond = (1000 / samplingtime);
  //(Number of samples/total sampling time for the samples in millisecond unit)samples read at one millisecond;
  Serial.println((String)"Sampling freq in millisecond = " + Sampling_Freq_per_millisecond);
  // Serial.println((String)"Sampling Interval = " + (millis()-start));
  // sensorvalue = analogRead(A0);
  // Serial.println(sensorvalue);

}
