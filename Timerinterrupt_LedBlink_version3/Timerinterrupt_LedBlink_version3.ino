//
// Basic example of ESP 32 interrupt timer sketch
//
// This example and code is in the public domain and
// may be used without restriction and without warranty.
//
volatile int interrupt_counter;
volatile int current_interrupt_counter;
int total_interrupt_counter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interrupt_counter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  timer = timerBegin(0, 80, true);    // prescale down to microsecond timing
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true); // step down to millisecond interrupts
  timerAlarmEnable(timer);
}

void loop() {
  // Take a copy of the current interrupt_counter value
  // so we may test it in our if/then process without
  // running into a conflict between our ISR and main
  // loop processing
  portENTER_CRITICAL(&timerMux);
  current_interrupt_counter = interrupt_counter;
  portEXIT_CRITICAL(&timerMux);
  if (current_interrupt_counter > 0) {
    // A timer interrupt has occurred
    portENTER_CRITICAL(&timerMux);
    interrupt_counter--;
    portEXIT_CRITICAL(&timerMux);
    total_interrupt_counter++;
    // report every 1000 interrupts, ie every 1 second
    if (total_interrupt_counter % 1000 == 0) {
      Serial.print("A timer interrupt has occurred. Total number: ");
      Serial.println(total_interrupt_counter);
      Serial.flush();
    }
  }
}