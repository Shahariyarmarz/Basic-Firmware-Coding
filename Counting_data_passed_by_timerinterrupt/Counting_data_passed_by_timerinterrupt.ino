/*
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.
 */

// Stop button is attached to PIN 0 (IO0)
#define buttonPin    42
#define LED1 0

uint16_t button_value;
// uint32_t isrCount = 0; // Do not declare at global space .........try to declare at local space

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore; 
//A semaphore is an integer variable, shared among multiple processes, a special kind of synchronization data that can be used only through specific synchronization primitives.
//transmit messages between distant points.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
// volatile uint32_t lastIsrAt = 0;

void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  if (button_value = 1-digitalRead(buttonPin)) {
  isrCounter++;
  // lastIsrAt = millis();
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  //For the semaphore to be used, it must be released with xSemaphoreGive(sem)
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {
  Serial.begin(115200);

  // Set BTN_STOP_ALARM to input mode
  pinMode(buttonPin, INPUT_PULLUP);

  xTaskCreate(
    blink1,      // Function name of the task
    "Blink 1",   // Name of the task (e.g. for debugging)
    2048,        // Stack size (bytes)
    NULL,        // Parameter to pass
    1,           // Task priority
    NULL         // Task handle
  );

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

void blink1(void *parameter) {
    pinMode(LED1, OUTPUT);
    while(1){
        digitalWrite(LED1, HIGH);
        delay(500); // Delay for Tasks 
        digitalWrite(LED1, LOW);
        delay(500);
    }
}

void loop() {
  // If Timer has fired
  // button_value = 1-digitalRead(buttonPin);
  // Serial.println(button_value);
  uint32_t isrCount = 0;
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    //For the semaphore to be used, it must be released with xSemaphoreGive(sem). To receive the semaphore, 
    //there is the function xSemaphoreTake(sem, xTicksToWait) with xTicksToWait as the maximum waiting time in ticks. 
    //If the semaphore was accepted within the waiting time, the function return value is pdTRUE, otherwise pdFALSE.
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    // isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
  }
  if (isrCount == 1000){
    Serial.print("ISR count 1000---> ");
    Serial.print(isrCount);
    Serial.println(" at ");
    // Serial.print(isrTime);
    // Serial.println(" ms");
  }
  if (isrCount==2000) {
    Serial.print("ISR count 2000--->");
    Serial.print(isrCount);
    Serial.println(" at ");
    // Serial.print(isrTime);
    // Serial.println(" ms");
    isrCounter=0;
    }
    // Serial.println("Main loop");
  // If button is pressed
  // if (digitalRead(BTN_STOP_ALARM) == LOW) {
  //   // If timer is still running
  //   if (timer) {
  //     // Stop and free timer
  //     timerEnd(timer);
  //     timer = NULL;
  //   }
  // }
}
