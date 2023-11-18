/*
   Timer Interrupt in ESP32, Alarm generation in ESP32
   www.electronicwings.com
 */ 

#define LED_PIN 00             	// In some ESP32 board have inbuilt LED

volatile int interruptCounter;  //for counting interrupt
int totalInterruptCounter;   	//total interrupt counting
int LED_STATE=HIGH;
hw_timer_t * timer = NULL;      //H/W timer defining (Pointer to the Structure)

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {      //Defining Inerrupt function with IRAM_ATTR for faster access
 portENTER_CRITICAL_ISR(&timerMux);
 interruptCounter++;
 portEXIT_CRITICAL_ISR(&timerMux);
//  Serial.println("Timer Interrupt");
}
 
void setup() {
 Serial.begin(115200);
 pinMode (LED_PIN, OUTPUT);
 
 timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
 timerAttachInterrupt(timer, &onTimer, true); 	// Attach interrupt
 timerAlarmWrite(timer, 9000000, true);  		// Match value= 1000000 for 1 sec. delay.
 timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)
//  Serial.println("Timer Interrupt");
}
 
void loop() {
 if (interruptCounter > 0) {
  // Serial.println("entering to interrupt");
  delay(1000);
  // Serial.println("Delay done");
  
 
 
  //  totalInterruptCounter++;         	//counting total interrupt

   LED_STATE= !LED_STATE;  				//toggle logic
   digitalWrite(LED_PIN, LED_STATE);    //Toggle LED
   Serial.println("LED toggled");
  //  delay(3000);
  //  Serial.print("An interrupt as occurred. Total number: ");
  //  Serial.println(totalInterruptCounter);
  portENTER_CRITICAL(&timerMux);
   Serial.print("interruptCounter= ");
   Serial.println(interruptCounter);
   interruptCounter--;
   Serial.print("interruptCounterdown= ");
   Serial.println(interruptCounter);
  portEXIT_CRITICAL(&timerMux);
 }
}