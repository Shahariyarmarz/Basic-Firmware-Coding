#include <SPI.h>
#include "SdFat.h"
#include <Arduino.h>

//Battery pin
#define PIN_VBAT 35

#define chipSelect    10
#define SPI_MOSI      11
#define SPI_MISO      13
#define SPI_SCK       12

#define FILE_BASE_NAME "Data"
#define error(msg) sd.errorHalt(F(msg))


const uint32_t ADC_BUF_SIZE = 9216 * 3; // index for the data in the ISR=======Max size try 9216
const uint32_t ADC_COPY_SIZE = 512; // index for the data in the ISR =======Max size try 3072
const uint16_t ADC_FREQ = 1024; // Samping frequency of the ADC data in Hz. This is also the sampling frequency of maternal sensation button press
const uint16_t TIMER_PRESCALER = 80;
const uint16_t TIMER_TICK = 10000;	// TimerTick = CPU_CLK_FREQ / (TIMER_PRESCALER * ADC_FREQ) Hard coded because u_16_integer size limit


// Variables related to the Wifi Sending of ADC data
volatile uint32_t ADC_BUFFER[1][ADC_BUF_SIZE] = { 0 }; // variable for buffering the data in the ISR
volatile uint32_t ADC_BUF_START_INDX = 0; // index for the data in the ISR
volatile uint32_t ADC_BUF_END_INDX = 1; // index for the data in the ISR
volatile uint32_t ADC_COUNT = 0; // variable to track the number of sampling has happened
bool DATA_READ_ENABLE;

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.dat";

hw_timer_t* Timer0_Cfg = NULL;

volatile uint64_t DATA_PACKET_COUNT = 0; // variable to track the number of data packet has been sent over wifi
volatile uint64_t SAMPLING_COUNT = 0; // variable to track the number of data packet has been sent over wifi
String payload = "";

//============>Push Button Variables
const int buttonPin = 42;  // the number of the pushbutton pin
const int led_red = 00;    // the number of the LED pin
const uint8_t led_yellow = 45;
// const uint8_t button_frequency = 4;
const uint8_t button_press_ON_threshold = 1;
const uint8_t button_press_OFF_threshold = 2;
uint8_t start_flag = 0;
uint8_t led_flag = 0;// uint8_t stop_flag = 0;
uint8_t button_value;
uint16_t button_count=0;

SdFat sd;
SdFile file;

void IRAM_ATTR Timer0_ISR() {
  read_analog_sensors();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  set_SD_card();
  set_pinMode();
  Serial.println("Press PUSH Button to START data Reading from SENSORS & writing to SD CARD");

  Timer0_Cfg = timerBegin(0, TIMER_PRESCALER, true); // timerBegin(timer, prescaler, counUp)
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, TIMER_TICK, true);// timerAlarmWrite(timer, TimerTick, autoReload)
  timerAlarmEnable(Timer0_Cfg);
}

void loop() {

  button_value = 1-digitalRead(buttonPin);
  // Serial.println(button_value);
  if (button_value==1) {
    button_count++;
    Serial.println(button_count);
    if (button_count==1024) {
    led_flag=1;
    }
  }
  
  if (button_count==button_press_ON_threshold*1024) {
    if (led_flag==1) {
    blink_LED_RED();
    led_flag=0;
    Serial.println("Data Reading from SENSORS & Writing to SD has been STARTED,Press PUSH BUTTON to STOP");
    // ============================>if we set SD card here,it will just open a single file and append 512 samples repeatedly and write on it <====================================================
    set_SD_card();
    if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
      error("file.open");
    }
    }
    // ===========================>if we set SD card here,it will open multiple file with 512 samples in a single file and write on it.<=========================================
    // set_SD_card();
    // if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
    //   error("file.open");
    // }
    start_flag=1;
  }

  if (start_flag==1) {
    //writing data to SD card
    DATA_READ_ENABLE=true;
    uint16_t ADC_buf_copy[1][ADC_COPY_SIZE]; // variable to contain (ADC_copy_size x 2) bytes copy of the buffer
    // long data_reading_interval = 600000; // sampling time in ms
    // long loop_start = millis(); // variables to store times in miliseconds
        if ((ADC_BUF_END_INDX - ADC_BUF_START_INDX + ADC_BUF_SIZE) % ADC_BUF_SIZE >= ADC_COPY_SIZE) {
            for (uint32_t i = 0; i < ADC_COPY_SIZE; i++) {
                ADC_buf_copy[0][i] = ADC_BUFFER[0][ADC_BUF_START_INDX];
                // ADC_buf_copy[1][i] = ADC_BUFFER[1][ADC_BUF_START_INDX];
                // ADC_buf_copy[2][i] = ADC_BUFFER[2][ADC_BUF_START_INDX];
                ADC_BUF_START_INDX = (ADC_BUF_START_INDX + 1) % ADC_BUF_SIZE;
            }
            if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
              error("file.open");
            }
            logData(ADC_buf_copy, ADC_COPY_SIZE);
            // set_SD_card();
            // if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
            //   error("file.open");
            //   }
        }
    }
  
  if (button_count==button_press_OFF_threshold*1024) {
    start_flag=0;
    button_count=0;
    DATA_READ_ENABLE=false;
    Serial.println("Data Reading from SENSORS & writing to SD has been STOPPED,Press PUSH Button to START");
    blink_LED_RED();
  }

}
