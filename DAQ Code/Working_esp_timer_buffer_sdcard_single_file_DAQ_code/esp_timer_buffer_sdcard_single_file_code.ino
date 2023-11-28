#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#define AX 01
#define AY 02
#define AZ 03

// SD card chip select pin
#define chipSelect    10 
#define SPI_MOSI      11       //Master Out Slave In------Master-out (transmit) and slave-in (receive).
#define SPI_MISO      13       //Master In Slave Out------Slave-out (transmit) and master-in (receive).
#define SPI_SCK       12   

/* -------------------------------------- Global Variables ------------------------------------------------------*/

// constant variables
const uint16_t ADC_BUF_SIZE = 4096 * 3; // index for the data in the ISR
const uint16_t ADC_COPY_SIZE = 512; // index for the data in the ISR
const uint16_t ADC_FREQ = 1000; // Samping frequency of the ADC data in Hz. This is also the sampling frequency of maternal sensation button press
const uint16_t TIMER_PRESCALER = 80;
const uint16_t TIMER_TICK = 100000;	// TimerTick = CPU_CLK_FREQ / (TIMER_PRESCALER * ADC_FREQ) Hard coded because u_16_integer size limit


// Variables related to the Wifi Sending of ADC data
volatile uint16_t ADC_BUFFER[3][ADC_BUF_SIZE] = { 0 }; // variable for buffering the data in the ISR
volatile uint16_t ADC_BUF_START_INDX = 0; // index for the data in the ISR
volatile uint16_t ADC_BUF_END_INDX = 1; // index for the data in the ISR
volatile uint32_t ADC_COUNT = 0; // variable to track the number of sampling has happened
volatile bool DATA_READ_ENABLE = true;

hw_timer_t* Timer0_Cfg = NULL;

volatile uint64_t DATA_PACKET_COUNT = 0; // variable to track the number of data packet has been sent over wifi
volatile uint64_t SAMPLING_COUNT = 0; // variable to track the number of data packet has been sent over wifi
String payload = "";

// char ssid[] = "WIFI SSID";
// const char* pw = "Wifi Password";

// const char* SOCKET_HOST = "192.168.0.106";
// const uint16_t SOCKET_PORT = 8000;



void IRAM_ATTR Timer0_ISR() {
	if (DATA_READ_ENABLE) {
		ADC_BUFFER[0][ADC_BUF_END_INDX] = (analogRead(AX) + analogRead(AX) + analogRead(AX)) / 3;
		ADC_BUFFER[1][ADC_BUF_END_INDX] = (analogRead(AY) + analogRead(AY) + analogRead(AY)) / 3;
		ADC_BUFFER[2][ADC_BUF_END_INDX] = (analogRead(AZ) + analogRead(AZ) + analogRead(AZ)) / 3;
		ADC_BUF_END_INDX = (ADC_BUF_END_INDX + 1) % ADC_BUF_SIZE;
		if (ADC_BUF_END_INDX == ADC_BUF_START_INDX) {
			Serial.println("Buffer Overflow!!!!!");
		}
		SAMPLING_COUNT++;
	}
	if (SAMPLING_COUNT % 10000 == 0) {
		// per 10000 sampling we are logging information 
		Serial.println("info:");
		Serial.println(SAMPLING_COUNT);
		Serial.println(DATA_PACKET_COUNT);
		Serial.println(SAMPLING_COUNT / DATA_PACKET_COUNT);
		Serial.println((ADC_BUF_END_INDX - ADC_BUF_START_INDX + ADC_BUF_SIZE) % ADC_BUF_SIZE);
	}
}

void write_data_to_sdcard(uint16_t ADC_buf_copy[3][ADC_COPY_SIZE], uint16_t size) {
    File dataFile = SD.open("/datalog.dat", FILE_WRITE);

    if (dataFile) {
        for (size_t i = 0; i < ADC_COPY_SIZE; i++) {
            dataFile.print(String(ADC_buf_copy[0][i]));
            dataFile.print(",");
            dataFile.print(String(ADC_buf_copy[1][i]));
            dataFile.print(",");
            dataFile.println(String(ADC_buf_copy[2][i]));
        }
        dataFile.close();
    } else {
        Serial.println("error opening datalog.dat");
    }

    DATA_PACKET_COUNT++;
}

void setup() {
    Serial.begin(115200);
    pinMode(14, INPUT_PULLUP);	//manuell switch to enable SD CARD
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, chipSelect);
    // SD.begin(SD_CS); 
    // uint8_t cardType = SD.cardType();

    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1);
    }
    Serial.println("card initialized.");
    File dataFile = SD.open("/datalog.dat");
    if(!dataFile) {
    Serial.println("File Opened");
    }
    dataFile.close();

    pinMode(AX, INPUT);
    pinMode(AY, INPUT);
    pinMode(AZ, INPUT);

    Timer0_Cfg = timerBegin(0, TIMER_PRESCALER, true); // timerBegin(timer, prescaler, counUp)
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, TIMER_TICK, true);// timerAlarmWrite(timer, TimerTick, autoReload)
    timerAlarmEnable(Timer0_Cfg);
}

void loop() {
    uint16_t ADC_buf_copy[3][ADC_COPY_SIZE]; // variable to contain (ADC_copy_size x 2) bytes copy of the buffer
    long data_reading_interval = 600000; // sampling time in ms
    long loop_start = millis(); // variables to store times in miliseconds

    // Loop for writing data to SD card for 600s
    while ((millis() - loop_start) <= data_reading_interval && DATA_READ_ENABLE) {
        if ((ADC_BUF_END_INDX - ADC_BUF_START_INDX + ADC_BUF_SIZE) % ADC_BUF_SIZE >= ADC_COPY_SIZE) {
            for (uint16_t i = 0; i < ADC_COPY_SIZE; i++) {
                ADC_buf_copy[0][i] = ADC_BUFFER[0][ADC_BUF_START_INDX];
                ADC_buf_copy[1][i] = ADC_BUFFER[1][ADC_BUF_START_INDX];
                ADC_buf_copy[2][i] = ADC_BUFFER[2][ADC_BUF_START_INDX];
                ADC_BUF_START_INDX = (ADC_BUF_START_INDX + 1) % ADC_BUF_SIZE;
            }
            write_data_to_sdcard(ADC_buf_copy, ADC_COPY_SIZE);
        }
        if (!DATA_READ_ENABLE)
            break;
    }
}