//Tech Trends Shameer
//Temperature Data Log into SD Card
#include "FS.h"
#include <SPI.h>        // Include SPI library (needed for the SD card)
#include <SD.h>
// #include <SdFat.h> 
#include <Arduino.h>         // Include SD library

#define SD_CS        10
#define SPI_MOSI      11
#define SPI_MISO      13
#define SPI_SCK       12

int piezoValue;
const int piezo = 02;
int dataMessage;

unsigned int count = 0;

 
// if (!dataFile.open(ADC_fileName, O_WRONLY | O_CREAT | O_APPEND)) {
//       // error("file.open");
//     }
 
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  pinMode(14, INPUT_PULLUP);	//manuell switch
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

  SD.begin(SD_CS);  
 
  uint8_t cardType = SD.cardType();

  File file = SD.open("/data.csv");
  if(!file) {
    writeFile(SD, "/data.csv", piezoValue);
  }
  file.close();
}
 
 
void loop() {
  if(digitalRead(14) == HIGH){
    count = count + 1;
    piezoValue = analogRead(piezo);
    logSDCard();
	}
  
}



void logSDCard() {
  dataMessage = count + piezoValue;
  for(int i=0;i <= 20; i++){
  dataMessage =piezoValue;
  // dataMessage = dataMessage + ";" + String(piezoValue)+"\r\n";
  }
  Serial.print("Save data: ");
  appendFile(SD, "/data.csv", dataMessage);
}




// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const int message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const int message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.println(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}