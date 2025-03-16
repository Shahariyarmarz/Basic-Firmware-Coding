#include <Arduino.h>
#include "AtmegaHexUploader.h"

#define ES_MISO 19
#define ES_MOSI 23
#define ES_SCK 18
#define ES_RST 15


unsigned long previous_millis = 0;
// unsigned long current_millis;

bool loop_count = true;

AtmegaHexUploader hexUploader(ES_MISO, ES_MOSI);  // Software: MISO, MOSI


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(ES_SCK, INPUT_PULLUP);

  if (hexUploader.begin(ES_RST, ES_SCK)) {  // Software: RESET, SCK
    Serial.println("Uploader Started.");
  }
  
  unsigned long current_millis = millis();
  while ((current_millis-previous_millis) <= 600) {
    if (ES_SCK==LOW) {
      if (Serial2.available()) {
        String received = Serial2.readString();
        Serial.println(String("Received Data: ") + received);
      }
      else {
      Serial.println("Data not found");
      }
    }
    else {
    Serial.println("Device not found");
    }
  
  current_millis = millis();
  }
}

void loop() {
  
}