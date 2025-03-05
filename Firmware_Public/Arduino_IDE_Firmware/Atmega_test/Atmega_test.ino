#include <Arduino.h>
#include <SoftwareSerial.h>
// #include "AtmegaHexUploader.h"

#define AT_MISO 12
#define AT_MOSI 11
#define AT_SCK 13
#define AT_RST 14


const uint8_t input1 = 5;
const uint8_t input2 = 6;
const uint8_t input3 = 7;

uint8_t in1_state = 0;
uint8_t in2_state = 0;
uint8_t in3_state = 0;

unsigned long previous_millis = 0;
// unsigned long current_millis;

bool loop_count = true;

SoftwareSerial new_serial(AT_MOSI, AT_MISO);
// AtmegaHexUploader hexUploader(AT_MOSI, AT_MISO);  // Software: MISO, MOSI

typedef enum {
  //constants
  Board_POE,
  Board_AUX,
  Board_SLB,
  Board_OB1,
  Board_OB2,
  Board_OB3,
  Board_OB4,
  Board_OB5
} Board_name;

void setup() {
  Serial.begin(115200);
  new_serial.begin(9600);
  // Serial2.begin(9600);
  pinMode(input1, INPUT);
  pinMode(input2, INPUT);
  pinMode(input3, INPUT);
  // pinMode(AT_SCK, OUTPUT);

  // if (hexUploader.begin(AT_RST, AT_SCK)) {  // Software: RESET, SCK
  //   Serial.println("Uploader Started.");
  // }
}

void loop() {
  in1_state = digitalRead(input1);
  in2_state = digitalRead(input2);
  in3_state = digitalRead(input3);
  pinMode(AT_SCK, OUTPUT);
  digitalWrite(AT_SCK, LOW);


  uint8_t in1_state_binary = bitRead(in1_state_binary, 0);
  uint8_t in2_state_binary = bitRead(in2_state_binary, 0);
  uint8_t in3_state_binary = bitRead(in3_state_binary, 0);

  uint8_t combined_input = (in3_state_binary << 2) | (in2_state_binary << 1) | (in1_state_binary);
  String formattedInput_board = "Dev_" + String(combined_input);  // Add "Dev_" before combined_input
  


  do {
    unsigned long current_millis = millis();
    if (current_millis - previous_millis >= 500) {
      pinMode(AT_SCK, INPUT);
      new_serial.write(formattedInput_board.c_str());  // Send the modified input for
      previous_millis = current_millis;
      if (new_serial.available() > 0) {
        loop_count = false;
      }
    }
  } while (new_serial.available() <= 0 && loop_count);  //Serial2.available()    new_serial.available()
  loop_count = true;
  delay(100);
}
