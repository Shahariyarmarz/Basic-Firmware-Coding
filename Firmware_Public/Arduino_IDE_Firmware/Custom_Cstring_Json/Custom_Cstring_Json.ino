#include <Arduino.h>
#include <cstring>

#define PAY_LOAD_ARRAY_LEN 2048
#define DEV_SETTINGS "setting"
#define DEV_ACCELERATION "acceleration"
#define DEV_RPM "rpm"
#define DEV_STEP_ANGLE "stepangle"
#define DEV_STEP_RES "stepResolution"
#define DEV_DIRECTION "direction"
#define DEV_MOTOR_EX_TYPE "motorExicutionType"
#define DEV_MOTOR_EX_VALUE "motorExicutionTypeValue"
#define DEV_WIFI "wifi"
#define DEV_SSID "ssid"
#define DEV_PASS "password"
#define DEV_TIME "time"
#define DEV_SCHEDULER "scheduler_on_off"

char payLoadArray[PAY_LOAD_ARRAY_LEN];
int payLoadArrayHead = 0;

int addField(const char *str) {
  int len = strlen(str);
  if (payLoadArrayHead + len + 3 < PAY_LOAD_ARRAY_LEN) {
    strcat(payLoadArray, "\"");
    strcat(payLoadArray, str);
    strcat(payLoadArray, "\":");
    payLoadArrayHead += len + 3;
    return payLoadArrayHead;
  }
  return -1; // Error: not enough space
}


int startSection(const char *str) {
  int len = strlen(str);
  if (payLoadArrayHead < PAY_LOAD_ARRAY_LEN) {
    strcat(payLoadArray, "\"");
    strcat(payLoadArray, str);
    strcat(payLoadArray, "\"");
    payLoadArrayHead = len + 2;
    return payLoadArrayHead;
  }
  return -1; // Error: not enough space
}


int addData(const char *str) {
  int len = strlen(str);
  if (payLoadArrayHead + len < PAY_LOAD_ARRAY_LEN) {
    strcat(payLoadArray, "\"");
    strcat(payLoadArray, str);
    strcat(payLoadArray, "\"");
    payLoadArrayHead += len;
    return payLoadArrayHead;
  }
  return -1; // Error: not enough space
}


int addData(int num) {
  char buffer[12];
  itoa(num, buffer, 10);
  strcat(payLoadArray, ",");
  return addData(buffer);
}

int addData(long num) {
  char buffer[22];
  ltoa(num, buffer, 10);
  strcat(payLoadArray, ",");
  return addData(buffer);
}

int addData(float num) {
  char buffer[32];
  dtostrf(num, 0, 2, buffer);
  strcat(payLoadArray, ",");
  return addData(buffer);
}

int addData(double num) {
  char buffer[32];
  dtostrf(num, 0, 2, buffer);
  strcat(payLoadArray, ",");
  return addData(buffer);
}



int startPayload() {
  memset((void *)&payLoadArray, 0x00, PAY_LOAD_ARRAY_LEN);
  payLoadArray[0] = '{';
  payLoadArrayHead = 1;
  return payLoadArrayHead;
}

int endPayload() {
  if (payLoadArrayHead > 0) {
    if (payLoadArray[payLoadArrayHead - 1] == ',') {
      payLoadArray[payLoadArrayHead - 1] = '\0';
      payLoadArrayHead--;
    }
    strcat(payLoadArray, "}");
    payLoadArrayHead++;
  }
  return payLoadArrayHead;
}



int endSection() {
  if (payLoadArrayHead > 0) {
    if (payLoadArray[payLoadArrayHead - 1] == ',') {
      payLoadArray[payLoadArrayHead - 1] = '\0';
      payLoadArrayHead--;
    }
    strcat(payLoadArray, "},");
    payLoadArrayHead += 2;
  }
  return payLoadArrayHead;
}

// void updateField(const char *field, const char *value) {
//   char *pos = strstr(payLoadArray, field);
//   if (pos) {
//     pos += strlen(field) + 1; // Move to the start of the value
//     char *end = strchr(pos, '"');
//     if (end) {
//       int oldLen = end - pos;
//       int newLen = strlen(value);
//       if (newLen != oldLen) {
//         memmove(pos + newLen, end, strlen(end) + 1); // Shift the remaining part of the string
//       }
//       strncpy(pos, value, newLen);
//       // pos[newLen] = '"'; // Properly terminate the updated value
//       // Add a comma and a double quote after the updated value
//       if (*(pos + newLen + 1) != ',') {
//         memmove(pos + newLen + 3, pos + newLen + 1, strlen(pos + newLen + 1) + 1);
//         *(pos + newLen + 1) = ',';
//         *(pos + newLen + 2) = '"';
//       }
//       pos[newLen+2] = '"'; // Properly terminate the updated value
//     }
//   }
// }

void setup() {
  Serial.begin(115200);

  // Start creating the JSON payload
  startPayload();

  // Add settings section
  startSection(DEV_SETTINGS);
  addField(DEV_ACCELERATION);
  addData("12");
  // strcat(payLoadArray, ",");
  addField(DEV_RPM);
  addData("12");
  // strcat(payLoadArray, ",");
  addField(DEV_STEP_ANGLE);
  addData("12");
  // strcat(payLoadArray, ",");
  addField(DEV_STEP_RES);
  addData("1");
  // strcat(payLoadArray, ",");
  addField(DEV_DIRECTION);
  addData("1");
  // strcat(payLoadArray, ",");
  addField(DEV_MOTOR_EX_TYPE);
  addData("2");
  // strcat(payLoadArray, ",");
  addField(DEV_MOTOR_EX_VALUE);
  addData("12");
  endSection();

  // Add wifi section
  startSection(DEV_WIFI);
  addField(DEV_SSID);
  addData("test234");
  // strcat(payLoadArray, ",");
  addField(DEV_PASS);
  addData("12334544");
  // strcat(payLoadArray, ",");
  addField(DEV_TIME);
  addData("12:53");
  endSection();

  // Add scheduler_on_off field
  addField(DEV_SCHEDULER);
  addData("false");

  // End the JSON payload
  endPayload();

  // Print the JSON string
  Serial.println(payLoadArray);

  // Update the values
  // updateField("\"acceleration\"", "62349596");
  // updateField("\"ssid\"", "gizan5500");

  // Print the updated JSON string
  // Serial.println("Updated JSON:");
  // Serial.println(payLoadArray);
}

void loop() {
  // Nothing to do here
}