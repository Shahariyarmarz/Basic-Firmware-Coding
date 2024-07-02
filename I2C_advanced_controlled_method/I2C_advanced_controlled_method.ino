#include "ICM20948.h"
#include <Wire.h>

# define SDApin 41
# define SCLpin 40

ICM20948 IMU(Wire, 0x68); // an ICM20948 object with the ICM-20948 sensor on I2C bus 0 with address 0x69
int status;
int count=0;
long start;
bool dataAvailable = false;
int dataTime = 0;
int lastDataTime = 0;

void setup() {
  // serial to display data
  Serial.begin(115200);
  Wire.begin (SDApin, SCLpin);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  IMU.configAccel(ICM20948::ACCEL_RANGE_16G, ICM20948::ACCEL_DLPF_BANDWIDTH_50HZ);
  IMU.configGyro(ICM20948::GYRO_RANGE_2000DPS, ICM20948::GYRO_DLPF_BANDWIDTH_51HZ);
  IMU.setGyroSrd(14); // Output data rate is = 1125/(1 + setGyrosrd) Hz------>control sampling frequency from setGyrosrd------>setGyrosrd value must be integer.
  IMU.setAccelSrd(14);
  IMU.enableDataReadyInterrupt();
  pinMode(45, INPUT);
  attachInterrupt(45, imuReady, RISING);
}

void imuReady() {
  // dataTime = micros();
  dataAvailable = true;
}

void loop() {
  if (count==0) {
  start = millis();
  }
  if (dataAvailable) {
    dataAvailable = false;
    int timeDiff = dataTime - lastDataTime;
    lastDataTime = dataTime;
    IMU.readSensor();
    // display the data
    // Serial.print(dataTime);
    // Serial.print("\t");
    // Serial.print(timeDiff);
    // Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(),6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(),6);
    count++;
  }
  if (count==1024) {
  Serial.println((String) "Sampling Interval: " + (millis() - start));
  count = 0;
  }

}