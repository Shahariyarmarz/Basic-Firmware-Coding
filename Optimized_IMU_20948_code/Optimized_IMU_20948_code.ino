/***************************************************************************
* Example sketch for the ICM20948_WE library
*
* This sketch shows how to retrieve accelerometer, gyroscope, temperature
* and magnetometer data from the ICM20948.
* 
* Further information can be found on:
*
* https://wolles-elektronikkiste.de/icm-20948-9-achsensensor-teil-i (German)
* https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i (English)
* 
***************************************************************************/

#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x69

ICM20948_WE myIMU = ICM20948_WE(&Wire, ICM20948_ADDR); //ICM20948_WE myIMU = ICM20948_WE(&wire2, ICM20948_ADDR)------- &wire2 = I2C bus selection,  ICM20948_ADDR = 0x68/0x69
int32_t accx;
void setup() {
  Wire.begin(41,40);
  Wire.setClock(100000);
  Serial.begin(115200);
  myIMU.reset_ICM20948();
  myIMU.init();
  myIMU.initMagnetometer();
  myIMU.enableI2CMaster(); 
  myIMU.reset_ICM20948();  //To use this function we need to edit the Library function and made this function private to public and save the header file using VScode.
  myIMU.init();
  myIMU.initMagnetometer();
  

  // myIMU.enableCycle(ICM20948_ACC_GYR_I2C_MST_CYCLE);
  // myIMU.sleep(false);
  // myIMU.setI2CMstSampleRate(13); 
  
  
  // -----------------------------------Basic Settings Range(Resolutions), Callibration, Digital filter, Sampling rate--------------------------------------------- 

  //_____________Accelerometer Callibration_____________
  /*Callibration: This is a method to calibrate. You have to determine the minimum and maximum raw acceleration values of the axes determined in the range +/- 2 g. 
  You can call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax); The parameters are floats.The calibration changes the slope / ratio of
  raw acceleration vs g. The zero point is set as (min + max)/2.*/

  // myIMU.setAccOffsets(0, 0, -16600.0, 0, 0, 16690.0);
    
  /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset. It assumes your 
  ICM20948 is positioned flat with its x,y-plane. The more you deviate from this, the less accurate will be your results. It overwrites the zero points of 
  setAccOffsets, but keeps the correction of the slope. The function also measures the offset of the gyroscope data. The gyroscope offset does not depend on 
  the positioning. This function needs to be called after setAccOffsets but before other settings since it will overwrite settings!*/

  //  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  //  delay(1000);
  //  myIMU.autoOffsets();
  //  Serial.println("Done!"); 

  //_____________Gyroscope Callibration_____________
  /* The gyroscope data is not zero, even if you don't move the ICM20948. To start at zero, you can apply offset values. These are the gyroscope raw values 
  you obtain using the +/- 250 degrees/s range. Use either autoOffset or setGyrOffsets, not both.*/
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);


  //_____________Accelerometer Range_____________
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
  */

  //_____________Accelerometer Digital Low Pass Filter_____________
  myIMU.setAccDLPF(ICM20948_DLPF_3);
  // Set Signal bandwidth to Level 3 (51Hz) through applying digital filter.

  /*  Choose a level for the Digital Low Pass Filter or switch it off.  
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  IMPORTANT: This needs to be ICM20948_DLPF_7 if DLPF is used in cycle mode!
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              246.0               1125/(1+ASRD) 
   *    1              246.0               1125/(1+ASRD)
   *    2              111.4               1125/(1+ASRD)
   *    3               50.4               1125/(1+ASRD)
   *    4               23.9               1125/(1+ASRD)
   *    5               11.5               1125/(1+ASRD)
   *    6                5.7               1125/(1+ASRD) 
   *    7              473.0               1125/(1+ASRD)
   *    OFF           1209.0               4500
   *    
   *    ASRD = Accelerometer Sample Rate Divider (0...4095)
   *    You achieve lowest noise using level 6  
  */
   
  //_____________Accelerometer Sampling Rate_____________
  // myIMU.setAccSampleRateDivider(14); //Sampling in 75 samples per second
  // ACC Data Sampling rate = (1125 / (1 + setAccSampleRateDivider) )
  /*  Acceleration sample rate ASRD divider(= setAccSampleRateDivider) divides the output rate of the accelerometer. It can only be applied if the corresponding 
  DLPF is not off! ASRD Divider is a number of 0...4095. If sampling rates are set for both the accelerometer and the gyroscope, the gyroscope sampling rate has
  priority.
  */

  //_____________Gyroscope Digital Low Pass Filter_____________
  myIMU.setGyrDLPF(ICM20948_DLPF_3);
  // Set Signal bandwidth to Level 3 (51Hz) through applying digital filter.
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
  */  

  //_____________Gyroscope Sampling Rate_____________
  // myIMU.setGyrSampleRateDivider(14);
  // Gyro Data Sampling rate = (1125 / (1 + setGyrSampleRateDivider) )
  /* Gyroscope sample rate GSRD divider(= setGyrSampleRateDivider) divides the output rate of the gyroscope. It can only be applied if the corresponding DLPF is 
  not OFF! Divider is a number of 0...255. If sample rates are set for the accelerometer and the gyroscope, the gyroscope sample rate has the priority.*/ 
  
  
  //_____________Gyroscope Resolution_____________
  //myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
  */

  //_____________Temperature Digital Low Pass Filter_____________
  myIMU.setTempDLPF(ICM20948_DLPF_3);
 

  //_____________Magnetometer Sampling rate_____________
  // myIMU.setMagOpMode(AK09916_CONT_MODE_100HZ);
  /* You can set the following modes for the magnetometer:
   * AK09916_PWR_DOWN          Power down to save energy
   * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by 
   *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
   * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
   * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
   * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
   * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
   */
}

void loop() {
  // myIMU.sleep(false);
  // myIMU.enableCycle(ICM20948_ACC_GYR_I2C_MST_CYCLE);
  myIMU.readSensor();
  xyzFloat gValue = myIMU.getCorrectedAccRawValues();
  xyzFloat gyr = myIMU.getCorrectedGyrRawValues();
  xyzFloat magValue = myIMU.getMagValues();
  // float temp = myIMU.getTemperature();
  // float resultantG = myIMU.getResultantG(gValue);
accx=gValue.x;
//Accelerometer
  Serial.print((String)"Ax:"+accx);
  Serial.print("   ");
  Serial.print((String)"Ay:"+gValue.y);
  Serial.print("   ");
  Serial.print((String)"Az:"+gValue.z);
  Serial.print("   ");
  Serial.print((String)"Gx:"+gyr.x);
  Serial.print("   ");
  Serial.print((String)"Gy:"+gyr.y);
  Serial.print("   ");
  Serial.print((String)"Gz:"+gyr.z);
  Serial.print("   ");
  Serial.print((String)"Mx:"+magValue.x);
  Serial.print("   ");
  Serial.print((String)"My:"+magValue.y);
  Serial.print("   ");
  Serial.print((String)"Mz:"+magValue.z);
  Serial.println("   ");



  delay(50);
}
