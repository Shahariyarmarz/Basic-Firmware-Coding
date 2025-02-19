/*****************************************************************
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
	LSM9DS1 --------- Arduino
	 SCL ---------- SCL (IO40)
	 SDA ---------- SDA (IO41)
	 VDD ------------- 3.3V
	 GND ------------- GND
  (CSG, CSXM, SDOG, and SDOXM should all be pulled high.
  Jumpers on the breakout board will do this for you.)
*****************************************************************/
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>   // The SFE_LSM9DS1 library requires both Wire and SPI be
#include <SparkFunLSM9DS1.h>

// Use the LSM9DS1 class to create an object. [imu] can be
LSM9DS1 imu;

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
// #define DECLINATION 0.33 // Declination (degrees) in Boulder, CO.

//Function definitions
// void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

void setup() {
  Serial.begin(115200);

  Wire.begin(41, 40);

  Serial.println("Initializing the LSM9DS1");

  //--------------------------------------------Gyro Settings--------------------------------------------------
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 2000; // Set scale to +/-245dps

  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9 LPF1 cut off 5Hz    4 = 238 LPF1 cut off 76Hz      (LPF1=Low Pass Filter 1)
  // 2 = 59.5 LPF1 cut off 19Hz   5 = 476 LPF1 cut off 100Hz
  // 3 = 119 LPF1 cut off 38Hz    6 = 952 LPF1 cut off 100Hz
  imu.settings.gyro.sampleRate = 5; // 476Hz sample rate with 100Hz cutoff low pass filter 

  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Allowed values: 0-3. Actual value of cutoff frequency of LPF depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;  // BW = 100Hz also try with 2

  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF enable
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.(Datasheet section 7.14)
  // High Pass Filter cut off frequency vary with respect to the sample rate of LPF1 (ODR)
  // 0 = 30Hz Cut off at ODR = 476Hz, 15Hz Cut off at ODR = 238Hz                      5 = 1Hz Cut off at ODR = 476Hz, 0.5Hz Cut off at ODR = 238Hz            
  // 1 = 15Hz Cut off at ODR = 476Hz, 8Hz Cut off at ODR = 238Hz                       6 = 0.5Hz Cut off at ODR = 476Hz, 0.2Hz Cut off at ODR = 238Hz 
  // 2 = 8Hz Cut off at ODR = 476Hz, 4Hz Cut off at ODR = 238Hz                        7 = 0.2Hz Cut off at ODR = 476Hz, 0.1Hz Cut off at ODR = 238Hz 
  // 3 = 4Hz Cut off at ODR = 476Hz, 2Hz Cut off at ODR = 238Hz                        8 = 0.1Hz Cut off at ODR = 476Hz, 0.05Hz Cut off at ODR = 238Hz
  // 4 = 2Hz Cut off at ODR = 476Hz, 1Hz Cut off at ODR = 238Hz                        9 = 0.05Hz Cut off at ODR = 476Hz, 0.02Hz Cut off at ODR = 238Hz
  imu.settings.gyro.HPFCutoff = 6; // HPF cutoff = 0.5Hz at ODR 476

  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z

  //-------------------------------------------Acceler Settings-------------------------------------------------
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 16; // Set accel scale to +/-16g.

  // [sampleRate] sets the output data rate (ODR) of the accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS DISABLED! 
  // Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 5; // Set accel to 10Hz.

  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 2; // BW = 105Hz

  // [highResEnable] enables or disables high resolution 
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  // imu.settings.accel.highResBandwidth = 2; 

  //---------------------------------------Magneto Settings---------------------------------------------------------
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 16; // Set mag scale to +/-16 Gs
  
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 7; // Set OD rate to 80Hz

  // [tempCompensationEnable] enables or disables 
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;

  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.

  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode

  if (imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire) == false)  // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will "
                   "work for an out of the box LSM9DS1 "
                   "Breakout, but may need to be modified "
                   "if the board jumpers are.");
    while (1)
      ;
  }


}

void loop() {
  // Update the sensor values whenever new data is available
  if (imu.gyroAvailable()) {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if (imu.accelAvailable()) {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if (imu.magAvailable()) {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  Serial.print((String) "Gx:" + imu.gx);
  Serial.print("  ");
  Serial.print((String) "Gy:" + imu.gy);
  Serial.print("  ");
  Serial.print((String) "Gz:" + imu.gz);
  Serial.print("  ");
  Serial.print((String) "Ax:" + imu.ax);
  Serial.print("  ");
  Serial.print((String) "Ay:" + imu.ay);
  Serial.print("  ");
  Serial.print((String) "Az:" + imu.az);
  Serial.print("  ");
  Serial.print((String) "Mx:" + imu.mx);
  Serial.print("  ");
  Serial.print((String) "My:" + imu.my);
  Serial.print("  ");
  Serial.print((String) "Mz:" + imu.mz);
  Serial.println("  ");
  delay(50);
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's mag x and y
  // axes are opposite to the accelerometer, so my, mx are
  // substituted for each other.
  // printAttitude(imu.ax, imu.ay, imu.az,
  //               -imu.my, -imu.mx, imu.mz);
  // Serial.println();
}


// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
// void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
// {
//   float roll = atan2(ay, az);
//   float pitch = atan2(-ax, sqrt(ay * ay + az * az));

//   float heading;
//   if (my == 0)
//     heading = (mx < 0) ? PI : 0;
//   else
//     heading = atan2(mx, my);

//   heading -= DECLINATION * PI / 180;

//   if (heading > PI) heading -= (2 * PI);
//   else if (heading < -PI) heading += (2 * PI);

//   // Convert everything from radians to degrees:
//   heading *= 180.0 / PI;
//   pitch *= 180.0 / PI;
//   roll  *= 180.0 / PI;

//   // Serial.print("Pitch, Roll: ");
//   // Serial.print((String) "PITCH:" + pitch);
//   // Serial.print("  ");
//   // Serial.print((String) "Roll:" + roll);
//   // Serial.print("  ");
//   // Serial.println((String) "HEADING:" + heading);
// }