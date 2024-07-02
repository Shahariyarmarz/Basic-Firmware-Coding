/*  ADS1115 4-Ch 16-bit ADC Test Program
 
    Basic code to test the ADS1115 ADC Module.  Takes a single-ended reading
    on module channel A0.  Uses default measurement range of of 6.144V 

    Module connections:
      VDD = 5V
      Gnd = Ground
      SCL = SCL / A5 on MCU
      SDA = SDA / A4 on MCU
      ADDR = Board has pull-down to ground (sets I2C address of 0x48)
      A0-A3 = 3.3V on MCU.  Voltage to be measured. Can be any voltage < 5V

    Uses Adafruit ADS1X15 library which can be downloaded via IDE     

    For 16-bit vs 12-bit verification, connect input to ground and verify the 
    minimum reading is less than 3mV.  Readings should change in approximately
    200uV (0.0002V) steps
*/
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

# define SDApin 38
# define SCLpin 39


Adafruit_ADS1115 ads;  
//===============================================================================
//  Initialization
//===============================================================================
void setup(void)
{
  Serial.begin(115200);
  Wire1.begin (SDApin, SCLpin);

  Serial.println("Read single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 0.1875mV");

  if (!ads.begin(0x48,&Wire1)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  ads.setDataRate(RATE_ADS1115_128SPS);
}
//===============================================================================
//  Main
//===============================================================================
void loop(void)
{
  int16_t adc0, adc1, adc2, adc3;
  // float volts0, volts1, volts2, volts3, volts;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  // volts0 = ads.computeVolts(adc0);
  // volts1 = ads.computeVolts(adc1);
  // volts2 = ads.computeVolts(adc2);
  // volts3 = ads.computeVolts(adc3);

  // Serial.println("-----------------------------------------------------------");

  Serial.print(adc0);
  Serial.print(",");
  Serial.print(adc1);
  Serial.print(",");
  Serial.print(adc2);
  Serial.print(",");
  Serial.print(adc3);

  // Serial.print("AIN0: ");Serial.print(volts0, 4); Serial.println("V");
  // Serial.print("AIN1: ");Serial.print(volts1, 4); Serial.println("V");
  // Serial.print("AIN2: ");Serial.print(volts2, 4); Serial.println("V");  
  // Serial.print("AIN3: ");Serial.print(volts3, 4); Serial.println("V");
  Serial.println();
  // delay(50);
}
