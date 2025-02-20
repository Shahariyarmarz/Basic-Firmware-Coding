#include "driver/mcpwm.h"  // "arduino-esp32-master" v2.0.14   (https://github.com/espressif/arduino-esp32/)

#define LED_5700K 13    // LED_5700K pin
#define LED_3000K 12    // LED_3000K pin  (this output pin has to be inverted in setup)
uint32_t frequency = 300000;  // chosen to have a nice display on the oscilloscope...
// the actual frequency is half this value (the counter counts up then down)
mcpwm_config_t pwm_config_0;  // initialize "pwm_config" structure
mcpwm_config_t pwm_config_1;

const int colorpotPin = 34;
const int brightpotPin = 35;
int colorPotValue = 0;
int brightPotValue = 0;
int colorPotValueMap = 0;
int brightPotValueMap = 0;  // percentage of the sum of the high states LED_5700K and LED_3000K
int valPwm_3000K = 0;       // PWM value for the 3000K LED
int valPwm_5700K = 0;       // PWM value for the 5700K LED

void setup() {
  Serial.begin(115200);
  // Serial.setTimeout(500);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LED_5700K);  // initializes gpio "LED_5700K" for MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, LED_3000K);  // initializes gpio "LED_3000K" for MCPWM
  GPIO.func_out_sel_cfg[LED_3000K].inv_sel = 1;       // <= this line inverts the corresponding output
  pwm_config_0.frequency = frequency;
  pwm_config_0.cmpr_a = 0;                                 // Duty cycle of PWMxA
  pwm_config_0.cmpr_b = (50);                             // Note: the duty cycle of PWMxB is inverted (from where "100-x")
  pwm_config_0.counter_mode = MCPWM_UP_COUNTER;      // creates symetrical vaweforms
  pwm_config_0.duty_mode = MCPWM_DUTY_MODE_0;              //
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_0);  // Configure PWM0A & PWM0B with settings

  
  pwm_config_1.frequency = frequency;
  pwm_config_1.cmpr_a = 0;                                 // Duty cycle of PWMxA
  pwm_config_1.cmpr_b = (50);                             // Note: the duty cycle of PWMxB is inverted (from where "100-x")
  pwm_config_1.counter_mode = MCPWM_UP_COUNTER;      // creates symetrical vaweforms
  pwm_config_1.duty_mode = MCPWM_DUTY_MODE_0;              //
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config_1);
}

void loop() {
  readInputsValues();
  valPwm_5700K = (brightPotValueMap * colorPotValueMap) / 100;
  valPwm_3000K = brightPotValueMap - valPwm_5700K;
  pwm_config_0.cmpr_a = 40;          // Duty cycle of PWMxA (HIGH level)
  pwm_config_1.cmpr_b = 50;  // The duty cycle of PWMxB is inverted (100-x)
  updatePWM();
  // printValues(); // uncomment if you want display on the serial console...
  delay(10);
}

void readInputsValues() {
  colorPotValue = 2000;
  colorPotValueMap = map(colorPotValue, 0, 4095, 0, 100);
  brightPotValue = 2000;
  brightPotValueMap = map(brightPotValue, 0, 4095, 0, 100);
}

void updatePWM() {
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_0);  // Configure PWM0A & PWM0B with settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config_1);
}
