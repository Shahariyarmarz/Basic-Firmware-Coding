#ifndef MYPWM_H
#define MYPWM_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

class mypwm {
public:
    mypwm();  // Constructor

    void setFrequency(float freq);
    bool setDutyCycle(float duty_cycle, volatile uint16_t &ocr_register);

private:
    uint32_t freq_icr;
    void setTimer1Prescaler(uint16_t prescaler);
    
    typedef struct {
        uint16_t prescaler;
        uint16_t divider;
    } prescaler_t;

    static const prescaler_t prescaler_table[5];
};

#endif
