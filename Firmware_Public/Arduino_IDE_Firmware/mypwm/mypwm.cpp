#include "mypwm.h"

const mypwm::prescaler_t mypwm::prescaler_table[5] = {
    { (1 << CS10), 1 },
    { (1 << CS11), 8 },
    { (1 << CS11) | (1 << CS10), 64 },
    { (1 << CS12), 256 },
    { (1 << CS12) | (1 << CS10), 1024 }
};

// Constructor
mypwm::mypwm() {
    freq_icr = 0;
}

// Set Timer1 Prescaler
void mypwm::setTimer1Prescaler(uint16_t prescaler) {
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // Clear previous prescaler
    TCCR1B |= prescaler;
}

// Set PWM Frequency
void mypwm::setFrequency(float freq) {
    freq_icr = floor(((float)F_CPU / freq) - 1);
    uint8_t index = 0;

    while (1) {
        if (freq_icr > 65535) {
            index++;
            freq_icr = floor(((float)F_CPU / freq / float(prescaler_table[index].divider)) - 1);
            continue;
        } else {
            setTimer1Prescaler(prescaler_table[index].prescaler);
            TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
            ICR1 = (uint16_t)freq_icr;
            break;
        }
    }
}

// Set Duty Cycle
bool mypwm::setDutyCycle(float duty_cycle, volatile uint16_t &ocr_register) {
    if (duty_cycle >= 0.0 && duty_cycle <= 100.0) {
        uint32_t duty_cycle_ocr = floor((duty_cycle / 100) * freq_icr);
        ocr_register = (uint16_t)duty_cycle_ocr;
        return true;
    }
    return false;
}
