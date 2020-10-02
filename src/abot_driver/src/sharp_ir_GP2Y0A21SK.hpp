
#ifndef SHARP_IR_GP2Y0A21SK_WIRING_PI_HPP_
#define SHARP_IR_GP2Y0A21SK_WIRING_PI_HPP_

#include <wiringPi.h>
#include "utility/gpio_expander_wiringpi.h"

#define GPIO_EXPANDER_ADC_RESOLUTION 12

#define GP2Y0A21SK_MIN_RANGE 0.10
#define GP2Y0A21SK_MAX_RANGE 0.80
#define GP2Y0A21SK_INF_RANGE 100.0

class SharpIR_GP2Y0A21SK_WiringPi {
public:
    SharpIR_GP2Y0A21SK_WiringPi(WiringPiGpioExpander* expander, int8_t pin);
    double getDistance();
    void setVref(float v_ref);
private:
    int8_t _pin;
    float _v_ref;

    WiringPiGpioExpander* _expander;
};

SharpIR_GP2Y0A21SK_WiringPi::SharpIR_GP2Y0A21SK_WiringPi(WiringPiGpioExpander* expander, int8_t pin) {
    _expander = expander;
    _pin = pin;

    _expander->pinMode(_pin, INPUT);
    ROS_INFO("Sharp IR sensor: Setup");
}

void SharpIR_GP2Y0A21SK_WiringPi::setVref(float v_ref) {
    _v_ref = v_ref;
}

double SharpIR_GP2Y0A21SK_WiringPi::getDistance() {

    uint16_t adc = _expander->analogRead(_pin);

    double volts = adc * _v_ref / pow(2, GPIO_EXPANDER_ADC_RESOLUTION);

    double distance = 29.988 * pow(volts, -1.173) / 100; // m
    
    if (distance > GP2Y0A21SK_MAX_RANGE)
        distance = GP2Y0A21SK_INF_RANGE;
    if (distance < GP2Y0A21SK_MIN_RANGE)  
        distance = GP2Y0A21SK_MIN_RANGE;
    return distance;
}

#endif // SHARP_IR_GP2Y0A21SK_WIRING_PI_HPP_