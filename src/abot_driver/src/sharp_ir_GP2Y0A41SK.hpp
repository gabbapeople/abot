
#ifndef SHARP_IR_GP2Y0A41SK_WIRING_PI_HPP_
#define SHARP_IR_GP2Y0A41SK_WIRING_PI_HPP_

#include <wiringPi.h>
#include "utility/gpio_expander_wiringpi.h"

#define GPIO_EXPANDER_ADC_RESOLUTION 12
#define RPI_V_REF 5.02

class SharpIR_GP2Y0A41SK_WiringPi {
public:
    SharpIR_GP2Y0A41SK_WiringPi(WiringPiGpioExpander* expander, int8_t pin);
    double getDistance();
private:
    int8_t _pin;
    WiringPiGpioExpander* _expander;
};

SharpIR_GP2Y0A41SK_WiringPi::SharpIR_GP2Y0A41SK_WiringPi(WiringPiGpioExpander* expander, int8_t pin) {
    _expander = expander;
    _pin = pin;

    _expander->pinMode(_pin, INPUT);
    ROS_INFO("Sharp IR sensor: Setup");
}

double SharpIR_GP2Y0A41SK_WiringPi::getDistance() {

    uint16_t adc = _expander->analogRead(_pin);

    double volts = adc * RPI_V_REF / pow(2, GPIO_EXPANDER_ADC_RESOLUTION);

    double distance = 12.08 * pow(volts, -1.058); // cm
    
    return distance;
}

#endif // SHARP_IR_GP2Y0A41SK_WIRING_PI_HPP_