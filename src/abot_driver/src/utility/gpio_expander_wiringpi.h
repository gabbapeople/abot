

#include <wiringPi.h>
#include <stdint.h>
#include <stdio.h>

#include <ros/ros.h>
#include "wiringPiI2C_utility.h" 

#ifndef GPIO_EXPANDER_WIRINGPI_H
#define GPIO_EXPANDER_WIRINGPI_H

#define GPIO_EXPANDER_DEFAULT_I2C_ADDRESS   0X2A
#define GPIO_EXPANDER_RESET                 0x01
#define GPIO_EXPANDER_PORT_MODE_INPUT       0x04
#define GPIO_EXPANDER_PORT_MODE_PULLUP      0x05
#define GPIO_EXPANDER_PORT_MODE_PULLDOWN    0x06
#define GPIO_EXPANDER_PORT_MODE_OUTPUT      0x07
#define GPIO_EXPANDER_DIGITAL_WRITE_HIGH    0x09
#define GPIO_EXPANDER_DIGITAL_WRITE_LOW     0x0A
#define GPIO_EXPANDER_ANALOG_WRITE          0x0B
#define GPIO_EXPANDER_ANALOG_READ           0x0C
#define GPIO_EXPANDER_PWM_FREQ              0x0D

#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

class WiringPiGpioExpander {
public:
    WiringPiGpioExpander(uint8_t i2cAddress);
    void reset();
    void pwmFreq(uint16_t freq);
    void digitalWrite(int pin, bool value);
    void analogWrite(int pin, uint16_t pulseWidth);
    uint16_t analogRead(int pin);
    void pinMode(int pin, uint8_t mode);

private:
    int _fd;
    uint8_t _i2cAddress;
    uint8_t _analogWriteResolution = 8;
    uint16_t _mapResolution(uint16_t value, uint8_t from, uint8_t to);
    uint16_t _reverse_uint16(uint16_t data);
};

WiringPiGpioExpander::WiringPiGpioExpander(uint8_t i2cAddress = GPIO_EXPANDER_DEFAULT_I2C_ADDRESS) {
    _i2cAddress = i2cAddress;

    if (wiringPiSetupGpio() < 0) {
        ROS_ERROR("Gpio Expander error: GPIO setup error");
        throw std::runtime_error("");
    }

    ROS_INFO("Gpio Expander: GPIO setup");

    int8_t fd = wiringPiI2CSetup((int)_i2cAddress);
    if (fd == -1) {
        ROS_ERROR("Gpio Expander error: I2C setup error");
        throw std::runtime_error("");
    } else {
        _fd = fd;
    }

    ROS_INFO("Gpio Expander: I2C setup error");
}

void WiringPiGpioExpander::reset() {
    wiringPiI2CWrite(_fd, GPIO_EXPANDER_RESET);
}

void WiringPiGpioExpander::pwmFreq(uint16_t freq) {
    wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_PWM_FREQ, _reverse_uint16(freq));
}

void WiringPiGpioExpander::pinMode(int pin, uint8_t mode) {
    uint16_t sendData = _reverse_uint16(1 << pin);
    if (mode == INPUT) {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_PORT_MODE_INPUT, sendData);
    } else if (mode == OUTPUT) {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_PORT_MODE_OUTPUT, sendData);
    } else if (mode == INPUT_PULLUP) {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_PORT_MODE_PULLUP, sendData);
    } else if (mode == INPUT_PULLDOWN) {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_PORT_MODE_PULLDOWN, sendData);
    }   
}

void WiringPiGpioExpander::digitalWrite(int pin, bool value) {
    uint16_t sendData = _reverse_uint16(1 << pin);
    if (value) {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_DIGITAL_WRITE_HIGH, sendData);
    } else {
        wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_DIGITAL_WRITE_LOW, sendData);
    }
}

void WiringPiGpioExpander::analogWrite(int pin, uint16_t pulseWidth) {
    uint16_t value = _mapResolution(pulseWidth, _analogWriteResolution, 16);
    
    uint8_t buff[3];
    buff[0] = pin;
    buff[1] = (value >> 8) & 0xff;
    buff[2] = value & 0xff;
          
    wiringPiI2CWriteRegBlock(_fd, GPIO_EXPANDER_ANALOG_WRITE, 3, buff);
}

uint16_t WiringPiGpioExpander::analogRead(int pin) {
    wiringPiI2CWriteReg16(_fd, GPIO_EXPANDER_ANALOG_READ, pin);
    return _reverse_uint16(wiringPiI2CReadReg16(_fd, GPIO_EXPANDER_ANALOG_READ));  
}

uint16_t WiringPiGpioExpander::_reverse_uint16(uint16_t data) {
    return ((data & 0xff) << 8) | ((data >> 8) & 0xff);
}

uint16_t WiringPiGpioExpander::_mapResolution(uint16_t value, uint8_t from, uint8_t to) {
    if (from == to) {
        return value;
    }
    if (from > to) {
        return value >> (from - to);
    }
    return value << (to - from);
}

#endif // GPIO_EXPANDER_WIRINGPI_H
