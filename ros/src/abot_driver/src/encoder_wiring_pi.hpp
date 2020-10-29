#ifndef ENCODER_WIRING_PI_HPP_
#define ENCODER_WIRING_PI_HPP_

#include <ros/ros.h>
#include <wiringPi.h>

#define ENCODER_1_PIN_A 17  // Wiring pi 0 = BCM 17
#define ENCODER_1_PIN_B 27  // Wiring pi 2 = BCM 27
#define ENCODER_2_PIN_A 24  // Wiring pi 5 = BCM 24
#define ENCODER_2_PIN_B 25  // Wiring pi 6 = BCM 25

#define TICKS_PER_REVOLUTION 1440

namespace EncoderWiringPiISR {

    volatile long encoderPosition1;
    volatile long encoderPosition2;
    volatile uint8_t encoderState1;
    volatile uint8_t encoderState2;

    void encoderISR(const int pinA, const int pinB, volatile long &encoderPosition, volatile uint8_t &encoderState) {
        uint8_t valA = digitalRead(pinA);
        uint8_t valB = digitalRead(pinB);
        uint8_t s = encoderState & 3;
        if (valA) s |= 4;
        if (valB) s |= 8; 
        encoderState = (s >> 2);
        switch (s) {
            case 1: case 7: case 8: case 14:
                encoderPosition++;
                return;
            case 2: case 4: case 11: case 13:
                encoderPosition--;
                return;
            case 3: case 12:
                encoderPosition += 2;
                return;
            case 6: case 9:
                encoderPosition -= 2;
                return;
        }
    }

    void encoderISR1(void) {
        encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B, encoderPosition1, encoderState1);
    }

    void encoderISR2(void) {
        encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B, encoderPosition2, encoderState2);
    }
}

class EncoderWiringPi {
public:
    EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition);
    long getPosition();
    double getRotation();
private:
    int _pinA;
    int _pinB;
    volatile long* _encoderPosition;
};

EncoderWiringPi::EncoderWiringPi(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition) {
    _encoderPosition = encoderPosition;

    if (wiringPiSetupSys() < 0) {
        ROS_ERROR("Encoder wiringPi error: GPIO setup error");
        throw std::runtime_error("");
    }
    ROS_INFO("Encoder wiringPi: GPIO setup");

    _pinA = pinA;
    _pinB = pinB;

    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);

    pullUpDnControl(_pinA, PUD_UP);
    pullUpDnControl(_pinB, PUD_UP);

    if (wiringPiISR(_pinA, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinA error");
        throw std::runtime_error("");
    }

    if (wiringPiISR(_pinB, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinB error");
        throw std::runtime_error("");
    }

    ROS_INFO("Encoder wiringPi: ISR setup");
}

long EncoderWiringPi::getPosition() {
    return *_encoderPosition;
}

double EncoderWiringPi::getRotation() {
    long position = *_encoderPosition;
    return 2 * M_PI * static_cast<double>(position) / TICKS_PER_REVOLUTION;
}

#endif // ENCODER_WIRING_PI_HPP_