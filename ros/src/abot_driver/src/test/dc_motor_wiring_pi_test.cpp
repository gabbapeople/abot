
#include "../dc_motor_wiring_pi.hpp"
#include "std_msgs/UInt16.h"

#define MOTOR_1_PIN_D 4 // Wiring pi 7 = BCM 4
#define MOTOR_1_PIN_E 18 // Wiring pi 1 = BCM 18
#define MOTOR_2_PIN_D 12 // Wiring pi 26 = BCM 12
#define MOTOR_2_PIN_E 13 // Wiring pi 23 = BCM 13

DCMotorWiringPi left_dc_motor(MOTOR_1_PIN_D, MOTOR_1_PIN_E);
DCMotorWiringPi right_dc_motor(MOTOR_2_PIN_D, MOTOR_2_PIN_E);

void leftMotorCallback(const std_msgs::Uint16& msg) {
    uint16_t pwm = msg.data;
    if (spd > 0) {
        left_dc_motor.cw(pwm);
    } else if (spd < 0) {
        left_dc_motor.ccw(pwm);
    } else if (spd == 0) {
        left_dc_motor.stop();
    }
}

void rightMotorCallback(const std_msgs::Uint16& msg) {
    uint16_t pwm = msg.data;
    if (spd > 0) {
        right_dc_motor.ccw(pwm);
    } else if (spd < 0) {
        right_dc_motor.cw(pwm);
    } else if (spd == 0) {
        right_dc_motor.stop();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dc_motor_wiringpi_test");
    ros::NodeHandle node;
    ros::Subscriber left_motor_sub = node.subscribe("left_motor", 1, &leftMotorCallback);
    ros::Subscriber right_motor_sub = node.subscribe("right_motor", 1, &rightMotorCallback);
    ros::spin();
    return 0;
}
