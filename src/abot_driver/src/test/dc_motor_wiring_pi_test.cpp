
#include "../dc_motor_wiring_pi.hpp"
#include "std_msgs/Float32.h"

#define MOTOR_1_PIN_D 4 // 4
#define MOTOR_1_PIN_E 18 // 18

DCMotorWiringPi motor1(MOTOR_1_PIN_D, MOTOR_1_PIN_E);

double mapSpeed(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void callback(const std_msgs::Float32& msg) {
    double spd = msg.data;
    uint16_t send_spd = mapSpeed(std::abs(spd), 0, 0.03, 0, 1023);

    ROS_INFO("Got angle spd: %f speed: %d", spd, send_spd);

    if (spd > 0) {
        motor1.cw(send_spd);
    } else if (spd < 0) {
        motor1.ccw(send_spd);
    } else if (spd == 0) {
        motor1.stop();
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dc_motor_wiringpi_test");

    ros::NodeHandle node;
    ros::Subscriber motor_sub = node.subscribe("motor", 1, &callback);

    ros::spin();

    return 0;
}