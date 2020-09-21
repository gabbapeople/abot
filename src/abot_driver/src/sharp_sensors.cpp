
#include "sharp_ir_GP2Y0A41SK.hpp"

#include "std_msgs/Float32.h"
#include <ros/console.h>
#include <ros/ros.h>

#define SENSOR_F_M_EXPANDER_PIN 0
#define SENSOR_F_R_EXPANDER_PIN 1
#define SENSOR_F_L_EXPANDER_PIN 2

double mapClipDistance(double raw_distance) {
    double distance; 
    if (raw_distance < 3.0) {
        distance = 0.030;
    } else if (raw_distance > 25.0) {
        distance = 10.0; // 10 meters
    } else {
        distance = raw_distance / 100; // cm to meters
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sharp_sensors");

    WiringPiGpioExpander* expander = new WiringPiGpioExpander(0X2A);

    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_middle(expander, SENSOR_F_M_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_left(expander, SENSOR_F_L_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_right(expander, SENSOR_F_R_EXPANDER_PIN);

    ros::NodeHandle node;

    ros::Rate sleep_rate(50);

    ros::Publisher ir_sensor_front_middle_pub = node.advertise<std_msgs::Float32>("abot/ir_sensor_front_middle", 1);
    ros::Publisher ir_sensor_front_left_pub = node.advertise<std_msgs::Float32>("abot/ir_sensor_front_left", 1);
    ros::Publisher ir_sensor_front_right_pub = node.advertise<std_msgs::Float32>("abot/ir_sensor_front_right", 1);

    double raw_ir_sensor_front_middle_distance;
    double raw_ir_sensor_front_left_distance;
    double raw_ir_sensor_front_right_distance;

    std_msgs::Float32 ir_sensor_front_middle_msg;
    std_msgs::Float32 ir_sensor_front_left_msg;
    std_msgs::Float32 ir_sensor_front_right_msg;

    while (ros::ok()) {

        raw_ir_sensor_front_middle_distance = ir_sensor_front_middle.getDistance();
        raw_ir_sensor_front_left_distance = ir_sensor_front_left.getDistance();
        raw_ir_sensor_front_right_distance = ir_sensor_front_right.getDistance();

        ir_sensor_front_middle_msg.data = mapClipDistance(raw_ir_sensor_front_middle_distance);
        ir_sensor_front_left_msg.data = mapClipDistance(raw_ir_sensor_front_left_distance);
        ir_sensor_front_right_msg.data = mapClipDistance(raw_ir_sensor_front_right_distance);

        ir_sensor_front_middle_pub.publish(ir_sensor_front_middle_msg);
        ir_sensor_front_left_pub.publish(ir_sensor_front_left_msg);
        ir_sensor_front_right_pub.publish(ir_sensor_front_right_msg);

        sleep_rate.sleep();
    }

    return 0;
}