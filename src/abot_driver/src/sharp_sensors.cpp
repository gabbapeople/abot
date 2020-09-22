
#include "sharp_ir_GP2Y0A41SK.hpp"

#include <sensor_msgs/Range.h>
#include <ros/console.h>
#include <ros/ros.h>

#define SENSOR_F_M_EXPANDER_PIN 0
#define SENSOR_F_R_EXPANDER_PIN 1
#define SENSOR_F_L_EXPANDER_PIN 2

#define SENSOR_MIN_RANGE 0.030
#define SENSOR_MAX_RANGE 0.370

double mapClipDistance(double raw_distance) {
    double distance;
    raw_distance = raw_distance / 100; // cm to meters

    // if (raw_distance < SENSOR_MIN_RANGE) {
    //     distance = 0;
    // } else if (raw_distance > SENSOR_MAX_RANGE) {
    //     distance = SENSOR_MAX_RANGE + 0.01; 
    // } else {
    //     distance = raw_distance;
    // }
    distance = raw_distance;
    return distance;
}

sensor_msgs::Range prepareRangeMsg(SharpIR_GP2Y0A41SK_WiringPi sensor, std::string frame) {

    sensor_msgs::Range range_msg;

    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = frame;
    range_msg.radiation_type = 1;
    range_msg.field_of_view = 0.25;
    range_msg.min_range = SENSOR_MIN_RANGE;
    range_msg.max_range = SENSOR_MAX_RANGE;
    range_msg.range = mapClipDistance(sensor.getDistance());

    return range_msg;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sharp_sensors");

    WiringPiGpioExpander* expander = new WiringPiGpioExpander(0X2A);

    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_middle(expander, SENSOR_F_M_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_left(expander, SENSOR_F_L_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_front_right(expander, SENSOR_F_R_EXPANDER_PIN);

    ros::NodeHandle node;

    ros::Rate sleep_rate(50);

    ros::Publisher ir_sensor_front_middle_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_middle", 1);
    ros::Publisher ir_sensor_front_left_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_left", 1);
    ros::Publisher ir_sensor_front_right_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_right", 1);

    sensor_msgs::Range ir_sensor_front_middle_msg;
    sensor_msgs::Range ir_sensor_front_left_msg;
    sensor_msgs::Range ir_sensor_front_right_msg;

    while (ros::ok()) {

        ir_sensor_front_middle_msg = prepareRangeMsg(ir_sensor_front_middle, "ir_fm");
        ir_sensor_front_middle_pub.publish(ir_sensor_front_middle_msg);

        ir_sensor_front_left_msg = prepareRangeMsg(ir_sensor_front_left, "ir_fl");
        ir_sensor_front_left_pub.publish(ir_sensor_front_left_msg);

        ir_sensor_front_right_msg = prepareRangeMsg(ir_sensor_front_right, "ir_fr");
        ir_sensor_front_right_pub.publish(ir_sensor_front_right_msg);

        sleep_rate.sleep();
    }

    return 0;
}