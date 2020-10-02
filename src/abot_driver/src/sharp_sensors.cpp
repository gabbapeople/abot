

#include "sharp_ir_GP2Y0A21SK.hpp"
#include "sharp_ir_GP2Y0A41SK.hpp"

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#define V_REF 3.27

#define SENSOR_F_EXPANDER_PIN 0
#define SENSOR_R_EXPANDER_PIN 1
#define SENSOR_L_EXPANDER_PIN 2

#define RADS_IN_DEGREE 0.017444444

sensor_msgs::Range prepareRangeMsg(std::string frame, double view_angle, double min_range, double max_range, double distance) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = frame;
    range_msg.radiation_type = 0;
    range_msg.field_of_view = view_angle * RADS_IN_DEGREE;
    range_msg.min_range = min_range;
    range_msg.max_range = max_range;
    range_msg.range = distance;
    return range_msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sharp_sensors");

    WiringPiGpioExpander* expander = new WiringPiGpioExpander(0X2A);

    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_f(expander, SENSOR_F_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_l(expander, SENSOR_L_EXPANDER_PIN);
    SharpIR_GP2Y0A41SK_WiringPi ir_sensor_r(expander, SENSOR_R_EXPANDER_PIN);

    ir_sensor_f.setVref(V_REF);
    ir_sensor_l.setVref(V_REF);
    ir_sensor_r.setVref(V_REF);

    ros::NodeHandle node;

    ros::Rate sleep_rate(20);

    double ir_sensor_f_distance;
    double ir_sensor_r_distance;
    double ir_sensor_l_distance;
    
    ros::Publisher ir_sensor_f_pub = node.advertise<sensor_msgs::Range>("abot/ir_sensor_f", 1);
    ros::Publisher ir_sensor_l_pub = node.advertise<sensor_msgs::Range>("abot/ir_sensor_l", 1);
    ros::Publisher ir_sensor_r_pub = node.advertise<sensor_msgs::Range>("abot/ir_sensor_r", 1);

    sensor_msgs::Range ir_sensor_f_msg;
    sensor_msgs::Range ir_sensor_l_msg;
    sensor_msgs::Range ir_sensor_r_msg;

    while (ros::ok()) {

        ir_sensor_f_distance = ir_sensor_f.getDistance();
        ir_sensor_r_distance = ir_sensor_r.getDistance();
        ir_sensor_l_distance = ir_sensor_l.getDistance();

        ir_sensor_f_msg = prepareRangeMsg("ir_f", 10.0, GP2Y0A41SK_MIN_RANGE, GP2Y0A41SK_INF_RANGE, ir_sensor_f_distance); 
        ir_sensor_f_pub.publish(ir_sensor_f_msg);
        
        ir_sensor_l_msg = prepareRangeMsg("ir_l", 10.0, GP2Y0A41SK_MIN_RANGE, GP2Y0A41SK_INF_RANGE, ir_sensor_l_distance); 
        ir_sensor_l_pub.publish(ir_sensor_l_msg);

        ir_sensor_r_msg = prepareRangeMsg("ir_r", 10.0, GP2Y0A41SK_MIN_RANGE, GP2Y0A41SK_INF_RANGE, ir_sensor_r_distance); 
        ir_sensor_r_pub.publish(ir_sensor_r_msg);

        sleep_rate.sleep();
    }

    return 0;
}