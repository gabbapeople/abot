
#include "sharp_ir_GP2Y0A21SK.hpp"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <ros/console.h>
#include <ros/ros.h>

#define SENSOR_F_M_EXPANDER_PIN 0
#define SENSOR_F_R_EXPANDER_PIN 1
#define SENSOR_F_L_EXPANDER_PIN 2

#define SENSOR_MIN_RANGE 0.100
#define SENSOR_MAX_RANGE 0.800

#define RADS_IN_DEGREE 0.017444444
// Fake params
unsigned int num_readings = 10;
double frequency = 100;
double view_angle = 10; // in degrees

sensor_msgs::LaserScan prepareScanMsg(std::string frame, double distance) {
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = frame;
    
    scan.angle_min = -1 * view_angle / 2 * RADS_IN_DEGREE;
    scan.angle_max = view_angle / 2 * RADS_IN_DEGREE;

    scan.angle_increment = view_angle * RADS_IN_DEGREE / num_readings;
    scan.time_increment = (1 / frequency) / (num_readings);

    scan.range_min = SENSOR_MIN_RANGE;
    scan.range_max = SENSOR_MAX_RANGE;

    scan.ranges.resize(num_readings);

    for(unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = distance;
    }

    return scan;
}

sensor_msgs::Range prepareRangeMsg(std::string frame, double distance) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = frame;
    range_msg.radiation_type = 1;
    range_msg.field_of_view = view_angle * RADS_IN_DEGREE;
    range_msg.min_range = SENSOR_MIN_RANGE;
    range_msg.max_range = SENSOR_MAX_RANGE;
    range_msg.range = distance;
    return range_msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sharp_sensors");

    WiringPiGpioExpander* expander = new WiringPiGpioExpander(0X2A);

    SharpIR_GP2Y0A21SK_WiringPi ir_sensor_front_middle(expander, SENSOR_F_M_EXPANDER_PIN);
    SharpIR_GP2Y0A21SK_WiringPi ir_sensor_front_left(expander, SENSOR_F_L_EXPANDER_PIN);
    SharpIR_GP2Y0A21SK_WiringPi ir_sensor_front_right(expander, SENSOR_F_R_EXPANDER_PIN);

    ros::NodeHandle node;

    ros::Rate sleep_rate(100);

    double ir_middle_distance;
    double ir_right_distance;
    double ir_left_distance;
    
    ros::Publisher ir_sensor_front_middle_scan_pub = node.advertise<sensor_msgs::LaserScan>("abot/ir_sensor_front_middle", 1);
    ros::Publisher ir_sensor_front_left_scan_pub = node.advertise<sensor_msgs::LaserScan>("abot/ir_sensor_front_left", 1);
    ros::Publisher ir_sensor_front_right_scan_pub = node.advertise<sensor_msgs::LaserScan>("abot/ir_sensor_front_right", 1);

    ros::Publisher ir_sensor_front_middle_range_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_middle", 1);
    ros::Publisher ir_sensor_front_left_range_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_left", 1);
    ros::Publisher ir_sensor_front_right_range_pub = node.advertise<sensor_msgs::Range>("ir_sensor_front_right", 1);

    sensor_msgs::LaserScan ir_sensor_front_middle_scan_msg;
    sensor_msgs::LaserScan ir_sensor_front_left_scan_msg;
    sensor_msgs::LaserScan ir_sensor_front_right_scan_msg;

    sensor_msgs::Range ir_sensor_front_middle_range_msg;
    sensor_msgs::Range ir_sensor_front_left_range_msg;
    sensor_msgs::Range ir_sensor_front_right_range_msg;

    while (ros::ok()) {

        ir_middle_distance = ir_sensor_front_middle.getDistance();
        if (ir_middle_distance >= SENSOR_MIN_RANGE && ir_middle_distance <= SENSOR_MAX_RANGE) {
            ir_sensor_front_middle_scan_msg = prepareScanMsg("ir_fm", ir_middle_distance); 
            ir_sensor_front_middle_scan_pub.publish(ir_sensor_front_middle_scan_msg);
        }
            ir_sensor_front_middle_range_msg = prepareRangeMsg("ir_fm", ir_middle_distance); 
            ir_sensor_front_middle_range_pub.publish(ir_sensor_front_middle_range_msg);

        ////}

        ir_left_distance = ir_sensor_front_left.getDistance();
        if (ir_left_distance >= SENSOR_MIN_RANGE && ir_left_distance <= SENSOR_MAX_RANGE) {
            ir_sensor_front_left_scan_msg = prepareScanMsg("ir_fl", ir_left_distance); 
            ir_sensor_front_left_scan_pub.publish(ir_sensor_front_left_scan_msg);
        }
            ir_sensor_front_left_range_msg = prepareRangeMsg("ir_fl", ir_left_distance); 
            ir_sensor_front_left_range_pub.publish(ir_sensor_front_left_range_msg);

        //}

        ir_right_distance = ir_sensor_front_right.getDistance();
        if (ir_right_distance >= SENSOR_MIN_RANGE && ir_right_distance <= SENSOR_MAX_RANGE) {
            ir_sensor_front_right_scan_msg = prepareScanMsg("ir_fr", ir_right_distance); 
            ir_sensor_front_right_scan_pub.publish(ir_sensor_front_right_scan_msg);
        }
            ir_sensor_front_right_range_msg = prepareRangeMsg("ir_fr", ir_right_distance); 
            ir_sensor_front_right_range_pub.publish(ir_sensor_front_right_range_msg);            
        //}        

        sleep_rate.sleep();
    }

    return 0;
}