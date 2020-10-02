

#include "sensors_array.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors_array");

    RangeSensorArray array("/ir_point_cloud", "base_link");


    array.addSensor("abot/ir_sensor_f", "ir_f");
    array.addSensor("abot/ir_sensor_r", "ir_r");
    array.addSensor("abot/ir_sensor_l", "ir_l");
    
    ros::spin();

    return 0;
}