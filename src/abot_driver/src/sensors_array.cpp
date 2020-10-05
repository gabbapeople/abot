

#include "sensors_array.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors_array");

    RangeSensorArray array("/ir_point_cloud", "base_link");


    array.addSensor("abot/ir_sensor_f", "ir_f");
    array.addSensor("abot/ir_sensor_fr", "ir_fr");
    array.addSensor("abot/ir_sensor_r", "ir_r");
    array.addSensor("abot/ir_sensor_br", "ir_br");

    array.addSensor("abot/ir_sensor_b", "ir_b");
    array.addSensor("abot/ir_sensor_bl", "ir_bl");
    array.addSensor("abot/ir_sensor_l", "ir_l");
    array.addSensor("abot/ir_sensor_fl", "ir_fl");

    ros::spin();

    return 0;
}