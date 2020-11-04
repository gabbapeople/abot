#include "abot_teleop.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "abot_teleop_node");
    ros::NodeHandle private_node("~");
    AbotTeleop abotTeleop(private_node);
    ros::spin();
}