
#include "../encoder_wiring_pi.hpp"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder_wiringpi_test");

    ros::NodeHandle node;
    ros::Publisher encoder_position_pub = node.advertise<std_msgs::Int32>("encoder_position", 1);
    ros::Publisher encoder_rotation_pub = node.advertise<std_msgs::Float32>("encoder_rotation", 1);

    EncoderWiringPi encoder1 = EncoderWiringPi(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1);

    ros::Rate sleep_rate(100);

    std_msgs::Int32 msg_position;
    std_msgs::Float32 msg_rotation;

    while (ros::ok()) {

        msg_position.data = encoder1.getPosition();
        encoder_position_pub.publish(msg_position);

        msg_rotation.data = encoder1.getRotation();
        encoder_rotation_pub.publish(msg_rotation);

        sleep_rate.sleep();
        
    }

    return 0;
}