
#include "../encoder_wiring_pi.hpp"
#include "std_msgs/Float32.h"

class EncodersPair {
public:
    EncodersPair(double update_rate);
private:
    ros::NodeHandle node;
    ros::Publisher left_wheel_angle_pub;
    ros::Publisher right_wheel_angle_pub;
    ros::Publisher left_wheel_velocity_pub;
    ros::Publisher right_wheel_velocity_pub;

    ros::Timer timer;

    EncoderWiringPi encoder_left;
    EncoderWiringPi encoder_right;

    double left_wheel_angle;
    double right_wheel_angle;

    void encodersCallback(const ros::TimerEvent& event);
};

EncodersPair::EncodersPair(double update_rate) :
    encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1),
    encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2) {
    
    left_wheel_angle_pub = node.advertise<std_msgs::Float32>("/abot/left_wheel_angle", 1);
    right_wheel_angle_pub = node.advertise<std_msgs::Float32>("/abot/right_wheel_angle", 1);

    timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {
    left_wheel_angle = -1 * encoder_left.getAngle();
    right_wheel_angle = 1 * encoder_right.getAngle();

    left_wheel_angle_msg.data = left_wheel_angle;
    right_wheel_angle_msg.data = right_wheel_angle;

    left_wheel_angle_pub.publish(left_wheel_angle_msg);
    right_wheel_angle_pub.publish(right_wheel_angle_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoders");
    EncodersPair encoders_pair(0.01);
    ros::spin();
    return 0;
}