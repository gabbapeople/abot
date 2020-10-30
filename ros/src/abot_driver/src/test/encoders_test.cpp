
#include "../encoder_wiring_pi.hpp"
#include "std_msgs/Float32.h"

double normalize(double angle) {
    angle += M_PI;
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    angle -= M_PI;
    return angle;
}

class EncodersPair {
public:
    EncodersPair(double update_rate);
private:
    ros::NodeHandle node;
    ros::Publisher left_wheel_angle_pub;
    ros::Publisher right_Wheel_angle_pub;

    ros::Timer timer;

    std_msgs::Float32 left_wheel_angle_msg;
    std_msgs::Float32 right_wheel_angle_msg;

    EncoderWiringPi encoder_left;
    EncoderWiringPi encoder_right;

    double left_wheel_angle;
    double right_wheel_angle;

    double initial_left_wheel_angle;
    double initial_right_wheel_angle;

    void encodersCallback(const ros::TimerEvent& event);
}

EncodersPair::EncodersPair(double update_rate) :
    encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1),
    encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2) {
    
    initial_left_wheel_angle = encoder_left.getRotation();
    initial_right_wheel_angle = encoder_right.getRotation();

    left_wheel_angle_pub = node.advertise<std_msgs::Float32>("/left_wheel_angle", 1);
    right_wheel_angle_pub = node.advertise<std_msgs::Float32>("/right_wheel_angle", 1);

    timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {
    double current_left_wheel_angle = -1 * encoder_left.getRotation();
    double current_right_wheel_angle = 1 * encoder_right.getRotation();

    left_wheel_angle_msg.data = normalize(current_left_wheel_angle - initial_left_wheel_angle);
    right_wheel_angle_msg.data = normalize(current_right_wheel_angle - initial_right_wheel_angle);

    left_wheel_angle_pub.publish(left_wheel_angle_msg);
    right_wheel_angle_pub.publish(right_wheel_angle_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoders_test");
    EncodersPair encoders_pair(1.0);
    ros::spin();
    return 0;
}