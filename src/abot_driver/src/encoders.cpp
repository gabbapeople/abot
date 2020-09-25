
#include "encoder_wiring_pi.hpp"
#include "std_msgs/Float32.h"

double normalize(double angle) {
    angle += M_PI;
    angle = fmod(angle, 2*M_PI);
    if (angle < 0) angle += 2*M_PI;
    angle -= M_PI;
    return angle;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoders");

    ros::NodeHandle node;
    ros::Publisher leftSideAnglePub = node.advertise<std_msgs::Float32>("abot/left_wheel_angle", 1);
    ros::Publisher rightSideAnglePub = node.advertise<std_msgs::Float32>("abot/right_wheel_angle", 1);

    EncoderWiringPi encoderLeft = EncoderWiringPi(ENCODER_1_PIN_A,
        ENCODER_1_PIN_B,
        &EncoderWiringPiISR::encoderISR1,
        &EncoderWiringPiISR::encoderPosition1);

    EncoderWiringPi encoderRight = EncoderWiringPi(ENCODER_2_PIN_A,
        ENCODER_2_PIN_B,
        &EncoderWiringPiISR::encoderISR2,
        &EncoderWiringPiISR::encoderPosition2);

    ros::Rate sleep_rate(100);

    std_msgs::Float32 left_wheel_msg;
    std_msgs::Float32 right_wheel_msg;

    double initial_angle_left;
    double initial_angle_right;
    
    double angle_left;
    double angle_right;
    
    initial_angle_left = encoderLeft.getRotation();
    initial_angle_right = encoderRight.getRotation();
    
    while (ros::ok()) {

        double current_angle_left = -1 * encoderLeft.getRotation();
        double current_angle_right = 1 * encoderRight.getRotation();
        
        angle_left = normalize(current_angle_left - initial_angle_left);
        angle_right = normalize(current_angle_right - initial_angle_right);
        
        left_wheel_msg.data = angle_left;
        right_wheel_msg.data = angle_right;

        leftSideAnglePub.publish(left_wheel_msg);
        rightSideAnglePub.publish(right_wheel_msg);

        sleep_rate.sleep();
    }

    return 0;
}