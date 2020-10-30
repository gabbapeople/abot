
#include "../encoder_wiring_pi.hpp"
#include "std_msgs/Float32.h"

#include <chrono>

typedef boost::chrono::steady_clock time_source;

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
    ros::Publisher right_wheel_angle_pub;
    ros::Publisher left_wheel_velocity_pub;
    ros::Publisher right_wheel_velocity_pub;

    ros::Timer timer;

    std_msgs::Float32 left_wheel_angle_msg;
    std_msgs::Float32 right_wheel_angle_msg;
    std_msgs::Float32 left_wheel_velocity_msg;
    std_msgs::Float32 right_wheel_velocity_msg;

    EncoderWiringPi encoder_left;
    EncoderWiringPi encoder_right;

    double left_wheel_angle;
    double right_wheel_angle;
    double left_wheel_velocity;
    double right_wheel_velocity;

    double left_wheel_position;
    double right_wheel_position;

    double initial_left_wheel_angle;
    double initial_right_wheel_angle;

    time_source::time_point last_time;

    void encodersCallback(const ros::TimerEvent& event);
};

EncodersPair::EncodersPair(double update_rate) :
    encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1),
    encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2) {
    
    initial_left_wheel_angle = encoder_left.getRotation();
    initial_right_wheel_angle = encoder_right.getRotation();

    left_wheel_angle_pub = node.advertise<std_msgs::Float32>("/abot/left_wheel_angle", 1);
    right_wheel_angle_pub = node.advertise<std_msgs::Float32>("/abot/right_wheel_angle", 1);

    left_wheel_velocity_pub = node.advertise<std_msgs::Float32>("/abot/left_wheel_velocity", 1);
    right_wheel_velocity_pub = node.advertise<std_msgs::Float32>("/abot/right_wheel_velocity", 1);

    timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
    last_time = time_source::now();
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {

    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    double raw_left_wheel_angle = -1 * encoder_left.getRotation();
    double raw_right_wheel_angle = 1 * encoder_right.getRotation();
    left_wheel_angle = normalize(raw_left_wheel_angle - initial_left_wheel_angle);
    right_wheel_angle = normalize(raw_right_wheel_angle - initial_right_wheel_angle);

    double delta_left_wheel = left_wheel_angle - left_wheel_position;
    double delta_right_wheel = right_wheel_angle - right_wheel_position;

    left_wheel_position += delta_left_wheel;
    left_wheel_velocity = delta_left_wheel / elapsed.toSec();

    right_wheel_position += delta_right_wheel;
    right_wheel_velocity = delta_right_wheel / elapsed.toSec();
 
    left_wheel_angle_msg.data = left_wheel_angle;
    right_wheel_angle_msg.data = right_wheel_angle;

    left_wheel_velocity_msg.data = left_wheel_velocity;
    right_wheel_velocity_msg.data = right_wheel_velocity;

    left_wheel_angle_pub.publish(left_wheel_angle_msg);
    right_wheel_angle_pub.publish(right_wheel_angle_msg);

    left_wheel_velocity_pub.publish(left_wheel_velocity_msg);
    right_wheel_velocity_pub.publish(right_wheel_velocity_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoders_test");
    EncodersPair encoders_pair(0.1);
    ros::spin();
    return 0;
}