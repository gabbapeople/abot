
#include "../dc_motor_wiring_pi.hpp"
#include "../encoder_wiring_pi.hpp"
#include "std_msgs/Float32.h"

#include <chrono>
#include <functional>

#define MOTOR_1_PIN_D 4 // Wiring pi 7 = BCM 4
#define MOTOR_1_PIN_E 18 // Wiring pi 1 = BCM 18

#define MOTOR_2_PIN_D 12 // Wiring pi 26 = BCM 12
#define MOTOR_2_PIN_E 13 // Wiring pi 23 = BCM 13

#define MOTOR_LEFT_PWM_THRESHOLD 0
#define MOTOR_RIGHT_PWM_THRESHOLD 0

#define MAX_ANGLUAR_WHEEL_SPEED 21

DCMotorWiringPi left_dc_motor(MOTOR_1_PIN_D, MOTOR_1_PIN_E);
DCMotorWiringPi right_dc_motor(MOTOR_2_PIN_D, MOTOR_2_PIN_E);

typedef std::chrono::steady_clock time_source;

double mapSpeed(double angluar_wheel_speed, double max_angluar_wheel_speed, double min_pwm, double max_pwm) {
    return angluar_wheel_speed * (max_pwm - min_pwm) / (max_angluar_wheel_speed - 0) + min_pwm;
}

void leftMotorCallback(const std_msgs::Float32& msg) {
    double spd = msg.data;
    uint16_t pwm = mapSpeed(std::abs(spd), MAX_ANGLUAR_WHEEL_SPEED, MOTOR_LEFT_PWM_THRESHOLD, RPI_MAX_PWM_VALUE);

    //ROS_INFO("LEFT MOTOR:    SPD: %f    PWM: %d", spd, pwm);
    
    if (spd > 0) {
        left_dc_motor.cw(pwm);
    } else if (spd < 0) {
        left_dc_motor.ccw(pwm);
    } else if (spd == 0) {
        left_dc_motor.stop();
    }
}

void rightMotorCallback(const std_msgs::Float32& msg) {
    double spd = msg.data;
    uint16_t pwm = mapSpeed(std::abs(spd), MAX_ANGLUAR_WHEEL_SPEED, MOTOR_LEFT_PWM_THRESHOLD, RPI_MAX_PWM_VALUE);

    //ROS_INFO("RIGHT MOTOR:    SPD: %f    PWM: %d", spd, pwm);
    
    if (spd > 0) {
        right_dc_motor.ccw(pwm);
    } else if (spd < 0) {
        right_dc_motor.cw(pwm);
    } else if (spd == 0) {
        right_dc_motor.stop();
    }
}

double normalize(double angle) {
    angle += M_PI;
    angle = fmod(angle, 2*M_PI);
    if (angle < 0) angle += 2*M_PI;
    angle -= M_PI;
    return angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dc_motor_wiringpi_test");

    EncoderWiringPi encoderLeft = EncoderWiringPi(ENCODER_1_PIN_A,
        ENCODER_1_PIN_B,
        &EncoderWiringPiISR::encoderISR1,
        &EncoderWiringPiISR::encoderPosition1);

    EncoderWiringPi encoderRight = EncoderWiringPi(ENCODER_2_PIN_A,
        ENCODER_2_PIN_B,
        &EncoderWiringPiISR::encoderISR2,
        &EncoderWiringPiISR::encoderPosition2);

    ros::NodeHandle node;
    ros::Subscriber left_motor_sub = node.subscribe("left_motor", 1, &leftMotorCallback);
    ros::Subscriber right_motor_sub = node.subscribe("right_motor", 1, &rightMotorCallback);

    ros::Publisher left_wheel_current_angle_pub = node.advertise<std_msgs::Float32>("current_left_motor_angle", 1);
    ros::Publisher right_wheel_current_angle_pub = node.advertise<std_msgs::Float32>("current_right_motor_angle", 1);

    ros::Publisher left_wheel_current_vel_pub = node.advertise<std_msgs::Float32>("current_left_motor_vel", 1);
    ros::Publisher right_wheel_current_vel_pub = node.advertise<std_msgs::Float32>("current_right_motor_vel", 1);

    ros::Publisher left_wheel_current_accel_pub = node.advertise<std_msgs::Float32>("current_left_motor_accel", 1);
    ros::Publisher right_wheel_current_accel_pub = node.advertise<std_msgs::Float32>("current_right_motor_accel", 1);

    ros::Publisher left_wheel_max_vel_pub = node.advertise<std_msgs::Float32>("max_left_motor_vel", 1);
    ros::Publisher right_wheel_max_vel_pub = node.advertise<std_msgs::Float32>("max_right_motor_vel", 1);

    ros::Publisher left_wheel_max_accel_pub = node.advertise<std_msgs::Float32>("max_left_motor_accel", 1);
    ros::Publisher right_wheel_max_accel_pub = node.advertise<std_msgs::Float32>("max_right_motor_accel", 1);

    time_source::time_point this_time;
    time_source::time_point last_time;
    
    ros::Rate loop_rate(50);
    
    this_time = time_source::now();
    last_time = this_time;

    double initial_angle_left;
    double initial_angle_right;
    
    double angle_left;
    double angle_right;
    
    initial_angle_left = encoderLeft.getRotation();
    initial_angle_right = encoderRight.getRotation();

    double left_wheel_angle;
    double right_wheel_angle;
    
    double left_wheel_pos;
    double right_wheel_pos;
    double left_wheel_pos_offset;
    double right_wheel_pos_offset;

    double left_wheel_vel;
    double right_wheel_vel;

    double left_wheel_vel_prev;
    double right_wheel_vel_prev;

    double left_wheel_accel;
    double right_wheel_accel;

    double max_left_wheel_accel = 0;
    double max_right_wheel_accel = 0;
    double max_left_wheel_vel = 0;
    double max_right_wheel_vel = 0;

    std_msgs::Float32 left_wheel_current_angle_msg;
    std_msgs::Float32 right_wheel_current_angle_msg;

    std_msgs::Float32 left_wheel_current_vel_msg;
    std_msgs::Float32 right_wheel_current_vel_msg;

    std_msgs::Float32 left_wheel_current_accel_msg;
    std_msgs::Float32 right_wheel_current_accel_msg;

    std_msgs::Float32 left_wheel_max_vel_msg;
    std_msgs::Float32 right_wheel_max_vel_msg;

    std_msgs::Float32 left_wheel_max_accel_msg;
    std_msgs::Float32 right_wheel_max_accel_msg;

    while (ros::ok()) {

        this_time = time_source::now();
        std::chrono::duration<double> elapsed_duration = this_time - last_time;
        ros::Duration elapsed(elapsed_duration.count());
        
        //ROS_INFO("PERIOD %f", elapsed.toSec());

        // Get encoders
        double current_angle_left = -1 * encoderLeft.getRotation();
        double current_angle_right = 1 * encoderRight.getRotation();
        
        angle_left = normalize(current_angle_left - initial_angle_left);
        angle_right = normalize(current_angle_right - initial_angle_right);

        left_wheel_angle = angle_left;
        right_wheel_angle = angle_right;

        left_wheel_current_angle_msg.data = angle_left;
        right_wheel_current_angle_msg.data = angle_right;

        left_wheel_current_angle_pub.publish(left_wheel_current_angle_msg);
        right_wheel_current_angle_pub.publish(right_wheel_current_angle_msg);

        // Travel delta
        double delta_left_wheel = left_wheel_angle - left_wheel_pos - left_wheel_pos_offset;
        double delta_right_wheel = right_wheel_angle - right_wheel_pos - right_wheel_pos_offset;
        
        if (std::abs(delta_left_wheel) < 1) {
            left_wheel_pos += delta_left_wheel;
            left_wheel_vel = delta_left_wheel / elapsed.toSec();

            left_wheel_accel = (left_wheel_vel - left_wheel_vel_prev) / elapsed.toSec();

            if (std::abs(left_wheel_vel) > max_left_wheel_vel) max_left_wheel_vel = std::abs(left_wheel_vel);
            if (std::abs(left_wheel_accel) > max_left_wheel_accel) max_left_wheel_accel = std::abs(left_wheel_accel);
        } else {
            left_wheel_pos_offset += delta_left_wheel;
        }

        if (std::abs(delta_right_wheel) < 1) {
            right_wheel_pos += delta_right_wheel;
            right_wheel_vel = delta_right_wheel / elapsed.toSec();

            right_wheel_accel = (right_wheel_vel - right_wheel_vel_prev) / elapsed.toSec();

            if (std::abs(right_wheel_vel) > max_right_wheel_vel) max_right_wheel_vel = std::abs(right_wheel_vel);
            if (std::abs(right_wheel_accel) > max_right_wheel_accel) max_right_wheel_accel = std::abs(right_wheel_accel);
        } else {
            right_wheel_pos_offset += delta_right_wheel;
        }   

        // Publish vel and accel
        left_wheel_current_vel_msg.data = left_wheel_vel;
        right_wheel_current_vel_msg.data = right_wheel_vel;

        left_wheel_current_vel_pub.publish(left_wheel_current_vel_msg);
        right_wheel_current_vel_pub.publish(right_wheel_current_vel_msg);

        left_wheel_current_accel_msg.data = left_wheel_accel;
        right_wheel_current_accel_msg.data = right_wheel_accel;

        left_wheel_current_accel_pub.publish(left_wheel_current_accel_msg);
        right_wheel_current_accel_pub.publish(right_wheel_current_accel_msg);

        left_wheel_max_vel_msg.data = max_left_wheel_vel;
        right_wheel_max_vel_msg.data = max_right_wheel_vel;

        left_wheel_max_vel_pub.publish(left_wheel_max_vel_msg);
        right_wheel_max_vel_pub.publish(right_wheel_max_vel_msg);

        left_wheel_max_accel_msg.data = max_left_wheel_accel;
        right_wheel_max_accel_msg.data = max_right_wheel_accel;

        left_wheel_max_accel_pub.publish(left_wheel_max_accel_msg);
        right_wheel_max_accel_pub.publish(right_wheel_max_accel_msg);

        //
        left_wheel_vel_prev = left_wheel_vel;
        right_wheel_vel_prev = right_wheel_vel;
        
        // End loop
        last_time = this_time;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
