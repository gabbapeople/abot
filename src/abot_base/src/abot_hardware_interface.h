
#ifndef ABOT_HARDWARE_INTERFACE_HPP_
#define ABOT_HARDWARE_INTERFACE_HPP_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/console.h>

class AbotHardwareInterface : public hardware_interface::RobotHW {
public:
    AbotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node);

    void updateJointsFromHardware(const ros::Duration& period);
    void writeCommandsToHardware();

private:
    ros::NodeHandle _node;
    ros::NodeHandle _private_node;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::VelocityJointInterface _velocity_joint_interface;

    ros::Subscriber _left_wheel_angle_sub;
    ros::Subscriber _right_wheel_angle_sub;

    ros::Publisher _left_wheel_vel_pub;
    ros::Publisher _right_wheel_vel_pub;

    ros::Publisher _left_wheel_current_vel_pub;
    ros::Publisher _right_wheel_current_vel_pub;

    double _max_wheel_angular_speed;

    struct Joint {
        double position;
        double position_offset;
        double velocity;
        double effort;
        double velocity_command;

        Joint()
            : position(0)
            , velocity(0)
            , effort(0)
            , velocity_command(0) {}
    } _joints[2];

    double _left_wheel_angle;
    double _right_wheel_angle;

    void registerControlInterfaces();

    void leftWheelAngleCallback(const std_msgs::Float32& msg);
    void rightWheelAngleCallback(const std_msgs::Float32& msg);

    void limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side);
};

AbotHardwareInterface::AbotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node)
    : _node(node)
    , _private_node(private_node) {

    registerControlInterfaces();

    _private_node.param<double>("max_wheel_angular_speed", _max_wheel_angular_speed, 0.0);

    _left_wheel_vel_pub = _node.advertise<std_msgs::Float32>("abot/left_wheel_vel", 1);
    _right_wheel_vel_pub = _node.advertise<std_msgs::Float32>("abot/right_wheel_vel", 1);

    _left_wheel_angle_sub = _node.subscribe("abot/left_wheel_angle", 1, &AbotHardwareInterface::leftWheelAngleCallback, this);
    _right_wheel_angle_sub = _node.subscribe("abot/right_wheel_angle", 1, &AbotHardwareInterface::rightWheelAngleCallback, this);

    _left_wheel_current_vel_pub = _node.advertise<std_msgs::Float32>("abot/current_left_wheel_vel", 1);
    _right_wheel_current_vel_pub = _node.advertise<std_msgs::Float32>("abot/current_right_wheel_vel", 1);

}

void AbotHardwareInterface::writeCommandsToHardware() {
    double diff_angle_speed_left = _joints[0].velocity_command;
    double diff_angle_speed_right = _joints[1].velocity_command;

    limitDifferentialSpeed(diff_angle_speed_left, diff_angle_speed_right);

    std_msgs::Float32 left_wheel_vel_msg;
    std_msgs::Float32 right_wheel_vel_msg;

    left_wheel_vel_msg.data = diff_angle_speed_left;
    right_wheel_vel_msg.data = diff_angle_speed_right;

    _left_wheel_vel_pub.publish(left_wheel_vel_msg);
    _right_wheel_vel_pub.publish(right_wheel_vel_msg);
}

void AbotHardwareInterface::updateJointsFromHardware(const ros::Duration& period) {

    double delta_left_wheel = _left_wheel_angle - _joints[0].position - _joints[0].position_offset;
    double delta_right_wheel = _right_wheel_angle - _joints[1].position - _joints[1].position_offset;

    std_msgs::Float32 left_wheel_current_vel_msg;
    std_msgs::Float32 right_wheel_current_vel_msg;

    if (std::abs(delta_left_wheel) < 1) {
        _joints[0].position += delta_left_wheel;
        _joints[0].velocity = delta_left_wheel / period.toSec();

        left_wheel_current_vel_msg.data = _joints[0].velocity;
        _left_wheel_current_vel_pub.publish(left_wheel_current_vel_msg);

    } else {
        _joints[0].position_offset += delta_left_wheel;
    }

    if (std::abs(delta_right_wheel) < 1) {
        _joints[1].position += delta_right_wheel;
        _joints[1].velocity = delta_right_wheel / period.toSec();

        right_wheel_current_vel_msg.data = _joints[1].velocity;
        _right_wheel_current_vel_pub.publish(right_wheel_current_vel_msg);

    } else {
        _joints[1].position_offset += delta_right_wheel;
    }
}

void AbotHardwareInterface::registerControlInterfaces() {
    ros::V_string joint_names = boost::assign::list_of("left_wheel_to_base")("right_wheel_to_base");

    for (unsigned int i = 0; i < joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
        _joint_state_interface.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
        _velocity_joint_interface.registerHandle(joint_handle);
    }
    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
}

void AbotHardwareInterface::leftWheelAngleCallback(const std_msgs::Float32& msg) {
    _left_wheel_angle = msg.data;
}

void AbotHardwareInterface::rightWheelAngleCallback(const std_msgs::Float32& msg) {
    _right_wheel_angle = msg.data;
}

void AbotHardwareInterface::limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side) {
    double large_speed = std::max(std::abs(diff_speed_left_side), std::abs(diff_speed_right_side));
    if (large_speed >  _max_wheel_angular_speed) {
        diff_speed_left_side *=  _max_wheel_angular_speed / large_speed;
        diff_speed_right_side *=  _max_wheel_angular_speed / large_speed;
    }
}

#endif // ABOT_HARDWARE_INTERFACE_HPP_