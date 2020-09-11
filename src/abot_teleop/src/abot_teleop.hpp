#ifndef ABOT_TELEOP_HPP_
#define ABOT_TELEOP_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define PS4_AXIS_STICK_LEFT_LEFTWARDS 0
#define PS4_AXIS_STICK_LEFT_UPWARDS 1
#define PS4_AXIS_STICK_RIGHT_LEFTWARDS 2
#define PS4_AXIS_STICK_RIGHT_UPWARDS 3

class AbotTeleop {
public:
    AbotTeleop(ros::NodeHandle private_node);

private:
    ros::NodeHandle _node;
    ros::NodeHandle _private_node;

    ros::Subscriber _joy_sub;
    ros::Publisher _cmd_vel_pub;

    double _linear_speed_scale;
    double _angular_speed_scale;
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

AbotTeleop::AbotTeleop(ros::NodeHandle private_node) :
    _private_node(private_node)
{

	_private_node.param<double>("linear_speed_scale", _linear_speed_scale, 0.0);
    _private_node.param<double>("angular_speed_scale", _angular_speed_scale, 0.0);

    _cmd_vel_pub = _node.advertise<geometry_msgs::Twist>("abot/mobile_abot/cmd_vel", 1);
    _joy_sub = _node.subscribe<sensor_msgs::Joy>("joy", 10, &AbotTeleop::joyCallback, this);

    ROS_INFO("Abot teleop node: Start");
}

void AbotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    geometry_msgs::Twist twist;

    twist.angular.z = 1 * _angular_speed_scale * joy->axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
    twist.linear.x = _linear_speed_scale * joy->axes[PS4_AXIS_STICK_LEFT_UPWARDS];

    _cmd_vel_pub.publish(twist);

}

#endif // ABOT_TELEOP_HPP_