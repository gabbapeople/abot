

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>


class AbotBumperMux {
public:
    AbotBumperMux();

private:
    ros::NodeHandle _node;

    ros::Subscriber _move_base_sub;
    ros::Subscriber _ir_f_sub;
    ros::Subscriber _ir_l_sub;
    ros::Subscriber _ir_r_sub;

    ros::Publisher _cmd_vel_pub;


    bool _left_bump;
    bool _right_bump;
    bool _front_bump;

    double _bumper_threshold_f;
    double _bumper_threshold_l;
    double _bumper_threshold_r;

    bool _recovery;

    void frontBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg);
    void leftBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg);
    void rightBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg);

    void moveBaseCallback(const geometry_msgs::TwistConstPtr& twistMsg);
};


AbotBumperMux::AbotBumperMux() {

    _move_base_sub = _node.subscribe<geometry_msgs::Twist>("/move_base/cmd_vel", 10, &AbotBumperMux::moveBaseCallback, this);
    _cmd_vel_pub = _node.advertise<geometry_msgs::Twist>("/mobile_abot/cmd_vel", 1);

    _ir_f_sub = _node.subscribe<sensor_msgs::Range>("/ir_sensor_front_middle", 10, &AbotBumperMux::frontBumperCallback, this);
    _ir_l_sub = _node.subscribe<sensor_msgs::Range>("/ir_sensor_front_left", 10, &AbotBumperMux::leftBumperCallback, this);
    _ir_r_sub = _node.subscribe<sensor_msgs::Range>("/ir_sensor_front_right", 10, &AbotBumperMux::rightBumperCallback, this);

    _bumper_threshold_f = 0.06;
    _bumper_threshold_r = 0.06;
    _bumper_threshold_l = 0.06;

    ROS_INFO("Abot Bumper Mutex node: Start");
}

void AbotBumperMux::frontBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg) {
    if (rangeMsg->range <= _bumper_threshold_f) {
        _recovery = true;
        ROS_WARN("Abot Bumper Mutex node: Front BUMP!");
        geometry_msgs::Twist twist;
        twist.linear.x = -0.1;
        twist.angular.z = 0.0;
        _cmd_vel_pub.publish(twist);

    } else {
        _recovery = false;
    }
}

void AbotBumperMux::leftBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg) {
    if (rangeMsg->range <= _bumper_threshold_l) {
        _recovery = true;
        ROS_WARN("Abot Bumper Mutex node: Left BUMP!");
        geometry_msgs::Twist twist;
        twist.linear.x = -0.05;
        twist.angular.z = -1.0;
        _cmd_vel_pub.publish(twist);

    } else {
        _recovery = false;
    }
}

void AbotBumperMux::rightBumperCallback(const sensor_msgs::RangeConstPtr& rangeMsg) {
    if (rangeMsg->range <= _bumper_threshold_r) {
        _recovery = true;
        ROS_WARN("Abot Bumper Mutex node: Right BUMP!");
        geometry_msgs::Twist twist;
        twist.linear.x = -0.05;
        twist.angular.z = 1.0;
        _cmd_vel_pub.publish(twist);

    } else {
        _recovery = false;
    }
}

void AbotBumperMux::moveBaseCallback(const geometry_msgs::TwistConstPtr& twistMsg) {
    geometry_msgs::Twist twist;
    
    if (_recovery == false) {
        twist.linear.x = twistMsg->linear.x;
        twist.angular.x = twistMsg->angular.x;
        _cmd_vel_pub.publish(twistMsg);
    } else {
        ROS_WARN("recovery!");
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "bumper_mutex_node");

	AbotBumperMux abm;

	ros::spin();

}