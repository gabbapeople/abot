
#include <chrono>
#include <functional>
#include <ros/callback_queue.h>

#include "abot_hardware_interface.h"

typedef boost::chrono::steady_clock time_source;

void controlLoop(AbotHardwareInterface& hardware,
    controller_manager::ControllerManager& cm,
    time_source::time_point& last_time) {

    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    hardware.updateJointsFromHardware(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hardware.writeCommandsToHardware();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "abot_base_node");

    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    double control_frequency;
    double max_speed;

    private_node.param<double>("control_frequency", control_frequency, 10.0);
    
    AbotHardwareInterface hardware(node, private_node);

    controller_manager::ControllerManager cm(&hardware, node);

    ros::CallbackQueue abot_queue;
    ros::AsyncSpinner abot_spinner(1, &abot_queue);

    time_source::time_point last_time = time_source::now();

    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency),
        boost::bind(controlLoop, std::ref(hardware), std::ref(cm), std::ref(last_time)), &abot_queue);

    ros::Timer control_loop = node.createTimer(control_timer);

    abot_spinner.start();

    ros::spin();

    return 0;
}