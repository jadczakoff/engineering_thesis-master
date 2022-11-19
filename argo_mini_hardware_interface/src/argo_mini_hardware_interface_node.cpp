#include "ros/ros.h"
#include <argo_mini_hardware_interface/argo_mini_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>


namespace AMHI = argo_mini_hardware_interface;

class TimerCallback {
public:
    TimerCallback(AMHI::HardwareInterface *robot, controller_manager::ControllerManager *cm) :
            robot_{robot}, controllerManager_{cm} {}

    void operator()(const ros::TimerEvent &e) {
        assert(robot_ && controllerManager_);
        const auto &period = e.current_real - e.last_real;
        robot_->read(e.current_real, period);
        controllerManager_->update(e.current_real, period);
        robot_->write(e.current_real, period);
    }

private:
    AMHI::HardwareInterface *robot_;
    controller_manager::ControllerManager *controllerManager_;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "argo_mini_hardware_interface_node");

    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    AMHI::HardwareInterface robot;

    if (!robot.init(nh, privateNh)) return EXIT_FAILURE;
    controller_manager::ControllerManager cm(&robot, nh);

    auto timer = privateNh.createTimer(ros::Duration(ros::Rate(50)), TimerCallback(&robot, &cm));
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
}