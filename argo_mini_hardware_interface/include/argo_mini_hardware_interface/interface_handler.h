#ifndef ARGO_MINI_HARDWARE_INTERFACE_INTERFACE_HANDLER_H
#define ARGO_MINI_HARDWARE_INTERFACE_INTERFACE_HANDLER_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include "common.h"

namespace argo_mini_hardware_interface {

    class InterfaceHandler {
    public:
        virtual bool
        registerHandles(ros::NodeHandle &privateNh, hardware_interface::RobotHW *hw, std::string logNamespace) = 0;

        virtual std::vector<uint8_t> getData() const = 0;

        virtual ReadStatus readData(std::vector<uint8_t> &data) = 0;
    };

}

#endif //ARGO_MINI_HARDWARE_INTERFACE_INTERFACE_HANDLER_H
