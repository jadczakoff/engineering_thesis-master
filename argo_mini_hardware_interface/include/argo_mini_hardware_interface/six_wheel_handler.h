#ifndef ARGO_MINI_HARDWARE_INTERFACE_SIX_WHEEL_HANDLER_H
#define ARGO_MINI_HARDWARE_INTERFACE_SIX_WHEEL_HANDLER_H

#include "interface_handler.h"
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace argo_mini_hardware_interface{

    class SixWheelHandler : public InterfaceHandler{
    public:
        bool registerHandles(ros::NodeHandle &privateNh, hardware_interface::RobotHW* hw, std::string logNamespace) override;

        std::vector<uint8_t> getData() const override;

        ReadStatus readData(std::vector<uint8_t> &data) override;

    private:

        hardware_interface::JointStateInterface jointStateInterface;
        hardware_interface::PositionJointInterface positionJointInterface;
        hardware_interface::VelocityJointInterface velocityJointInterface;

        struct HardwareInfo {
            double frontLeftSteer;
            double frontRightSteer;
            double midLeftSteer;
            double midRightSteer;
            double rearLeftSteer;
            double rearRightSteer;

            double frontLeftWheel;
            double frontRightWheel;
            double midLeftWheel;
            double midRightWheel;
            double rearLeftWheel;
            double rearRightWheel;
        } commands, velocities, positions, efforts;

        double wheelCommandCoefficient_;
        double steerCommandCoefficient_;

        std::string name_;
    };
}
#endif //ARGO_MINI_HARDWARE_INTERFACE_SIX_WHEEL_HANDLER_H
