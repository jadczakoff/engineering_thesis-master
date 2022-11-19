#ifndef ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H
#define ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <serial/serial.h>
#include "interface_handler.h"

#include <utility>

namespace argo_mini_hardware_interface {

    class HardwareInterface : public hardware_interface::RobotHW {

    public:
        ~HardwareInterface() override = default;

        bool init(ros::NodeHandle &rootNh, ros::NodeHandle &privateNh) override;

        void read(const ros::Time &time, const ros::Duration &duration) override;

        void write(const ros::Time &time, const ros::Duration &duration) override;

    private:

        bool initSerial(ros::NodeHandle &privateNh);

        bool initDrive(ros::NodeHandle &privateNh);

        std::string name_;

        std::unique_ptr<serial::Serial> serial_;

        std::unique_ptr<InterfaceHandler> ifaceHandler_;

        std::vector<uint8_t> readBuffer_;

        class SerialTimer {
        public:
            SerialTimer(double frequency, const ros::Time &initTime) :
                    period_(1/frequency), lastWrite_(initTime) {}

            SerialTimer() = default;

            bool timeForWrite(const ros::Time &now) {
                if ((now - lastWrite_) >= period_) {
                    lastWrite_ = now;
                    return true;
                } else return false;
            }

        private:
            ros::Duration period_;
            ros::Time lastWrite_;
        } serialTimer_;
    };

}


#endif //ARGO_MINI_HARDWARE_INTERFACE_ARGO_MINI_HARDWARE_INTERFACE_H
