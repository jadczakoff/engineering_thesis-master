#include <argo_mini_hardware_interface/argo_mini_hardware_interface.h>
#include "ostream"
#include <argo_mini_hardware_interface/six_wheel_handler.h>

namespace argo_mini_hardware_interface {

    bool HardwareInterface::init(ros::NodeHandle &rootNh, ros::NodeHandle &privateNh) {

        std::string completeNs = privateNh.getNamespace();
        auto id = completeNs.find_last_of('/');
        name_ = completeNs.substr(id + 1);

        if (!initDrive(privateNh)) return false;
        if (!initSerial(privateNh)) return false;

        if (!ifaceHandler_->registerHandles(privateNh, this, name_)) return false;

        return true;
    }

    void HardwareInterface::read(const ros::Time &time1, const ros::Duration &duration) {
        serial_->read(readBuffer_,14);

        auto status = ifaceHandler_->readData(readBuffer_);
        std::stringstream ss;
        std::copy(readBuffer_.begin(), readBuffer_.end(), std::ostream_iterator<int>(ss << std::hex, " "));

        switch(status){
            case ReadStatus::Success:
                ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "serial_comm", "Received frame: " << ss.str());
            case ReadStatus::Error:
                readBuffer_.clear();
                break;
            default:
                break;
        }
    }

    void HardwareInterface::write(const ros::Time &time, const ros::Duration &duration) {
        if (serialTimer_.timeForWrite(time)) {
            const auto &data = ifaceHandler_->getData();
            std::stringstream ss;
            std::copy(data.begin(), data.end(), std::ostream_iterator<int>(ss << std::hex, " "));
            ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "serial_comm", "Sending frame: " << ss.str());
            serial_->write(data);
        }
    }

    bool HardwareInterface::initSerial(ros::NodeHandle &privateNh) {
        std::string serialPath = "argo_mini/serial/";
        double freq;
        int baudrate;
        std::string serialDev;

        if (!privateNh.param(serialPath + "write_frequency", freq, 50.))
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/write_frequency param. Defaulting to 50Hz.");
        if (freq <= 0.) {
            ROS_ERROR_STREAM_NAMED(name_, "serial/write_frequency have to be >0. Aborting ...");
            return false;
        }

        if (!privateNh.param(serialPath + "baudrate", baudrate, 115200))
            ROS_INFO_STREAM_NAMED(name_, "Couldn't get serial/baudrate param. Defaulting to 115200.");

        if (!privateNh.getParam(serialPath + "device", serialDev)) {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't get serial/device param. Aborting...");
            return false;
        }

        serialTimer_ = SerialTimer(freq, ros::Time::now());

        try {
            serial_ = std::make_unique<serial::Serial>(
                    serialDev, (uint32_t) baudrate, serial::Timeout::simpleTimeout(50));
        } catch (const serial::PortNotOpenedException &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial port opening exception: " << e.what());
            return false;
        } catch (const std::invalid_argument &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial invalid argument: " << e.what());
            return false;
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM_NAMED(name_, "Serial exception: " << e.what());
            return false;
        }

        // if successful
        ROS_INFO_STREAM_NAMED(name_, "Serial parameters:\n"
                << "- device: " << serial_->getPort() << "\n"
                << "- baudrate: " << serial_->getBaudrate() << "\n"
                << "- write frequency: " << freq << " Hz"
        );
        return true;
    }

    bool HardwareInterface::initDrive(ros::NodeHandle &privateNh) {
        std::string driveType;
        if (!privateNh.getParam("argo_mini/drive", driveType)) {
            ROS_ERROR_STREAM_NAMED(name_, "Drive type not specified. Aborting ...");
            return false;
        }

        if (driveType == "6w")
            ifaceHandler_ = std::make_unique<SixWheelHandler>();
        else {
            ROS_ERROR_STREAM_NAMED(name_, "Not supported drive type (" << driveType << ')');
            return false;
        }

        return true;
    }

}

