#ifndef SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H
#define SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H

#include "common.h"
#include <hardware_interface/joint_command_interface.h>

namespace six_wheel_steering_controller {

    class WheelUnit {
    public:

        WheelUnit(hardware_interface::JointHandle steeringHandle, hardware_interface::JointHandle wheelHandle);

        WheelUnitInfo getInfo() const;

        void setCommand(WheelUnitInfo command);


    private:
        hardware_interface::JointHandle _steeringHandle;
        hardware_interface::JointHandle _wheelHandle;
        double _wheelRadius = 0;
        double _angleMin = 0;
        double _angleMax = 0;
        double _rotationalSpeedMax = 0;

        void optimizeAngleExceeded(WheelUnitInfo &command) const;

        void clampAngle(WheelUnitInfo &command) const;
        static void optimizeCommand(WheelUnitInfo &command);

    public:

        double getWheelRadius() const;

        void setWheelRadius(double wheelRadius);

        double getAngleMin() const;

        void setAngleMin(double angleMin);

        double getAngleMax() const;

        void setAngleMax(double angleMax);

        double getRotationalSpeedMax() const;

        void setRotationalSpeedMax(double rotationalSpeedMax);
    };
}


#endif //SIX_WHEEL_STEERING_CONTROLLER_WHEEL_UNIT_H
