#ifndef SIX_WHEEL_STEERING_CONTROLLER_COMMON_H
#define SIX_WHEEL_STEERING_CONTROLLER_COMMON_H

#include <array>

namespace six_wheel_steering_controller {

    struct Twist {
        double linX=0;
        double linY=0;
        double angZ=0;
    };

    struct OdometryInfo {
        double x=0;
        double y=0;
        double yaw=0;

        double vX=0;
        double vY=0;
        double vYaw=0;

        std::array<double, 36> positionCovariance{0};
        std::array<double, 36> velocityCovariance{0};
    };

    struct WheelUnitInfo {
        double angle;
        double speed;
    };

    struct DriveInfo {
        WheelUnitInfo frontRight;
        WheelUnitInfo frontLeft;
        WheelUnitInfo midRight;
        WheelUnitInfo midLeft;
        WheelUnitInfo rearRight;
        WheelUnitInfo rearLeft;
    };
}

#endif // SIX_WHEEL_STEERING_CONTROLLER_COMMON_H