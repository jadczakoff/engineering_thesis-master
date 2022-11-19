#include <six_wheel_steering_controller/wheel_unit.h>
#include <stdexcept>
#include <utility>

namespace six_wheel_steering_controller {

    double WheelUnit::getWheelRadius() const {
        return _wheelRadius;
    }

    void WheelUnit::setWheelRadius(double wheelRadius) {
        if (wheelRadius < 0)
            throw std::runtime_error("Wheel radius have to be greater or equal to 0");
        _wheelRadius = wheelRadius;
    }

    double WheelUnit::getAngleMin() const {
        return _angleMin;
    }

    void WheelUnit::setAngleMin(double angleMin) {
        _angleMin = angleMin;
    }

    double WheelUnit::getAngleMax() const {
        return _angleMax;
    }

    void WheelUnit::setAngleMax(double angleMax) {
        _angleMax = angleMax;
    }

    double WheelUnit::getRotationalSpeedMax() const {
        return _rotationalSpeedMax;
    }

    void WheelUnit::setRotationalSpeedMax(double rotationalSpeedMax) {
        if (rotationalSpeedMax < 0)
            throw std::runtime_error("Maximal rotational speed have to be greater or equal to 0");
        _rotationalSpeedMax = rotationalSpeedMax;
    }

    WheelUnit::WheelUnit(hardware_interface::JointHandle steeringHandle, hardware_interface::JointHandle wheelHandle)
            :
            _steeringHandle(std::move(steeringHandle)), _wheelHandle(std::move(wheelHandle)) {

    }

    ///
    /// \return info: \b angle [radians]; \b speed [m/s]
    WheelUnitInfo WheelUnit::getInfo() const {
        return {_steeringHandle.getPosition(), _wheelHandle.getVelocity() * _wheelRadius};
    }

    void WheelUnit::setCommand(WheelUnitInfo command) {
        if (_wheelRadius == 0)
            throw std::domain_error("Wheel radius cannot be 0");

        optimizeCommand(command);
        optimizeAngleExceeded(command);
        clampAngle(command);

        _steeringHandle.setCommand(command.angle);

        double rotSpeed = command.speed / _wheelRadius;

        if (std::fabs(rotSpeed) > _rotationalSpeedMax && _rotationalSpeedMax != 0) {
            double sign = std::copysign(1., rotSpeed);
            rotSpeed = _rotationalSpeedMax * sign;
        }

        _wheelHandle.setCommand(rotSpeed);
    }

    void WheelUnit::optimizeAngleExceeded(WheelUnitInfo &command) const {
        if (_angleMax != _angleMin){
            if(command.angle>_angleMax)
            {
                command.angle -= M_PI;
                command.speed  = -command.speed;
            }
            else if(command.angle<_angleMin){
                command.angle += M_PI;
                command.speed = -command.speed;
            }
        }
    }

    void WheelUnit::optimizeCommand(WheelUnitInfo &command) {
        command.angle = std::fmod(command.angle, 2 * M_PI);
        if (command.angle >= M_PI)
            command.angle -= 2 * M_PI;
        else if (command.angle <= -M_PI)
            command.angle += 2 * M_PI;

        if (command.angle >= M_PI_2){
            command.angle -= M_PI;
            command.speed *=-1;
        }
        else if (command.angle <= -M_PI_2){
            command.angle += M_PI;
            command.speed *= -1;
        }
    }

    void WheelUnit::clampAngle(WheelUnitInfo &command) const {
        if (_angleMax != _angleMin) {
            if (command.angle > _angleMax)
                command.angle = _angleMax;
            else if (command.angle < _angleMin)
                command.angle = _angleMin;
        }
    }
}

