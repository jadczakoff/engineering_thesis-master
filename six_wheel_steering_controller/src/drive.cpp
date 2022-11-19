#include <six_wheel_steering_controller/drive.h>
#include <six_wheel_steering_controller/mode.h>


namespace six_wheel_steering_controller {

    void Drive::setMode(std::unique_ptr<Mode> mode) {
        _mode = std::move(mode);
        _mode->setDrive(this);
    }

    double Drive::getY_AxisSpacing() const {
        return _yAxisSpacing;
    }

    void Drive::setY_AxisSpacing(double spacing) {
        if (spacing < 0)
            throw std::runtime_error("Spacing have to be greater or equal to 0");
        _yAxisSpacing = spacing;
    }

    double Drive::getDistanceToFront() const {
        return _distanceToFront;
    }

    void Drive::setDistanceToFront(double distanceToFront) {
        if (distanceToFront < 0)
            throw std::runtime_error("Distance to front have to be greater or equal to 0");
        _distanceToFront = distanceToFront;
    }

    double Drive::getDistanceToRear() const {
        return _distanceToRear;
    }

    void Drive::setDistanceToRear(double distanceToRear) {
        if (distanceToRear < 0)
            throw std::logic_error("Distance to rear have to be greater or equal to 0");
        _distanceToRear = distanceToRear;
    }

    void Drive::setCommand(DriveInfo command) {
        if (!_mode)
            throw std::logic_error("Mode is not specified");

        if (!_frontRight)
            throw std::logic_error("Front right wheel unit is not specified");
        if (!_frontLeft)
            throw std::logic_error("Front left wheel unit is not specified");
        if (!_midRight)
            throw std::logic_error("Mid right wheel unit is not specified");
        if (!_midLeft)
            throw std::logic_error("Mid left wheel unit is not specified");
        if (!_rearRight)
            throw std::logic_error("Rear right wheel unit is not specified");
        if (!_rearLeft)
            throw std::logic_error("Rear left wheel unit is not specified");

        _mode->setCommand(command);
    }

    const std::unique_ptr<WheelUnit> &Drive::getFrontRight() const {
        return _frontRight;
    }

    void Drive::setFrontRight(std::unique_ptr<WheelUnit> frontRight) {
        _frontRight = std::move(frontRight);
    }

    const std::unique_ptr<WheelUnit> &Drive::getFrontLeft() const {
        return _frontLeft;
    }

    void Drive::setFrontLeft(std::unique_ptr<WheelUnit> frontLeft) {
        _frontLeft = std::move(frontLeft);
    }

    const std::unique_ptr<WheelUnit> &Drive::getMidRight() const {
        return _midRight;
    }

    void Drive::setMidRight(std::unique_ptr<WheelUnit> midRight) {
        _midRight = std::move(midRight);
    }

    const std::unique_ptr<WheelUnit> &Drive::getMidLeft() const {
        return _midLeft;
    }

    void Drive::setMidLeft(std::unique_ptr<WheelUnit> midLeft) {
        _midLeft = std::move(midLeft);
    }

    const std::unique_ptr<WheelUnit> &Drive::getRearRight() const {
        return _rearRight;
    }

    void Drive::setRearRight(std::unique_ptr<WheelUnit> rearRight) {
        _rearRight = std::move(rearRight);
    }

    const std::unique_ptr<WheelUnit> &Drive::getRearLeft() const {
        return _rearLeft;
    }

    void Drive::setRearLeft(std::unique_ptr<WheelUnit> rearLeft) {
        _rearLeft = std::move(rearLeft);
    }

    void Drive::brake() {
        setCommand({
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0},
            {0,0}
        });
    }

    const Mode * Drive::getCurrentMode() const {
        return _mode.get();
    }
}

