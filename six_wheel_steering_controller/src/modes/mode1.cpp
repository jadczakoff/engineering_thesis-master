#include "six_wheel_steering_controller/modes/mode1.h"
#include <six_wheel_steering_controller/drive.h>

void six_wheel_steering_controller::modes::Mode1::doSetCommand(six_wheel_steering_controller::DriveInfo command) {

    _drive->getFrontLeft()->setCommand(command.frontLeft);
    _drive->getFrontRight()->setCommand(command.frontRight);
    _drive->getMidLeft()->setCommand(command.midLeft);
    _drive->getMidRight()->setCommand(command.midRight);
    _drive->getRearLeft()->setCommand(command.rearLeft);
    _drive->getRearRight()->setCommand(command.rearRight);
}

six_wheel_steering_controller::EModes six_wheel_steering_controller::modes::Mode1::modeCode() const {
    return EModes::Mode1;
}
