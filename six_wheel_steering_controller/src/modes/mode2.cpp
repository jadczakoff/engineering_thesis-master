#include "six_wheel_steering_controller/modes/mode2.h"
#include <stdexcept>
#include <cmath>

namespace six_wheel_steering_controller {
    namespace modes {
        void Mode2::doSetCommand(DriveInfo command) {
            checkAndRun(_drive->getRearRight(),command.rearRight);
            checkAndRun(_drive->getRearLeft(),command.rearLeft);
            checkAndRun(_drive->getMidRight(),command.midRight);
            checkAndRun(_drive->getMidLeft(),command.midLeft);
            checkAndRun(_drive->getFrontRight(),command.frontRight);
            checkAndRun(_drive->getFrontLeft(),command.frontLeft);
        }

        double Mode2::getMaxAngleError() const {
            return _maxAngleError;
        }

        void Mode2::setMaxAngleError(double maxAngleError) {
            if (maxAngleError < 0)
                throw std::runtime_error("Max angle error have to be greater or equal 0");
            _maxAngleError = maxAngleError;
        }

        void Mode2::checkAndRun(const std::unique_ptr<WheelUnit> &wheelUnit, WheelUnitInfo command) const {
            const auto currentState = wheelUnit->getInfo();
            if(std::fabs(currentState.angle - command.angle)> _maxAngleError){
                command.speed = 0;
            }
            wheelUnit->setCommand(command);
        }

        EModes Mode2::modeCode() const {
            return EModes::Mode1;
        }

        Mode2::Mode2(double angleError): _maxAngleError(angleError) {

        }

    }
}
