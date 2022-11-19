#include <six_wheel_steering_controller/mode.h>
#include <stdexcept>

namespace six_wheel_steering_controller {
    Drive *Mode::getDrive() const {
        return _drive;
    }

    void Mode::setDrive(Drive *drive) {
        _drive = drive;
    }

    void Mode::setCommand(DriveInfo command) {
        if(! _drive)
            throw std::runtime_error("Drive is not specified");
        doSetCommand(command);
    }

}
