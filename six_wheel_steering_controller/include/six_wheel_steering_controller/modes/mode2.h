#ifndef SIX_WHEEL_STEERING_CONTROLLER_MODE2_H
#define SIX_WHEEL_STEERING_CONTROLLER_MODE2_H

#include "../mode.h"
#include "../drive.h"

namespace six_wheel_steering_controller {

    namespace modes {
        class Mode2 : public Mode {


        protected:
            void doSetCommand(DriveInfo command) override;

        private:
            double _maxAngleError = 1e-3;
        public:
            double getMaxAngleError() const;

            Mode2() = default;

            Mode2(double angleError);

            EModes modeCode() const override;

            void setMaxAngleError(double maxAngleError);

        private:

            void checkAndRun(const std::unique_ptr<WheelUnit> &wheelUnit, WheelUnitInfo command) const;
        };

    }

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_MODE2_H
