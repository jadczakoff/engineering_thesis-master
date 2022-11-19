#ifndef SIX_WHEEL_STEERING_CONTROLLER_MODE1_H
#define SIX_WHEEL_STEERING_CONTROLLER_MODE1_H

#include "../mode.h"

namespace six_wheel_steering_controller {

    namespace modes{
        class Mode1 : public Mode{

        public:
            EModes modeCode() const override;

        protected:
            void doSetCommand(DriveInfo command) override;

        };

    }

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_MODE1_H
