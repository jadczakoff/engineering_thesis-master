#ifndef SIX_WHEEL_STEERING_CONTROLLER_SPEED_LIMITER_H
#define SIX_WHEEL_STEERING_CONTROLLER_SPEED_LIMITER_H


namespace six_wheel_steering_controller {

    class SpeedLimiter {
    public:
        bool hasVelocityLimits = false;
        bool hasAccelerationLimits = false;
        bool hasJerkLimits = false;

        // Velocity limits:
        double minVelocity = 0.;
        double maxVelocity = 0.;

        // Acceleration limits:
        double minAcceleration = 0.;
        double maxAcceleration = 0.;

        // Jerk limits:
        double minJerk = 0.;
        double maxJerk = 0.;

        double limit(double v, double dt);
        void reset();

    private:
        double v1 = 0.;
        double v2 = 0.;

        void limitVelocity(double &v) const;
        void limitAcceleration(double &v, double dt) const;
        void limitJerk(double &v, double dt) const;

    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_SPEED_LIMITER_H
