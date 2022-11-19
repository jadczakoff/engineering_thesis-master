#include <six_wheel_steering_controller/speed_limiter.h>
#include <algorithm>

template<typename T>
T clamp(T value, T min, T max) {
    return std::max(std::min(value, max), min);
}

namespace six_wheel_steering_controller {


    double SpeedLimiter::limit(double v, double dt) {
        if (hasJerkLimits) {
            limitJerk(v, dt);
        }

        if (hasAccelerationLimits) {
            limitAcceleration(v, dt);
        }

        if (hasVelocityLimits) {
            limitVelocity(v);
        }

        v2 = v1;
        v1 = v;
        return v;
    }

    void SpeedLimiter::limitVelocity(double &v) const {
        clamp(v, minVelocity, maxVelocity);
    }

    void SpeedLimiter::limitAcceleration(double &v, double dt) const {
        const double dvMin = minAcceleration * dt;
        const double dvMax = maxAcceleration * dt;

        const double dv = clamp(v - v1, dvMin, dvMax);

        v = v1 + dv;
    }

    void SpeedLimiter::limitJerk(double &v, double dt) const {
        const double dv = v - v1;
        const double dv1 = v1 - v2;

        const double dt2 = 2. * dt * dt;

        const double daMin = minJerk * dt2;
        const double daMax = maxJerk * dt2;

        const double da = clamp(dv - dv1, daMin, daMax);

        v = v1 + dv1 + da;
    }

    void SpeedLimiter::reset() {
        v1 = 0;
        v2 = 0;
    }
}