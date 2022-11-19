#ifndef SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H
#define SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H

#include "common.h"
#include "drive.h"
#include <Eigen/Dense>

namespace six_wheel_steering_controller {

    class Kinematics {

    public:
        void init(std::shared_ptr<Drive> drive);

        void setVelocity(Twist velocity);

        OdometryInfo getOdometry(double dt) const;

    private:
        using Matrix63 = Eigen::Matrix<double, 6, 3>;
        using Matrix12_3 = Eigen::Matrix<double, 12, 3>;
        using Matrix3_12 = Eigen::Matrix<double, 3, 12>;
        using Vector12d = Eigen::Matrix<double, 12, 1>;

        std::shared_ptr<Drive> _drive;
        mutable struct {
            Eigen::Vector2d position = Eigen::Vector2d::Zero();
            double yaw = 0;
        }
                _lastPose{};

        Matrix63 R;
        Matrix3_12 CInv;

        static Matrix63 getR(const std::shared_ptr<const Drive> &drive);

        static Matrix3_12 getCInv(const std::shared_ptr<const Drive> &drive);

        static Vector12d getP(const std::shared_ptr<const Drive> &drive);
    };

}


#endif //SIX_WHEEL_STEERING_CONTROLLER_KINEMATICS_H
