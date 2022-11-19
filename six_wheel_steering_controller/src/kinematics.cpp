#include "six_wheel_steering_controller/kinematics.h"
#include <cmath>
#include <utility>
#include <Eigen/Geometry>

namespace six_wheel_steering_controller {


    void Kinematics::setVelocity(Twist velocity) {
        assert(_drive);
        // planar
        auto planarComponent = Eigen::Vector2d(velocity.linX, velocity.linY);

        auto rotationalComponent = Eigen::Vector3d{0, 0, velocity.angZ};

        using VkType = Eigen::Matrix<double, 6, 2>;
        VkType Vk;

        for (int i = 0; i < Vk.rows(); i++) {
            Vk.row(i) = rotationalComponent.cross((R.row(i)).transpose()).head(2) + planarComponent;
        }


        // turning
        Eigen::Vector2d frontL = Vk.row(0).transpose();
        Eigen::Vector2d frontR = Vk.row(1).transpose();
        Eigen::Vector2d midL = Vk.row(2).transpose();
        Eigen::Vector2d midR = Vk.row(3).transpose();
        Eigen::Vector2d rearL = Vk.row(4).transpose();
        Eigen::Vector2d rearR = Vk.row(5).transpose();


        auto getWheelUnitInfo = [](const Eigen::Vector2d &v) {
            return WheelUnitInfo{std::atan2(v(1), v(0)), v.norm()};
        };

        _drive->setCommand({
                                   getWheelUnitInfo(frontR),
                                   getWheelUnitInfo(frontL),
                                   getWheelUnitInfo(midR),
                                   getWheelUnitInfo(midL),
                                   getWheelUnitInfo(rearR),
                                   getWheelUnitInfo(rearL)
                           });

    }

    OdometryInfo Kinematics::getOdometry(double dt) const {
        assert(_drive);
        const auto P = getP(_drive);

        const Eigen::Vector3d X = CInv * P;

        Eigen::Rotation2Dd r(_lastPose.yaw);
        _lastPose.position += r * X.head(2) * dt;
        _lastPose.yaw += X(2) * dt;

        return {
                .x = _lastPose.position(0),
                .y = _lastPose.position(1),
                .yaw = _lastPose.yaw,
                .vX = X(0),
                .vY = X(1),
                .vYaw = X(2)
        };
    }

    Kinematics::Matrix63 Kinematics::getR(const std::shared_ptr<const Drive> &drive) {
        const double yspacing2 = drive->getY_AxisSpacing() / 2;
        const double mid2Front = drive->getDistanceToFront();
        const double mid2Rear = drive->getDistanceToRear();

        Matrix63 ret;
        ret << mid2Front, yspacing2, 0,
                mid2Front, -yspacing2, 0,
                0, yspacing2, 0,
                0, -yspacing2, 0,
                -mid2Rear, yspacing2, 0,
                -mid2Rear, -yspacing2, 0;

        return ret;
    }

    Kinematics::Matrix3_12 Kinematics::getCInv(const std::shared_ptr<const Drive> &drive) {
        Matrix12_3 C;

        const double yspacing2 = drive->getY_AxisSpacing() / 2;
        const double mid2Front = drive->getDistanceToFront();
        const double mid2Rear = drive->getDistanceToRear();

        C << 1, 0, -yspacing2,
                0, 1, mid2Front,

                1, 0, yspacing2,
                0, 1, mid2Front,

                1, 0, -yspacing2,
                0, 1, 0,

                1, 0, yspacing2,
                0, 1, 0,

                1, 0, -yspacing2,
                0, 1, -mid2Rear,

                1, 0, yspacing2,
                0, 1, -mid2Rear;

        return (C.transpose() * C).inverse() * C.transpose();
    }

    Kinematics::Vector12d Kinematics::getP(const std::shared_ptr<const Drive> &drive) {
        auto vectorFromWheelUnit = [](const WheelUnit &wheelUnit) {
            const auto &info = wheelUnit.getInfo();
            return Eigen::Vector2d{std::cos(info.angle) * info.speed, std::sin(info.angle) * info.speed};
        };

        Vector12d ret;

        ret.block<2, 1>(0, 0) = vectorFromWheelUnit(*drive->getFrontLeft());
        ret.block<2, 1>(2, 0) = vectorFromWheelUnit(*drive->getFrontRight());
        ret.block<2, 1>(4, 0) = vectorFromWheelUnit(*drive->getMidLeft());
        ret.block<2, 1>(6, 0) = vectorFromWheelUnit(*drive->getMidRight());
        ret.block<2, 1>(8, 0) = vectorFromWheelUnit(*drive->getRearLeft());
        ret.block<2, 1>(10, 0) = vectorFromWheelUnit(*drive->getRearRight());

        return ret;
    }

    void Kinematics::init(std::shared_ptr<Drive> drive) {
        _drive = std::move(drive);
        R = getR(_drive);
        CInv = getCInv(_drive);
    }

}
