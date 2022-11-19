#include <six_wheel_steering_controller/six_wheel_steering_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <iomanip>
#include <six_wheel_steering_controller/mode.h>
#include <six_wheel_steering_controller/modes/mode1.h>
#include <six_wheel_steering_controller/modes/mode2.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_datatypes.h>
#include <boost/range/algorithm.hpp>

#include <urdf_geometry_parser/urdf_geometry_parser.h>

namespace six_wheel_steering_controller {

    void SixWheelSteeringController::update(const ros::Time &time, const ros::Duration &period) {

        Command *currentCommand = _currentCommand.readFromRT();

        currentCommand->twist.linX = _xLimiter.limit(currentCommand->twist.linX, period.toSec());
        currentCommand->twist.linY = _yLimiter.limit(currentCommand->twist.linY, period.toSec());
        currentCommand->twist.angZ = _yawLimiter.limit(currentCommand->twist.angZ, period.toSec());

        if ((time - currentCommand->stamp).toSec() > _cmdVelTimeout) {
            _drive->brake();
            _xLimiter.reset();
            _yLimiter.reset();
            _yawLimiter.reset();
            ROS_DEBUG_STREAM_NAMED(_name, "Discarding outdated cmd_vel message.");
        } else {
            _kinematics->setVelocity(currentCommand->twist);
        }

        auto odomInfo = _kinematics->getOdometry(period.toSec());

        if (time - _lastOdomPublishTime >= _publishPeriod) {
            _lastOdomPublishTime = time;

            const geometry_msgs::Quaternion &orientationQuaternion = tf::createQuaternionMsgFromYaw(odomInfo.yaw);
            if (_odomPub.trylock()) {
                _odomPub.msg_.header.stamp = time;

                _odomPub.msg_.pose.pose.position.x = odomInfo.x;
                _odomPub.msg_.pose.pose.position.y = odomInfo.y;

                _odomPub.msg_.pose.pose.orientation = orientationQuaternion;
                boost::range::copy(odomInfo.positionCovariance, _odomPub.msg_.pose.covariance.begin());

                _odomPub.msg_.twist.twist.linear.x = odomInfo.vX;
                _odomPub.msg_.twist.twist.linear.y = odomInfo.vY;
                _odomPub.msg_.twist.twist.angular.z = odomInfo.vYaw;
                boost::range::copy(odomInfo.velocityCovariance, _odomPub.msg_.twist.covariance.begin());

                _odomPub.unlockAndPublish();
            }

            if (_tf2OdomEnable && _tfPub.trylock()) {
                auto &transform = _tfPub.msg_.transforms[0];

                transform.header.stamp = time;
                transform.transform.translation.x = odomInfo.x;
                transform.transform.translation.y = odomInfo.y;

                transform.transform.rotation = orientationQuaternion;

                _tfPub.unlockAndPublish();
            }
        }
    }

    void SixWheelSteeringController::stopping(const ros::Time &time) {
        _drive->brake();
    }

    void SixWheelSteeringController::waiting(const ros::Time &time) {
    }

    void SixWheelSteeringController::aborting(const ros::Time &time) {
        _drive->brake();
    }

    bool SixWheelSteeringController::init(hardware_interface::RobotHW *hw,
                                          ros::NodeHandle &rootNH,
                                          ros::NodeHandle &controllerNH) {

        const auto &complete_ns = controllerNH.getNamespace();
        auto id = complete_ns.find_last_of('/');
        _name = complete_ns.substr(id + 1);

        _helloMessage << "six_wheel_steering_controller started with following parameteres:\n";

        if (!initDrive(hw, rootNH, controllerNH)) return false;
        if (!initOdom(rootNH, controllerNH)) return false;
        if (!initLimiters(controllerNH)) return false;

        _currentModePub = controllerNH.advertise<std_msgs::UInt8>("current_mode", 5, true);
        _cmdVelSub = rootNH.subscribe("cmd_vel", 50, &SixWheelSteeringController::cmdVelCallback, this);

        _modeChangeServiceServer = controllerNH.advertiseService("mode_change",
                                                                 &SixWheelSteeringController::modeChangeCallback, this);

        ROS_INFO_STREAM_NAMED(_name, _helloMessage.str());
        _helloMessage.clear();
        return true;
    }

    void six_wheel_steering_controller::SixWheelSteeringController::starting(const ros::Time &time) {
        publishCurrentMode();
        _lastOdomPublishTime = time;
        controller_interface::ControllerBase::starting(time);
    }

    SixWheelSteeringController::SixWheelSteeringController() :
            _drive(std::make_shared<Drive>()),
            _mode2AngleError(0.),
            _cmdVelTimeout(0.15),
            _tf2OdomEnable(false) {
        _drive->setMode(std::make_unique<modes::Mode1>());
        _kinematics = std::make_unique<Kinematics>();
    }

    std::unique_ptr<WheelUnit>
    SixWheelSteeringController::getWheelUnit(hardware_interface::RobotHW *hw, const std::string &steeringJointName,
                                             const std::string &wheelJointName, double wheelRadius) {
        const auto &steeringHandle = hw->get<hardware_interface::PositionJointInterface>()->getHandle(
                steeringJointName);
        const auto &wheelHandle = hw->get<hardware_interface::VelocityJointInterface>()->getHandle(wheelJointName);

        auto ret = std::make_unique<WheelUnit>(steeringHandle, wheelHandle);
        ret->setWheelRadius(wheelRadius);
        return ret;
    }

    void SixWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist &msg) {
        if (isRunning()) {
            if (std::isnan(msg.linear.x) || std::isnan(msg.linear.y) || std::isnan(msg.angular.z)) {
                ROS_WARN_NAMED(_name, "Received NaN values. Ignoring message.");
            } else {
                _currentCommandNonRT.twist.linX = msg.linear.x;
                _currentCommandNonRT.twist.linY = msg.linear.y;
                _currentCommandNonRT.twist.angZ = msg.angular.z;
                _currentCommandNonRT.stamp = ros::Time::now();

                _currentCommand.writeFromNonRT(_currentCommandNonRT);
                ROS_DEBUG_STREAM_NAMED(_name, "Received message "
                                              "linear x: " << msg.linear.x <<
                                                           " linear y: " << msg.linear.y <<
                                                           " angular z: " << msg.angular.z);
            }
        } else {
            ROS_ERROR_STREAM_NAMED(_name, "Controller is not running. Ignoring message.");
        }
    }

    bool SixWheelSteeringController::modeChangeCallback(six_wheel_steering_msgs::ModeChangeRequest &req,
                                                        six_wheel_steering_msgs::ModeChangeResponse &resp) {

        if (!isRunning()) {
            ROS_ERROR_STREAM_NAMED(_name, "Controller is not running. Cannot call service.");
            return false;
        }
        std::unique_ptr<Mode> mode;

        switch ((EModes) req.mode) {
            case EModes::Mode1:
                mode = std::make_unique<modes::Mode1>();
                break;
            case EModes::Mode2:
                mode = std::make_unique<modes::Mode2>(_mode2AngleError);
                break;
            default:
                resp.success = false;
                return true;
        }
        _drive->setMode(std::move(mode));
        resp.success = true;
        publishCurrentMode();
        return true;
    }

    void SixWheelSteeringController::publishCurrentMode() {
        auto msg = std_msgs::UInt8{};
        msg.data = (uint8_t) _drive->getCurrentMode()->modeCode();
        _currentModePub.publish(msg);
    }

    bool SixWheelSteeringController::initOdom(ros::NodeHandle &rootNH, ros::NodeHandle &controllerNH) {
        _odomPub.init(rootNH, "odom", 20);

        _odomPub.msg_.header.frame_id = "odom";
        _odomPub.msg_.child_frame_id = _baseFrameId;
        _odomPub.msg_.pose.pose.position.z = 0;

        _odomPub.msg_.twist.twist.linear.z = 0;

        _odomPub.msg_.twist.covariance[0] = -1;
        _odomPub.msg_.twist.covariance[7] = -1;
        _odomPub.msg_.twist.covariance[14] = -1;
        _odomPub.msg_.twist.covariance[21] = -1;
        _odomPub.msg_.twist.covariance[28] = -1;
        _odomPub.msg_.twist.covariance[35] = -1;

        _odomPub.msg_.pose.covariance[0] = -1;
        _odomPub.msg_.pose.covariance[7] = -1;
        _odomPub.msg_.pose.covariance[14] = -1;
        _odomPub.msg_.pose.covariance[21] = -1;
        _odomPub.msg_.pose.covariance[28] = -1;
        _odomPub.msg_.pose.covariance[35] = -1;

        std::vector<double> poseCovariance, twistCovariance;

        if (controllerNH.getParam("twist_covariance", twistCovariance)) {
            if (twistCovariance.size() != 36) {
                ROS_ERROR_STREAM("Wrong size of twist_covariance_diagonal current = "
                                         << twistCovariance.size() << " expected: 36");
                return false;
            }
            std::copy(twistCovariance.begin(), twistCovariance.end(), _odomPub.msg_.twist.covariance.begin());
        }

        if (controllerNH.getParam("pose_covariance", poseCovariance)) {
            if (poseCovariance.size() != 36) {
                ROS_ERROR_STREAM("Wrong size of pose_covariance_diagonal current = "
                                         << poseCovariance.size() << " expected: 36");
                return false;
            }
            std::copy(poseCovariance.begin(), poseCovariance.end(), _odomPub.msg_.pose.covariance.begin());
        }

        double pubRate;
        controllerNH.param("publish_rate", pubRate, 50.);
        _publishPeriod = ros::Duration(1 / pubRate);

        controllerNH.param("enable_odom_tf", _tf2OdomEnable, false);

        if (_tf2OdomEnable) {
            _tfPub.init(rootNH, "/tf", 50);

            _tfPub.msg_.transforms.resize(1);
            _tfPub.msg_.transforms[0].header.frame_id = "odom";
            _tfPub.msg_.transforms[0].child_frame_id = _baseFrameId;
            _tfPub.msg_.transforms[0].transform.translation.z = 0;
        }

        _helloMessage << "\tpublish_rate: " << pubRate << "\n"
                      << "\tenable_odom_tf: " << (_tf2OdomEnable ? "true" : "false") << '\n'
                      << "\tpose_covariance:";

        for (int i = 0; i < 6; i++) {
            _helloMessage << "\n\t\t";
            std::copy(_odomPub.msg_.twist.covariance.begin() + i * 6,
                      _odomPub.msg_.twist.covariance.begin() + (i + 1) * 6,
                      std::ostream_iterator<double>(_helloMessage << std::fixed, " "));
        }
        _helloMessage << "\n\ttwist_covariance:";
        for (int i = 0; i < 6; i++) {
            _helloMessage << "\n\t\t";
            std::copy(_odomPub.msg_.pose.covariance.begin() + i * 6,
                      _odomPub.msg_.pose.covariance.begin() + (i + 1) * 6,
                      std::ostream_iterator<double>(_helloMessage << std::fixed, " "));
        }
        return true;
    }

    bool SixWheelSteeringController::initDrive(hardware_interface::RobotHW *hw, ros::NodeHandle &rootNH,
                                               ros::NodeHandle &controllerNH) {
        //region joints_init
#define GET_JOINT_PARAM(var, paramName) \
        if(!controllerNH.getParam(paramName, var)){ \
        ROS_ERROR_STREAM_NAMED("Init","Could not get " paramName " parameter."); \
        return false;\
        }

        std::string frontRightSteeringJointName;
        GET_JOINT_PARAM(frontRightSteeringJointName, "front_right_steering_joint")
        std::string frontLeftSteeringJointName;
        GET_JOINT_PARAM(frontLeftSteeringJointName, "front_left_steering_joint")
        std::string midRightSteeringJointName;
        GET_JOINT_PARAM(midRightSteeringJointName, "mid_right_steering_joint")
        std::string midLeftSteeringJointName;
        GET_JOINT_PARAM(midLeftSteeringJointName, "mid_left_steering_joint")
        std::string rearRightSteeringJointName;
        GET_JOINT_PARAM(rearRightSteeringJointName, "rear_right_steering_joint")
        std::string rearLeftSteeringJointName;
        GET_JOINT_PARAM(rearLeftSteeringJointName, "rear_left_steering_joint")
        std::string frontRightWheelJointName;
        GET_JOINT_PARAM(frontRightWheelJointName, "front_right_wheel_joint")
        std::string frontLeftWheelJointName;
        GET_JOINT_PARAM(frontLeftWheelJointName, "front_left_wheel_joint")
        std::string midRightWheelJointName;
        GET_JOINT_PARAM(midRightWheelJointName, "mid_right_wheel_joint")
        std::string midLeftWheelJointName;
        GET_JOINT_PARAM(midLeftWheelJointName, "mid_left_wheel_joint")
        std::string rearRightWheelJointName;
        GET_JOINT_PARAM(rearRightWheelJointName, "rear_right_wheel_joint")
        std::string rearLeftWheelJointName;
        GET_JOINT_PARAM(rearLeftWheelJointName, "rear_left_wheel_joint")
// endregion


        controllerNH.param("base_frame_id", _baseFrameId, std::string("base_link"));

        double ySpacing, midToFront, midToRear, wheelRadius;

        bool obtainY_Spacing = !controllerNH.getParam("y_spacing", ySpacing);
        bool obtainMidToFront = !controllerNH.getParam("mid_to_front", midToFront);
        bool obtainMidToRear = !controllerNH.getParam("mid_to_rear", midToRear);
        bool obtainWheelRadius = !controllerNH.getParam("wheel_radius", wheelRadius);

        urdf_geometry_parser::UrdfGeometryParser ugp(rootNH, _baseFrameId);


        if (obtainY_Spacing) {

            if (!ugp.getDistanceBetweenJoints(frontLeftSteeringJointName, frontRightSteeringJointName, ySpacing)) {
                ROS_ERROR_STREAM_NAMED(_name + "/init", "Couldn't obtain y_spacing. Aborting...");
                return false;
            } else {
                controllerNH.setParam("y_spacing", ySpacing);
            }
        }

        if (obtainMidToFront) {

            if (!ugp.getDistanceBetweenJoints(frontLeftSteeringJointName, midLeftSteeringJointName, midToFront)) {
                ROS_ERROR_STREAM_NAMED(_name + "/init", "Couldn't obtain mid_to_front parameter. Aborting...");
                return false;

            } else {
                controllerNH.setParam("mid_to_front", midToFront);
            }
        }
        if (obtainMidToRear) {

            if (!ugp.getDistanceBetweenJoints(rearLeftSteeringJointName, midLeftSteeringJointName, midToRear)) {
                ROS_ERROR_STREAM_NAMED(_name + "/init", "Couldn't obtain mid_to_rear parameter. Aborting...");
                return false;
            } else {
                controllerNH.setParam("mid_to_rear", midToRear);
            }
        }
        if (obtainWheelRadius) {

            if (!ugp.getJointRadius(frontLeftWheelJointName, wheelRadius)) {
                ROS_ERROR_STREAM_NAMED(_name + "/init", "Couldn't obtain wheel_radius parameter. Aborting...");
                return false;
            } else {
                controllerNH.setParam("wheel_radius", wheelRadius);
            }
        }


        _drive->setFrontRight(getWheelUnit(hw, frontRightSteeringJointName, frontRightWheelJointName, wheelRadius));
        _drive->setFrontLeft(getWheelUnit(hw, frontLeftSteeringJointName, frontLeftWheelJointName, wheelRadius));
        _drive->setMidRight(getWheelUnit(hw, midRightSteeringJointName, midRightWheelJointName, wheelRadius));
        _drive->setMidLeft(getWheelUnit(hw, midLeftSteeringJointName, midLeftWheelJointName, wheelRadius));
        _drive->setRearRight(getWheelUnit(hw, rearRightSteeringJointName, rearRightWheelJointName, wheelRadius));
        _drive->setRearLeft(getWheelUnit(hw, rearLeftSteeringJointName, rearLeftWheelJointName, wheelRadius));

        _drive->setDistanceToFront(midToFront);
        _drive->setDistanceToRear(midToRear);
        _drive->setY_AxisSpacing(ySpacing);

        _kinematics->init(_drive);

        _mode2AngleError = controllerNH.param("mode2_angle_error", 1e-3);


        _helloMessage
                << "\tbase_frame_id: " << _baseFrameId << "\n"
                << "\tsteering joints:\n"
                << "\t\t- " << frontLeftSteeringJointName << "\n"
                << "\t\t- " << frontRightSteeringJointName << "\n"
                << "\t\t- " << midLeftSteeringJointName << "\n"
                << "\t\t- " << midRightSteeringJointName << "\n"
                << "\t\t- " << rearLeftSteeringJointName << "\n"
                << "\t\t- " << rearRightSteeringJointName << "\n"
                << "\twheel_joints:\n"
                << "\t\t- " << frontLeftWheelJointName << "\n"
                << "\t\t- " << frontRightWheelJointName << "\n"
                << "\t\t- " << midLeftWheelJointName << "\n"
                << "\t\t- " << midRightWheelJointName << "\n"
                << "\t\t- " << rearLeftWheelJointName << "\n"
                << "\t\t- " << rearRightWheelJointName << "\n"
                << "\tmid_to_front_distance: " << midToFront << "\n"
                << "\tmid_to_rear_distance: " << midToRear << "\n"
                << "\ty_spacing: " << ySpacing << "\n"
                << "\twheel_radius: " << wheelRadius << "\n";
        return true;
    }

    bool SixWheelSteeringController::initLimiters(ros::NodeHandle &controllerNH) {

#define OBTAIN_PARAMS(parameters_namespace, limiter)\
        controllerNH.getParam(parameters_namespace "/has_velocity_limits", (limiter).hasVelocityLimits);\
        controllerNH.getParam(parameters_namespace "/max_velocity", (limiter).maxVelocity);\
        controllerNH.getParam(parameters_namespace "/min_velocity", (limiter).minVelocity); \
        \
        controllerNH.getParam(parameters_namespace "/has_acceleration_limits", (limiter).hasAccelerationLimits);\
        controllerNH.getParam(parameters_namespace "/max_acceleration", (limiter).maxAcceleration);\
        controllerNH.getParam(parameters_namespace "/min_acceleration", (limiter).minAcceleration);\
        \
        controllerNH.getParam(parameters_namespace "/has_jerk_limits", (limiter).hasJerkLimits);\
        controllerNH.getParam(parameters_namespace "/max_jerk", (limiter).maxJerk);\
        controllerNH.getParam(parameters_namespace "/min_jerk", (limiter).minJerk);

#define ADD_TO_HELLO_MESSAGE(parameter, limiter) \
        _helloMessage<<                                \
            "\n\t\t\t" parameter ":"<<\
            "\n\t\t\t\t- has_velocity_limits: "   << ((limiter).hasVelocityLimits ? "true": "false")<<\
            "\n\t\t\t\t- max_velocity: "    << (limiter).maxVelocity<<\
            "\n\t\t\t\t- min_velocity: "    <<(limiter).minVelocity<<\
            "\n\t\t\t\t- has_acceleration_limits: " << ((limiter).hasAccelerationLimits ? "true": "false")<<\
            "\n\t\t\t\t- max_acceleration: " << (limiter).maxAcceleration <<\
            "\n\t\t\t\t- min_acceleration: " << (limiter).minAcceleration <<\
            "\n\t\t\t\t- has_jerk_limits: " << ((limiter).hasJerkLimits ? "true": "false")<<\
            "\n\t\t\t\t- max_jerk: " << (limiter).maxJerk <<\
            "\n\t\t\t\t- min_jerk: " << (limiter).minJerk;


        OBTAIN_PARAMS("limits/linear/x", _xLimiter)
        OBTAIN_PARAMS("limits/linear/y", _yLimiter)
        OBTAIN_PARAMS("limits/angular/z", _yawLimiter)

        _helloMessage << "\n\tlimits:\n\t\tlinear:";
        ADD_TO_HELLO_MESSAGE( "x", _xLimiter)
        ADD_TO_HELLO_MESSAGE( "y", _yLimiter)
        _helloMessage<<"\n\t\tangular:";
        ADD_TO_HELLO_MESSAGE( "z", _yawLimiter)

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(six_wheel_steering_controller::SixWheelSteeringController, controller_interface::ControllerBase)
