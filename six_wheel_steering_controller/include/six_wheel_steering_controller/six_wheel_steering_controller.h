#ifndef SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H
#define SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H


#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "kinematics.h"
#include "drive.h"
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <six_wheel_steering_msgs/ModeChange.h>
#include <tf/tfMessage.h>
#include <sstream>
#include "speed_limiter.h"

namespace six_wheel_steering_controller{

    class SixWheelSteeringController: public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
            hardware_interface::PositionJointInterface>{
    public:
        SixWheelSteeringController();

        void starting(const ros::Time &time) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void stopping(const ros::Time &time) override;

        void waiting(const ros::Time &time) override;

        void aborting(const ros::Time &time) override;

        bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &rootNH, ros::NodeHandle &controllerNH) override;

    private:
        std::string _name;
        std::stringstream _helloMessage;

        std::unique_ptr<Kinematics> _kinematics;
        std::shared_ptr<Drive> _drive;

        SpeedLimiter _xLimiter;
        SpeedLimiter _yLimiter;
        SpeedLimiter _yawLimiter;

        std::string _baseFrameId;

        realtime_tools::RealtimePublisher<nav_msgs::Odometry> _odomPub;
        realtime_tools::RealtimePublisher<tf::tfMessage> _tfPub;
        ros::Publisher _currentModePub;
        ros::Subscriber _cmdVelSub;
        ros::ServiceServer _modeChangeServiceServer;
        ros::Duration _publishPeriod;
        ros::Time _lastOdomPublishTime;

        double _mode2AngleError;
        double _cmdVelTimeout;

        bool _tf2OdomEnable;

        struct Command{
            Twist twist{};
            ros::Time stamp;
        }_currentCommandNonRT{};

        realtime_tools::RealtimeBuffer<Command> _currentCommand;

        void cmdVelCallback(const geometry_msgs::Twist &msg);
        bool initOdom(ros::NodeHandle &rootNH, ros::NodeHandle &controllerNH);
        bool initDrive(hardware_interface::RobotHW *hw, ros::NodeHandle &rootNH,
                       ros::NodeHandle &controllerNH);
        bool initLimiters(ros::NodeHandle &controllerNH);

        bool modeChangeCallback(six_wheel_steering_msgs::ModeChangeRequest &req, six_wheel_steering_msgs::ModeChangeResponse & resp);
        void publishCurrentMode();
        static std::unique_ptr<WheelUnit>
        getWheelUnit(hardware_interface::RobotHW *hw, const std::string &steeringJointName,
                     const std::string &wheelJointName, double wheelRadius);
    };
}

#endif //SIX_WHEEL_STEERING_CONTROLLER_SIX_WHEEL_STEERING_CONTROLLER_H
