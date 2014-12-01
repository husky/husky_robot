#include <husky_base/horizon_legacy_api/Message.h>
#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#ifndef HUSKY_BASE_HUSKY_HARDWARE_H
#define HUSKY_BASE_HUSKY_HARDWARE_H

namespace husky_base
{


    class HuskyHardware : public hardware_interface::RobotHW
    {

    public:
        HuskyHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double control_frequency, double diagnostic_frequency);
        ~HuskyHardware();
        void updateJointsFromHardware();
        void writeCommandsToHardware();

    private:

        //Communication init/deinit
        void connect(std::string port);
        void subscribe(double control_frequency, double diagnostic_frequency);
        void unsubscribe();

        //Helper Function
        void resetTravelOffset();
        double travelToAngle(const double &travel) const;
        double angleToTravel(const double &angle) const;
        void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

        ros::NodeHandle nh_, private_nh_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        //ROS Params
        double wheel_diameter_, max_accel_, max_speed_;

        struct Joint
        {
            double position;
            double position_offset;
            double velocity;
            double effort;
            double velocity_command;

            Joint() : position(0), velocity(0), effort(0), velocity_command(0)
            {
            }
        } joints_[4];

    };

} //husky_base
#endif