#include <husky_base/husky_hardware.h>
#include <boost/assign/list_of.hpp>
#include <husky_base/horizon_legacy_api/clearpath.h>

namespace {
    const uint16_t UNSUBSCRIBE = 0xFFFF;
    const uint8_t LEFT = 0, RIGHT = 1;
};

namespace husky_base {

    HuskyHardware::HuskyHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double control_frequency, double diagnostic_frequency) :
            nh_(nh), private_nh_(private_nh_)
    {

        private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3555);
        private_nh_.param<double>("max_accel", max_accel_, 5.0);
        private_nh_.param<double>("max_speed", max_speed_, 1.0);

        std::string port;
        nh_.param<std::string>("port", port, "/dev/prolific");

        connect(port);
        subscribe(control_frequency, diagnostic_frequency);

        ros::V_string joint_names = boost::assign::list_of("joint_front_left_wheel")
                ("joint_front_right_wheel")("joint_back_left_wheel")("joint_back_right_wheel");

        for (unsigned int i = 0; i < joint_names.size(); i++) {
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                    &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(
                    joint_state_handle, &joints_[i].velocity_command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);

    }

    HuskyHardware::~HuskyHardware(){
        unsubscribe();
    }

    void HuskyHardware::connect(std::string port) {
        try {
            clearpath::Transport::instance().configure(port.c_str(), 3 /* max retries*/);
        } catch (clearpath::Exception *ex) {
            ROS_ERROR_STREAM("Error connecting to Husky: " << ex->message);
        }

        try {
            clearpath::SetMaxAccel(max_accel_, max_accel_).send();
            clearpath::SetMaxSpeed(max_speed_, max_speed_).send();
        } catch (clearpath::Exception *ex) {
            ROS_ERROR_STREAM("Error setting Husky parameters (accel=" << max_accel_ << ",speed=" << max_speed_ << "): " << ex->message);
        }
    }

    void HuskyHardware::subscribe(double control_frequency, double diagnostic_frequency) {
        resetTravelOffset();
        clearpath::DataEncoders::subscribe(control_frequency);

        //TODO subscribe to diagnostics
    }

    void HuskyHardware::unsubscribe() {
        clearpath::DataEncoders::subscribe(UNSUBSCRIBE);

        //TODO unsubscribe to diagnostics
    }

    void HuskyHardware::resetTravelOffset(){
        clearpath::DataEncoders *enc = clearpath::DataEncoders::getUpdate(1.0);
        if (enc){
            for (int i = 0; i < 4; i++)
            {
                joints_[i].position_offset = travelToAngle(enc->getTravel(i % 2));
            }
        }else{
            throw std::runtime_error("Cannot get encoder data");
        }
    }

    void HuskyHardware::updateJointsFromHardware()
    {
        clearpath::DataEncoders *enc = 0;
        while(enc = clearpath::DataEncoders::popNext()) {
            for (int i = 0; i < 4; i++)
            {
                joints_[i].position = travelToAngle(enc->getTravel(i % 2)) - joints_[i % 2].position_offset;
                joints_[i].velocity = travelToAngle(enc->getSpeed(i % 2));
            }
        }
    }

    void HuskyHardware::writeCommandsToHardware()
    {
        double diff_speed_left = angleToTravel(joints_[LEFT].velocity_command);
        double diff_speed_right = angleToTravel(joints_[RIGHT].velocity_command);

        limitDifferentialSpeed(diff_speed_left, diff_speed_right);

        try {
            clearpath::SetDifferentialSpeed(
                    diff_speed_left, diff_speed_right,
                    max_accel_, max_accel_
            ).send();
        }catch(clearpath::Exception *ex) {
            ROS_ERROR_STREAM("Error setting speed/velocity command: " << ex->message);
        }

    };

    void HuskyHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right) {

        double large_speed = std::max(diff_speed_left, diff_speed_right);

        if(large_speed > max_speed_){
            diff_speed_left *= max_speed_ / large_speed;
            diff_speed_right *= max_speed_ / large_speed;
        }

    }

    double HuskyHardware::travelToAngle(const double &travel) const {
        return travel / wheel_diameter_;
    }

    double HuskyHardware::angleToTravel(const double &angle) const {
        return angle * wheel_diameter_;
    }

}