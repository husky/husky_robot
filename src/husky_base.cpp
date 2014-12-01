#include <controller_manager/controller_manager.h>

#include "ros/ros.h"
#include "husky_base/husky_hardware.h"

void diagnosticLoop(const ros::TimerEvent& event, husky_base::HuskyHardware &husky)
{
//    husky.{};
}

void controlLoop(const ros::TimerEvent& event, husky_base::HuskyHardware &husky, controller_manager::ControllerManager &cm)
{
    husky.updateJointsFromHardware();
    cm.update(event.current_real, event.current_real - event.last_real);
    husky.writeCommandsToHardware();
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "husky_base");
    ros::NodeHandle nh, private_nh("~");

    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

    husky_base::HuskyHardware husky(nh, private_nh, control_frequency, diagnostic_frequency);
    controller_manager::ControllerManager cm(&husky, nh);

    ros::Timer control_loop = nh.createTimer(ros::Duration(1/control_frequency), boost::bind(controlLoop, _1, boost::ref(husky), boost::ref(cm)));
    ros::Timer diagnostic_loop = nh.createTimer(ros::Duration(1/diagnostic_frequency), boost::bind(diagnosticLoop, _1, boost::ref(husky)));

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;

}