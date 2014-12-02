/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"
#include "husky_base/husky_hardware.h"

void diagnosticLoop(const ros::TimerEvent &event, husky_base::HuskyHardware &husky)
{
  husky.updateDiagnostics();
}

void controlLoop(const ros::TimerEvent &event, husky_base::HuskyHardware &husky,
    controller_manager::ControllerManager &cm)
{

  //Check that Husky hardware meets expected control frequency
#ifdef ROS_DEBUG
  double frequency = 1 / (event.current_real - event.last_real).toSec();
  double expected_frequency = 1 / (event.current_expected - event.last_expected).toSec();
  ROS_WARN_STREAM_COND(frequency > expected_frequency * 10.1,
      "Husky control loop target frequency " << expected_frequency << " Hz, actual frequency " << frequency << " Hz");
#endif

  husky.updateJointsFromHardware();
  cm.update(event.current_real, event.current_real - event.last_real);
  husky.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "husky_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  husky_base::HuskyHardware husky(nh, private_nh);
  controller_manager::ControllerManager cm(&husky, nh);

  // Setup loops to process latest control and diagnostic messages
  ros::CallbackQueue husky_queue;
  ros::TimerOptions control_timer(
      ros::Duration(1 / control_frequency),
      boost::bind(controlLoop, _1, boost::ref(husky), boost::ref(cm)),
      &husky_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(
      ros::Duration(1 / diagnostic_frequency),
      boost::bind(diagnosticLoop, _1, boost::ref(husky)),
      &husky_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  // Process all hardware-related timer callbacks serially on separate queue, libhorizon_legacy not threadsafe
  ros::AsyncSpinner husky_spinner(1, &husky_queue);
  husky_spinner.start();

  // Process remainder of ROS callbacks serially, mainly ControlManager related
  ros::spin();

  return 0;
}
