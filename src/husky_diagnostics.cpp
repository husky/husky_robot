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

#include "husky_base/husky_diagnostics.h"
#include <algorithm>

namespace
{
  const int UNDERVOLT_ERROR = 18;
  const int UNDERVOLT_WARN = 19;
  const int OVERVOLT_ERROR = 30;
  const int OVERVOLT_WARN = 29;
  const int DRIVER_OVERTEMP_ERROR = 50;
  const int DRIVER_OVERTEMP_WARN = 30;
  const int MOTOR_OVERTEMP_ERROR = 80;
  const int MOTOR_OVERTEMP_WARN = 70;
  const double LOWPOWER_ERROR = 0.2;
  const double LOWPOWER_WARN = 0.3;
  const unsigned int SAFETY_TIMEOUT = 0x1;
  const unsigned int SAFETY_LOCKOUT = 0x2;
  const unsigned int SAFETY_ESTOP = 0x8;
  const unsigned int SAFETY_CCI = 0x10;
  const unsigned int SAFETY_PSU = 0x20;
  const unsigned int SAFETY_CURRENT = 0x40;
  const unsigned int SAFETY_WARN = (SAFETY_TIMEOUT | SAFETY_CCI | SAFETY_PSU);
  const unsigned int SAFETY_ERROR = (SAFETY_LOCKOUT | SAFETY_ESTOP | SAFETY_CURRENT);
}  // namespace

namespace husky_base
{

  template<>
  HuskyDiagnosticTask<clearpath::DataSystemStatus>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg)
      : DiagnosticTask("system_status"),
        msg_(msg) { }

  template<>
  void HuskyDiagnosticTask<clearpath::DataSystemStatus>::update(diagnostic_updater::DiagnosticStatusWrapper &stat,
      Msg<clearpath::DataSystemStatus>::Ptr &system_status)
  {
    msg_.uptime = system_status->getUptime();
    msg_.bus_voltage = system_status->getVoltage(0);
    msg_.left_driver_voltage = system_status->getVoltage(1);
    msg_.right_driver_voltage = system_status->getVoltage(2);
    msg_.left_driver_temp = system_status->getTemperature(0);
    msg_.right_driver_temp = system_status->getTemperature(1);
    msg_.left_motor_temp = system_status->getTemperature(2);
    msg_.right_motor_temp = system_status->getTemperature(3);

    stat.add("Uptime", msg_.uptime);
    stat.add("Bus Voltage", msg_.bus_voltage);
    stat.add("Left motor driver (C)", msg_.left_driver_temp);
    stat.add("Right motor driver (C)", msg_.right_driver_temp);
    stat.add("Left motor (C)", msg_.left_motor_temp);
    stat.add("Right motor (C)", msg_.right_motor_temp);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System Status OK");
    if (msg_.bus_voltage > OVERVOLT_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main bus voltage too high");
    }
    else if (msg_.bus_voltage > OVERVOLT_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main bus voltage too high");
    }
    else if (msg_.bus_voltage < UNDERVOLT_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main bus voltage too low");
    }
    else if (msg_.bus_voltage < UNDERVOLT_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main bus voltage too low");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Voltage OK");
    }

    if (std::max(msg_.left_driver_temp, msg_.right_driver_temp) > DRIVER_OVERTEMP_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor drivers too hot");
    }
    else if (std::max(msg_.left_driver_temp, msg_.right_driver_temp) > DRIVER_OVERTEMP_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor drivers too hot");
    }
    else if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > MOTOR_OVERTEMP_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motors too hot");
    }
    else if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > MOTOR_OVERTEMP_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motors too hot");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK");
    }
  }

  template<>
  HuskyDiagnosticTask<clearpath::DataPowerSystem>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg)
      : DiagnosticTask("power_status"),
        msg_(msg) { }

  template<>
  void HuskyDiagnosticTask<clearpath::DataPowerSystem>::update(diagnostic_updater::DiagnosticStatusWrapper &stat,
      Msg<clearpath::DataPowerSystem>::Ptr &power_status)
  {
    msg_.charge_estimate = power_status->getChargeEstimate(0);
    msg_.capacity_estimate = power_status->getCapacityEstimate(0);

    stat.add("Charge (%)", msg_.charge_estimate);
    stat.add("Battery Capacity (Wh)", msg_.capacity_estimate);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Power System OK");
    if (msg_.charge_estimate < LOWPOWER_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Low power");
    }
    else if (msg_.charge_estimate < LOWPOWER_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Low power");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Charge OK");
    }
  }

  template<>
  HuskyDiagnosticTask<clearpath::DataSafetySystemStatus>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg)
      : DiagnosticTask("safety_status"),
        msg_(msg) { }

  template<>
  void HuskyDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
      diagnostic_updater::DiagnosticStatusWrapper &stat, Msg<clearpath::DataSafetySystemStatus>::Ptr &safety_status)
  {
    msg_.flags = safety_status->getFlags();
    msg_.timeout = (msg_.flags & SAFETY_TIMEOUT) > 0;
    msg_.lockout = (msg_.flags & SAFETY_LOCKOUT) > 0;
    msg_.e_stop = (msg_.flags & SAFETY_ESTOP) > 0;
    msg_.ros_pause = (msg_.flags & SAFETY_CCI) > 0;
    msg_.no_battery = (msg_.flags & SAFETY_PSU) > 0;
    msg_.current_limit = (msg_.flags & SAFETY_CURRENT) > 0;

    stat.add("Timeout", static_cast<bool>(msg_.timeout));
    stat.add("Lockout", static_cast<bool>(msg_.lockout));
    stat.add("Emergency Stop", static_cast<bool>(msg_.e_stop));
    stat.add("ROS Pause", static_cast<bool>(msg_.ros_pause));
    stat.add("No battery", static_cast<bool>(msg_.no_battery));
    stat.add("Current limit", static_cast<bool>(msg_.current_limit));

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safety System OK");
    if ((msg_.flags & SAFETY_ERROR) > 0)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error");
    }
    else if ((msg_.flags & SAFETY_WARN) > 0)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Warning");
    }
  }

}  // namespace husky_base
