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

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "husky_base/horizon_legacy/clearpath.h"
#include "husky_base/horizon_legacy_wrapper.h"
#include "husky_msgs/HuskyStatus.h"

#ifndef HUSKY_BASE_HUSKY_DIAGNOSTICS_H
#define HUSKY_BASE_HUSKY_DIAGNOSTICS_H

namespace husky_base
{

  template<typename T>
  class HuskyDiagnosticTask : public diagnostic_updater::DiagnosticTask
  {
  public:
    HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      typename Msg<T>::Ptr latest = Msg<T>::getUpdate();
      if (latest)
      {
        update(stat, latest);
      }
    }

    void update(diagnostic_updater::DiagnosticStatusWrapper &stat, typename Msg<T>::Ptr &status);

  private:
    husky_msgs::HuskyStatus &msg_;

  };

  template<>
  HuskyDiagnosticTask<clearpath::DataSystemStatus>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg);

  template<>
  HuskyDiagnosticTask<clearpath::DataPowerSystem>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg);

  template<>
  HuskyDiagnosticTask<clearpath::DataSafetySystemStatus>::HuskyDiagnosticTask(husky_msgs::HuskyStatus &msg);

  template<>
  void HuskyDiagnosticTask<clearpath::DataSystemStatus>::update(
      diagnostic_updater::DiagnosticStatusWrapper &stat, Msg<clearpath::DataSystemStatus>::Ptr &status);

  template<>
  void HuskyDiagnosticTask<clearpath::DataPowerSystem>::update(
      diagnostic_updater::DiagnosticStatusWrapper &stat, Msg<clearpath::DataPowerSystem>::Ptr &status);

  template<>
  void HuskyDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
      diagnostic_updater::DiagnosticStatusWrapper &stat, Msg<clearpath::DataSafetySystemStatus>::Ptr &status);

} // namespace husky_base
#endif  // HUSKY_BASE_HUSKY_DIAGNOSTICS_H