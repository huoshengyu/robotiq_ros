// Copyright (c) 2026 Robotiq
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_controllers/object_status_goal.hpp"

#include <algorithm>

#include "rclcpp/logging.hpp"
#include "robotiq_controllers/gripper_status.hpp"

namespace robotiq_controllers::object_status_goal {
namespace {
constexpr const char* kInterface = gripper_status::kInterfaceNames.at(gripper_status::OBJECT_STATUS);
} // namespace

std::string interfaceName(const std::string& joint)
{
   return joint + "/" + kInterface;
}

std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> findInterface(
   std::vector<hardware_interface::LoanedStateInterface>& interfaces,
   const std::string& joint,
   const rclcpp::Logger& logger)
{
   const auto object_status =
      std::find_if(interfaces.begin(), interfaces.end(), [&](const hardware_interface::LoanedStateInterface& i) {
         return i.get_prefix_name() == joint && i.get_interface_name() == kInterface;
      });
   if(object_status == interfaces.end())
   {
      RCLCPP_ERROR(logger,
                   "%s is set but joint '%s' exports no %s. Mock and topic-based hardware do not report it; "
                   "use the driver against a gripper or its simulation, or unset the parameter.",
                   kUseParameter,
                   joint.c_str(),
                   kInterface);
      return std::nullopt;
   }
   return *object_status;
}

void Verdict::reset(const rclcpp::Time& time,
                    const std::optional<Robotiq::ObjectDetection>& objectDetection,
                    double timeout)
{
   timed_from_ = time;
   timeout_ = timeout;
   baseline_ = objectDetection;
}

std::optional<Outcome> Verdict::decide(const rclcpp::Time& time,
                                       const std::optional<Robotiq::ObjectDetection>& objectDetection)
{
   if(!baseline_)
   {
      // No reading at acceptance: the first one stands in for it.
      baseline_ = objectDetection;
   }
   else if(objectDetection && objectDetection != baseline_)
   {
      if(objectDetection == Robotiq::ObjectDetection::Moving)
      {
         // Motion seen: whatever the gripper settles on next is this goal's
         // verdict, even the value it started from, as when it tightens on the
         // object it already held.
         baseline_ = objectDetection;
         timed_from_ = time;
      }
      else
      {
         const bool reached = objectDetection == Robotiq::ObjectDetection::AtRequestedPosition;
         return Outcome{reached, !reached};
      }
   }
   if((time - timed_from_).seconds() >= timeout_)
   {
      return Outcome{false, false};
   }
   return std::nullopt;
}
} // namespace robotiq_controllers::object_status_goal
