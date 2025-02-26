/*
 *  Copyright 2025, MASCOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    MASCOR
 *
 */
#include "mbf_octo_nav/octo_controller_execution.h"

namespace mbf_octo_nav
{
OctoControllerExecution::OctoControllerExecution(const std::string& name,
                                                 const mbf_octo_core::OctoController::Ptr& controller_ptr,
                                                 const mbf_utility::RobotInformation::ConstPtr& robot_info,
                                                 const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& vel_pub,
                                                 const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& goal_pub,
                                                 const rclcpp::Node::SharedPtr& node)
  : AbstractControllerExecution(name, controller_ptr, robot_info, vel_pub, goal_pub, node)
{
}

OctoControllerExecution::~OctoControllerExecution()
{
}

uint32_t OctoControllerExecution::computeVelocityCmd(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                     const geometry_msgs::msg::TwistStamped& robot_velocity,
                                                     geometry_msgs::msg::TwistStamped& vel_cmd, std::string& message)
{
  // Lock the octo while planning, but following issue #4, we allow to move the
  // responsibility to the planner itself
  // Note: we do not save the octo map globallly at this stage.
  //if (lock_mesh_)
  //{
    // TODO
    // boost::lock_guard<mesh::Mesh::mutex_t> lock(*(mesh_ptr_->getMutex()));
    // return controller_->computeVelocityCommands(robot_pose, robot_velocity,
    // vel_cmd, message);
  //}
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

} /* namespace mbf_octo_nav */
