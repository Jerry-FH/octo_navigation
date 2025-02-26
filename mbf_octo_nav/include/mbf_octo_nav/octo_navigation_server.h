/*
 *  Copyright 2020, MASCOR INSTITUE. All rights reserved.
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
 *    MASCOR INSTITUE.
 *
 */

#ifndef MBF_OCTO_NAV__OCTO_NAVIGATION_SERVER_H
#define MBF_OCTO_NAV__OCTO_NAVIGATION_SERVER_H

#include <mutex>

#include <mbf_abstract_nav/abstract_navigation_server.h>

#include "octo_controller_execution.h"
#include "octo_planner_execution.h"
#include "octo_recovery_execution.h"

#include <mbf_msgs/srv/check_path.hpp>
#include <mbf_msgs/srv/check_pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mbf_simple_core/simple_planner.h>
#include <mbf_simple_core/simple_controller.h>
#include <mbf_simple_core/simple_recovery.h>

#include <pluginlib/class_loader.hpp>

namespace mbf_octo_nav
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */

/**
 * @brief The OctoNavigationServer makes Move Base Flex backwards compatible to
 * the old move_base. It combines the execution classes which use the
 * nav_core/BaseLocalPlanner, nav_core/BaseOctoPlanner and the
 *        nav_core/RecoveryBehavior base classes as plugin interfaces. These
 * plugin interface are the same for the old move_base
 *
 * Supports both octo_core and simple_core plugins.
 *
 * @ingroup navigation_server move_base_server
 */
class OctoNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:

  typedef std::shared_ptr<OctoNavigationServer> Ptr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  OctoNavigationServer(const TFPtr& tf_listener_ptr, const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Destructor
   */
  virtual ~OctoNavigationServer();

  virtual void stop();

private:
  //! shared pointer to a new @ref planner_execution "PlannerExecution"
  virtual mbf_abstract_nav::AbstractPlannerExecution::Ptr
  newPlannerExecution(const std::string &plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr);

  //! shared pointer to a new @ref controller_execution "ControllerExecution"
  virtual mbf_abstract_nav::AbstractControllerExecution::Ptr
  newControllerExecution(const std::string &plugin_name, const mbf_abstract_core::AbstractController::Ptr plugin_ptr);

  //! shared pointer to a new @ref recovery_execution "RecoveryExecution"
  virtual mbf_abstract_nav::AbstractRecoveryExecution::Ptr
  newRecoveryExecution(const std::string &plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr);

  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type);

  /**
   * @brief Initializes the controller plugin with its name and pointer to the
   * octo
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the
   * name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializePlannerPlugin(const std::string& name,
                                       const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr);

  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller
   * plugin was loaded successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type);

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the
   * TransformListener and pointer to the octo
   * @param name The name of the controller
   * @param controller_ptr pointer to the controller object which corresponds to
   * the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeControllerPlugin(const std::string& name,
                                          const mbf_abstract_core::AbstractController::Ptr& controller_ptr);

  /**
   * @brief Loads a Recovery plugin associated with given recovery type
   * parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded
   * successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type);

  /**
   * @brief Initializes a recovery behavior plugin with its name and pointers to
   * the global and local octos
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which
   * corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeRecoveryPlugin(const std::string& name,
                                        const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr);

  /**
   * @brief Callback method for the check_pose_cost service
   * @param request Request object, see the mbf_msgs/srv/CheckPose service
   * definition file.
   * @param response Response object, see the mbf_msgs/srv/CheckPose service
   * definition file.
   */
  void callServiceCheckPoseCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPose::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPose::Response> response);

  /**
   * @brief Callback method for the check_path_cost service
   * @param request Request object, see the mbf_msgs/srv/CheckPath service
   * definition file.
   * @param response Response object, see the mbf_msgs/srv/CheckPath service
   * definition file.
   */
  void callServiceCheckPathCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPath::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPath::Response> response);

  /**
   * @brief Callback method for the make_plan service
   * @param request Empty request object.
   * @param response Empty response object.
   */
  void callServiceClearOcto(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

  //! plugin class loader for recovery behaviors plugins
  pluginlib::ClassLoader<mbf_octo_core::OctoRecovery> recovery_plugin_loader_;
  pluginlib::ClassLoader<mbf_simple_core::SimpleRecovery> simple_recovery_plugin_loader_;

  //! plugin class loader for controller plugins
  pluginlib::ClassLoader<mbf_octo_core::OctoController> controller_plugin_loader_;
  pluginlib::ClassLoader<mbf_simple_core::SimpleController> simple_controller_plugin_loader_;

  //! plugin class loader for planner plugins
  pluginlib::ClassLoader<mbf_octo_core::OctoPlanner> planner_plugin_loader_;
  pluginlib::ClassLoader<mbf_simple_core::SimplePlanner> simple_planner_plugin_loader_;

  //! Shared pointer to the common global octo
  //OctoPtr octo_ptr_;

  //! Service Server to clear the octo
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_octo_srv_;

  //! Service Server for the check_pose_cost service
  rclcpp::Service<mbf_msgs::srv::CheckPose>::SharedPtr check_pose_cost_srv_;

  //! Service Server for the check_path_cost service
  rclcpp::Service<mbf_msgs::srv::CheckPath>::SharedPtr check_path_cost_srv_;

  //! Start/stop octos mutex; concurrent calls to start can lead to segfault
  std::mutex check_octos_mutex_;
};

} /* namespace mbf_octo_nav */

#endif /* MBF_OCTO_NAV__OCTO_NAVIGATION_SERVER_H */
