// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Eigen>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_fr3_arm_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string arm_id_;
  std::string namespace_prefix_;
  std::string robot_description_;
  bool load_gripper_{false};
  std::string tip_link_;
  std::string base_link_;

  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double k_alpha_{0.99};

  Vector7d q_hold_;
  Vector7d q_goal_last_;
  bool have_valid_q_goal_{false};

  double max_linear_velocity_{0.25};
  double max_angular_velocity_{1.0};
  int ik_max_iter_{150};
  double ik_eps_{1e-5};

  std::string target_pose_topic_{"/max/goal_pose"};

  // Target pose published by the user (latest only, mutex-protected).
  std::mutex target_mutex_;
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};
  std::atomic<bool> have_target_{false};

  // Interpolated pose sent to IK each cycle.
  Eigen::Vector3d interp_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond interp_orientation_{Eigen::Quaterniond::Identity()};
  bool interp_initialized_{false};

  // KDL
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

  void updateJointStates_();
  Vector7d calculateTauDGains_(const Vector7d& q_goal);
  bool validateGains_(const std::vector<double>& gains, const std::string& gains_name);
  bool initializeKdl_();
  bool computeForwardKinematics_(const Vector7d& q,
                                 Eigen::Vector3d& position,
                                 Eigen::Quaterniond& orientation);
  void interpolateTowardTarget_(const rclcpp::Duration& period);
  bool solveIk_(const Eigen::Vector3d& position,
                const Eigen::Quaterniond& orientation,
                const Vector7d& q_seed,
                Vector7d& q_out);
  void poseCallback_(const geometry_msgs::msg::PoseStamped& msg);
};

}  // namespace franka_fr3_arm_controllers
