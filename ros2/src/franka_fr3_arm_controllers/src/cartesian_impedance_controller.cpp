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

#include <franka_fr3_arm_controllers/cartesian_impedance_controller.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace franka_fr3_arm_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/velocity");
  }
  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates_();

  if (!interp_initialized_) {
    if (!computeForwardKinematics_(q_, interp_position_, interp_orientation_)) {
      auto tau_d = calculateTauDGains_(q_hold_);
      for (int i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
      }
      return controller_interface::return_type::OK;
    }
    q_goal_last_ = q_;
    have_valid_q_goal_ = true;
    interp_initialized_ = true;
  }

  if (!have_target_.load()) {
    auto tau_d = calculateTauDGains_(q_hold_);
    for (int i = 0; i < kNumJoints; ++i) {
      command_interfaces_[i].set_value(tau_d(i));
    }
    return controller_interface::return_type::OK;
  }

  interpolateTowardTarget_(period);

  Vector7d q_ik;
  if (solveIk_(interp_position_, interp_orientation_, q_, q_ik)) {
    q_goal_last_ = q_ik;
    have_valid_q_goal_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "IK failed; holding last valid joint goal.");
  }

  const Vector7d& q_goal = have_valid_q_goal_ ? q_goal_last_ : q_hold_;
  auto tau_d = calculateTauDGains_(q_goal);
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<bool>("load_gripper", false);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("k_alpha", 0.99);
    auto_declare<std::string>("target_pose_topic", "/max/goal_pose");
    auto_declare<double>("max_linear_velocity", 0.25);
    auto_declare<double>("max_angular_velocity", 1.0);
    auto_declare<int>("ik_max_iter", 150);
    auto_declare<double>("ik_eps", 1e-5);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  load_gripper_ = get_node()->get_parameter("load_gripper").as_bool();

  namespace_prefix_ = get_node()->get_namespace();
  if (namespace_prefix_ == "/" || namespace_prefix_.empty()) {
    namespace_prefix_.clear();
  } else {
    namespace_prefix_ = namespace_prefix_.substr(1) + "_";
  }

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto k_alpha = get_node()->get_parameter("k_alpha").as_double();

  if (!validateGains_(k_gains, "k_gains") || !validateGains_(d_gains, "d_gains")) {
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }
  if (k_alpha < 0.0 || k_alpha > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_alpha should be in the range [0, 1]");
    return CallbackReturn::FAILURE;
  }
  k_alpha_ = k_alpha;

  target_pose_topic_ = get_node()->get_parameter("target_pose_topic").as_string();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  ik_max_iter_ = get_node()->get_parameter("ik_max_iter").as_int();
  ik_eps_ = get_node()->get_parameter("ik_eps").as_double();

  base_link_ = namespace_prefix_ + arm_id_ + "_link0";
  tip_link_ = namespace_prefix_ + arm_id_ + (load_gripper_ ? "_hand_tcp" : "_link8");

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (result.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::FAILURE;
  }
  robot_description_ = result[0].value_to_string();

  if (!initializeKdl_()) {
    return CallbackReturn::FAILURE;
  }

  target_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped& msg) { poseCallback_(msg); });

  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianImpedanceController configured: base=%s tip=%s topic=%s",
              base_link_.c_str(), tip_link_.c_str(), target_pose_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates_();
  q_hold_ = q_;
  q_goal_last_ = q_;
  have_valid_q_goal_ = false;
  have_target_.store(false);
  interp_initialized_ = false;
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::updateJointStates_() {
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

auto CartesianImpedanceController::calculateTauDGains_(const Vector7d& q_goal) -> Vector7d {
  dq_filtered_ = (1 - k_alpha_) * dq_filtered_ + k_alpha_ * dq_;
  Vector7d tau_d_calculated;
  tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  return tau_d_calculated;
}

bool CartesianImpedanceController::validateGains_(const std::vector<double>& gains,
                                                  const std::string& gains_name) {
  if (gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s parameter not set", gains_name.c_str());
    return false;
  }
  if (gains.size() != static_cast<uint>(kNumJoints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s should be of size %d but is of size %ld",
                 gains_name.c_str(), kNumJoints, gains.size());
    return false;
  }
  return true;
}

bool CartesianImpedanceController::initializeKdl_() {
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description_, tree)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to parse KDL tree from robot_description.");
    return false;
  }
  if (!tree.getChain(base_link_, tip_link_, kdl_chain_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to extract KDL chain from '%s' to '%s'.",
                 base_link_.c_str(), tip_link_.c_str());
    return false;
  }
  if (kdl_chain_.getNrOfJoints() != static_cast<unsigned>(kNumJoints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "KDL chain has %u joints, expected %d. Check base/tip link names.",
                 kdl_chain_.getNrOfJoints(), kNumJoints);
    return false;
  }
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, ik_eps_, ik_max_iter_);
  return true;
}

bool CartesianImpedanceController::computeForwardKinematics_(const Vector7d& q,
                                                             Eigen::Vector3d& position,
                                                             Eigen::Quaterniond& orientation) {
  KDL::JntArray q_kdl(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    q_kdl(i) = q(i);
  }
  KDL::Frame frame;
  if (fk_solver_->JntToCart(q_kdl, frame) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Forward kinematics failed.");
    return false;
  }
  position.x() = frame.p.x();
  position.y() = frame.p.y();
  position.z() = frame.p.z();
  double qx, qy, qz, qw;
  frame.M.GetQuaternion(qx, qy, qz, qw);
  orientation = Eigen::Quaterniond(qw, qx, qy, qz);
  orientation.normalize();
  return true;
}

void CartesianImpedanceController::interpolateTowardTarget_(const rclcpp::Duration& period) {
  Eigen::Vector3d target_position;
  Eigen::Quaterniond target_orientation;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position = target_position_;
    target_orientation = target_orientation_;
  }

  const double dt = std::max(period.seconds(), 1e-6);

  Eigen::Vector3d delta = target_position - interp_position_;
  const double max_step_lin = max_linear_velocity_ * dt;
  if (delta.norm() > max_step_lin) {
    interp_position_ += delta.normalized() * max_step_lin;
  } else {
    interp_position_ = target_position;
  }

  if (interp_orientation_.coeffs().dot(target_orientation.coeffs()) < 0.0) {
    target_orientation.coeffs() *= -1.0;
  }
  Eigen::Quaterniond q_err = target_orientation * interp_orientation_.conjugate();
  q_err.normalize();
  Eigen::AngleAxisd aa(q_err);
  double angle = aa.angle();
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  const double max_step_ang = max_angular_velocity_ * dt;
  if (std::abs(angle) > max_step_ang) {
    double ratio = max_step_ang / std::abs(angle);
    Eigen::AngleAxisd aa_step(angle * ratio, aa.axis());
    interp_orientation_ = Eigen::Quaterniond(aa_step) * interp_orientation_;
    interp_orientation_.normalize();
  } else {
    interp_orientation_ = target_orientation;
  }
}

bool CartesianImpedanceController::solveIk_(const Eigen::Vector3d& position,
                                            const Eigen::Quaterniond& orientation,
                                            const Vector7d& q_seed,
                                            Vector7d& q_out) {
  KDL::JntArray q_seed_kdl(kNumJoints);
  KDL::JntArray q_out_kdl(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) {
    q_seed_kdl(i) = q_seed(i);
  }
  KDL::Rotation rot = KDL::Rotation::Quaternion(orientation.x(), orientation.y(), orientation.z(),
                                                orientation.w());
  KDL::Frame target(rot, KDL::Vector(position.x(), position.y(), position.z()));
  if (ik_solver_->CartToJnt(q_seed_kdl, target, q_out_kdl) < 0) {
    return false;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    q_out(i) = q_out_kdl(i);
  }
  return true;
}

void CartesianImpedanceController::poseCallback_(const geometry_msgs::msg::PoseStamped& msg) {
  if (!msg.header.frame_id.empty() && msg.header.frame_id != base_link_ &&
      msg.header.frame_id != arm_id_ + "_link0") {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Received pose with frame_id='%s' but controller expects base frame '%s'. "
        "Pose is used as-is (no TF lookup).",
        msg.header.frame_id.c_str(), base_link_.c_str());
  }
  Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                       msg.pose.orientation.z);
  if (q.norm() < 1e-6) {
    RCLCPP_WARN(get_node()->get_logger(), "Received zero-norm quaternion; ignoring.");
    return;
  }
  q.normalize();
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_position_ = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y,
                                       msg.pose.position.z);
    target_orientation_ = q;
  }
  have_target_.store(true);
}

}  // namespace franka_fr3_arm_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_fr3_arm_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
