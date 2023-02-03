/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <string>
#include <neo_srvs2/srv/reset_omni_wheels.hpp>

#include "../include/OmniKinematics.h"
#include "../include/VelocitySolver.h"

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <boost/array.hpp>
#include <neo_srvs2/srv/lock_platform.hpp>
#include <neo_srvs2/srv/unlock_platform.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class NeoOmniDriveNode : public rclcpp::Node
{
public:
  NeoOmniDriveNode()
  : Node("neo_omnidrive_node")
  {
    // Declaring parameters
    this->declare_parameter<double>("control_rate", 50.0);
    this->declare_parameter<bool>("broadcast_tf", true);
    this->declare_parameter<int>("num_wheels", 3);
    this->declare_parameter<double>("wheel_radius", 0.0);
    this->declare_parameter<double>("cmd_timeout", 0.0);
    this->declare_parameter<int>("homeing_button", 0);
    this->declare_parameter<int>("steer_reset_button", 0);
    this->declare_parameter<double>("wheel_lever_arm", 5.0);
    this->declare_parameter<double>("zero_vel_threshold", 0.0);
    this->declare_parameter<double>("small_vel_threshold", 0.0);
    this->declare_parameter<double>("steer_hysteresis", 0.0);
    this->declare_parameter<double>("steer_hysteresis_dynamic", 0.0);
    this->declare_parameter<bool>("reset_odom", false);

    if (!this->get_parameter("num_wheels", m_num_wheels)) {
      throw std::logic_error("missing num_wheels param");
    }

    if (this->get_parameter("wheel_radius", m_wheel_radius) == 0.0) {
      throw std::logic_error("missing wheel_radius param");
    }

    if (this->get_parameter("wheel_lever_arm", m_wheel_lever_arm) == 0.0) {
      throw std::logic_error("missing wheel_lever_arm param");
    }
    this->get_parameter_or("broadcast_tf", m_broadcast_tf, true);
    this->get_parameter_or("cmd_timeout", m_cmd_timeout, 0.1);
    this->get_parameter_or("homeing_button", m_homeing_button, 0);
    this->get_parameter_or("steer_reset_button", m_steer_reset_button, 1);
    this->get_parameter_or("control_rate", m_control_rate, 50.0);

    if (m_num_wheels < 1) {
      throw std::logic_error("invalid num_wheels param");
    }
    m_wheels.resize(m_num_wheels);

    for (int i = 0; i < m_num_wheels; ++i) {
      m_wheels[i].lever_arm = m_wheel_lever_arm;
      this->declare_parameter<std::string>("drive" + std::to_string(i) + ".joint_name", "random");
      this->declare_parameter<std::string>("steer" + std::to_string(i) + ".joint_name", "random");
      this->declare_parameter<double>("steer" + std::to_string(i) + ".center_pos_x", 1.0);
      this->declare_parameter<double>("steer" + std::to_string(i) + ".center_pos_y", 1.0);
      this->declare_parameter<double>("steer" + std::to_string(i) + ".home_angle", 1.0);

      if (!this->get_parameter(
          "drive" + std::to_string(i) + ".joint_name",
          m_wheels[i].drive_joint_name))
      {
        throw std::logic_error("joint_name param missing for drive motor" + std::to_string(i));
      }
      if (!this->get_parameter(
          "steer" + std::to_string(i) + ".joint_name",
          m_wheels[i].steer_joint_name))
      {
        throw std::logic_error("joint_name param missing for steering motor" + std::to_string(i));
      }
      if (!this->get_parameter(
          "steer" + std::to_string(i) + ".center_pos_x",
          m_wheels[i].center_pos_x))
      {
        throw std::logic_error("center_pos_x param missing for steering motor" + std::to_string(i));
      }
      if (!this->get_parameter(
          "steer" + std::to_string(i) + ".center_pos_y",
          m_wheels[i].center_pos_y))
      {
        throw std::logic_error("center_pos_y param missing for steering motor" + std::to_string(i));
      }
      if (!this->get_parameter(
          "steer" + std::to_string(i) + ".home_angle",
          m_wheels[i].home_angle))
      {
        throw std::logic_error("home_angle param missing for steering motor" + std::to_string(i));
      }
      m_wheels[i].home_angle = M_PI * m_wheels[i].home_angle / 180.;
      m_wheels[i].set_wheel_angle(0);
    }

    m_pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    m_pub_joint_trajectory = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "drives/joint_trajectory", 1);
    m_tf_odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_sub_cmd_vel =
      this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      std::bind(&NeoOmniDriveNode::cmd_vel_callback, this, _1));
    m_sub_joint_state =
      this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&NeoOmniDriveNode::joint_state_callback, this, _1));
    m_sub_joy =
      this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&NeoOmniDriveNode::joy_callback, this, _1));

    this->m_srv_lock_platform =
      this->create_service<neo_srvs2::srv::LockPlatform>(
      "lock_platform", std::bind(
        &NeoOmniDriveNode::lock_platform, this, _1,
        _2));
    this->m_srv_unlock_platform =
      this->create_service<neo_srvs2::srv::UnlockPlatform>(
      "unlock_platform", std::bind(
        &NeoOmniDriveNode::unlock_platform, this, _1,
        _2));
    this->m_srv_reset_omni_wheels =
      this->create_service<neo_srvs2::srv::ResetOmniWheels>(
      "reset_omni_wheels", std::bind(
        &NeoOmniDriveNode::reset_omni_wheels, this, _1,
        _2));

    m_kinematics = std::make_shared<OmniKinematics>(m_num_wheels);
    m_velocity_solver = std::make_shared<VelocitySolver>(m_num_wheels);

    this->get_parameter_or("zero_vel_threshold", m_kinematics->zero_vel_threshold, 0.005);
    this->get_parameter_or("small_vel_threshold", m_kinematics->small_vel_threshold, 0.03);
    this->get_parameter_or("steer_hysteresis", m_kinematics->steer_hysteresis, 30.0);
    this->get_parameter_or("steer_hysteresis_dynamic", m_kinematics->steer_hysteresis_dynamic, 5.0);
    this->get_parameter("reset_odom", m_reset_odom);

    m_kinematics->steer_hysteresis = M_PI * m_kinematics->steer_hysteresis / 180;
    m_kinematics->steer_hysteresis_dynamic = M_PI * m_kinematics->steer_hysteresis_dynamic / 180;
    m_kinematics->initialize(m_wheels);
  }

  void control_step()
  {
    std::lock_guard<std::mutex> lock(m_node_mutex);

    auto now = rclcpp::Clock().now();

    // check for input timeout
    if ((now - m_last_cmd_time).seconds() > m_cmd_timeout) {
      if (!is_cmd_timeout && !m_last_cmd_time.seconds() == 0 &&
        (m_last_cmd_vel.linear.x != 0 || m_last_cmd_vel.linear.y != 0 ||
        m_last_cmd_vel.angular.z != 0))
      {
        RCLCPP_WARN(this->get_logger(), "cmd_vel input timeout! Stopping now.");
      }
      // reset values to zero
      // m_last_cmd_vel = geometry_msgs::msg::Twist();
      is_cmd_timeout = true;
    } else {
      is_cmd_timeout = false;
    }

    // check if platform is locked
    if (is_locked) {
      m_last_cmd_vel.linear.x = std::numeric_limits<double>::min();  // use zero cmd_vel
      m_last_cmd_vel.linear.y = std::numeric_limits<double>::min();
      m_last_cmd_vel.angular.z = std::numeric_limits<double>::min();
    }

    // compute new wheel angles and velocities
    auto cmd_wheels = m_kinematics->compute(
      m_wheels, m_last_cmd_vel.linear.x,
      m_last_cmd_vel.linear.y, m_last_cmd_vel.angular.z);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;

    joint_trajectory.header.stamp = now;

    trajectory_msgs::msg::JointTrajectoryPoint point;

    for (const auto & wheel : cmd_wheels) {
      joint_trajectory.joint_names.push_back(wheel.drive_joint_name);
      joint_trajectory.joint_names.push_back(wheel.steer_joint_name);
      {
        const double drive_rot_vel = wheel.wheel_vel / m_wheel_radius;
        point.positions.push_back(0);
        point.velocities.push_back(drive_rot_vel);
      }
      {
        point.positions.push_back(wheel.wheel_angle);
        point.velocities.push_back(0);
      }
    }
    joint_trajectory.points.push_back(point);
    m_pub_joint_trajectory->publish(joint_trajectory);
  }

  inline double get_control_rate()
  {
    return m_control_rate;
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    m_last_cmd_time = rclcpp::Clock().now();
    m_last_cmd_vel.linear.x = twist->linear.x;
    m_last_cmd_vel.linear.y = twist->linear.y;
    m_last_cmd_vel.angular.z = twist->angular.z;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
  {
    std::lock_guard<std::mutex> lock(m_node_mutex);
    geometry_msgs::msg::Quaternion quat_msg1;
    const size_t num_joints = joint_state->name.size();
    if (joint_state->position.size() < num_joints) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "joint_state->position.size() < num_joints");
      return;
    }
    if (joint_state->velocity.size() < num_joints) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "joint_state->velocity.size() < num_joints");
      return;
    }
    // update wheels with new data
    for (size_t i = 0; i < num_joints; ++i) {
      for (auto & wheel : m_wheels) {
        if (joint_state->name[i] == wheel.drive_joint_name) {
          // update wheel velocity
          wheel.wheel_vel = -1 * joint_state->velocity[i] * m_wheel_radius;
        }
        if (joint_state->name[i] == wheel.steer_joint_name) {
          // update wheel steering angle and wheel position (due to lever arm)
          wheel.set_wheel_angle(joint_state->position[i] + M_PI);
        }
      }
    }

    // compute velocities
    m_velocity_solver->solve(m_wheels);

    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.header.stamp.sec = joint_state->header.stamp.sec;
    odometry.header.stamp.nanosec = joint_state->header.stamp.nanosec;
    odometry.child_frame_id = "base_link";

    // integrate odometry (using second order midpoint method)
    if (m_curr_odom_time.sec != 0.000) {
      // Convert nanoseconds to seconds add them with the seconds
      const double dt =
        (joint_state->header.stamp.sec + (joint_state->header.stamp.nanosec) *
        (1.0 / 1000000000.0)) -
        (m_curr_odom_time.sec + (m_curr_odom_time.nanosec) * (1.0 / 1000000000.0) );

      // check for valid delta time
      if (dt > 0 && dt < 1) {
        // compute second order midpoint velocities
        const double vel_x_mid = 0.5 *
          (m_velocity_solver->move_vel_x + m_curr_odom_twist.linear.x);
        const double vel_y_mid = 0.5 *
          (m_velocity_solver->move_vel_y + m_curr_odom_twist.linear.y);
        const double yawrate_mid = 0.5 *
          (m_velocity_solver->move_yawrate + m_curr_odom_twist.angular.z);

        // compute midpoint yaw angle
        const double yaw_mid = m_curr_odom_yaw + 0.5 * yawrate_mid * dt;

        // integrate position using midpoint velocities and yaw angle
        m_curr_odom_x += vel_x_mid * dt * cos(yaw_mid) + vel_y_mid * dt * -sin(yaw_mid);
        m_curr_odom_y += vel_x_mid * dt * sin(yaw_mid) + vel_y_mid * dt * cos(yaw_mid);

        // integrate yaw angle using midpoint yawrate
        m_curr_odom_yaw += yawrate_mid * dt;
      } else {
        RCLCPP_WARN(this->get_logger(), "invalid joint state delta time");
      }
    }
    m_curr_odom_time = joint_state->header.stamp;

    // assign odometry pose
    odometry.pose.pose.position.x = m_curr_odom_x;
    odometry.pose.pose.position.y = m_curr_odom_y;
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, m_curr_odom_yaw);
    quat_msg1 = tf2::toMsg(q);

    odometry.pose.pose.orientation.x = quat_msg1.x;
    odometry.pose.pose.orientation.y = quat_msg1.y;
    odometry.pose.pose.orientation.z = quat_msg1.z;
    odometry.pose.pose.orientation.w = quat_msg1.w;

    // assign odometry twist
    m_curr_odom_twist.linear.x = m_velocity_solver->move_vel_x;
    m_curr_odom_twist.linear.y = m_velocity_solver->move_vel_y;
    m_curr_odom_twist.linear.z = 0;
    m_curr_odom_twist.angular.x = 0;
    m_curr_odom_twist.angular.y = 0;
    m_curr_odom_twist.angular.z = m_velocity_solver->move_yawrate;
    odometry.twist.twist = m_curr_odom_twist;

    // assign bogus covariance values
    odometry.pose.covariance.fill(0.1);
    odometry.twist.covariance.fill(0.1);

    // publish odometry
    m_pub_odometry->publish(odometry);

    // broadcast odometry
    if (m_broadcast_tf) {
      // compose and publish transform for tf package
      geometry_msgs::msg::TransformStamped odom_tf;
      geometry_msgs::msg::Quaternion quat_msg;
      // compose header
      std::string robot_namespace(this->get_namespace());
      odom_tf.header.stamp = joint_state->header.stamp;
      if (robot_namespace != "/") {
        robot_namespace.erase(
          std::remove(robot_namespace.begin(),
          robot_namespace.end(),
          '/'), robot_namespace.end());
        odom_tf.header.frame_id = robot_namespace + "odom";
        odom_tf.child_frame_id = robot_namespace + "base_link";
      } else {
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
      }
      // compose data container
      odom_tf.transform.translation.x = m_curr_odom_x;
      odom_tf.transform.translation.y = m_curr_odom_y;
      odom_tf.transform.translation.z = 0;
      odom_tf.transform.rotation.x = quat_msg1.x;
      odom_tf.transform.rotation.y = quat_msg1.y;
      odom_tf.transform.rotation.z = quat_msg1.z;
      odom_tf.transform.rotation.w = quat_msg1.w;
      // publish the transform

      m_tf_odom_broadcaster->sendTransform(odom_tf);
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    std::lock_guard<std::mutex> lock(m_node_mutex);

    if (m_homeing_button >= 0 && static_cast<int>(joy->buttons.size()) > m_homeing_button) {
      if (joy->buttons[m_homeing_button]) {
        ::usleep(500 * 1000);         // wait for homeing to start before sending new commands
        m_kinematics->initialize(m_wheels);   // reset stop position to home
        if (m_reset_odom) {
          m_curr_odom_x = std::numeric_limits<double>::min();
          m_curr_odom_y = std::numeric_limits<double>::min();
          m_curr_odom_yaw = std::numeric_limits<double>::min();
        }
      }
    }
    if (m_steer_reset_button >= 0 && static_cast<int>(joy->buttons.size()) > m_steer_reset_button) {
      if (joy->buttons[m_steer_reset_button]) {
        m_kinematics->initialize(m_wheels);   // reset stop position to home
      }
    }
  }

  bool lock_platform(
    const std::shared_ptr<neo_srvs2::srv::LockPlatform::Request>/*request*/,
    std::shared_ptr<neo_srvs2::srv::LockPlatform::Response> response)
  {
    if (fabs(m_last_cmd_vel.linear.x) < 0.1 &&
      fabs(m_last_cmd_vel.linear.y) < 0.1 &&
      fabs(m_last_cmd_vel.angular.z) < 0.1)
    {
      is_locked = true;
      response->success = true;
      return true;
    }
    response->success = false;
    return false;
  }

  bool unlock_platform(
    std::shared_ptr<neo_srvs2::srv::UnlockPlatform::Request>/*request*/,
    std::shared_ptr<neo_srvs2::srv::UnlockPlatform::Response> response)
  {
    is_locked = false;
    response->success = true;
    return true;
  }

  bool reset_omni_wheels(
    std::shared_ptr<neo_srvs2::srv::ResetOmniWheels::Request> request,
    std::shared_ptr<neo_srvs2::srv::ResetOmniWheels::Response> response)
  {
    if (m_num_wheels >= static_cast<int>(request->steer_angles_rad.size())) {
      response->success = true;
      for (size_t i = 0; i < request->steer_angles_rad.size(); ++i) {
        m_kinematics->last_stop_angle[i] = request->steer_angles_rad[i];
        response->success = response->success && !m_kinematics->is_driving[i];
      }
      if (m_reset_odom) {
        m_curr_odom_x = std::numeric_limits<double>::min();
        m_curr_odom_y = std::numeric_limits<double>::min();
        m_curr_odom_yaw = std::numeric_limits<double>::min();
      }
      return true;
    }
    response->success = false;
    return false;
  }

private:
  std::mutex m_node_mutex;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odometry;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_pub_joint_trajectory;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_cmd_vel;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_sub_joint_state;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub_joy;

  rclcpp::Service<neo_srvs2::srv::LockPlatform>::SharedPtr m_srv_lock_platform;
  rclcpp::Service<neo_srvs2::srv::UnlockPlatform>::SharedPtr m_srv_unlock_platform;
  rclcpp::Service<neo_srvs2::srv::ResetOmniWheels>::SharedPtr m_srv_reset_omni_wheels;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_odom_broadcaster;

  bool m_broadcast_tf = false;
  int m_num_wheels = 0;
  int m_homeing_button = -1;
  int m_steer_reset_button = -1;
  double m_wheel_radius = 0;
  double m_wheel_lever_arm = 0;
  double m_cmd_timeout = 0;
  double m_control_rate = 0;

  std::vector<OmniWheel> m_wheels;

  std::shared_ptr<OmniKinematics> m_kinematics;
  std::shared_ptr<VelocitySolver> m_velocity_solver;

  rclcpp::Time m_last_cmd_time;
  geometry_msgs::msg::Twist m_last_cmd_vel;
  bool is_cmd_timeout = false;
  bool is_locked = false;
  bool m_reset_odom = false;

  std_msgs::msg::Header::_stamp_type m_curr_odom_time;
  double m_curr_odom_x = std::numeric_limits<double>::min();
  double m_curr_odom_y = std::numeric_limits<double>::min();
  double m_curr_odom_yaw = std::numeric_limits<double>::min();
  geometry_msgs::msg::Twist m_curr_odom_twist;
};

int main(int argc, char ** argv)
{
  // initialize ROS
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoOmniDriveNode>();
  double control_rate = nh->get_control_rate();   // [1/s]
  rclcpp::Rate loop_rate(control_rate);

  try {
    while (rclcpp::ok()) {
      rclcpp::spin_some(nh);
      nh->control_step();
      loop_rate.sleep();
    }
  } catch (std::exception & ex) {
    std::cout << "NeoOmniDriveNode: " << ex.what();
  }
  return 0;
}
