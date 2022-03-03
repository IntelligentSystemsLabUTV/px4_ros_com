/**
 * ROS 2 Node for the microRTPS Agent, to add specific functionalities.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * March 1, 2022
 */
/**
 * Copyright © 2022 Intelligent Systems Lab
 */
/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
 */

#include "microRTPS_agent_node.hpp"

#include <rmw/qos_profiles.h>

using namespace px4_msgs::msg;

namespace MicroRTPSAgentNode
{

/**
 * @brief Creates a new AgentNode.
 *
 * @param node_name New node name.
 */
AgentNode::AgentNode(std::string node_name, rclcpp::NodeOptions & opts)
: Node(node_name, opts)
{
  // Samples must be published in a timely manner, so use sensor_data QoS
  rmw_qos_profile_t samples_qos = rmw_qos_profile_sensor_data;
  samples_qos.depth = 1;

  // Initialize publishers
  local_pos_pub_ = this->create_publisher<VehicleLocalPositionStamped>(
    "~/vehicle_local_position_stamped/out",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(samples_qos)));
  attitude_pub_ = this->create_publisher<VehicleAttitudeStamped>(
    "~/vehicle_attitude_stamped/out",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(samples_qos)));
  timestamp_pub_ = this->create_publisher<PX4Timestamp>(
    "~/px4_timestamp/out",
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(samples_qos)));

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Publishes a new local position sample.
 *
 * @param timestamp Message timestamp.
 * @param timestamp_sample Sample acquisition timestamp.
 * @param xy_valid (X, Y) validity flag.
 * @param z_valid Z validity flag.
 * @param v_xy_valid (Vx, Vy) validity flag.
 * @param v_z_valid Vz validity flag.
 * @param x X position.
 * @param y Y position.
 * @param z Z position.
 * @param vx X velocity.
 * @param vy Y velocity.
 * @param vz Z velocity.
 * @param ax X acceleration.
 * @param ay Y acceleration.
 * @param az Z acceleration.
 */
void AgentNode::publish_local_position_sample(
  uint64_t timestamp, uint64_t timestamp_sample,
  bool xy_valid, bool z_valid, bool v_xy_valid, bool v_z_valid,
  float x, float y, float z,
  float vx, float vy, float vz,
  float ax, float ay, float az)
{
  // Create a new message with a timestamp and send it
  VehicleLocalPositionStamped pos_msg{};
  pos_msg.set__timestamp(timestamp);
  pos_msg.set__timestamp_sample(timestamp_sample);
  pos_msg.set__xy_valid(xy_valid);
  pos_msg.set__z_valid(z_valid);
  pos_msg.set__v_xy_valid(v_xy_valid);
  pos_msg.set__v_z_valid(v_z_valid);
  pos_msg.set__x(x);
  pos_msg.set__y(y);
  pos_msg.set__z(z);
  pos_msg.set__vx(vx);
  pos_msg.set__vy(vy);
  pos_msg.set__vz(vz);
  pos_msg.set__ax(ax);
  pos_msg.set__ay(ay);
  pos_msg.set__az(az);
  pos_msg.header.set__frame_id("world");
  pos_msg.header.set__stamp(get_clock()->now());

  local_pos_pub_->publish(pos_msg);
}

/**
 * @brief Publishes a new attitude sample.
 *
 * @param timestamp Message timestamp.
 * @param timestamp_sample Sample acquisition timestamp.
 * @param q1 Quaternion w component.
 * @param q2 Quaternion i component.
 * @param q3 Quaternion j component.
 * @param q4 Quaternion k component.
 */
void AgentNode::publish_attitude_sample(
  uint64_t timestamp, uint64_t timestamp_sample,
  float q1, float q2, float q3, float q4)
{
  // Create a new message with a timestamp and send it
  VehicleAttitudeStamped attitude_msg{};
  attitude_msg.set__timestamp(timestamp);
  attitude_msg.set__timestamp_sample(timestamp_sample);
  attitude_msg.q[0] = q1;
  attitude_msg.q[1] = q2;
  attitude_msg.q[2] = q3;
  attitude_msg.q[3] = q4;
  attitude_msg.header.set__frame_id("world");
  attitude_msg.header.set__stamp(get_clock()->now());

  attitude_pub_->publish(attitude_msg);
}

/**
 * @brief Publishes a new valid PX4 timestamp.
 *
 * @param timestamp New timestamp to be published.
 */
void AgentNode::publish_timestamp(uint64_t timestamp)
{
  PX4Timestamp clock_msg{};
  clock_msg.set__timestamp(timestamp);
  timestamp_pub_->publish(clock_msg);
}

} // namespace MicroRTPSAgentNode
