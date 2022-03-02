/**
 * ROS 2 Node for the microRTPS agent, to add specific functionalities.
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

#ifndef PX4_ROS_COM_MICRORTPS_AGENT_MICRORTPS_AGENT_NODE_HPP_
#define PX4_ROS_COM_MICRORTPS_AGENT_MICRORTPS_AGENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <px4_msgs/msg/vehicle_local_position_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude_stamped.hpp>
#include <px4_msgs/msg/px4_timestamp.hpp>

namespace MicroRTPSAgentNode
{

/**
 * Adds some ROS 2-specific functionalities over those offered by the Agent:
 *  - Publishes timestamped fused local position samples.
 *  - Publishes timestamped fused attitude quaternion samples.
 */
class AgentNode : public rclcpp::Node
{
public:
  AgentNode(std::string node_name, std::string node_namespace, rclcpp::NodeOptions & opts);
  void publish_local_position_sample(
    uint64_t timestamp, uint64_t timestamp_sample,
    bool xy_valid, bool z_valid, bool v_xy_valid, bool v_z_valid,
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az);
  void publish_attitude_sample(
    uint64_t timestamp, uint64_t timestamp_sample,
    float q1, float q2, float q3, float q4);
  void publish_timestamp(uint64_t timestamp);

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeStamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<px4_msgs::msg::PX4Timestamp>::SharedPtr timestamp_pub_;
};

} // namespace MicroRTPSAgentNode

#endif // PX4_ROS_COM_MICRORTPS_AGENT_MICRORTPS_AGENT_NODE_HPP_
