#ifndef COMESY_MOVEIT_DRIVER_HPP_
#define COMESY_MOVEIT_DRIVER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MlReceiverMoveitNode : public rclcpp::Node
{
public:
    MlReceiverMoveitNode(const rclcpp::NodeOptions options);

private:
    void onPointReceiveCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_subscription_;
    geometry_msgs::msg::Quaternion multiplyQuaternion(const geometry_msgs::msg::Quaternion q1,  const geometry_msgs::msg::Quaternion q2);
    geometry_msgs::msg::Quaternion inversQuaternion(geometry_msgs::msg::Quaternion q);
    geometry_msgs::msg::Quaternion normalizQuaternion(geometry_msgs::msg::Quaternion q);
    geometry_msgs::msg::Quaternion createRotationX180();
    moveit::planning_interface::MoveGroupInterface move_group_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    moveit::core::JointModelGroup* joint_model_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif  // ML_RECEIVER_NODE_DRIVER_HPP_
