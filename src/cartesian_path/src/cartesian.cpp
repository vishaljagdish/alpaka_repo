#include <cartesian.hpp>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "ur_manipulator";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("alpaka_welding_cobot_moveit");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP);
  move_group_arm.setPoseReferenceFrame("base");
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);

  const moveit::core::JointModelGroup* joint_model_group_arm = 
    current_state_arm->getJointModelGroup(PLANNING_GROUP);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

// Set velocity scaling factor (0.1 = 10% of max speed)
  move_group_arm.setMaxVelocityScalingFactor(0.1);  // Adjust this value (0.0-1.0)
  //go home
  RCLCPP_INFO(LOGGER, "going home");
  joint_group_positions_arm[0] = 1.0411;
  joint_group_positions_arm[1] = 0.5191;
  joint_group_positions_arm[2] = -1.5423; 
  joint_group_positions_arm[3] = -0.1065;
  joint_group_positions_arm[4] = 0.8487;
  joint_group_positions_arm[5] = 0.5561;

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = -0.002037;
  target_pose1.position.y = -1.097061;
  target_pose1.position.z = 0.697058;
  target_pose1.orientation.x = 0.356231; 
  target_pose1.orientation.y = 0.189883;
  target_pose1.orientation.z = 0.093213;
  target_pose1.orientation.w = 0.910140;

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = -0.135309;
  target_pose2.position.y = -1.232684;
  target_pose2.position.z = 0.698240;
  target_pose2.orientation.x = 0.356219; 
  target_pose2.orientation.y = 0.189880;
  target_pose2.orientation.z = 0.093212;
  target_pose2.orientation.w = 0.910146;  

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  approach_waypoints.push_back(target_pose1);
  approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
  RCLCPP_INFO(LOGGER, "move");
  move_group_arm.execute(trajectory_approach);

  rclcpp::shutdown();
  return 0;

}