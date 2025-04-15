#include <cartesian.hpp>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "ur_manipulator";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("alpaka_welding_cobot_moveit");

double deg2rad(double joint_angle){
  return joint_angle * (M_PI/180.0);
}

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
  joint_group_positions_arm[0] = deg2rad(45.05);
  joint_group_positions_arm[1] = deg2rad(31.22);
  joint_group_positions_arm[2] = deg2rad(-89.05); 
  joint_group_positions_arm[3] = deg2rad(-5.75);
  joint_group_positions_arm[4] = deg2rad(67.31);
  joint_group_positions_arm[5] = deg2rad(25.79);

  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  move_group_arm.setPlanningTime(5.0);   
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = -0.658431;
  target_pose1.position.y = -1.030103;
  target_pose1.position.z = 0.685928;
  target_pose1.orientation.x = 0.293958; 
  target_pose1.orientation.y = 0.025905;
  target_pose1.orientation.z = -0.116543;
  target_pose1.orientation.w = 0.948333;

  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = -0.479880;
  target_pose2.position.y = -1.114142;
  target_pose2.position.z = 0.682587;
  target_pose2.orientation.x = 0.293955; 
  target_pose2.orientation.y = 0.025916;
  target_pose2.orientation.z = -0.116534;
  target_pose2.orientation.w = 0.948335;  

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  approach_waypoints.push_back(target_pose1);
  approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
  RCLCPP_INFO(LOGGER, "move");  
  move_group_arm.execute(trajectory_approach);
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(LOGGER, "going home again ;)");
  move_group_arm.execute(my_plan_arm);

  rclcpp::shutdown();
  return 0;

}