#include <ml_node_receiver_driver.hpp>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "ur_manipulator";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("alpaka_welding_cobot_moveit");

MlReceiverMoveitNode::MlReceiverMoveitNode(const rclcpp::NodeOptions options):rclcpp::Node("ml_receiver", options),
    move_group_(std::make_shared<rclcpp::Node>(this->get_name()), PLANNING_GROUP),
    robot_model_loader_(std::make_shared<rclcpp::Node>(this->get_name()), "robot_description"),
    robot_model_(robot_model_loader_.getModel()),
    robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    joint_model_group_ = robot_model_->getJointModelGroup(PLANNING_GROUP);  
    //setup subs and timer ...
    point_subscription_= this->create_subscription<geometry_msgs::msg::Point>("/points", 1, std::bind(&MlReceiverMoveitNode::onPointReceiveCallback, this, _1));
    move_group_.setPoseReferenceFrame("base");
    move_group_.setPlanningTime(5);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_.setPlannerId("LIN");
    move_group_.setEndEffector("tool0");
    //move_group_.setGoalOrientationTolerance(0.001);
    //move_group_.setGoalPositionTolerance(0.0001);

    
    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Planner: %s", move_group_.getPlannerId().c_str());
    RCLCPP_INFO(LOGGER, "Planning Pipeline: %s", move_group_.getPlanningPipelineId().c_str());
    RCLCPP_INFO(LOGGER, "PlanningTime: %f", move_group_.getPlanningTime());
    RCLCPP_INFO(LOGGER, "EndEffector: %s", move_group_.getEndEffector().c_str());
    RCLCPP_INFO(LOGGER, "EndEffectorLink: %s", move_group_.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "PositionTolerance: %f", move_group_.getGoalPositionTolerance());
    RCLCPP_INFO(LOGGER, "OrientationTolerance: %f", move_group_.getGoalOrientationTolerance());
}

void MlReceiverMoveitNode::onPointReceiveCallback(const geometry_msgs::msg::Point::SharedPtr msg){


    RCLCPP_INFO(LOGGER, "Kamera Target Pose");
    RCLCPP_INFO(LOGGER, "P_x: %f", msg->x);
    RCLCPP_INFO(LOGGER, "P_y: %f", msg->y);
    RCLCPP_INFO(LOGGER, "P_z: %f", msg->z);


    geometry_msgs::msg::TransformStamped transform_tcp_tool0;
    try {
        transform_tcp_tool0 = tf_buffer_.lookupTransform("tcp_endpoint_link","tool0", rclcpp::Time(0),tf2::Duration(std::chrono::seconds(50)));
    } catch (const tf2::TransformException & ex){
        RCLCPP_WARN(get_logger(),"Could not find transform tcp_endpoint_link-tool0.");
        RCLCPP_WARN(get_logger(),"%s",ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose = move_group_interface.getCurrentPose().pose;

    // Print the current pose of the end effector
    RCLCPP_INFO(node->get_logger(), "Current pose: X:%f Y:%f Z:%f x:%f y:%f z:%f w:%f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);

    RCLCPP_INFO(LOGGER, "TF Tcp Tool0");
    RCLCPP_INFO(LOGGER, "P_x: %f", transform_tcp_tool0.transform.translation.x);
    RCLCPP_INFO(LOGGER, "P_y: %f", transform_tcp_tool0.transform.translation.y);
    RCLCPP_INFO(LOGGER, "P_z: %f", transform_tcp_tool0.transform.translation.z);
    RCLCPP_INFO(LOGGER, "O_x: %f", transform_tcp_tool0.transform.rotation.x);
    RCLCPP_INFO(LOGGER, "O_y: %f", transform_tcp_tool0.transform.rotation.y);
    RCLCPP_INFO(LOGGER, "O_z: %f", transform_tcp_tool0.transform.rotation.z);
    RCLCPP_INFO(LOGGER, "O_w: %f", transform_tcp_tool0.transform.rotation.w);

    geometry_msgs::msg::TransformStamped transform_camera_base;
    try {
        transform_camera_base = tf_buffer_.lookupTransform("base","sensor_d435i_color_optical_frame", tf2::TimePointZero,tf2::Duration(std::chrono::seconds(50)));
    } catch (const tf2::TransformException & ex){
        RCLCPP_WARN(get_logger(),"Could not find transform sensor_d435i_color_optical_frame-world.");
        RCLCPP_WARN(get_logger(),"%s",ex.what());
        return;
    }

    RCLCPP_INFO(LOGGER, "TF Base Kamera");
    RCLCPP_INFO(LOGGER, "P_x: %f", transform_camera_base.transform.translation.x);
    RCLCPP_INFO(LOGGER, "P_y: %f", transform_camera_base.transform.translation.y);
    RCLCPP_INFO(LOGGER, "P_z: %f", transform_camera_base.transform.translation.z);
    RCLCPP_INFO(LOGGER, "O_x: %f", transform_camera_base.transform.rotation.x);
    RCLCPP_INFO(LOGGER, "O_y: %f", transform_camera_base.transform.rotation.y);
    RCLCPP_INFO(LOGGER, "O_z: %f", transform_camera_base.transform.rotation.z);
    RCLCPP_INFO(LOGGER, "O_w: %f", transform_camera_base.transform.rotation.w);
    

    geometry_msgs::msg::Quaternion cameraTargetPoint;
    geometry_msgs::msg::Quaternion nomBaseCameraRot ;

    cameraTargetPoint.w = 0.0;
    cameraTargetPoint.x = msg->x;
    cameraTargetPoint.y = msg->y;
    cameraTargetPoint.z = msg->z;

    nomBaseCameraRot = normalizQuaternion(transform_camera_base.transform.rotation);

    cameraTargetPoint = this->multiplyQuaternion(nomBaseCameraRot, cameraTargetPoint);
    cameraTargetPoint = this->multiplyQuaternion(cameraTargetPoint, this->inversQuaternion(nomBaseCameraRot));

    RCLCPP_INFO(LOGGER, "Kamera Target Pos after rot");
    RCLCPP_INFO(LOGGER, "P_x: %f", cameraTargetPoint.x );
    RCLCPP_INFO(LOGGER, "P_y: %f", cameraTargetPoint.y );
    RCLCPP_INFO(LOGGER, "P_z: %f", cameraTargetPoint.z );

    geometry_msgs::msg::Quaternion pointOffset;
    geometry_msgs::msg::Quaternion nomQuaternion;
    geometry_msgs::msg::Pose target_pose;

    pointOffset.w = 0.0;
    pointOffset.x = transform_tcp_tool0.transform.translation.x;
    pointOffset.y = transform_tcp_tool0.transform.translation.y;
    pointOffset.z = transform_tcp_tool0.transform.translation.z;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;


    nomQuaternion = normalizQuaternion(target_pose.orientation);
    target_pose.orientation = this->multiplyQuaternion(nomQuaternion, transform_tcp_tool0.transform.rotation);
    //target_pose.orientation = this->multiplyQuaternion(target_pose.orientation, rotation_x_180);
    pointOffset = this->multiplyQuaternion(nomQuaternion, pointOffset);
    pointOffset = this->multiplyQuaternion(pointOffset, this->inversQuaternion(nomQuaternion));

    RCLCPP_INFO(LOGGER, "PointOffset");
    RCLCPP_INFO(LOGGER, "P_x: %f", pointOffset.x);
    RCLCPP_INFO(LOGGER, "P_y: %f", pointOffset.y);
    RCLCPP_INFO(LOGGER, "P_z: %f", pointOffset.z);

    target_pose.position.x = cameraTargetPoint.x + transform_camera_base.transform.translation.x;
    target_pose.position.y = cameraTargetPoint.y + transform_camera_base.transform.translation.y;
    target_pose.position.z = cameraTargetPoint.z + transform_camera_base.transform.translation.z;

    RCLCPP_INFO(LOGGER, "Target in Base");
    RCLCPP_INFO(LOGGER, "P_x: %f", target_pose.position.x);
    RCLCPP_INFO(LOGGER, "P_y: %f", target_pose.position.y);
    RCLCPP_INFO(LOGGER, "P_z: %f", target_pose.position.z);
    RCLCPP_INFO(LOGGER, "O_x: %f", target_pose.orientation.x);
    RCLCPP_INFO(LOGGER, "O_y: %f", target_pose.orientation.y);
    RCLCPP_INFO(LOGGER, "O_z: %f", target_pose.orientation.z);
    RCLCPP_INFO(LOGGER, "O_w: %f", target_pose.orientation.w);

    target_pose.position.x = cameraTargetPoint.x + pointOffset.x + transform_camera_base.transform.translation.x;
    target_pose.position.y = cameraTargetPoint.y + pointOffset.y + transform_camera_base.transform.translation.y;
    target_pose.position.z = cameraTargetPoint.z + pointOffset.z + transform_camera_base.transform.translation.z;

    RCLCPP_INFO(LOGGER, "UR_Pose after calculation");
    RCLCPP_INFO(LOGGER, "P_x: %f", target_pose.position.x);
    RCLCPP_INFO(LOGGER, "P_y: %f", target_pose.position.y);
    RCLCPP_INFO(LOGGER, "P_z: %f", target_pose.position.z);
    RCLCPP_INFO(LOGGER, "O_x: %f", target_pose.orientation.x);
    RCLCPP_INFO(LOGGER, "O_y: %f", target_pose.orientation.y);
    RCLCPP_INFO(LOGGER, "O_z: %f", target_pose.orientation.z);
    RCLCPP_INFO(LOGGER, "O_w: %f", target_pose.orientation.w);
    move_group_.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::msg::MoveItErrorCodes error = move_group_.plan(my_plan);
    bool success2 = (error.val == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER,"Planning successful: %d", success2);
    if (success2){
        move_group_.stop();
        move_group_.asyncExecute(my_plan);
        RCLCPP_INFO(LOGGER,"Moving!");
    }
    else{
        RCLCPP_INFO(LOGGER,"Moveit Error Code: %d", error.val);
        RCLCPP_INFO(LOGGER,"Moveit Error: %s", moveit::core::error_code_to_string(error.val).c_str());
    }
    
}


geometry_msgs::msg::Quaternion MlReceiverMoveitNode::multiplyQuaternion(const geometry_msgs::msg::Quaternion q1,  const geometry_msgs::msg::Quaternion q2){
    geometry_msgs::msg::Quaternion result;

    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

    return result;
}

geometry_msgs::msg::Quaternion MlReceiverMoveitNode::inversQuaternion(geometry_msgs::msg::Quaternion q){
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    
    return q;
}

geometry_msgs::msg::Quaternion MlReceiverMoveitNode::normalizQuaternion(geometry_msgs::msg::Quaternion q){
    double nom = sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    q.x = q.x / nom;
    q.y = q.y / nom;
    q.z = q.z / nom;
    q.w = q.w / nom;
    return q;
}


geometry_msgs::msg::Quaternion MlReceiverMoveitNode::createRotationX180() {
    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI);
    return tf2::toMsg(q);
}