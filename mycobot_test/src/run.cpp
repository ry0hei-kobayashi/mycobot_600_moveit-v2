#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

void visualizeGoal(const rclcpp::Node::SharedPtr& node, const geometry_msgs::msg::Pose& pose, const std::string& frame_id)
{
  static auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "goal_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = pose;
  marker.scale.x = 0.05; 
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.0;  
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;   
  marker.lifetime = rclcpp::Duration(std::chrono::seconds(0));  

  marker_pub->publish(marker);

  RCLCPP_INFO(LOGGER, "Published goal marker at position: x=%f, y=%f, z=%f",
              pose.position.x, pose.position.y, pose.position.z);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  move_group.setMaxVelocityScalingFactor(1.0); 
  move_group.setMaxAccelerationScalingFactor(1.0);

  //for neutral pose
  const std::string neutral_pose = "neutral";
  RCLCPP_INFO(LOGGER, "Setting target pose to: %s", neutral_pose.c_str());
  move_group.setNamedTarget(neutral_pose);
  move_group.move();
  //for neutral pose

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  RCLCPP_INFO(LOGGER, "Current Position: x=%f, y=%f, z=%f",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z);

  RCLCPP_INFO(LOGGER, "Current Orientation: x=%f, y=%f, z=%f, w=%f",
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);

  geometry_msgs::msg::Pose target_pose = current_pose;

  //1st
  target_pose.position.z -= 0.02;
  
  visualizeGoal(move_group_node, target_pose, move_group.getPlanningFrame());

  move_group.setPlanningTime(20.0);
  move_group.setPoseTarget(target_pose);
  move_group.move();


  //2nd
  target_pose.position.z -= 0.02;
  move_group.setPoseTarget(target_pose);


  visualizeGoal(move_group_node, target_pose, move_group.getPlanningFrame());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(LOGGER, "Plan successful, executing...");
    move_group.move();
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Plan failed.");
  }

  rclcpp::shutdown();
  return 0;
}

