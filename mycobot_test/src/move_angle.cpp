#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("moveit_joint_control");

    auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, "arm_group");

    RCLCPP_INFO(node->get_logger(), "Initialized MoveGroupInterface for 'arm_group'");

    std::vector<double> target_joint_positions = {0.0, 0.0, 0.0, 0.0, -1.57, 0.0};
    move_group_interface->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface->plan(plan));

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        move_group_interface->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to the target joint positions.");
    }

    rclcpp::shutdown();
    return 0;
}

