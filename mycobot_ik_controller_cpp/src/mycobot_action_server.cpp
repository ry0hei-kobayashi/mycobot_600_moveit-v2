#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mycobot_action_interfaces/action/move_end_effector.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>  

class MyCobotActionServer : public rclcpp::Node
{
public:
    using MoveEndEffector = mycobot_action_interfaces::action::MoveEndEffector;
    using GoalHandleMoveEndEffector = rclcpp_action::ServerGoalHandle<MoveEndEffector>;

    MyCobotActionServer() : Node("mycobot_action_server"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "arm_group")
    {
    RCLCPP_INFO(this->get_logger(), "MyCobotActionServer node initialized.");

    action_server_ = rclcpp_action::create_server<MoveEndEffector>(
        this,
        "move_end_effector",
        std::bind(&MyCobotActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MyCobotActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MyCobotActionServer::handle_accepted, this, std::placeholders::_1));
    }
private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp_action::Server<MoveEndEffector>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveEndEffector::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle)
    {
        std::thread{[this, goal_handle]() {
          this->execute(goal_handle);
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing IK goal...");

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveEndEffector::Result>();
        auto feedback = std::make_shared<MoveEndEffector::Feedback>();
      
        // エンドエフェクタの目標姿勢をセット
        geometry_msgs::msg::Pose target_pose = goal->target_pose;
        move_group_interface_.setPoseTarget(target_pose);
      
        feedback->status_message = "Planning IK to target pose...";
        goal_handle->publish_feedback(feedback);
      
        // パス計画を実行（内部で逆運動学を使う）
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "IK failed to find a solution.");
          result->success = false;
          result->message = "IK failed to plan to the given pose.";
          goal_handle->abort(result);
          return;
        }
      
        // 逆運動学で求められたジョイント角度を取得
        std::vector<double> joint_values;
        move_group_interface_.getCurrentState()->copyJointGroupPositions(move_group_interface_.getName(), joint_values);
      
        // ログ出力
        std::ostringstream joint_str;
        for (size_t i = 0; i < joint_values.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "Joint %ld: %.3f rad", i + 1, joint_values[i]);
          joint_str << "Joint " << (i + 1) << ": " << joint_values[i] << " rad\n";
        }

        // ここにmycobotを動かすため、FollowJointTrajectoryアクションを使って、joint_trajectory_controllerに目標姿勢を送信するプログラムを作成

      
        // 成功レスポンスを返す
        result->success = true;
        result->message = "????\n";
        goal_handle->succeed(result);
      
        RCLCPP_INFO(this->get_logger(), "????");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCobotActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
