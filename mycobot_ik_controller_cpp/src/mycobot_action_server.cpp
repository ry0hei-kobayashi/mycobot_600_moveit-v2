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
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>  // 時間リテラルに必要
using namespace std::chrono_literals;  // 3sなどを使うため


class MyCobotActionServer : public rclcpp::Node
{
public:
    using MoveEndEffector = mycobot_action_interfaces::action::MoveEndEffector;
    using GoalHandleMoveEndEffector = rclcpp_action::ServerGoalHandle<MoveEndEffector>;

    MyCobotActionServer() : Node("mycobot_action_server"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "arm_group")
    {
    RCLCPP_INFO(this->get_logger(), "MyCobotActionServer node initialized.");

    trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/mycobot_controller/follow_joint_trajectory");

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
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;


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
      
        // 逆運動学で求められた目標姿勢のジョイント角度を取得
        std::vector<double> goal_joint_values;
        const auto & traj_points = plan.trajectory_.joint_trajectory.points;
        if (!traj_points.empty()) {
            // ここでは最終点だけ取り出す
            goal_joint_values = traj_points.back().positions;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory has no points!");
            result->success = false;
            result->message = "Trajectory is empty.";
            goal_handle->abort(result);
            return;
        }

        // 現在の関節角度 (MoveIt の状態から取得)
        std::vector<double> current_joint_values = move_group_interface_.getCurrentJointValues();
      
        // ログ出力
        for (size_t i = 0; i < goal_joint_values.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "goal Joint %ld: %.3f rad", i + 1, goal_joint_values[i]);
        }

        // ログ出力
        for (size_t i = 0; i < current_joint_values.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "current Joint %ld: %.3f rad", i + 1, current_joint_values[i]);
            }

        
        // サーバが立ち上がるまで待つ（サーバーが既に立ち上がっていたら待たないが、立ち上がっていなかったら最大3秒待つ）
        if (!trajectory_client_->wait_for_action_server(3s)) {
            RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available");
            result->success = false;
            result->message = "Action server not available.";
            goal_handle->abort(result);
            return;
        }

        // FollowJointTrajectoryアクションを使って、joint_trajectory_controllerに目標姿勢を送信する
        // trajectory生成
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = move_group_interface_.getJointNames();

        // ポイント0: 現在姿勢
        trajectory_msgs::msg::JointTrajectoryPoint start_pt;
        start_pt.positions = current_joint_values;
        start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);  // 0秒

        // ポイント1: 目標姿勢
        trajectory_msgs::msg::JointTrajectoryPoint goal_pt;
        goal_pt.positions = goal_joint_values;
        goal_pt.time_from_start = rclcpp::Duration::from_seconds(0.005);

        // 複数ポイントを登録
        trajectory.points.push_back(start_pt);
        trajectory.points.push_back(goal_pt);

        // Goal メッセージ作成
        control_msgs::action::FollowJointTrajectory::Goal trajectory_goal;
        trajectory_goal.trajectory = trajectory;

        // 非同期で送信
        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        
        // feedback
        send_goal_options.feedback_callback =
        [this, goal_handle](typename rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
                            const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> msg)
        {
          // msg->desired, msg->actual, msg->error などが格納されている
          auto feedback_for_user = std::make_shared<MoveEndEffector::Feedback>();
  
          // 例として、actual.positions を文字列にまとめる
          std::ostringstream oss;
          oss << "Feedback from controller:\n";
          if (!msg->actual.positions.empty()) {
            oss << "  actual[0]: " << msg->actual.positions[0] << "\n";
          }
          if (!msg->error.positions.empty()) {
            oss << "  error[0]: " << msg->error.positions[0] << "\n";
          }
  
          feedback_for_user->status_message = oss.str();
          // ここでユーザ独自フィードバック情報を加えてもOK
          // feedback_for_user->some_other_field = ...
  
          // 自前のアクションサーバーのフィードバックを送信
          goal_handle->publish_feedback(feedback_for_user);
        };
        
        // result
        send_goal_options.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & wrapped_result) {
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
                result->success = true;
                result->message = "Trajectory executed successfully.";
                goal_handle->succeed(result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed.");
                result->success = false;
                result->message = "Trajectory execution failed.";
                goal_handle->abort(result);
            }
        };

        trajectory_client_->async_send_goal(trajectory_goal, send_goal_options);
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
