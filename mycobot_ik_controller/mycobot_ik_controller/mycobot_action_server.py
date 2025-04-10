import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from mycobot_action_interfaces.action import MoveEndEffector
from geometry_msgs.msg import Pose

from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize


class MyCobotActionServer(Node):
    def __init__(self):
        super().__init__('mycobot_action_server')
        roscpp_initialize([])

        # MoveIt初期化
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("arm_group")  # SRDFに定義されたグループ名に合わせる

        # アクションサーバー作成
        self._action_server = ActionServer(
            self,
            MoveEndEffector,
            'move_end_effector',
            self.execute_callback
        )

        self.get_logger().info('MyCobot Action Server has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal.')

        feedback = MoveEndEffector.Feedback()
        result = MoveEndEffector.Result()

        pose = goal_handle.request.target_pose

        # フィードバック送信
        feedback.status_message = 'Setting pose target...'
        goal_handle.publish_feedback(feedback)

        # 目標Poseをセット
        self.group.set_pose_target(pose)

        feedback.status_message = 'Planning to target...'
        goal_handle.publish_feedback(feedback)

        success, _, _, _ = self.group.plan()
        if not success:
            result.success = False
            result.message = "Failed to plan or solve IK."
            self.get_logger().error(result.message)
            return goal_handle.abort(result)

        joint_values = self.group.get_joint_value_target()
        joint_str = ', '.join([f"{v:.3f}" for v in joint_values])

        # 各ジョイント角度を個別に表示
        for i, angle in enumerate(joint_values):
            self.get_logger().info(f"Joint {i+1}: {angle:.3f} rad")

        result.success = True
        result.message = f"IK succeeded. Joint angles: [{joint_str}]"
        feedback.status_message = 'IK succeeded.'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(result.message)
        return goal_handle.succeed(result)


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
