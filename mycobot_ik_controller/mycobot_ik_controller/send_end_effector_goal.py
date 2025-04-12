import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mycobot_action_interfaces.action import MoveEndEffector
from geometry_msgs.msg import Pose


class EndEffectorGoalSender(Node):
    def __init__(self):
        super().__init__('end_effector_goal_sender')
        self._client = ActionClient(self, MoveEndEffector, 'move_end_effector')

    def send_goal(self):
        self._client.wait_for_server()

        goal_msg = MoveEndEffector.Goal()

        # 目標Poseを設定（ここを任意に変更してOK）
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.4
        pose.orientation.x = 0.0
        pose.orientation.y = 0.8
        pose.orientation.z = 0.0
        pose.orientation.w = 0.6

        goal_msg.target_pose = pose

        self.get_logger().info('Sending end effector goal pose...')
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.status_message}")

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Success: {result.message}")
        else:
            self.get_logger().error(f"Failed: {result.message}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorGoalSender()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
