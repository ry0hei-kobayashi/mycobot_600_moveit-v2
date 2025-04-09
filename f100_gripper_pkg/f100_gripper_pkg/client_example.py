#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# ここでは、my_robot_interfaces パッケージ内の SetHandAngle サービスを利用する場合の例です
from f100_gripper_interfaces.srv import SetHandAngle

class HandAngleClient(Node):
    def __init__(self):
        super().__init__('hand_angle_client')
        # 'set_hand_angle' という名前のサービスに接続するクライアントを作成
        self.cli = self.create_client(SetHandAngle, 'set_hand_angle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス "set_hand_angle" が利用可能になるまで待っています...')
        self.req = SetHandAngle.Request()

    def send_request(self, angle):
        self.req.angle = angle  # 送信するハンドの開閉角度
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = HandAngleClient()
    
    # 例として角度45を送信（0〜100 の範囲で調整）
    angle_value = 45
    client.get_logger().info(f"ハンド角度リクエスト送信: {angle_value}")
    
    future = client.send_request(angle_value)
    
    # サービス呼び出しの完了を待機
    rclpy.spin_until_future_complete(client, future)
    
    if future.result() is not None:
        client.get_logger().info(f"サービス応答: {future.result().message}")
    else:
        client.get_logger().error("サービス呼び出しに失敗しました")
        
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()