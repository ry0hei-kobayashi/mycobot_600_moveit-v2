#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from f100_gripper_interfaces.srv import SetHandAngle  # カスタムサービスのインポート
from f100_gripper_pkg.elegripper import Gripper
import time

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        # サービス名 'set_hand_angle' でサービスサーバーを作成し、コールバック関数を指定
        self.srv = self.create_service(SetHandAngle, 'set_hand_angle', self.handle_set_hand_angle)
        self.get_logger().info('グリッパーサービスサーバーが起動しました。')

        self.g=Gripper("/dev/ttyACM0",baudrate=115200,id=14)##Fill in the actual serial port number, baud rate and gripper ID
        print("The actual ID of the gripper is:",self.g.get_gripper_Id())

        # テスト用
        # print(g.set_gripper_value(100,100))
        # time.sleep(2)
        # print(g.set_gripper_value(0,100))
        # time.sleep(2)

    def handle_set_hand_angle(self, request, response):
        self.get_logger().info('リクエスト受信: angle = %f' % request.angle)
        # 例：有効な角度の範囲は 0～100 とする
        if 0.0 <= request.angle <= 100.0:
            self.g.set_gripper_value(request.angle,100)
            response.success = True
            response.message = f'ハンドの角度を {request.angle} 度に設定しました。'
        else:
            response.success = False
            response.message = f'無効な角度です: {request.angle}。有効な範囲は 0～100です。'
        return response

def main(args=None):
    rclpy.init(args=args)
    server = GripperServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('グリッパーサービスサーバーを終了します...')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
