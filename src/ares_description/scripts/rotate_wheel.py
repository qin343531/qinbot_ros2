#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class AllWheelRotator(Node):
    def __init__(self):
        super().__init__('all_wheel_rotator')
        # 创建发布者，发布到/joint_states话题
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # 定时发布（每0.1秒一次）
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.angle = 0.0  # 初始角度

    def timer_callback(self):
        msg = JointState()
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        # 所有关节名（与/joint_states输出一致）
        msg.name = ['wheel_lf_joint', 'wheel_lb_joint', 'wheel_rf_joint', 'wheel_rb_joint']
        
        # 四个轮子同时转动：角度同步递增
        # 左前轮(lf)、左后轮(lb)、右前轮(rf)、右后轮(rb)
        self.angle += 0.1  # 每次增加0.1弧度（可调整速度）
        msg.position = [
            self.angle,    # 左前轮
            self.angle,    # 左后轮
            self.angle,    # 右前轮
            self.angle     # 右后轮
        ]
        
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        # 打印当前角度（方便观察）
        self.get_logger().info(f"轮子角度: {self.angle:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    all_wheel_rotator = AllWheelRotator()
    try:
        rclpy.spin(all_wheel_rotator)
    except KeyboardInterrupt:
        all_wheel_rotator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    