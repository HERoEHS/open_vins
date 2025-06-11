import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUChecker(Node):
    def __init__(self):
        super().__init__('imu_checker')
        self.sub = self.create_subscription(Imu, '/edie8/sensor/lpf_imu', self.imu_callback, 10)
        self.buffer = []

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.buffer.append([ax, ay, az])

        # 정지 상태에서 약 100개 수집하면 분석
        if len(self.buffer) >= 100:
            acc = np.array(self.buffer)
            acc_mean = np.mean(acc, axis=0)
            print(f"[평균] ax: {acc_mean[0]:.4f}, ay: {acc_mean[1]:.4f}, az: {acc_mean[2]:.4f}")
            if abs(acc_mean[0]) < 0.2 and abs(acc_mean[1]) < 0.2 and 9.6 < acc_mean[2] < 10.0:
                print("✅ IMU 정렬 OK")
            else:
                print("❌ IMU 정렬 문제 있음")
            self.buffer.clear()
        

rclpy.init()
node = IMUChecker()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
