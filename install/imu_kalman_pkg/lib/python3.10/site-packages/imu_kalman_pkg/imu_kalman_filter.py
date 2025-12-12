#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from math import atan2, asin

class IMUKalmanFilter(Node):
    def __init__(self):
        super().__init__('imu_kalman_filter')
        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # Parameter Kalman filter sederhana
        self.Q_angle = 0.001   # process noise
        self.Q_bias = 0.003
        self.R_measure = 0.03  # measurement noise

        # State variabel
        self.angle = 0.0
        self.bias = 0.0
        self.P = np.zeros((2, 2))

        self.prev_time = self.get_clock().now()
        self.get_logger().info("Kalman Filter IMU Node started!")

    def kalman_update(self, new_rate, new_angle, dt):
        # Prediction step
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Update covariance matrix
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Measurement update
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        y = new_angle - self.angle

        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

    def imu_callback(self, msg):
        # Hitung delta waktu
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        # Ambil data linear_acceleration & angular_velocity
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Estimasi pitch & roll dari accelerometer
        roll_acc = atan2(ay, az)
        pitch_acc = atan2(-ax, np.sqrt(ay**2 + az**2))

        # Update Kalman filter untuk roll dan pitch
        roll = self.kalman_update(gx, roll_acc, dt)
        pitch = self.kalman_update(gy, pitch_acc, dt)

        # Buat quaternion sederhana (tanpa yaw)
        qx = np.sin(roll/2)
        qy = np.sin(pitch/2)
        qz = 0.0
        qw = np.cos(roll/2) * np.cos(pitch/2)

        # Buat pesan IMU baru
        imu_filtered = Imu()
        imu_filtered.header = msg.header
        imu_filtered.orientation.x = qx
        imu_filtered.orientation.y = qy
        imu_filtered.orientation.z = qz
        imu_filtered.orientation.w = qw

        imu_filtered.linear_acceleration = msg.linear_acceleration
        imu_filtered.angular_velocity = msg.angular_velocity

        self.pub_imu.publish(imu_filtered)


def main(args=None):
    rclpy.init(args=args)
    node = IMUKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Kalman filter node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
