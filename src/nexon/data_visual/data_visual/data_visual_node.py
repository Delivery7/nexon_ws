#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import time
from collections import deque

class DataVisualizer(Node):
    def __init__(self):
        super().__init__('data_visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/checking_input',
            self.listener_callback,
            10)

        # Buffer untuk menyimpan history data (100 sampel terakhir)
        self.buffer_size = 100
        self.target_left = deque(maxlen=self.buffer_size)
        self.feedback_left = deque(maxlen=self.buffer_size)
        self.target_right = deque(maxlen=self.buffer_size)
        self.feedback_right = deque(maxlen=self.buffer_size)

        # Setup plot
        plt.ion()
        plt.style.use('seaborn-darkgrid')
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))

        # Subplot kiri
        self.line_target_left, = self.ax1.plot([], [], 'r-', label='Target Left')
        self.line_feedback_left, = self.ax1.plot([], [], 'b-', label='Feedback Left')
        self.ax1.set_title("Left Motor PID Response")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("RPS")
        self.ax1.legend()

        # Subplot kanan
        self.line_target_right, = self.ax2.plot([], [], 'r-', label='Target Right')
        self.line_feedback_right, = self.ax2.plot([], [], 'b-', label='Feedback Right')
        self.ax2.set_title("Right Motor PID Response")
        self.ax2.set_xlabel("Sample")
        self.ax2.set_ylabel("RPS")
        self.ax2.legend()

        self.fig.tight_layout()
        self.fig.show()
        self.fig.canvas.draw()

    def listener_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            # Ambil data sesuai urutan
            target_L = -msg.data[0]
            feedback_L = -msg.data[1]
            target_R = -msg.data[2]
            feedback_R = -msg.data[3]

            # Simpan ke buffer
            self.target_left.append(target_L)
            self.feedback_left.append(feedback_L)
            self.target_right.append(target_R)
            self.feedback_right.append(feedback_R)

            # Log di terminal
            self.get_logger().info(f"Target L: {target_L:.2f}, Feedback L: {feedback_L:.2f} | Target R: {target_R:.2f}, Feedback R: {feedback_R:.2f}")

    def update_plot(self):
        if len(self.target_left) == 0:
            return

        x = range(len(self.target_left))

        # Update data kiri
        self.line_target_left.set_data(x, self.target_left)
        self.line_feedback_left.set_data(x, self.feedback_left)
        self.ax1.relim()
        self.ax1.autoscale_view()

        # Update data kanan
        x2 = range(len(self.target_right))
        self.line_target_right.set_data(x2, self.target_right)
        self.line_feedback_right.set_data(x2, self.feedback_right)
        self.ax2.relim()
        self.ax2.autoscale_view()

        # Refresh grafik
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = DataVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
