#!/usr/bin/env python3

import time
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class GamePad(Node):
    def __init__(self):
        super().__init__("gamepad_bt")

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_btn = self.create_publisher(Int32MultiArray, 'button', 10)

        # Initialize pygame joystick
        pygame.init()
        pygame.joystick.init()

        self.speed = 0.5
        self.min_speed = 0.0
        self.max_speed = 1.0

        # Tombol untuk adjust speed
        self.button_increase_pressed = False
        self.button_decrease_pressed = False

        # Connect joystick (USB)
        self.connect_joystick()

        # Timer untuk membaca axis & button
        self.create_timer(0.05, self.axis_callback)   # 20 Hz
        self.create_timer(0.1, self.button_callback)  # 10 Hz

    def connect_joystick(self):
        """Connects to the first available joystick."""
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                name = self.joystick.get_name()
                self.get_logger().info(f"PS3 Controller connected via USB: {name}")
                break
            else:
                self.get_logger().warn("No joystick detected. Connect PS3 via USB. Retrying in 2s...")
                time.sleep(2)

    def button_callback(self):
        pygame.event.pump()
        msg = Int32MultiArray()
        msg.data = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        self.pub_btn.publish(msg)
        self.get_logger().debug(f"Button states: {msg.data}")

    def axis_callback(self):
        pygame.event.pump()

        # L1 (button 4) / R1 (button 5) untuk kontrol kecepatan
        if self.joystick.get_button(4) and not self.button_decrease_pressed:
            self.speed = max(self.min_speed, self.speed - 0.1)
            self.button_decrease_pressed = True
        elif not self.joystick.get_button(4):
            self.button_decrease_pressed = False

        if self.joystick.get_button(5) and not self.button_increase_pressed:
            self.speed = min(self.max_speed, self.speed + 0.1)
            self.button_increase_pressed = True
        elif not self.joystick.get_button(5):
            self.button_increase_pressed = False

        # Axis mapping PS3
        linear_x = -self.joystick.get_axis(1) * self.speed
        linear_y = self.joystick.get_axis(0) * self.speed
        angular_z = self.joystick.get_axis(2) * self.speed

        # Deadzone kecil
        deadzone = 0.05
        if abs(linear_x) < deadzone: linear_x = 0.0
        if abs(linear_y) < deadzone: linear_y = 0.0
        if abs(angular_z) < deadzone: angular_z = 0.0

        # Publish hanya jika ada pergerakan
        if linear_x != 0.0 or linear_y != 0.0 or angular_z != 0.0:
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.angular.z = angular_z
            self.pub_cmd.publish(twist)
            self.get_logger().info(f"Publishing Twist: x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = GamePad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting PS3 teleop node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
