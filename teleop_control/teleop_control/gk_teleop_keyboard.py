import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import sys
import termios
import tty
import os

# Robot parameters
WHEEL_RADIUS = 0.05  # 5 cm
WHEEL_BASE = 0.35    # Distance between wheels (0.175 * 2)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('gk_teleop_keyboard') 
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None
        self.run()

    def get_key(self):
        if os.name == 'nt':
            import msvcrt
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def compute_wheel_velocities(self, linear_vel, angular_vel):
        left_wheel_vel = (linear_vel - (WHEEL_BASE / 2.0) * angular_vel) / WHEEL_RADIUS
        right_wheel_vel = (linear_vel + (WHEEL_BASE / 2.0) * angular_vel) / WHEEL_RADIUS
        return left_wheel_vel, right_wheel_vel

    def run(self):
        self.get_logger().info("Teleop Node Started! Use 'w/a/s/d/x' to move, space to stop, CTRL+C to exit.")
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    self.target_linear_velocity += 0.05
                elif key == 'x':
                    self.target_linear_velocity -= 0.05
                elif key == 'a':
                    self.target_angular_velocity += 0.1
                elif key == 'd':
                    self.target_angular_velocity -= 0.1
                elif key == 's':
                    self.target_linear_velocity = 0.0
                    self.target_angular_velocity = 0.0
                elif key == '\x03':  # CTRL+C
                    break
                
                left_wheel_vel, right_wheel_vel = self.compute_wheel_velocities(
                    self.target_linear_velocity, self.target_angular_velocity
                )
                
                twist = Twist()
                twist.linear.x = self.target_linear_velocity
                twist.angular.z = self.target_angular_velocity
                self.pub.publish(twist)
                self.get_logger().info(
                    f"Linear: {twist.linear.x}, Angular: {twist.angular.z}, "
                    f"Left Wheel: {left_wheel_vel:.2f}, Right Wheel: {right_wheel_vel:.2f}"
                )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) if self.settings else None
            self.stop_robot()
            self.get_logger().info("Teleop Node Stopped.")
    
    def stop_robot(self):
        twist = Twist()
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = TeleopNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
