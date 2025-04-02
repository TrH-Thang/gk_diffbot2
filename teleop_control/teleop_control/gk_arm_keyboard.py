import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_keyboard')
        
        # Action clients for controlling the joints
        self.joint_base_mid_action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_base_mid_position_controller/follow_joint_trajectory'
        )
        self.joint_mid_top_action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_mid_top_position_controller/follow_joint_trajectory'
        )
        
        self.run()

    def send_joint_command(self, joint_name, position):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 2
        trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        if joint_name == 'joint_base_mid':
            self.joint_base_mid_action_client.wait_for_server()
            self.joint_base_mid_action_client.send_goal_async(goal_msg)
            self.get_logger().info(f"Moving joint_base_mid to {position}")
        elif joint_name == 'joint_mid_top':
            self.joint_mid_top_action_client.wait_for_server()
            self.joint_mid_top_action_client.send_goal_async(goal_msg)
            self.get_logger().info(f"Moving joint_mid_top to {position}")

    def run(self):
        self.get_logger().info("Joint Control Node Started! Enter the desired position for the joints.")
        self.get_logger().info("Press 'q' to exit.")
        
        try:
            while rclpy.ok():
                # Get input for joint_base_mid
                self.get_logger().info("Enter position for joint_base_mid (or 'q' to quit):")
                joint_base_mid_input = input("Joint Base Mid Position: ")
                if joint_base_mid_input == 'q':
                    break
                try:
                    joint_base_mid_position = float(joint_base_mid_input)
                    self.send_joint_command('joint_base_mid', joint_base_mid_position)
                except ValueError:
                    self.get_logger().warn("Invalid input for joint_base_mid. Please enter a valid number.")
                
                # Get input for joint_mid_top
                self.get_logger().info("Enter position for joint_mid_top (or 'q' to quit):")
                joint_mid_top_input = input("Joint Mid Top Position: ")
                if joint_mid_top_input == 'q':
                    break
                try:
                    joint_mid_top_position = float(joint_mid_top_input)
                    self.send_joint_command('joint_mid_top', joint_mid_top_position)
                except ValueError:
                    self.get_logger().warn("Invalid input for joint_mid_top. Please enter a valid number.")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.get_logger().info("Joint Control Node Stopped.")

def main():
    rclpy.init()
    node = JointControlNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
