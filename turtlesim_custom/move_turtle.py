#!/opt/ros_venv/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import pandas as pd
import math


class TurtleMover(Node):
    def __init__(self, data):
        super().__init__('turtle_mover')

        # Publisher for control turtle's movement
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to track the turtle's position
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()

        self.data = data
        self.index = 0
        self.target_x = self.data.iloc[self.index]['Scaled_X']
        self.target_y = self.data.iloc[self.index]['Scaled_Y']

        self.timer = self.create_timer(0.1, self.update_turtle)

    def pose_callback(self, msg):
        self.current_pose = msg

    def update_turtle(self):
        if self.index >= len(self.data):
            self.get_logger().info('Finished moving turtle!')
            self.destroy_node()
            return

        # Check if turtle is at current target
        if self.is_at_target():
            self.index += 1  # Move to next target
            if self.index < len(self.data):
                self.target_x = self.data.iloc[self.index]['Scaled_X']
                self.target_y = self.data.iloc[self.index]['Scaled_Y']
            return

        twist = Twist() # ROS motion message format
        twist.angular.z = self.calculate_angular_velocity()

        # Turn, to point
        if abs(twist.angular.z) > 0.1:
            self.publisher.publish(twist)
            return

        # Drive to point
        twist.linear.x = self.calculate_linear_velocity()
        twist.angular.z = 0.0

        # Publish the movement command
        self.publisher.publish(twist)

    def is_at_target(self, tolerance=0.1):
        distance = math.sqrt(
            (self.current_pose.x - self.target_x) ** 2 +
            (self.current_pose.y - self.target_y) ** 2
        )
        return distance < tolerance

    def calculate_angular_velocity(self):
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        angle_to_target = math.atan2(dy, dx)

        # Calculate the angular velocity to align with the target
        angle_diff = angle_to_target - self.current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize angle

        # Simple P controller
        angular_velocity = 6.0 * angle_diff
        return angular_velocity

    def calculate_linear_velocity(self):
        # Calculate how far turtle is from next price point
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y

        linear_velocity = 2.5 * math.sqrt(dx**2 + dy**2)

        # Cap linear velocity to prevent skipping past point
        return min(linear_velocity, 3.0)


def main():
    data = pd.read_csv("/root/ros2_ws/src/turtlesim_custom/turtlesim_custom/scaled_sp500_data.csv")

    rclpy.init()
    turtle_mover = TurtleMover(data)
    rclpy.spin(turtle_mover)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
