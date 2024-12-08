#!/opt/ros_venv/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
import math

class TurtleCatcher(Node):
    def __init__(self):
        super().__init__('turtle_catcher')

        self.spawn_client = self.create_client(turtlesim.srv.Spawn, '/spawn')
        self.spawn_turtle2()

        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Initialize variables
        self.is_moving = False
        self.current_pose = None # Pose()
        self.predicted_x = None
        self.predicted_y = None
        self.turtle1_pose = Pose()
        self.timer = None 

        # Subscriptions
        self.pose_subscriber = self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)
        self.predicted_pose_subscriber = self.create_subscription(Pose, '/predicted_pose', self.predicted_pose_callback, 10)
        self.turtle1_pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)

    def spawn_turtle2(self):
        request = turtlesim.srv.Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        #request.name = 'turtle2'
        self.spawn_client.call_async(request)
        self.get_logger().info('Spawning turtle2 at position (2, 2).')

    def pose_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f'Received pose: {self.current_pose}')
        
        # Attempt to catch every 20 seconds
        if self.timer is None:
            self.timer = self.create_timer(20.0, self.move_to_predicted_position)

    def predicted_pose_callback(self, msg):
        self.predicted_x = msg.x
        self.predicted_y = msg.y

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg

    def move_to_predicted_position(self):
        # Debugging
        if self.predicted_x is None or self.predicted_y is None:
            self.get_logger().info('predicted position not received yet')
            return

        if self.is_moving:
            self.get_logger().info('Waiting for previous move to complete')
            return

        self.is_moving = True
        self.get_logger().info(f'Moving to predicted position: ({self.predicted_x}, {self.predicted_y})')

        # Move to target
        while not self.is_at_target(self.predicted_x, self.predicted_y):
            twist = Twist()
            twist.angular.z = self.calculate_angular_velocity(self.predicted_x, self.predicted_y)

            if abs(twist.angular.z) > 0.1:  # Rotate until facing target
                self.publisher.publish(twist)
                continue

            # Then move forward
            twist.linear.x = self.calculate_linear_velocity(self.predicted_x, self.predicted_y)
            twist.angular.z = 0.0
            self.publisher.publish(twist)

        self.publisher.publish(Twist())  # Stop once target is reached
        self.is_moving = False

        # Check proximity to stock turtle, win/lose if within 1.5 unit radius, future method
        distance_to_turtle1 = math.sqrt((self.current_pose.x - self.turtle1_pose.x) ** 2 +(self.current_pose.y - self.turtle1_pose.y) ** 2)
        if distance_to_turtle1 <= 1.5:
            self.get_logger().info('GAIN')
        else:
            self.get_logger().info('LOSS')

    def is_at_target(self, target_x, target_y, tolerance=0.1):
        if self.current_pose is None:
            return False
        distance = math.sqrt((self.current_pose.x - target_x) ** 2 +(self.current_pose.y - target_y) ** 2)
        return distance < tolerance

    def calculate_angular_velocity(self, target_x, target_y):
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) # Normalize angle
        angular_velocity = 4.0 * angle_diff
        return max(min(angular_velocity, 2.0), -2.0)

    def calculate_linear_velocity(self, target_x, target_y):
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        linear_velocity = 2.5 * math.sqrt(dx ** 2 + dy ** 2)
        return min(linear_velocity, 3.0)

def main():
    rclpy.init()
    turtle_catcher = TurtleCatcher()
    rclpy.spin(turtle_catcher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
