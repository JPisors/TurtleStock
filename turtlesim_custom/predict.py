#!/opt/ros_venv/bin/python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import pandas as pd


class PredictionPublisher(Node):
    def __init__(self, data):
        super().__init__('prediction_publisher')

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher for the predicted position
        self.predicted_publisher = self.create_publisher(Pose, '/predicted_pose', 10)

        # Parameters for prediction
        self.data = data
        self.position_buffer = []  # Buffer to store recent positions
        self.window_size = 25   # Number of positions to average

    def pose_callback(self, msg):
        # Update the position buffer, rolling window
        self.position_buffer.append((msg.x, msg.y))
        if len(self.position_buffer) > self.window_size:
            self.position_buffer.pop(0)  # Remove the oldest position

        # Once we have enough data, make prediction and publish it
        if len(self.position_buffer) == self.window_size:
            predicted_pose = self.predict_next_position()
            if predicted_pose:
                self.predicted_publisher.publish(predicted_pose)
                self.get_logger().info(f'Predicted Pose: x={predicted_pose.x:.2f}, y={predicted_pose.y:.2f}')

    def predict_next_position(self):
        if len(self.position_buffer) < self.window_size:
            return None

        # Calculate the averages
        avg_x = sum([pos[0] for pos in self.position_buffer]) / self.window_size
        avg_y = sum([pos[1] for pos in self.position_buffer]) / self.window_size

        # Return a ROS Pose() message with the predicted position
        predicted_pose = Pose()
        predicted_pose.x = avg_x
        predicted_pose.y = avg_y
        predicted_pose.theta = 0.0 # Placeholder for orientation, not used
        return predicted_pose


def main():
    data = pd.read_csv("/root/ros2_ws/src/turtlesim_custom/turtlesim_custom/scaled_sp500_data.csv")

    rclpy.init()
    prediction_publisher = PredictionPublisher(data)
    rclpy.spin(prediction_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
