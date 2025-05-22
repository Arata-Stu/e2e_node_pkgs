import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import torch
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class CNNNode(Node):
    def __init__(self):
        super().__init__('lidar_drive_node')

        # Declare parameters
        self.declare_parameter('model_path', 'checkpoint.pt')
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('downsample_num', 1081)

        # Load parameters
        self.load_parameters()

        # Load model
        self.model = self.load_model(self.model_path)

        # Register dynamic param callback
        self.add_on_set_parameters_callback(self.on_param_change)

        # ROS I/O
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(
            AckermannDrive,
            '/cmd_drive',
            10)

        self.get_logger().info(
            f"[STARTED] model: {self.model_path}, downsample_num: {self.downsample_num}, max_range: {self.max_range}"
        )

    def load_parameters(self):
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.downsample_num = self.get_parameter('downsample_num').get_parameter_value().integer_value

    def on_param_change(self, params):
        success = True
        reason = ""

        for param in params:
            if param.name == 'model_path':
                try:
                    self.model = self.load_model(param.value)
                    self.model_path = param.value
                    self.get_logger().info(f"[RELOADED] Model from {param.value}")
                except Exception as e:
                    success = False
                    reason += f" Model reload failed: {e}"
            elif param.name == 'max_range':
                self.max_range = param.value
                self.get_logger().info(f"[UPDATED] max_range: {self.max_range}")
            elif param.name == 'downsample_num':
                self.downsample_num = param.value
                self.get_logger().info(f"[UPDATED] downsample_num: {self.downsample_num}")

        return SetParametersResult(successful=success, reason=reason)
