#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, FluidPressure # Consolidated imports

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

class AUVController(Node):
    def __init__(self):
        super().__init__('aqua_pilot')
        self.get_logger().info('AUV Pilot is alive and searching for sensors...')

        # 1. Initialize PIDs (To be tuned on competition day) [cite: 122, 255]
        self.heading_pid = PIDController(1.5, 0.01, 0.5)
        self.depth_pid = PIDController(2.0, 0.0, 0.2)
        
        # 2. State Tracking
        self.current_yaw = 0.0
        self.current_depth = 0.0
        self.target_depth = -2.0 # Operating depth range [cite: 196]

        # 3. Waypoint Mission [cite: 61, 222]
        self.waypoints = [0.0, 0.5, -0.3, 1.2] 
        self.current_waypoint_index = 0
        self.target_yaw = self.waypoints[self.current_waypoint_index]

        # 4. Publishers & Subscribers [cite: 58]
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.depth_sub = self.create_subscription(FluidPressure, '/depth', self.depth_callback, 10)

        # 5. Control Loop Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def imu_callback(self, msg):
        self.current_yaw = msg.orientation.z # [cite: 81]
        self.get_logger().info(f'Heading: {self.current_yaw:.2f}', once=False)

    def depth_callback(self, msg):
        self.current_depth = msg.fluid_pressure # [cite: 83]

    def control_loop(self):
        # Calculate Errors
        yaw_error = self.target_yaw - self.current_yaw
        depth_error = self.target_depth - self.current_depth
        
        # Compute PID [cite: 254]
        yaw_correction = self.heading_pid.compute(yaw_error, 0.1)
        depth_correction = self.depth_pid.compute(depth_error, 0.1)
        
        # Prepare Command
        cmd = Twist()
        cmd.linear.x = 1.0  # Surge [cite: 77]
        cmd.linear.z = depth_correction # Heave
        cmd.angular.z = yaw_correction # Yaw
        
        self.publisher_.publish(cmd)

        # Waypoint Switching [cite: 16]
        if abs(yaw_error) < 0.05:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.target_yaw = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(f'Switching to Waypoint: {self.target_yaw}')

def main(args=None):
    rclpy.init(args=args)
    node = AUVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()