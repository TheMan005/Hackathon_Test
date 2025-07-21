#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import os
import csv
import json
from datetime import datetime

MAX_LINEAR_VEL = 0.22 # m/s
MAX_ANGULAR_VEL = 2.84 # rad/s


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')  

        # Ziegler nichols parameters
        self.ziegler_nichols_mode = True  
        self.is_oscillating = False
        self.ku_linear = 0.0
        self.tu_linear = 0.0
        self.oscillation_start_time = None
        self.oscillation_peak_time = None
        self.oscillation_direction = None
        self.last_direction = None
        self.prev_distance_error_sign = None

        # Declare PID parameters - UNCOMMENTED AND FIXED
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_linear', 0.5),
                ('ki_linear', 0.0),
                ('kd_linear', 0.1),
                ('kp_angular', 1.0),
                ('ki_angular', 0.0),
                ('kd_angular', 0.2),
                ('goal_x', 1.0),  # Goal position x-coordinate
                ('goal_y', 0.0),  # Goal position y-coordinate
                ('distance_tolerance', 0.05),  # Tolerance to consider goal reached
                ('waypoint_index', 0)
            ]
        )
        
        # Initialize variables
        self.current_position = None
        self.current_orientation = None
        self.current_yaw = 0.0
        self.prev_linear_error = 0.0
        self.integral_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_angular_error = 0.0
        
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
        ]
        self.current_waypoint_index = 0

        # Initialize file paths and create directories if they don't exist
        self.data_dir = os.path.expanduser("~/challenge1/src/pid_challenge/data")
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.waypoint_log_file = os.path.join(self.data_dir, "waypointlogs.csv")
        self.tuning_results_file = os.path.join(self.data_dir, "tuning_results.json")
        
        # Initialize CSV file with headers
        self.init_csv_file()
        
        # Initialize tuning results structure
        self.tuning_results = {
            "session_start": datetime.now().isoformat(),
            "parameters_tested": [],
            "performance_metrics": {},
            "best_parameters": {}
        }

        # the odometer subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # publisher for cmdvel
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop with freq 10hz.
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('PID Controller Node Initialized')
        
        # Log current parameters
        self.log_current_parameters()

    def init_csv_file(self):
        """Initialize CSV file with headers if it doesn't exist"""
        if not os.path.exists(self.waypoint_log_file):
            with open(self.waypoint_log_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'waypoint_index', 'current_x', 'current_y', 'goal_x', 'goal_y',
                    'distance_error', 'angular_error', 'linear_vel', 'angular_vel',
                    'kp_linear', 'ki_linear', 'kd_linear', 'kp_angular', 'ki_angular', 'kd_angular'
                ])

    def log_current_parameters(self):
        """Log current PID parameters"""
        kp_linear = self.get_parameter('kp_linear').value
        ki_linear = self.get_parameter('ki_linear').value
        kd_linear = self.get_parameter('kd_linear').value
        kp_angular = self.get_parameter('kp_angular').value
        ki_angular = self.get_parameter('ki_angular').value
        kd_angular = self.get_parameter('kd_angular').value
        
        self.get_logger().info(
            f'Current PID Parameters - Linear: Kp={kp_linear}, Ki={ki_linear}, Kd={kd_linear}, '
            f'Angular: Kp={kp_angular}, Ki={ki_angular}, Kd={kd_angular}'
        )

    def log_waypoint_data(self, goal_x, goal_y, distance_error, angular_error, linear_vel, angular_vel):
        """Log waypoint data to CSV file"""
        try:
            with open(self.waypoint_log_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    datetime.now().isoformat(),
                    self.current_waypoint_index,
                    self.current_position.x if self.current_position else 0,
                    self.current_position.y if self.current_position else 0,
                    goal_x,
                    goal_y,
                    distance_error,
                    angular_error,
                    linear_vel,
                    angular_vel,
                    self.get_parameter('kp_linear').value,
                    self.get_parameter('ki_linear').value,
                    self.get_parameter('kd_linear').value,
                    self.get_parameter('kp_angular').value,
                    self.get_parameter('ki_angular').value,
                    self.get_parameter('kd_angular').value
                ])
        except Exception as e:
            self.get_logger().error(f'Failed to log waypoint data: {e}')

    def save_tuning_results(self):
        """Save tuning results to JSON file"""
        try:
            self.tuning_results["session_end"] = datetime.now().isoformat()
            self.tuning_results["total_waypoints"] = len(self.waypoints)
            self.tuning_results["waypoints_reached"] = self.current_waypoint_index
            
            with open(self.tuning_results_file, 'w') as jsonfile:
                json.dump(self.tuning_results, jsonfile, indent=4)
                
            self.get_logger().info(f'Tuning results saved to {self.tuning_results_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save tuning results: {e}')

    def odom_callback(self, msg):
        # Extract position and orientation
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(self.current_orientation)
        
        # Debug logging
        self.get_logger().info(
            f'Position: ({self.current_position.x:.2f}, {self.current_position.y:.2f}), '
            f'Yaw: {self.current_yaw:.2f}'
        )

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_position is None:
            return
        
        # Get parameters
        kp_linear = self.get_parameter('kp_linear').value
        ki_linear = self.get_parameter('ki_linear').value
        kd_linear = self.get_parameter('kd_linear').value
        kp_angular = self.get_parameter('kp_angular').value
        ki_angular = self.get_parameter('ki_angular').value
        kd_angular = self.get_parameter('kd_angular').value
        distance_tolerance = self.get_parameter('distance_tolerance').value

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            
            # Save final tuning results
            self.save_tuning_results()
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]

        # Compute distance error
        dx = goal_x - self.current_position.x
        dy = goal_y - self.current_position.y
        distance_error = math.sqrt(dx**2 + dy**2)

        # Angular error calculation
        desired_yaw = math.atan2(dy, dx)
        angular_error = desired_yaw - self.current_yaw

        # Normalize angular error to [-pi, pi]
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Stop if within tolerance
        if distance_error < distance_tolerance:
            cmd_vel = Twist()
            self.current_waypoint_index += 1
            self.get_logger().info(f'Waypoint no {self.current_waypoint_index} has been reached. Coordinates are {goal_x},{goal_y}')
            self.vel_pub.publish(cmd_vel)
            
            # Log waypoint completion
            self.log_waypoint_data(goal_x, goal_y, distance_error, angular_error, 0.0, 0.0)
            return

        # Compute PID for linear velocity
        self.integral_linear_error += distance_error * 0.1  
        derivative_linear_error = (distance_error - self.prev_linear_error) / 0.1
        linear_vel = (
            kp_linear * distance_error +
            ki_linear * self.integral_linear_error +
            kd_linear * derivative_linear_error
        )
        self.prev_linear_error = distance_error

        # Compute PID for angular velocity
        self.integral_angular_error += angular_error * 0.1
        derivative_angular_error = (angular_error - self.prev_angular_error) / 0.1
        angular_vel = (
            kp_angular * angular_error +
            ki_angular * self.integral_angular_error +
            kd_angular * derivative_angular_error
        )
        self.prev_angular_error = angular_error

        # Apply velocity limits
        linear_vel = max(min(linear_vel, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)  
        angular_vel = max(min(angular_vel, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)  

        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.vel_pub.publish(cmd_vel)

        # Log current control data
        self.log_waypoint_data(goal_x, goal_y, distance_error, angular_error, linear_vel, angular_vel)

        self.get_logger().info(
            f'Distance Error: {distance_error:.2f}, Angular Error: {angular_error:.2f}, '
            f'Linear Vel: {linear_vel:.2f}, Angular Vel: {angular_vel:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()