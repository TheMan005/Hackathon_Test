import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos
import tf_transformations

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter("Kp_linear", 1.0)
        self.declare_parameter("Ki_linear", 0.0)
        self.declare_parameter("Kd_linear", 0.1)
        self.declare_parameter("Kp_angular", 4.0)
        self.declare_parameter("Ki_angular", 0.0)
        self.declare_parameter("Kd_angular", 0.1)

        self.Kp_linear = self.get_parameter("Kp_linear").value
        self.Ki_linear = self.get_parameter("Ki_linear").value
        self.Kd_linear = self.get_parameter("Kd_linear").value
        self.Kp_angular = self.get_parameter("Kp_angular").value
        self.Ki_angular = self.get_parameter("Ki_angular").value
        self.Kd_angular = self.get_parameter("Kd_angular").value

        self.prev_linear_error = 0.0
        self.integral_linear = 0.0
        self.prev_angular_error = 0.0
        self.integral_angular = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.waypoints = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        self.current_wp = 0
        self.tolerance = 0.1

        self.pose = None
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("PID controller node initialized")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def control_loop(self):
        if self.pose is None or self.current_wp >= len(self.waypoints):
            return

        x = self.pose.position.x
        y = self.pose.position.y
        orientation = self.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y,
            orientation.z, orientation.w
        ])

        target_x, target_y = self.waypoints[self.current_wp]
        dx = target_x - x
        dy = target_y - y
        distance_error = sqrt(dx**2 + dy**2)
        angle_to_target = atan2(dy, dx)
        angular_error = angle_to_target - yaw
        angular_error = atan2(sin(angular_error), cos(angular_error))

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now
        if dt == 0:
            dt = 1e-2

        self.integral_linear += distance_error * dt
        derivative_linear = (distance_error - self.prev_linear_error) / dt
        linear_output = (
            self.Kp_linear * distance_error +
            self.Ki_linear * self.integral_linear +
            self.Kd_linear * derivative_linear
        )
        self.prev_linear_error = distance_error

        self.integral_angular += angular_error * dt
        derivative_angular = (angular_error - self.prev_angular_error) / dt
        angular_output = (
            self.Kp_angular * angular_error +
            self.Ki_angular * self.integral_angular +
            self.Kd_angular * derivative_angular
        )
        self.prev_angular_error = angular_error

        linear_output = max(-0.22, min(0.22, linear_output))
        angular_output = max(-2.84, min(2.84, angular_output))

        if distance_error < self.tolerance:
            self.get_logger().info(f"Reached waypoint {self.current_wp+1}")
            self.current_wp += 1
            self.integral_linear = 0.0
            self.integral_angular = 0.0
            linear_output = 0.0
            angular_output = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_output
        cmd.twist.angular.z = angular_output
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
