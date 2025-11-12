import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_bot_node')

        # Publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/ttb_lidar/out',  # topic that matches my actual robot
            self.lidar_callback,
            10)

        self.get_logger().info('Wall Follower Node Initialized')

        # Wall-following control parameters
        self.desired_right_distance = 0.5  # meters
        self.kp = 1.0  # Proportional gain

    def lidar_callback(self, msg: LaserScan):
        twist = Twist()

        # Get angles in radians corresponding to right (-90°) and front (0°)
        angle_right = -np.pi / 2  # -90 degrees
        angle_front = 0.0         # 0 degrees
        angle_tolerance = 0.1     # ~5-6 degrees

        # Generate angle array from LIDAR message
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Filter ranges near right
        right_indices = [i for i, angle in enumerate(angles) if abs(angle - angle_right) < angle_tolerance]
        right_distances = [msg.ranges[i] for i in right_indices if not np.isinf(msg.ranges[i])]

        # Filter ranges near front
        front_indices = [i for i, angle in enumerate(angles) if abs(angle - angle_front) < angle_tolerance]
        front_distances = [msg.ranges[i] for i in front_indices if not np.isinf(msg.ranges[i])]

        # Get minimum valid distances or fallback
        distance_right = min(right_distances) if right_distances else float('inf')
        distance_front = min(front_distances) if front_distances else float('inf')

        # Control logic
        if distance_front < 0.3:
            # Obstacle ahead — turn left
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            # Follow right wall
            error = self.desired_right_distance - distance_right
            twist.linear.x = 0.2
            twist.angular.z = self.kp * error

        # Publish command
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
