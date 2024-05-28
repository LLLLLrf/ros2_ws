import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        self.leader_pose = None
        self.pose = None

        self.create_subscription(
            Pose,
            'turtle1/pose',
            self.update_leader_pose,
            QoSProfile(depth=10)
        )

        self.create_subscription(
            Pose,
            'turtle2/pose',
            self.update_pose,
            QoSProfile(depth=10)
        )

        self.vel_publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

    def update_leader_pose(self, msg):
        self.leader_pose = msg

    def update_pose(self, msg):
        self.pose = msg
        if self.leader_pose:
            self.follow_leader()

    def follow_leader(self):
        if self.pose is None or self.leader_pose is None:
            return

        twist = Twist()
        dx = self.leader_pose.x - self.pose.x
        dy = self.leader_pose.y - self.pose.y
        angle_to_leader = math.atan2(dy, dx)
        distance_to_leader = math.sqrt(dx**2 + dy**2)

        if distance_to_leader > 1.0:
            if abs(angle_to_leader - self.pose.theta) > 0.1:
                twist.angular.z = 1.5 * (angle_to_leader - self.pose.theta)
            else:
                twist.linear.x = 1.5 * (distance_to_leader - 1.0)
            self.vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
