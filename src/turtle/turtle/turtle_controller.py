import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.update_pose, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.turtles = {}
        self.target_turtle = None
        self.pose = Pose()

        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        
        self.timer = self.create_timer(1.0, self.find_and_chase_turtle)

    def update_pose(self, msg):
        self.pose = msg

    def find_and_chase_turtle(self):
        if not self.turtles:
            self.get_logger().info('No turtles to chase.')
            return

        if self.target_turtle is None or self.target_turtle not in self.turtles:
            self.target_turtle = min(self.turtles, key=lambda name: self.distance_to(self.turtles[name]))

        turtle_pose = self.turtles[self.target_turtle]
        distance = self.distance_to(turtle_pose)
        
        if distance < 0.5:
            self.catch_turtle()
        else:
            self.move_to(turtle_pose)

    def distance_to(self, turtle_pose):
        return sqrt((turtle_pose.x - self.pose.x) ** 2 + (turtle_pose.y - self.pose.y) ** 2)

    def move_to(self, turtle_pose):
        msg = Twist()
        angle_to_target = atan2(turtle_pose.y - self.pose.y, turtle_pose.x - self.pose.x)
        
        if abs(angle_to_target - self.pose.theta) > 0.1:
            msg.angular.z = 2.0 * (angle_to_target - self.pose.theta)
        else:
            msg.linear.x = 2.0 * self.distance_to(turtle_pose)
        
        self.velocity_publisher.publish(msg)

    def catch_turtle(self):
        self.get_logger().info('Caught turtle: %s' % self.target_turtle)
        kill_request = Kill.Request()
        kill_request.name = self.target_turtle
        self.future = self.kill_client.call_async(kill_request)
        self.future.add_done_callback(self.turtle_killed)

    def turtle_killed(self, future):
        try:
            response = future.result()
            self.get_logger().info('Killed turtle: %s' % self.target_turtle)
            del self.turtles[self.target_turtle]
            self.target_turtle = None
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
