import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("The turtle follower node has started")
        
        self.spawn_timer = self.create_timer(3.0, self.spawn_turtle)
        self.follow_timers = []

        self.turtle_count = 2
        self.turtle_positions = {}
        self.unhit_turtles = set()
        self.catched_list = []
        
        self.x1 = self.y1 = self.theta1 = 0.0
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.chase_timer = self.create_timer(0.2, self.chase_closest_turtle)

    def pose_callback(self, data):
        self.x1 = data.x
        self.y1 = data.y
        self.theta1 = data.theta

        for turtle_name, (x2, y2, _) in self.turtle_positions.items():
            if turtle_name in self.unhit_turtles and self.turtles_collide(self.x1, self.y1, x2, y2):
                self.unhit_turtles.remove(turtle_name)
                self.catched_list.append(turtle_name)
                # self.get_logger().info(f'{turtle_name} starts following')
                self.get_logger().info('list: {}'.format(self.catched_list))
                follow_timer = self.create_timer(0.2, self.create_follow_callback(turtle_name))
                self.follow_timers.append(follow_timer)

    def spawn_turtle(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        request = Spawn.Request()
        request.x = random.uniform(1, 10)
        request.y = random.uniform(1, 10)
        request.theta = random.uniform(-math.pi, math.pi)
        request.name = f'turtle{self.turtle_count}'
        
        future = client.call_async(request)
        future.add_done_callback(self.turtle_spawned_callback)

    def turtle_spawned_callback(self, future):
        try:
            response = future.result()
            turtle_name = response.name
            # self.get_logger().info(f'Successfully spawned {turtle_name}')
            
            self.turtle_positions[turtle_name] = [random.uniform(1, 10), random.uniform(1, 10), 0.0]
            self.unhit_turtles.add(turtle_name)
            self.create_subscription(Pose, f'/{turtle_name}/pose', self.create_pose_callback(turtle_name), 10)
            self.turtle_count += 1
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {str(e)}')

    def create_pose_callback(self, turtle_name):
        def pose_callback(data):
            self.turtle_positions[turtle_name] = [data.x, data.y, data.theta]
        return pose_callback

    def create_follow_callback(self, turtle_name):
        def follow_callback():
            if turtle_name not in self.turtle_positions:
                return
            if turtle_name == "turtle1":
                return
            elif self.catched_list.index(turtle_name)==0:
                x1, y1 = self.x1, self.y1
                ind=0
            elif turtle_name not in self.unhit_turtles:
                ind = self.catched_list.index(turtle_name)
                x1, y1 = self.turtle_positions[self.catched_list[ind-1]][:2]
                # self.get_logger().info('{} is following {}'.format(turtle_name, self.catched_list[ind-1]))

            x2, y2, theta2 = self.turtle_positions[turtle_name]
            distance = math.sqrt((y1 - y2)**2 + (x1 - x2)**2)
            angle_to_goal = math.atan2(y1 - y2, x1 - x2)
            angle_diff = angle_to_goal - theta2

            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            twist_msg = Twist()
            # 跟随海龟的速度
            # if 0.3<distance<=1.0 and abs(angle_diff)<0.2:
            #     twist_msg.linear.x=4.1 + 0.1*distance
            #     twist_msg.angular.z=1.0*angle_diff
            # elif 0.3<distance<=1.0 and abs(angle_diff)>0.2:
            #     twist_msg.linear.x=3.0 + 0.1*distance
            #     twist_msg.angular.z=4.0*angle_diff
            # elif distance<=0.3:
            #     twist_msg.linear.x=2 + 0.1*distance
            #     twist_msg.angular.z=2.0*angle_diff
            # elif distance>1.0 and abs(angle_diff)>0.2:
            #     twist_msg.linear.x=3.8+2.0*distance
            #     twist_msg.angular.z=7.0*angle_diff
            # elif distance>1.0 and abs(angle_diff)<0.2:
            #     twist_msg.linear.x=3.8+3.8*distance
            #     twist_msg.angular.z=3.0*angle_diff
            if distance<=0.6:
                twist_msg.linear.x=0.0
                twist_msg.angular.z=3.0*angle_diff
            else:
                twist_msg.linear.x=3.0*distance
                twist_msg.angular.z=5.0*angle_diff
                
                
            publisher = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            publisher.publish(twist_msg)
        return follow_callback

    def turtles_collide(self, x1, y1, x2, y2, threshold=0.5):
        distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return distance < threshold

    def chase_closest_turtle(self):
        if not self.unhit_turtles:
            return

        closest_turtle = None
        min_distance = float('inf')

        for turtle_name in self.unhit_turtles:
            x2, y2, _ = self.turtle_positions[turtle_name]
            distance = math.sqrt((self.x1 - x2)**2 + (self.y1 - y2)**2)
            if distance < min_distance:
                min_distance = distance
                closest_turtle = turtle_name

        if closest_turtle:
            x2, y2, _ = self.turtle_positions[closest_turtle]
            angle_to_goal = math.atan2(y2 - self.y1, x2 - self.x1)
            angle_diff = angle_to_goal - self.theta1

            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            # 追逐目标海龟
            twist_msg = Twist()
            # twist_msg.linear.x = 2.0
            # twist_msg.angular.z = 4.0 * angle_diff
            twist_msg.linear.x = 4.2
            twist_msg.angular.z = 6.0 * angle_diff
            
            self.publisher1.publish(twist_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
