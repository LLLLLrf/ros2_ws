import math
import rclpy
from rclpy.node import Node
import random
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from math import sqrt, atan2

# 定义TurtleCatcher类，继承自Node类
class TurtleCatcher(Node):
    def __init__(self):
        super().__init__('turtle_catcher')
        self.master_turtle_pose = Pose()
        self.captured_turtles = {}  # 存储被捕获的海龟的位姿
        self.captured_turtles_pub = {} # 存储被捕获的海龟的发布器
        self.turtle_poses = {}  # 存储所有海龟的位姿信息   
        self.captured_turtles_sub = {}

        self.turtle_spawner = self.create_timer(3, self.spawn_turtle)  # 每3秒调用一次spawn_turtle函数
        self.turtle_chaser = self.create_timer(0.1, self.chase_turtle)  # 每0.1秒调用一次chase_turtle函数
        self.turtle_follower = self.create_timer(0.1, self.follow_turtle)  # 每0.1秒调用一次follow_turtle函数
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)  # 创建发布者，用于控制海龟的移动
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)  # 创建订阅者，用于获取主海龟的位姿信息
        self.get_logger().info('Turtle catcher node started')  # 打印日志信息
        self.num = []

    # 生成新的海龟
    def spawn_turtle(self):
        client = self.create_client(Spawn, 'spawn')  # 创建服务客户端，用于请求生成新的海龟
        req = Spawn.Request()  # 创建请求对象
        req.x = random.uniform(1, 10)  # 随机生成x坐标
        req.y = random.uniform(1, 10)  # 随机生成y坐标
        req.name = 'turtle' + str(len(self.num) + 2)  # 生成海龟名称
        future = client.call_async(req)
        self.spawn_response_callback(req)

    # 处理生成海龟的响应
    def spawn_response_callback(self, req):
        try:
            response = req
            self.get_logger().info(f"Spawned new turtle: {response.name}")  # 打印日志信息
            new_turtle_pose = Pose()
            new_turtle_pose.x = response.x  # 获取新海龟的 x 坐标
            new_turtle_pose.y = response.y  # 获取新海龟的 y 坐标
            self.turtle_poses[response.name] = new_turtle_pose  # 将新海龟的位姿信息添加到字典中
            self.num.append(1)
            self.create_turtle_subscriber(response.name)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")  # 打印错误信息

    def create_turtle_subscriber(self, turtle_name):
        topic = f'/{turtle_name}/pose'
        self.captured_turtles_sub[turtle_name] = self.create_subscription(
            Pose,
            topic,
            lambda msg, name=turtle_name: self.turtle_pose_callback(msg, name),
            10
        )
    def turtle_pose_callback(self, msg, name):
        if name in self.captured_turtles:
            self.captured_turtles[name].x = msg.x
            self.captured_turtles[name].y = msg.y
            self.captured_turtles[name].theta = msg.theta

    # 主海龟追逐最近的海龟
    def chase_turtle(self):
        if self.master_turtle_pose is None:
            return
        min_dist = float('inf')
        target_turtle = None
        for name, pose in self.turtle_poses.items():
            dist = sqrt((pose.x - self.master_turtle_pose.x)**2 + (pose.y - self.master_turtle_pose.y)**2)  # 计算距离
            if dist < min_dist:
                min_dist = dist
                target_turtle = name
        if target_turtle is not None:
            # # 移动到目标海龟
            target_pose = self.turtle_poses[target_turtle]
            # # msg = Twist()
            # # msg.linear.x = 1 * (target_pose.x - self.master_turtle_pose.x)  # 控制前进方向
            # # msg.linear.y = 1 * (target_pose.y - self.master_turtle_pose.y)  # 控制前进方向
            # # self.cmd_pub.publish(msg)
            # msg = Twist()
            # distance = sqrt((target_pose.x - self.master_turtle_pose.x) ** 2 + (target_pose.y - self.master_turtle_pose.y) ** 2)
            # if distance >= 0.5:
            #     msg.linear.x = 1.5 * distance
            #     msg.angular.z = 4 * (atan2(target_pose.y - self.master_turtle_pose.y, target_pose.x - self.master_turtle_pose.x) - self.master_turtle_pose.theta)
            # else:
            #     msg.linear.x = 0.0
            #     msg.angular.z = 0.0
            # self.cmd_pub.publish(msg)
            distance_x = target_pose.x - self.master_turtle_pose.x
            distance_y = target_pose.y - self.master_turtle_pose.y
            distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

            msg = Twist()

            if distance > 0.5:
                # position
                msg.linear.x = 2 * distance

                # orientation
                goal_theta = math.atan2(distance_y, distance_x)
                diff = goal_theta - self.master_turtle_pose.theta

                # Normalize the angle difference to the range [-π, π]
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi

                msg.angular.z = 6 * diff

            else:
                # target reached
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                target_pose = None
            self.cmd_pub.publish(msg)

        if min_dist < 0.5 and target_turtle is not None and target_turtle not in self.captured_turtles:
            self.captured_turtles[target_turtle] = self.turtle_poses.pop(target_turtle)

    # 被捕获的海龟跟随主海龟
    def follow_turtle(self):
        for name, pose in self.captured_turtles.items():
            topic = name + '/cmd_vel'
            if topic not in self.captured_turtles_pub:
                self.captured_turtles_pub[topic] = self.create_publisher(Twist, topic, 10)
            pub = self.captured_turtles_pub[topic]

            relative_x = self.master_turtle_pose.x - pose.x
            relative_y = self.master_turtle_pose.y - pose.y

            distance = sqrt(relative_x**2 + relative_y**2)
            angle_to_master = atan2(relative_y, relative_x)
            angle_diff = angle_to_master - pose.theta
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            msg = Twist()
            if distance > 0.5:
                # position
                msg.linear.x = 2 * distance

                # orientation
                goal_theta = math.atan2(relative_y, relative_x)
                diff = self.master_turtle_pose.theta - goal_theta

                # Normalize the angle difference to the range [-π, π]
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi

                msg.angular.z = 6 * diff
            pub.publish(msg)

    # 更新主海龟的位姿信息
    def pose_callback(self, msg):
        self.master_turtle_pose = msg

# 主函数
def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    catcher = TurtleCatcher()  # 创建TurtleCatcher对象
    rclpy.spin(catcher)  # 进入事件循环
    rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':
    main()
