import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class TurtleSpawner(Node):

    def __init__(self):
        super().__init__('turtle_spawner')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.timer = self.create_timer(3.0, self.spawn_turtle)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(0.5, 10.5)
        request.y = random.uniform(0.5, 10.5)
        request.theta = random.uniform(-3.14, 3.14)
        request.name = 'turtle' + str(random.randint(0, 10000))
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.turtle_spawned)

    def turtle_spawned(self, future):
        try:
            response = future.result()
            self.get_logger().info('Spawned turtle: %s' % response.name)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
