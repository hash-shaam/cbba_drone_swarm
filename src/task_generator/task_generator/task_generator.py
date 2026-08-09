import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class TaskGenerator(Node):

    def __init__(self):

        super().__init__("task_generator")

        self.publisher = self.create_publisher(
            Point,
            "/task",
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.publish_task
        )

        self.task_x = 10.0
        self.task_y = 5.0

    def publish_task(self):

        msg = Point()

        msg.x = self.task_x
        msg.y = self.task_y
        msg.z = 0.0

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Task published at ({msg.x}, {msg.y})"
        )


def main(args=None):

    rclpy.init(args=args)

    node = TaskGenerator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()