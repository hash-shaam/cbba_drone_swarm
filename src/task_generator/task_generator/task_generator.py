import rclpy
from rclpy.node import Node
import csv

from geometry_msgs.msg import Point


class TaskGenerator(Node):

    def __init__(self):
        super().__init__("task_generator")

        self.publisher = self.create_publisher(Point, "/task", 10)

        # (delay_seconds_since_start, x, y, task_id)
        self.tasks = [
            (8.0, 10.0, 5.0, 1),
            (48.0, 0.0, 10.0, 2),
            (88.0, -8.0, 2.0, 3),
        ]
        self.next_task_index = 0

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.log_path = "/tmp/tasks.csv"
        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["task_id", "x", "y", "time"])

        self.timer = self.create_timer(0.5, self.publish_task)

    def publish_task(self):

        if self.next_task_index >= len(self.tasks):
            self.timer.cancel()
            return

        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        delay, tx, ty, task_id = self.tasks[self.next_task_index]

        if elapsed < delay:
            return

        msg = Point()
        msg.x = tx
        msg.y = ty
        msg.z = float(task_id)
        self.publisher.publish(msg)

        self.get_logger().info(f"Task {task_id} published at ({tx}, {ty})")

        t = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([task_id, tx, ty, t])
        self.log_file.flush()

        self.next_task_index += 1

        if self.next_task_index >= len(self.tasks):
            self.timer.cancel()
            self.log_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = TaskGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()