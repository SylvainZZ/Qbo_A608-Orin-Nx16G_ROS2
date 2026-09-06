import rclpy
from rclpy.executors import MultiThreadedExecutor
from .home_intent_node import HomeIntentNode


def main(args=None):
    rclpy.init(args=args)
    node = HomeIntentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
