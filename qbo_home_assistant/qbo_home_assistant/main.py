import rclpy
from .ha_bridge_node import HaBridgeNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
