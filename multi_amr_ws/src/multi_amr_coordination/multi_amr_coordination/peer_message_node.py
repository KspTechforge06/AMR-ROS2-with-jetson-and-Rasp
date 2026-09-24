"""Demonstrate direct ROS 2 message exchange between simulated AMR nodes."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PeerMessageNode(Node):
    """Logs messages published by other robots on their namespaced topics."""

    def __init__(self):
        super().__init__('peer_message_node')
        self.declare_parameter('robot_name', 'r1')
        self.declare_parameter('peer_names', ['r1', 'r2', 'r3', 'r4', 'r5'])
        self.robot_name = self.get_parameter('robot_name').value
        peer_names = list(self.get_parameter('peer_names').value)

        for peer in peer_names:
            self.create_subscription(
                String,
                f'/{peer}/message',
                lambda message, sender=peer: self.message_callback(sender, message),
                10,
            )
        self.get_logger().info(
            f'{self.robot_name}: listening for messages from {", ".join(peer_names)}')

    def message_callback(self, sender, message):
        if sender != self.robot_name:
            self.get_logger().info(f'message from {sender}: {message.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PeerMessageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
