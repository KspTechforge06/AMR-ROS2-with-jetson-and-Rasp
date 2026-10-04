#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WarehouseTrafficController(Node):
    def __init__(self):
        super().__init__('warehouse_traffic_controller')

        self.cmd_publishers = {
            'forklift_1': self.create_publisher(Twist, '/model/forklift_1/cmd_vel', 10),
            'forklift_2': self.create_publisher(Twist, '/model/forklift_2/cmd_vel', 10),
            'forklift_3': self.create_publisher(Twist, '/model/forklift_3/cmd_vel', 10),
            'human_1': self.create_publisher(Twist, '/model/human_1/cmd_vel', 10),
            'human_2': self.create_publisher(Twist, '/model/human_2/cmd_vel', 10),
            'cart_1': self.create_publisher(Twist, '/model/cart_1/cmd_vel', 10),
            'cart_2': self.create_publisher(Twist, '/model/cart_2/cmd_vel', 10),
            'cart_3': self.create_publisher(Twist, '/model/cart_3/cmd_vel', 10),
        }

        self.dt = 0.05
        self.t = 0.0

        self.current_speed = {
            'forklift_1': 0.0,
            'forklift_2': 0.0,
            'forklift_3': 0.0,
            'human_1': 0.0,
            'human_2': 0.0,
            'cart_1': 0.0,
            'cart_2': 0.0,
            'cart_3': 0.0,
        }

        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Fixed safe warehouse traffic controller started with reduced realistic dynamic speeds.')

    def target_speed(self, speed, move_time, stop_time, phase=0.0):
        cycle = 2.0 * move_time + 2.0 * stop_time
        local_t = (self.t + phase) % cycle

        if local_t < move_time:
            return speed

        if local_t < move_time + stop_time:
            return 0.0

        if local_t < 2.0 * move_time + stop_time:
            return -speed

        return 0.0

    def ramp(self, name, target, accel_limit):
        current = self.current_speed[name]
        max_step = accel_limit * self.dt

        if target > current + max_step:
            current += max_step
        elif target < current - max_step:
            current -= max_step
        else:
            current = target

        self.current_speed[name] = current
        return current

    def make_twist(self, linear_x):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        return msg

    def update(self):
        self.t += self.dt

        targets = {
            'forklift_1': self.target_speed(0.20, 6.0, 0.5, 0.0),
            'forklift_2': self.target_speed(0.22, 5.0, 0.5, 1.0),
            'forklift_3': self.target_speed(0.22, 5.0, 0.5, 2.0),
            'human_1': self.target_speed(0.26, 14.0, 0.5, 0.0),
            'human_2': self.target_speed(0.26, 14.0, 0.5, 0.0),
            'cart_1': self.target_speed(0.20, 4.0, 0.5, 1.0),
            'cart_2': self.target_speed(0.18, 6.0, 0.5, 0.0),
            'cart_3': self.target_speed(0.14, 8.5, 0.5, 1.5),
        }

        accel_limits = {
            'forklift_1': 0.16,
            'forklift_2': 0.16,
            'forklift_3': 0.16,
            'human_1': 0.22,
            'human_2': 0.22,
            'cart_1': 0.16,
            'cart_2': 0.14,
            'cart_3': 0.11,
        }

        for name, target in targets.items():
            safe_speed = self.ramp(name, target, accel_limits[name])
            self.cmd_publishers[name].publish(self.make_twist(safe_speed))


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseTrafficController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for pub in node.cmd_publishers.values():
            pub.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()