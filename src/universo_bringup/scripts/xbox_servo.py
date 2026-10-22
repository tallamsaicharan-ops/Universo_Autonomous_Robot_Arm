#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class XboxServo(Node):
    def __init__(self):
        super().__init__('xbox_servo')

        self.last_joy = None
        self.last_joy_time = 0.0
        self.deadzone = 0.12

        self.speed_x = 1.0
        self.speed_y = 1.0
        self.speed_z = 1.0

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        self.timer = self.create_timer(0.02, self.publish_command)

        self.get_logger().info(
            'Ready: hold LB to enable. RT=+X, RB=-X, left horizontal=Y, right vertical=Z.'
        )

    def joy_callback(self, msg):
        self.last_joy = msg
        self.last_joy_time = time.monotonic()

    def stick(self, value):
        return 0.0 if abs(value) < self.deadzone else float(value)

    def trigger_pressed(self, value):
        # Your controller: +1 at rest, -1 fully pressed.
        # Convert to 0 at rest and 1 when fully pressed.
        amount = (1.0 - float(value)) / 2.0
        return 0.0 if amount < self.deadzone else min(1.0, amount)

    def publish_command(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'arm_tip'

        joy = self.last_joy
        fresh = (
            joy is not None
            and time.monotonic() - self.last_joy_time < 0.25
        )

        enabled = (
            fresh
            and len(joy.buttons) > 7
            and len(joy.axes) > 4
            and joy.buttons[6] == 1
        )

        if enabled:
            forward = self.trigger_pressed(joy.axes[4])
            backward = 1.0 if joy.buttons[7] else 0.0

            # Opposing controls cancel each other.
            cmd.twist.linear.x = self.speed_x * (forward - backward)

            # Reverse left horizontal direction.
            cmd.twist.linear.y = self.stick(joy.axes[0]) * self.speed_y

            # Right vertical controls Z.
            cmd.twist.linear.z = self.stick(joy.axes[3]) * self.speed_z

        # When LB is released, publish zero velocity.
        self.cmd_pub.publish(cmd)

    def stop(self):
        for _ in range(5):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'arm_tip'
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = XboxServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
