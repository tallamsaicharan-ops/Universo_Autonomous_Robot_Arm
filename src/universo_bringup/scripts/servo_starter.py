#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoStarter(Node):
    def __init__(self):
        super().__init__("servo_starter")
        self.client = self.create_client(
            Trigger, "/servo_node/start_servo"
        )
        self.timer = self.create_timer(1.0, self.try_start)
        self.started = False
        self.attempts = 0

    def try_start(self):
        if self.started:
            return

        if not self.client.service_is_ready():
            self.attempts += 1
            if self.attempts % 5 == 0:
                self.get_logger().info(
                    "Waiting for /servo_node/start_servo..."
                )
            return

        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self.on_response)
        self.timer.cancel()

    def on_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.started = True
                self.get_logger().info(
                    "MoveIt Servo started successfully."
                )
            else:
                self.get_logger().error(
                    f"Servo did not start: {response.message}"
                )
        except Exception as exc:
            self.get_logger().error(f"Servo start failed: {exc}")


def main():
    rclpy.init()
    node = ServoStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
