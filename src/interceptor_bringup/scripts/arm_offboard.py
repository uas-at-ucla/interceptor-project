#!/usr/bin/env python3
"""Switch the drone to OFFBOARD and arm it, then exit.

SIM / TEST TOOL ONLY. On a real flight the safety pilot arms the drone and
switches to OFFBOARD from the RC. Never run this on the real drone.

The manager node never arms the drone or changes its mode. This script does
that once the manager is ready:
  1. Wait for MAVROS to connect to PX4.
  2. Wait until the manager has streamed setpoints for about a second.
     PX4 rejects OFFBOARD unless setpoints are already arriving.
  3. Request OFFBOARD, then arming, retrying every second until both stick.
"""

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node

# About one second of the manager's 20 Hz setpoint stream.
SETPOINTS_BEFORE_OFFBOARD = 20


class ArmOffboard(Node):

    def __init__(self):
        super().__init__("arm_offboard")
        self.state = State()
        self.setpoint_count = 0
        self.done = False

        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(
            PositionTarget, "/mavros/setpoint_raw/local", self.on_setpoint, 10)
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        # Once a second: slow enough not to spam PX4 with repeated requests.
        self.create_timer(1.0, self.step)

    def on_state(self, msg):
        self.state = msg

    def on_setpoint(self, msg):
        self.setpoint_count += 1

    def step(self):
        if not self.state.connected:
            self.get_logger().info("Waiting for MAVROS to connect to PX4...")
        elif self.setpoint_count < SETPOINTS_BEFORE_OFFBOARD:
            self.get_logger().info("Waiting for the manager's setpoints...")
        elif self.state.mode != "OFFBOARD":
            self.get_logger().info(f"Requesting OFFBOARD (mode is {self.state.mode})")
            self.mode_client.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        elif not self.state.armed:
            self.get_logger().info("Requesting arm")
            self.arm_client.call_async(CommandBool.Request(value=True))
        else:
            self.get_logger().info("Armed and in OFFBOARD; done")
            self.done = True


def main():
    rclpy.init()
    node = ArmOffboard()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
