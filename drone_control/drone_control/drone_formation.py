#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import threading
import time
import json
import os


class OffboardControl(Node):
    def __init__(self, drone_id):
        super().__init__(f'offboard_cli_control_{drone_id}')
        self.drone_id = drone_id

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        prefix = f'/px4_{drone_id}/fmu'

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', qos)
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f'{prefix}/out/vehicle_local_position', self.local_position_cb, qos)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, f'{prefix}/out/vehicle_status', self.status_cb, qos)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_started = False
        self.current_setpoint = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.offboard_counter = 0
        self.flying_mode = 0

        self.timer = self.create_timer(0.1, self.timer_cb)

    def local_position_cb(self, msg):
        self.vehicle_local_position = msg

    def status_cb(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        msg.velocity = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, vx, vy, vz, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        if self.flying_mode == 0:
            msg.position = [x, y, z]
            msg.velocity = [0.0, 0.0, 0.0]
        elif self.flying_mode == 1:
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        msg.target_system = self.drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_cb(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(*self.current_setpoint, *self.current_velocity, self.current_yaw)
        if not self.offboard_started:
            if self.offboard_counter == 10:
                self.engage_offboard()
                self.arm()
                self.offboard_started = True
            else:
                self.offboard_counter += 1


def multi_drone_move(targets: dict):
    rclpy.init()
    drone_nodes = {}

    def run_node(drone_id, position):
        node = OffboardControl(drone_id)
        drone_nodes[drone_id] = node

        def spin_node():
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin()

        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()

        # Wait for publisher setup
        time.sleep(1.0)

        node.current_setpoint = position

        for _ in range(15):
            node.publish_offboard_control_mode()
            node.publish_trajectory_setpoint(*position, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)

        node.engage_offboard()
        node.arm()

        print(f"Drone {drone_id} is flying to {position}")


    for drone_id_str, position in targets.items():
        drone_id = int(drone_id_str)
        run_node(drone_id, position)
        time.sleep(0.2)  # slight delay between drone launches

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down all drones...")
    finally:
        for node in drone_nodes.values():
            node.destroy_node()
        rclpy.shutdown()


def main():
    # Load drone target points from JSON file
    json_path = "points/drone_points.json"
    if not os.path.exists(json_path):
        print(f"File not found: {json_path}")
        return

    with open(json_path, "r") as f:
        drone_targets = json.load(f)

    multi_drone_move(drone_targets)


if __name__ == '__main__':
    main()
