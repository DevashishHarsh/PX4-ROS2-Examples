#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sshkeyboard import listen_keyboard
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
        self.base_setpoint = [0.0, 0.0, 0.0]  # Store initial formation position
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
    movement = {"x": 0.0, "y": 0.0, "z": 0.0}
    yaw = 0.0
    manual_running = False

    def run_node(drone_id, final_position):
        node = OffboardControl(drone_id)
        drone_nodes[drone_id] = node
        node.base_setpoint = final_position.copy()  # Store initial formation position

        def spin_node():
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin()

        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()

        # Wait for ROS setup
        time.sleep(1.0)

        # Takeoff to z = -5 
        takeoff_position = [0, 0, -5.0]
        node.current_setpoint = takeoff_position
        print(f"Drone {drone_id} taking off")

        for _ in range(15):
            node.publish_offboard_control_mode()
            node.publish_trajectory_setpoint(*takeoff_position, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)

        node.engage_offboard()
        node.arm()

        time.sleep(1.0)  # hover at takeoff altitude

        # Move to final destination 
        node.current_setpoint = final_position
        print(f"Drone {drone_id} moving to final destination {final_position}")

        for _ in range(15):
            node.publish_trajectory_setpoint(*final_position, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)

    def press(key):
        nonlocal movement, yaw, manual_running
        movement_step = 3.0
        yaw_step = 0.2
        if key == "w":
            movement["x"] = movement_step
        elif key == "s":
            movement["x"] = -movement_step
        elif key == "a":
            movement["y"] = movement_step
        elif key == "d":
            movement["y"] = -movement_step
        elif key == "i":
            movement["z"] = -movement_step
        elif key == "k":
            movement["z"] = movement_step
        elif key == "j":
            yaw = yaw_step
        elif key == "l":
            yaw = -yaw_step
        elif key == "esc":
            manual_running = False
            for node in drone_nodes.values():
                node.flying_mode = 0
                node.current_velocity = [0.0, 0.0, 0.0]
                node.current_yaw = 0.0
            print("\nExited manual mode.")

    def release(key):
        nonlocal movement, yaw
        if key in ["w", "s"]:
            movement["x"] = 0.0
        elif key in ["a", "d"]:
            movement["y"] = 0.0
        elif key in ["i", "k"]:
            movement["z"] = 0.0
        elif key in ["j", "l"]:
            yaw = 0.0

    def update_loop():
        while manual_running:
            for node in drone_nodes.values():
                node.current_velocity = [movement["y"], movement["x"], movement["z"]]
                node.current_yaw += yaw * 0.1
            time.sleep(0.01)

    # Start all drones
    for drone_id_str, position in targets.items():
        drone_id = int(drone_id_str)
        world_x = position[0] - drone_id
        world_pos = [world_x, position[1], position[2]]
        run_node(drone_id, world_pos)
        time.sleep(2)  # slight delay between each drone

    # Wait for all drones to reach formation
    time.sleep(5.0)
    print("\nFormation complete. Entering manual mode. Use W/A/S/D/I/J/K/L to control all drones, ESC to stop.")

    # Start manual control
    manual_running = True
    for node in drone_nodes.values():
        node.flying_mode = 1

    loop_thread = threading.Thread(target=update_loop, daemon=True)
    loop_thread.start()

    listen_keyboard(
        on_press=press,
        on_release=release,
        sequential=False
    )

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down all drones...")
    finally:
        for node in drone_nodes.values():
            node.land()
            time.sleep(1)
            node.disarm()
            node.destroy_node()
        rclpy.shutdown()


def main():
    json_path = "points/drone_points.json"
    if not os.path.exists(json_path):
        print(f"File not found: {json_path}")
        return

    with open(json_path, "r") as f:
        drone_targets = json.load(f)

    multi_drone_move(drone_targets)


if __name__ == '__main__':
    main()