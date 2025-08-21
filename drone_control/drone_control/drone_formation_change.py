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
import numpy as np


class OffboardControl(Node):
    def __init__(self, drone_id):
        super().__init__(f'offboard_cli_control_{drone_id}')
        self.drone_id = drone_id

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        prefix = f'/px4_{drone_id}/fmu'

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', qos_pub)
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f'{prefix}/out/vehicle_local_position', self.local_position_cb, qos_sub)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, f'{prefix}/out/vehicle_status', self.status_cb, qos_sub)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_started = False
        self.current_setpoint = [0.0, 0.0, 0.0]
        self.base_setpoint = [0.0, 0.0, 0.0]
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
            msg.velocity = [vx, vy, vz]
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


def multi_drone_move(targets: list):
    rclpy.init()
    drone_nodes = {}
    drone_number = len(targets[0])
    movement = {"x": 0.0, "y": 0.0, "z": 0.0}
    yaw = 0.0
    manual_running = False
    leader_id =  1
    current_set_index = 0

    def compute_formation(set_index):
        target = targets[set_index]
        x_vals = [float(pos[0]) for k, pos in target.items()]
        y_vals = [float(pos[1]) for k, pos in target.items()]
        center_of_mass = [float(np.mean(x_vals)) - leader_id, float(np.mean(y_vals)), -10.0]
        offsets = [None] * (drone_number + 2)
        for drone_id_str, position in target.items():
            drone_id = int(drone_id_str)
            world_x = position[0] - drone_id
            offsets[drone_id] = [
                world_x - center_of_mass[0],
                position[1] - center_of_mass[1],
                position[2] - center_of_mass[2]
            ]
        offsets[leader_id] = [0.0, 0.0, 0.0]
        return center_of_mass, offsets

    center_of_mass, offsets = compute_formation(current_set_index)
    print(f"Initial center of mass: {center_of_mass}")
    print(f"Initial offsets: {offsets}")

    def run_node(drone_id, final_position, is_leader=False):
        node = OffboardControl(drone_id)
        drone_nodes[drone_id] = node
        node.base_setpoint = final_position.copy()

        def spin_node():
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin()

        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()

        time.sleep(2.0)

        takeoff_position = [0.0, 0.0, -10.0 if is_leader else -5.0]
        node.current_setpoint = takeoff_position
        print(f"Drone {drone_id}{' (Leader)' if is_leader else ''} taking off to {takeoff_position}")

        for _ in range(20):
            node.publish_offboard_control_mode()
            node.publish_trajectory_setpoint(*takeoff_position, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)

        node.engage_offboard()
        node.arm()

        time.sleep(2.0)

        if not is_leader:
            node.current_setpoint = final_position
            print(f"Drone {drone_id} moving to final destination {final_position}")
            for _ in range(20):
                node.publish_trajectory_setpoint(*final_position, 0.0, 0.0, 0.0, 0.0)
                time.sleep(0.1)
        else:
            node.current_setpoint = final_position
            print(f"Drone {drone_id} (Leader) at center of mass {final_position}")

        if not is_leader:
            node.current_setpoint = final_position.copy()

    def press(key):
        nonlocal movement, yaw, manual_running, current_set_index
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
        elif key == "n":
            if current_set_index + 1 < len(targets):
                current_set_index += 1
                print(f"Moving to point set {current_set_index}")
                nonlocal center_of_mass, offsets
                center_of_mass, offsets = compute_formation(current_set_index)
                print(f"New center of mass: {center_of_mass}")
                print(f"New offsets: {offsets}")
                for drone_id, node in drone_nodes.items():
                    node.flying_mode = 0
                    node.current_velocity = [0.0, 0.0, 0.0]
                    node.current_yaw = 0.0
                    if drone_id == leader_id:
                        node.base_setpoint = center_of_mass.copy()
                        node.current_setpoint = center_of_mass.copy()
                    else:
                        world_x = targets[current_set_index][str(drone_id)][0] - drone_id
                        final_position = [world_x, targets[current_set_index][str(drone_id)][1], targets[current_set_index][str(drone_id)][2]]
                        node.base_setpoint = final_position.copy()
                        node.current_setpoint = final_position.copy()
                    print(f"Drone {drone_id} moving to new setpoint {node.current_setpoint}")
                stabilize_formation()
                print(f"Reached point set {current_set_index}. Resuming manual mode.")
                drone_nodes[leader_id].flying_mode = 1
                drone_nodes[leader_id].current_setpoint = [float('nan'), float('nan'), float('nan')]
                drone_nodes[leader_id].current_velocity = [0.0, 0.0, 0.0]
            else:
                print("No more point sets available.")
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

    def is_valid_position(pos):
        return all(isinstance(v, float) and not np.isnan(v) and abs(v) < 1e6 for v in [pos.x, pos.y, pos.z])

    def stabilize_formation():
        print("Stabilizing formation...")
        start_time = time.time()
        while time.time() - start_time < 15.0:
            all_ready = True
            for drone_id, node in drone_nodes.items():
                if is_valid_position(node.vehicle_local_position):
                    error = [
                        node.current_setpoint[0] - node.vehicle_local_position.x,
                        node.current_setpoint[1] - node.vehicle_local_position.y,
                        node.current_setpoint[2] - node.vehicle_local_position.z
                    ]
                    if any(abs(e) > 0.5 for e in error):  # 0.5m tolerance
                        all_ready = False
                        print(f"Drone {drone_id}: Error {error}, stabilizing...")
                else:
                    all_ready = False
                    print(f"Drone {drone_id}: Invalid position [{node.vehicle_local_position.x}, {node.vehicle_local_position.y}, {node.vehicle_local_position.z}]")
                node.publish_trajectory_setpoint(*node.current_setpoint, 0.0, 0.0, 0.0, 0.0)
            if all_ready:
                print("All drones reached new setpoints.")
                break
            time.sleep(0.1)

    def update_loop():
        kp = 0.5
        dt = 0.01
        while manual_running:
            leader_node = drone_nodes[leader_id]
            leader_node.flying_mode = 1
            leader_node.current_velocity = [movement["y"], movement["x"], movement["z"]]
            leader_node.current_setpoint = [float('nan'), float('nan'), float('nan')]
            leader_node.current_yaw += yaw * dt

            leader_pos = [
                leader_node.vehicle_local_position.x,
                leader_node.vehicle_local_position.y,
                leader_node.vehicle_local_position.z
            ]

            for drone_id, node in drone_nodes.items():
                if drone_id != leader_id:
                    desired_pos = [
                        leader_pos[0] + offsets[drone_id][0],
                        leader_pos[1] + offsets[drone_id][1],
                        leader_pos[2] + offsets[drone_id][2]
                    ]
                    pos_error = [0.0, 0.0, 0.0]
                    if is_valid_position(node.vehicle_local_position):
                        pos_error = [
                            desired_pos[0] - node.vehicle_local_position.x,
                            desired_pos[1] - node.vehicle_local_position.y,
                            desired_pos[2] - node.vehicle_local_position.z
                        ]
                    node.flying_mode = 0
                    node.current_velocity = [kp * e for e in pos_error]
                    node.current_setpoint = desired_pos
                    node.current_yaw += yaw * dt

            time.sleep(dt)

    for drone_id_str, position in targets[0].items():
        drone_id = int(drone_id_str)
        world_x = position[0] - drone_id
        world_pos = [world_x, position[1], position[2]]
        run_node(drone_id, world_pos, is_leader=False)
        time.sleep(3.0)

    run_node(leader_id, center_of_mass, is_leader=True)

    print("Waiting for initial formation to stabilize...")
    stabilize_formation()

    print("\nFormation complete. Entering manual mode. Use W/A/S/D/I/J/K/L to control, N for next point set, ESC to stop.")
    print(f"Drone nodes: {list(drone_nodes.keys())}")

    manual_running = True
    for node in drone_nodes.values():
        node.flying_mode = 0
        if node.drone_id == leader_id:
            node.flying_mode = 1
            node.current_setpoint = [float('nan'), float('nan'), float('nan')]
            node.current_velocity = [0.0, 0.0, 0.0]

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
    json_path = "/mnt/c/Users/daisu/Downloads/Projects/Drone/drone_points.json"
    if not os.path.exists(json_path):
        print(f"File not found: {json_path}")
        return

    with open(json_path, "r") as f:
        drone_targets = json.load(f)

    # Convert dict to list of dicts for sequential access
    max_index = max(int(k) for k in drone_targets.keys())
    targets_list = [drone_targets[str(i)] for i in range(max_index + 1)]
    multi_drone_move(targets_list)


if __name__ == '__main__':
    main()