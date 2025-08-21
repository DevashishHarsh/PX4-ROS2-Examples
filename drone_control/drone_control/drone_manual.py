#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import threading
import time
from sshkeyboard import listen_keyboard

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

    def publish_trajectory_setpoint(self, x, y, z, vx,vy,vz , yaw):
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

def user_input_loop():
    global current_node
    current_node = None
    ros_thread = None
    current_drone = 1

    def start_node(drone_id):
        global current_node 
        nonlocal ros_thread
        if current_node:
            rclpy.shutdown()
            current_node.destroy_node()
            time.sleep(0.5)

        rclpy.init()
        current_node = OffboardControl(drone_id)
        ros_thread = threading.Thread(target=rclpy.spin, args=(current_node,), daemon=True)
        ros_thread.start()

    start_node(current_drone)


    print("\nCommands:")
    print("  arm")
    print("  takeoff")
    print("  move x y z")
    print("  manual")
    print("  drone sel 1,2,3")
    print("  land")
    print("  disarm")
    print("  exit\n")

    while True:
        try:
            cmd = input(">>> ").strip()
            if cmd == "":
                continue

            if cmd == "arm":
                current_node.arm()
                current_node.engage_offboard()

            elif cmd == "takeoff":
                current_node.current_setpoint = [0.0, 0.0, -5.0]
                print("Taking off to -5m...")

            elif cmd.startswith("move"):
                parts = cmd.split()
                if len(parts) != 4:
                    print("Usage: move x y z")
                    continue
                y, x, z = map(float, parts[1:])
                current_node.current_setpoint = [x, y, z]
                print(f"Moving to: {x}, {y}, {z}")

            elif cmd == "land":
                current_node.land()

            elif cmd == "disarm":
                current_node.disarm()

            elif cmd == "manual":
                current_node.flying_mode = 1
                print("Entering manual mode. Use W/A/S/D/I/J/K/L to move,  ESC to stop.")
                movement_step = 3
                yaw_step = 0.2
                movement = {"x": 0.0, "y": 0.0, "z": 0.0}
                yaw = 0.0
                manual_running = True

                def press(key):
                    nonlocal movement, yaw, movement_step
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
                    elif key == "q":
                        movement_step += 0.5
                    elif key == "e":
                        movement_step -= 0.5
                    elif key == "j":
                        yaw = yaw_step
                    elif key == "l":
                        yaw = -yaw_step
                    elif key == "esc":
                        nonlocal manual_running
                        manual_running = False
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
                        cur_yaw = current_node.current_yaw
                        new_pos = [
                            movement["y"],
                            movement["x"],
                            movement["z"]
                        ]
                        new_yaw = cur_yaw + yaw * 0.1
                        current_node.current_velocity = new_pos
                        current_node.current_yaw = new_yaw
                        time.sleep(0.01)

                loop_thread = threading.Thread(target=update_loop, daemon=True)
                loop_thread.start()

                listen_keyboard(
                    on_press=press,
                    on_release=release,
                    sequential=False
                )



            elif cmd.startswith("drone sel"):
                parts = cmd.split()
                if len(parts) != 3 or not parts[2].isdigit():
                    print("Usage: drone sel <1-3>")
                    continue
                new_drone = int(parts[2])
                if new_drone != current_drone:
                    current_drone = new_drone
                    print(f"Switching to drone {current_drone}...")
                    start_node(current_drone)
                else:
                    print(f"Drone {current_drone} already selected.")

            elif cmd == "exit":
                print("Shutting down...")
                break

            else:
                print("Unknown command.")

        except KeyboardInterrupt:
            print("Exiting.")
            break

    if current_node:
        current_node.destroy_node()
    rclpy.shutdown()


def main():
    user_input_loop()


if __name__ == '__main__':
    main()
