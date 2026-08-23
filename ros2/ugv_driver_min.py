#!/usr/bin/env python3
"""Minimal Waveshare UGV ESP32 bridge for headless ROS 2 teleop.

Matches the stock ugv_bringup/ugv_driver serial contract used by Flask ros_motion:
  /cmd_vel  → JSON {"T":13,"X":lin,"Z":ang}
  /joint_states or /ugv/joint_states (PT joint names) → JSON T:134 degrees
  /ugv/led_ctrl (Float32MultiArray) → JSON T:132 IO4/IO5 (base/head lights)

Runs inside dudulrx0601/ugv_rpi_ros_humble (rclpy + pyserial) without full ugv_ws.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

try:
    import serial
except ImportError as e:  # pragma: no cover
    print("pyserial required:", e, file=sys.stderr)
    raise


def _serial_port() -> str:
    return (
        os.environ.get("UGV_SERIAL_DEV")
        or os.environ.get("UGV_SERIAL_PORT")
        or "/dev/ttyAMA0"
    )


class UgvDriverMin(Node):
    def __init__(self) -> None:
        super().__init__("ugv_driver")
        port = _serial_port()
        baud = int(os.environ.get("UGV_SERIAL_BAUD") or "115200")
        self.get_logger().info(f"Opening serial {port} @ {baud}")
        self.ser = serial.Serial(port, baud, timeout=1)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)
        # Stock Waveshare uses ugv/joint_states; Flask ros_motion uses /joint_states
        self.create_subscription(JointState, "ugv/joint_states", self._joint_cb, 10)
        self.create_subscription(JointState, "joint_states", self._joint_cb, 10)
        # Stock ugv_driver: /ugv/led_ctrl → T:132 (base/head 12V switches)
        self.create_subscription(Float32MultiArray, "ugv/led_ctrl", self._led_cb, 10)
        # Lights off on start so a stuck-on state from prior Flask ROS drop is cleared
        self._write({"T": 132, "IO4": 0, "IO5": 0})
        self.get_logger().info(
            "ugv_driver_min ready: cmd_vel + joint_states + ugv/led_ctrl → ESP32 JSON"
        )

    def _write(self, payload: dict) -> None:
        line = (json.dumps(payload) + "\n").encode("utf-8")
        try:
            self.ser.write(line)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"serial write failed: {e}")

    def _cmd_vel_cb(self, msg: Twist) -> None:
        linear_velocity = float(msg.linear.x)
        angular_velocity = float(msg.angular.z)
        # Stock threshold when spinning in place
        if linear_velocity == 0.0:
            if 0.0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0.0:
                angular_velocity = -0.2
        # Stock uses string "13" for T; ESP32 accepts it
        self._write({"T": "13", "X": linear_velocity, "Z": angular_velocity})

    def _joint_cb(self, msg: JointState) -> None:
        try:
            names = list(msg.name)
            pos = list(msg.position)
            x_rad = pos[names.index("pt_base_link_to_pt_link1")]
            y_rad = pos[names.index("pt_link1_to_pt_link2")]
        except (ValueError, IndexError):
            return
        x_deg = (180.0 * x_rad) / math.pi
        y_deg = (180.0 * y_rad) / math.pi
        self._write({"T": 134, "X": x_deg, "Y": y_deg, "SX": 600, "SY": 600})

    def _led_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data) if msg.data is not None else []
        io4 = float(data[0]) if len(data) > 0 else 0.0
        io5 = float(data[1]) if len(data) > 1 else 0.0
        io4 = max(0.0, min(255.0, io4))
        io5 = max(0.0, min(255.0, io5))
        self._write({"T": 132, "IO4": io4, "IO5": io5})

    def destroy_node(self) -> bool:
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    # Retry serial open a few times (UART handoff races with Flask release)
    last_err: Exception | None = None
    for attempt in range(1, 11):
        try:
            rclpy.init()
            node = UgvDriverMin()
            break
        except Exception as e:
            last_err = e
            print(f"[ugv_driver_min] serial/init attempt {attempt}/10 failed: {e}")
            try:
                rclpy.shutdown()
            except Exception:
                pass
            time.sleep(1.0)
    else:
        raise SystemExit(f"Failed to start ugv_driver_min: {last_err}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
