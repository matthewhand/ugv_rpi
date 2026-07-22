"""
ROS 2 motion bridge for the Flask AI agent.

Flask (host) → rosbridge WebSocket → ROS graph in ugv_ros2 (host network)
  → /cmd_vel → ugv_bringup → ESP32 serial
  → /joint_states (PT joint names) → ugv_bringup → T:133 serial (physical robot)
  → /pt_joint_position_controller/commands → ros2_control (gazebo / full stack)

Configure via env (ugv_rpi/.env):
  UGV_MOTION_BACKEND=ros2|serial|none   (default: none)
  UGV_PT_BACKEND=auto|ros2|serial       (default: auto)
  ROSBRIDGE_URL=ws://127.0.0.1:9090
  UGV_CMD_VEL_TOPIC=/cmd_vel
  UGV_PT_JOINT_TOPIC=/pt_joint_position_controller/commands
  UGV_JOINT_STATES_TOPIC=/joint_states
  UGV_MAX_LINEAR=0.35
  UGV_MAX_ANGULAR=0.8
  UGV_MAX_DRIVE_MS=4000

Prerequisites for ROS chassis/PT:
  ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Set, Tuple

try:
    import websocket  # websocket-client
except ImportError:  # pragma: no cover
    websocket = None


def motion_backend() -> str:
    return (os.environ.get('UGV_MOTION_BACKEND') or 'none').strip().lower()


def pt_backend() -> str:
    """auto | ros2 | serial — how Flask should command pan/tilt."""
    return (os.environ.get('UGV_PT_BACKEND') or 'auto').strip().lower()


def rosbridge_url() -> str:
    return (os.environ.get('ROSBRIDGE_URL') or 'ws://127.0.0.1:9090').strip()


def cmd_vel_topic() -> str:
    return (os.environ.get('UGV_CMD_VEL_TOPIC') or '/cmd_vel').strip()


def pt_joint_topic() -> str:
    return (os.environ.get('UGV_PT_JOINT_TOPIC') or '/pt_joint_position_controller/commands').strip()


def joint_states_topic() -> str:
    return (os.environ.get('UGV_JOINT_STATES_TOPIC') or '/joint_states').strip()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _limits() -> Tuple[float, float, int]:
    max_lin = float(os.environ.get('UGV_MAX_LINEAR') or 0.35)
    max_ang = float(os.environ.get('UGV_MAX_ANGULAR') or 0.8)
    max_ms = int(os.environ.get('UGV_MAX_DRIVE_MS') or 4000)
    return max_lin, max_ang, max_ms


class RosbridgeClient:
    """Rosbridge publisher. Can be used one-shot or kept open for high-rate PT."""

    def __init__(self, url: Optional[str] = None, timeout: float = 3.0):
        self.url = url or rosbridge_url()
        self.timeout = timeout
        self._ws = None
        self._id = 0
        self._lock = threading.Lock()
        self._advertised: Set[Tuple[str, str]] = set()

    def _next_id(self) -> str:
        self._id += 1
        return str(self._id)

    def connect(self) -> None:
        if websocket is None:
            raise RuntimeError('websocket-client not installed')
        self._ws = websocket.create_connection(self.url, timeout=self.timeout)
        self._advertised.clear()

    def close(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None
            self._advertised.clear()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def ensure(self) -> None:
        if self._ws is None:
            self.connect()

    def send(self, payload: dict) -> None:
        if self._ws is None:
            raise RuntimeError('not connected')
        self._ws.send(json.dumps(payload))

    def advertise(self, topic: str, msg_type: str) -> None:
        key = (topic, msg_type)
        if key in self._advertised:
            return
        self.send({
            'op': 'advertise',
            'id': f'advertise:{topic}:{self._next_id()}',
            'topic': topic,
            'type': msg_type,
        })
        self._advertised.add(key)

    def publish(self, topic: str, msg: dict, msg_type: Optional[str] = None) -> None:
        if msg_type:
            self.advertise(topic, msg_type)
        self.send({
            'op': 'publish',
            'id': f'publish:{topic}:{self._next_id()}',
            'topic': topic,
            'msg': msg,
        })

    def unadvertise(self, topic: str) -> None:
        self.send({
            'op': 'unadvertise',
            'id': f'unadvertise:{topic}:{self._next_id()}',
            'topic': topic,
        })
        self._advertised = {k for k in self._advertised if k[0] != topic}


# Persistent client for stick-rate pan/tilt (avoid open/close every move).
# NOTE: do not name the global the same as the accessor function (that shadows
# the function after first call → "'RosbridgeClient' object is not callable").
_pt_ws: Optional[RosbridgeClient] = None
_pt_client_lock = threading.Lock()
_pt_last_pub = 0.0
_PT_MIN_INTERVAL = 0.04  # ~25 Hz max


def _get_pt_client() -> RosbridgeClient:
    global _pt_ws
    with _pt_client_lock:
        if _pt_ws is None:
            _pt_ws = RosbridgeClient(timeout=2.0)
        try:
            _pt_ws.ensure()
        except Exception:
            try:
                _pt_ws.close()
            except Exception:
                pass
            _pt_ws = RosbridgeClient(timeout=2.0)
            _pt_ws.ensure()
        return _pt_ws


def _twist_msg(linear_x: float = 0.0, angular_z: float = 0.0) -> dict:
    return {
        'linear': {'x': float(linear_x), 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular_z)},
    }


def publish_cmd_vel(linear_x: float = 0.0, angular_z: float = 0.0) -> Dict[str, Any]:
    topic = cmd_vel_topic()
    msg_type = os.environ.get('UGV_CMD_VEL_TYPE') or 'geometry_msgs/msg/Twist'
    msg = _twist_msg(linear_x, angular_z)
    with RosbridgeClient() as client:
        client.publish(topic, msg, msg_type)
        time.sleep(0.02)
    return {
        'ok': True,
        'backend': 'ros2',
        'topic': topic,
        'linear_x': linear_x,
        'angular_z': angular_z,
    }


def ui_xy_to_radians(x_deg: float, y_deg: float) -> Tuple[float, float]:
    """Map Waveshare Flask stick degrees (T:133 X/Y) → ROS joint radians.

    ugv_bringup does: X_cmd = -x_deg, Y_cmd = y_deg where x_deg = 180*rad/pi
    so pan_rad = -X * pi/180, tilt_rad = Y * pi/180
    """
    pan = -float(x_deg) * math.pi / 180.0
    tilt = float(y_deg) * math.pi / 180.0
    # Match keyboard_ctrl limits
    pan = _clamp(pan, -3.14, 3.14)
    tilt = _clamp(tilt, -0.523, 1.57)
    return pan, tilt


def publish_gimbal(pan_rad: float, tilt_rad: float, throttle: bool = True) -> Dict[str, Any]:
    """Publish pan/tilt for physical + sim stacks.

    1) sensor_msgs/JointState on /joint_states — ugv_bringup maps to ESP32 T:133
    2) Float64MultiArray on pt_joint_position_controller/commands — joy/vision/gazebo
    """
    global _pt_last_pub
    if throttle:
        now = time.time()
        if now - _pt_last_pub < _PT_MIN_INTERVAL:
            return {'ok': True, 'backend': 'ros2', 'throttled': True}
        _pt_last_pub = now

    pan_rad = _clamp(float(pan_rad), -3.14, 3.14)
    tilt_rad = _clamp(float(tilt_rad), -0.523, 1.57)

    js_topic = joint_states_topic()
    js_type = os.environ.get('UGV_JOINT_STATES_TYPE') or 'sensor_msgs/msg/JointState'
    joint_msg = {
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
        'name': ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2'],
        'position': [pan_rad, tilt_rad],
        'velocity': [],
        'effort': [],
    }

    cmd_topic = pt_joint_topic()
    cmd_type = os.environ.get('UGV_PT_JOINT_TYPE') or 'std_msgs/msg/Float64MultiArray'
    cmd_msg = {'data': [pan_rad, tilt_rad]}

    client = _get_pt_client()
    with _pt_client_lock:
        client.publish(js_topic, joint_msg, js_type)
        client.publish(cmd_topic, cmd_msg, cmd_type)

    return {
        'ok': True,
        'backend': 'ros2',
        'topics': [js_topic, cmd_topic],
        'pan_rad': pan_rad,
        'tilt_rad': tilt_rad,
    }


def publish_gimbal_from_ui(x_deg: float, y_deg: float, throttle: bool = True) -> Dict[str, Any]:
    """UI stick / T:133 X,Y (degrees-ish) → ROS pan/tilt."""
    pan, tilt = ui_xy_to_radians(x_deg, y_deg)
    out = publish_gimbal(pan, tilt, throttle=throttle)
    out['ui_x_deg'] = float(x_deg)
    out['ui_y_deg'] = float(y_deg)
    return out


def prefer_ros_for_pt() -> bool:
    """Whether Flask should route PT over rosbridge (vs serial T:133)."""
    mode = pt_backend()
    if mode == 'serial':
        return False
    if mode == 'ros2':
        return True
    # auto: use ROS if motion backend is ros2 and bridge is up
    if motion_backend() != 'ros2':
        return False
    return bool(rosbridge_status().get('ok'))


def ros_stop() -> Dict[str, Any]:
    return publish_cmd_vel(0.0, 0.0)


def ros_drive(linear_x: float, angular_z: float = 0.0, duration_ms: int = 0) -> Dict[str, Any]:
    max_lin, max_ang, max_ms = _limits()
    lin = _clamp(float(linear_x), -max_lin, max_lin)
    ang = _clamp(float(angular_z), -max_ang, max_ang)
    dur = int(duration_ms or 0)
    if dur < 0:
        dur = 0
    if dur > max_ms:
        dur = max_ms

    result = publish_cmd_vel(lin, ang)
    result['duration_ms'] = dur
    result['clamped'] = {
        'linear_x': lin,
        'angular_z': ang,
        'max_linear': max_lin,
        'max_angular': max_ang,
        'max_drive_ms': max_ms,
    }
    if dur > 0:
        time.sleep(dur / 1000.0)
        stop = ros_stop()
        result['stopped'] = stop.get('ok', False)
    return result


def rosbridge_status() -> Dict[str, Any]:
    """Probe rosbridge connectivity (does not require bringup)."""
    url = rosbridge_url()
    if websocket is None:
        return {'ok': False, 'url': url, 'error': 'websocket-client missing'}
    try:
        with RosbridgeClient(url, timeout=2.0) as client:
            client.send({'op': 'status', 'level': 'none', 'msg': 'ping'})
        return {'ok': True, 'url': url}
    except Exception as e:
        return {'ok': False, 'url': url, 'error': str(e)}


def openai_motion_tools() -> list:
    """OpenAI Chat Completions `tools` entries for chassis/gimbal motion."""
    return [
        {
            'type': 'function',
            'function': {
                'name': 'send_motor_command',
                'description': (
                    'Drive the rover chassis. Path follows Flask control_mode '
                    '(direct serial T:13 or ROS 2 /cmd_vel). '
                    'linear_x is forward m/s (positive=forward), angular_z is yaw rad/s '
                    '(positive=left). Optional duration_ms auto-stops after that time. '
                    'Values are hard-clamped server-side for safety.'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'linear_x': {'type': 'number', 'description': 'Forward velocity m/s'},
                        'angular_z': {'type': 'number', 'description': 'Yaw rate rad/s'},
                        'duration_ms': {
                            'type': 'integer',
                            'description': 'Drive duration then stop (0 = continuous until stop_motors)',
                        },
                    },
                    'required': ['linear_x'],
                },
            },
        },
        {
            'type': 'function',
            'function': {
                'name': 'stop_motors',
                'description': 'Emergency stop wheels on the active control path.',
                'parameters': {'type': 'object', 'properties': {}},
            },
        },
        {
            'type': 'function',
            'function': {
                'name': 'send_gimbal_command',
                'description': (
                    'Move pan/tilt camera head (direct serial T:133 or ROS joints per control_mode). '
                    'pan_rad / tilt_rad in radians (approx ±1.0 safe).'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'pan_rad': {'type': 'number'},
                        'tilt_rad': {'type': 'number'},
                    },
                    'required': ['pan_rad', 'tilt_rad'],
                },
            },
        },
    ]


def execute_motion_tool(name: str, arguments: dict) -> Dict[str, Any]:
    """Execute motion over rosbridge. Caller (app.py) chooses direct vs ros2.

    Kept for ROS path only; direct serial is handled in app._execute_motion_via_mode.
    """
    args = arguments or {}
    try:
        if name == 'stop_motors':
            out = ros_stop()
            out['control_mode'] = 'ros2'
            return out
        if name == 'send_motor_command':
            out = ros_drive(
                linear_x=float(args.get('linear_x', 0.0)),
                angular_z=float(args.get('angular_z', 0.0)),
                duration_ms=int(args.get('duration_ms') or 0),
            )
            out['control_mode'] = 'ros2'
            return out
        if name == 'send_gimbal_command':
            pan = _clamp(float(args.get('pan_rad', 0.0)), -1.2, 1.2)
            tilt = _clamp(float(args.get('tilt_rad', 0.0)), -1.0, 0.6)
            out = publish_gimbal(pan, tilt)
            out['control_mode'] = 'ros2'
            return out
        return {'ok': False, 'error': f'unknown motion tool: {name}'}
    except Exception as e:
        return {'ok': False, 'error': str(e), 'tool': name, 'control_mode': 'ros2'}
