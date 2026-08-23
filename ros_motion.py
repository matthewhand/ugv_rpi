"""
ROS 2 motion bridge for the Flask AI agent.

Flask (host) → rosbridge WebSocket → ROS graph in ugv_ros2 (host network :9090)
  → /cmd_vel → chassis driver (ugv_driver_min or ugv_bringup) → ESP32 serial
  → /joint_states (PT joint names) → ugv_driver_min T:134 or ugv_bringup T:133
  → /pt_joint_position_controller/commands → ros2_control (gazebo / full stack)

Beast compose runs ugv_driver_min (no ugv_ws required). Rover images may have ugv_bringup.

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


def preferred_motion_path(control_mode: str, rosbridge_ok: bool) -> str:
    """Choose software path for chassis/gimbal commands.

    - direct → always serial
    - ros2 + rosbridge up → ros2
    - ros2 + rosbridge down → serial_fallback (caller must reclaim UART)

    Pure helper so unit tests can drive path selection without Flask/hardware.
    """
    mode = (control_mode or 'direct').strip().lower()
    if mode in ('serial', 'raw'):
        mode = 'direct'
    if mode == 'ros2' and bool(rosbridge_ok):
        return 'ros2'
    if mode == 'ros2':
        return 'serial_fallback'
    return 'direct'


def parse_ugv_bringup_pids(ps_text: Optional[str]) -> list:
    """Parse `ps` / `pgrep -af` lines for real ugv_bringup PIDs (not wrappers).

    Used when leaving ROS 2 so Flask can reclaim UART without `pkill -f`
    matching the docker-exec / bash wrapper.
    """
    pids = []
    if not ps_text:
        return pids
    skip = ('pgrep', 'pkill', 'grep -', 'awk', 'bash -lc', 'docker exec')
    seen = set()
    for line in str(ps_text).splitlines():
        raw = line.strip()
        if not raw or 'ugv_bringup' not in raw:
            continue
        low = raw.lower()
        if any(tok in low for tok in skip):
            continue
        pid = None
        for tok in raw.split():
            if tok.isdigit():
                pid = int(tok)
                break
        if pid is None or pid in seen:
            continue
        seen.add(pid)
        pids.append(pid)
    return pids


def parse_chassis_driver_pids(ps_text: Optional[str]) -> list:
    """PIDs for ugv_driver_min and/or ugv_bringup (not RoArm, not wrappers).

    Beast compose runs python3 .../ugv_driver_min.py.
    Rover images may run the stock ugv_bringup binary.
    """
    pids = list(parse_ugv_bringup_pids(ps_text))
    if not ps_text:
        return pids
    skip = ('pgrep', 'pkill', 'grep -', 'awk', 'bash -lc', 'docker exec', 'roarm_')
    seen = set(pids)
    for line in str(ps_text).splitlines():
        raw = line.strip()
        if not raw or 'ugv_driver_min' not in raw:
            continue
        low = raw.lower()
        if any(tok in low for tok in skip):
            continue
        pid = None
        for tok in raw.split():
            if tok.isdigit():
                pid = int(tok)
                break
        if pid is None or pid in seen:
            continue
        seen.add(pid)
        pids.append(pid)
    return pids


def prefer_ugv_driver_min(env_value: Optional[str] = None, *, driver_min_available: bool = False, bringup_available: bool = False) -> str:
    """Pure helper: which chassis ROS node to prefer.

    Never requires ugv_ws-ugv_ros2. Prefers ugv_driver_min when available.
    """
    env = (env_value if env_value is not None else (os.environ.get('UGV_CHASSIS_DRIVER') or 'auto')).strip().lower()
    if env in ('ugv_driver_min', 'driver_min', 'min'):
        return 'ugv_driver_min'
    if env in ('ugv_bringup', 'bringup'):
        return 'ugv_bringup'
    if driver_min_available:
        return 'ugv_driver_min'
    if bringup_available:
        return 'ugv_bringup'
    return 'none'


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


# Default short timed move when duration_ms is missing/0 (unless continuous=true).
_DEFAULT_DRIVE_MS = 1000

# Module-level scheduled auto-stop (daemon timer). Cancelled on new drive/stop.
_drive_timer_lock = threading.Lock()
_drive_timer: Optional[threading.Timer] = None
_drive_timer_gen = 0


def _cancel_scheduled_drive_stop() -> None:
    """Cancel any pending timed auto-stop (call under lock or via public stop)."""
    global _drive_timer, _drive_timer_gen
    with _drive_timer_lock:
        _drive_timer_gen += 1
        if _drive_timer is not None:
            try:
                _drive_timer.cancel()
            except Exception:
                pass
            _drive_timer = None


def _schedule_drive_stop(duration_ms: int) -> None:
    """Schedule ros_stop after duration_ms; supersedes any previous timer."""
    global _drive_timer, _drive_timer_gen
    with _drive_timer_lock:
        _drive_timer_gen += 1
        gen = _drive_timer_gen
        if _drive_timer is not None:
            try:
                _drive_timer.cancel()
            except Exception:
                pass
            _drive_timer = None

        def _fire():
            global _drive_timer
            with _drive_timer_lock:
                if gen != _drive_timer_gen:
                    return
                _drive_timer = None
            try:
                publish_cmd_vel(0.0, 0.0)
            except Exception:
                pass

        t = threading.Timer(max(0.0, duration_ms) / 1000.0, _fire)
        t.daemon = True
        _drive_timer = t
        t.start()


def _resolve_drive_duration(
    duration_ms: Any,
    continuous: bool,
    max_ms: int,
) -> Tuple[int, bool]:
    """Return (duration_ms, continuous). Missing/0 → default short timed move."""
    if continuous:
        return 0, True
    try:
        if duration_ms is None or duration_ms == '' or int(duration_ms) == 0:
            dur = _DEFAULT_DRIVE_MS
        else:
            dur = int(duration_ms)
    except (TypeError, ValueError):
        dur = _DEFAULT_DRIVE_MS
    if dur < 0:
        dur = _DEFAULT_DRIVE_MS
    if dur > max_ms:
        dur = max_ms
    return dur, False


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
    _cancel_scheduled_drive_stop()
    return publish_cmd_vel(0.0, 0.0)


def ros_drive(
    linear_x: float,
    angular_z: float = 0.0,
    duration_ms: Any = None,
    continuous: bool = False,
) -> Dict[str, Any]:
    """Publish cmd_vel; default is a short timed move with async auto-stop.

    duration_ms missing/0 → 1000 ms timed move (unless continuous=True).
    continuous=True → no auto-stop (caller must stop_motors).
    Timed moves schedule stop on a daemon timer and return immediately.
    """
    max_lin, max_ang, max_ms = _limits()
    lin = _clamp(float(linear_x), -max_lin, max_lin)
    ang = _clamp(float(angular_z), -max_ang, max_ang)
    dur, is_continuous = _resolve_drive_duration(duration_ms, bool(continuous), max_ms)

    # New command supersedes any previous scheduled stop
    _cancel_scheduled_drive_stop()
    result = publish_cmd_vel(lin, ang)
    result['duration_ms'] = dur
    result['continuous'] = is_continuous
    result['stopped'] = False
    result['async_stop'] = False
    result['clamped'] = {
        'linear_x': lin,
        'angular_z': ang,
        'max_linear': max_lin,
        'max_angular': max_ang,
        'max_drive_ms': max_ms,
    }
    if not is_continuous and dur > 0:
        _schedule_drive_stop(dur)
        result['scheduled_stop_ms'] = dur
        result['async_stop'] = True
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


# ---- USB RoArm joint topics (independent of base UART / chassis) ------------
# Command: /ugv/roarm/joint_command  name=[roarm_base, roarm_shoulder, roarm_elbow, roarm_hand]
# State:   /ugv/roarm/joint_states
_ROARM_JOINT_NAMES = [
    'roarm_base',
    'roarm_shoulder',
    'roarm_elbow',
    'roarm_hand',
]
_roarm_last_pub = 0.0
_roarm_last_cmd_pub = 0.0
_ROARM_MIN_INTERVAL = 0.03  # ~33 Hz cap for stick spam


def roarm_joint_states_topic() -> str:
    return (os.environ.get('UGV_ROARM_JOINT_STATES_TOPIC') or '/ugv/roarm/joint_states').strip()


def roarm_joint_command_topic() -> str:
    return (os.environ.get('UGV_ROARM_JOINT_CMD_TOPIC') or '/ugv/roarm/joint_command').strip()


def roarm_usb_owner() -> str:
    """Who opens the RoArm CP2102 USB serial.

    flask  (default) — Flask roarm_ctrl owns USB (hybrid: also publishes ROS topics)
    driver           — exclusive ROS/host driver owns USB; Flask only publishes command
    """
    raw = (os.environ.get('UGV_ROARM_USB_OWNER') or 'flask').strip().lower()
    if raw in ('driver', 'ros', 'ros2', 'bridge', 'exclusive'):
        return 'driver'
    return 'flask'


def _roarm_joint_msg(
    base: float,
    shoulder: float,
    elbow: float,
    hand: float,
) -> dict:
    return {
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'roarm_base_link'},
        'name': list(_ROARM_JOINT_NAMES),
        'position': [float(base), float(shoulder), float(elbow), float(hand)],
        'velocity': [],
        'effort': [],
    }


def publish_roarm_joint_command(
    base: float,
    shoulder: float,
    elbow: float,
    hand: float,
    *,
    throttle: bool = True,
    also_states: bool = False,
) -> Dict[str, Any]:
    """Publish RoArm target joints on /ugv/roarm/joint_command (sensor_msgs/JointState).

    Does not open USB — roarm_driver_min / bridge / Flask hybrid write hardware.
    """
    global _roarm_last_cmd_pub
    if throttle:
        now = time.time()
        if now - _roarm_last_cmd_pub < _ROARM_MIN_INTERVAL:
            return {'ok': True, 'backend': 'ros2', 'throttled': True, 'kind': 'command'}
        _roarm_last_cmd_pub = now

    positions = [float(base), float(shoulder), float(elbow), float(hand)]
    js_type = os.environ.get('UGV_JOINT_STATES_TYPE') or 'sensor_msgs/msg/JointState'
    msg = _roarm_joint_msg(base, shoulder, elbow, hand)
    topics = []
    try:
        client = _get_pt_client()
        with _pt_client_lock:
            cmd_topic = roarm_joint_command_topic()
            client.publish(cmd_topic, msg, js_type)
            topics.append(cmd_topic)
            if also_states:
                st_topic = roarm_joint_states_topic()
                client.publish(st_topic, msg, js_type)
                topics.append(st_topic)
        return {
            'ok': True,
            'backend': 'ros2',
            'kind': 'command',
            'topics': topics,
            'positions': positions,
            'names': list(_ROARM_JOINT_NAMES),
        }
    except Exception as e:
        return {
            'ok': False,
            'backend': 'ros2',
            'kind': 'command',
            'error': str(e),
            'topics': topics,
        }


def publish_roarm_joints(
    base: float,
    shoulder: float,
    elbow: float,
    hand: float,
    *,
    throttle: bool = True,
    also_as_command: bool = False,
) -> Dict[str, Any]:
    """Publish RoArm joint positions on rosbridge (state mirror; optional command)."""
    global _roarm_last_pub
    if throttle:
        now = time.time()
        if now - _roarm_last_pub < _ROARM_MIN_INTERVAL:
            return {'ok': True, 'backend': 'ros2', 'throttled': True}
        _roarm_last_pub = now

    positions = [float(base), float(shoulder), float(elbow), float(hand)]
    js_type = os.environ.get('UGV_JOINT_STATES_TYPE') or 'sensor_msgs/msg/JointState'
    msg = _roarm_joint_msg(base, shoulder, elbow, hand)
    topics = []
    try:
        client = _get_pt_client()
        with _pt_client_lock:
            st_topic = roarm_joint_states_topic()
            client.publish(st_topic, msg, js_type)
            topics.append(st_topic)
            if also_as_command:
                cmd_topic = roarm_joint_command_topic()
                client.publish(cmd_topic, msg, js_type)
                topics.append(cmd_topic)
        return {
            'ok': True,
            'backend': 'ros2',
            'topics': topics,
            'positions': positions,
            'names': list(_ROARM_JOINT_NAMES),
        }
    except Exception as e:
        return {'ok': False, 'backend': 'ros2', 'error': str(e), 'topics': topics}


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
                    '(positive=left). Default is a short timed move (~1000 ms, clamped); '
                    'prefer duration_ms 500–2000. Auto-stop is scheduled asynchronously. '
                    'Pass continuous=true only if you will call stop_motors yourself. '
                    'Values are hard-clamped server-side for safety.'
                ),
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'linear_x': {'type': 'number', 'description': 'Forward velocity m/s'},
                        'angular_z': {'type': 'number', 'description': 'Yaw rate rad/s'},
                        'duration_ms': {
                            'type': 'integer',
                            'description': (
                                'Timed drive duration in ms then auto-stop. '
                                'Missing/0 defaults to 1000 ms (not continuous). '
                                'Recommend 500–2000; hard-clamped to max.'
                            ),
                        },
                        'continuous': {
                            'type': 'boolean',
                            'description': (
                                'If true, no auto-stop (drive until stop_motors). '
                                'Default false — prefer short timed moves.'
                            ),
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
                'description': (
                    'Emergency stop wheels on the active control path. '
                    'Cancels any pending timed auto-stop.'
                ),
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
            out['pending_stop_cancelled'] = True
            return out
        if name == 'send_motor_command':
            cont = args.get('continuous')
            if isinstance(cont, str):
                cont = cont.strip().lower() in ('1', 'true', 'yes', 'on')
            else:
                cont = bool(cont)
            out = ros_drive(
                linear_x=float(args.get('linear_x', 0.0)),
                angular_z=float(args.get('angular_z', 0.0)),
                duration_ms=args.get('duration_ms'),
                continuous=cont,
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
