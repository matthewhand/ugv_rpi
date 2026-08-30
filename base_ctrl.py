import serial  
import json
import queue
import threading
import yaml
import os
import time
import glob
import numpy as np

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

# LD19 / STL-19P / D300: 47-byte frames at 230400 on USB UART (CP2102 or CDC ACM).
LIDAR_BAUD = 230400
LIDAR_FRAME_LEN = 47
LIDAR_HEADER = 0x54
LIDAR_VERLEN = 0x2C
LIDAR_POINTS_PER_FRAME = 12
SENSOR_BAUD = 115200
# RoArm uses CP2102N; lidar USB adapters on this kit are CP2102 (no N).
_ROARM_BY_ID_GLOB = (
	"/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_*-if00-port0"
)
_LIDAR_BY_ID_GLOB = (
	"/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0"
)


def _uniq_existing_paths(paths, exists=os.path.exists, realpath=os.path.realpath):
	out = []
	seen = set()
	for raw in paths:
		if not raw:
			continue
		if not exists(raw):
			continue
		rp = realpath(raw)
		if rp in seen:
			continue
		seen.add(rp)
		out.append(rp)
	return out


def roarm_usb_ports(globber=glob.glob, realpath=os.path.realpath, exists=os.path.exists):
	"""Real paths of CP2102N RoArm adapters — lidar must not steal these."""
	return set(_uniq_existing_paths(globber(_ROARM_BY_ID_GLOB), exists=exists, realpath=realpath))


def lidar_port_candidates(
	env=None,
	globber=glob.glob,
	exists=os.path.exists,
	realpath=os.path.realpath,
):
	"""USB lidar ports: env, ttyACM (legacy Waveshare), CP2102 (not CP2102N), leftover ttyUSB."""
	if env is None:
		env = os.environ.get("UGV_LIDAR_SERIAL") or os.environ.get("UGV_LIDAR_PORT")
	skip = roarm_usb_ports(globber=globber, realpath=realpath, exists=exists)
	ordered = []
	if env:
		ordered.append(env)
	ordered.extend(sorted(globber("/dev/ttyACM*")))
	ordered.extend(sorted(globber(_LIDAR_BY_ID_GLOB)))
	for usb in sorted(globber("/dev/ttyUSB*")):
		rp = realpath(usb) if exists(usb) else usb
		if rp in skip:
			continue
		ordered.append(usb)
	out = []
	for p in _uniq_existing_paths(ordered, exists=exists, realpath=realpath):
		if p in skip:
			continue
		out.append(p)
	return out


def parse_ld19_frame(data):
	"""Parse one 47-byte LD19 frame. Returns (start_deg, end_deg, distances_mm, intensities) or None."""
	if data is None or len(data) < LIDAR_FRAME_LEN:
		return None
	if data[0] != LIDAR_HEADER:
		return None
	start_angle = ((data[5] << 8) | data[4]) * 0.01
	end_angle = ((data[43] << 8) | data[42]) * 0.01
	distances = []
	intensities = []
	for i in range(LIDAR_POINTS_PER_FRAME):
		offset = 6 + i * 3
		distances.append((data[offset + 1] << 8) | data[offset])
		intensities.append(data[offset + 2])
	return start_angle, end_angle, distances, intensities


def _open_usb_serial(port, baud, timeout=0.2, exclusive=True):
	kwargs = dict(
		port=port,
		baudrate=baud,
		timeout=timeout,
		write_timeout=timeout,
		xonxoff=False,
		rtscts=False,
		dsrdtr=False,
	)
	ser = None
	if exclusive:
		try:
			ser = serial.Serial(exclusive=True, **kwargs)
		except TypeError:
			ser = None
		except Exception:
			raise
	if ser is None:
		ser = serial.Serial(**kwargs)
	try:
		ser.dtr = False
		ser.rts = False
	except Exception:
		pass
	return ser


class ReadLine:
	def __init__(self, s):
		self.buf = bytearray()
		self.s = s

		self.sensor_data = []
		self.sensor_list = []
		self.sensor_data_ser = None
		self.sensor_port = None
		self.sensor_data_max_len = 51

		self.lidar_ser = None
		self.lidar_port = None
		self.ANGLE_PER_FRAME = LIDAR_POINTS_PER_FRAME
		self.HEADER = LIDAR_HEADER
		self.lidar_angles = []
		self.lidar_distances = []
		self.lidar_angles_show = []
		self.lidar_distances_show = []
		self.last_start_angle = 0
		self.lidar_updated_at = None
		self.lidar_last_error = None
		self._lidar_lock = threading.Lock()
		self._lidar_retry_at = 0.0

	def readline(self):
		i = self.buf.find(b"\n")
		if i >= 0:
			r = self.buf[:i+1]
			self.buf = self.buf[i+1:]
			return r
		while True:
			if not self.s:
				time.sleep(0.1)
				return b""
			i = max(1, min(512, self.s.in_waiting))
			data = self.s.read(i)
			i = data.find(b"\n")
			if i >= 0:
				r = self.buf + data[:i+1]
				self.buf[0:] = data[i+1:]
				return r
			else:
				self.buf.extend(data)

	def clear_buffer(self):
		if not self.s:
			self.buf = bytearray()
			return
		try:
			self.s.reset_input_buffer()
		except Exception:
			pass
		self.buf = bytearray()

	def _claimed_ports(self):
		claimed = set()
		for ser in (self.lidar_ser, self.sensor_data_ser):
			port = getattr(ser, "port", None) if ser is not None else None
			if port and getattr(ser, "is_open", False):
				try:
					claimed.add(os.path.realpath(port))
				except Exception:
					claimed.add(port)
		return claimed

	def open_extra_sensor(self):
		if self.sensor_data_ser is not None and getattr(self.sensor_data_ser, "is_open", False):
			return True
		claimed = self._claimed_ports()
		for port in sorted(glob.glob("/dev/ttyUSB*")):
			rp = os.path.realpath(port)
			if rp in claimed:
				continue
			if rp in roarm_usb_ports():
				continue
			try:
				self.sensor_data_ser = _open_usb_serial(port, SENSOR_BAUD, timeout=0.2, exclusive=True)
				self.sensor_port = rp
				print(f"[base_ctrl] extra sensor connected {rp} @ {SENSOR_BAUD}")
				return True
			except Exception as e:
				print(f"[base_ctrl] extra sensor open {port} failed: {e}")
		self.sensor_data_ser = None
		self.sensor_port = None
		return False

	def close_extra_sensor(self):
		ser = self.sensor_data_ser
		self.sensor_data_ser = None
		self.sensor_port = None
		if ser is None:
			return
		try:
			ser.close()
		except Exception:
			pass

	def open_lidar(self):
		with self._lidar_lock:
			return self._open_lidar_locked()

	def _open_lidar_locked(self):
		if self.lidar_ser is not None and getattr(self.lidar_ser, "is_open", False):
			return True
		claimed = self._claimed_ports()
		last_err = None
		for port in lidar_port_candidates():
			if port in claimed:
				continue
			try:
				self.lidar_ser = _open_usb_serial(port, LIDAR_BAUD, timeout=0.2, exclusive=True)
				self.lidar_port = port
				self.lidar_last_error = None
				try:
					self.lidar_ser.reset_input_buffer()
				except Exception:
					pass
				print(f"[base_ctrl] lidar connected {port} @ {LIDAR_BAUD}")
				return True
			except Exception as e:
				last_err = e
				self.lidar_ser = None
				self.lidar_port = None
				print(f"[base_ctrl] lidar open {port} failed: {e}")
		self.lidar_last_error = str(last_err) if last_err else "no USB lidar port"
		return False

	def close_lidar(self):
		with self._lidar_lock:
			ser = self.lidar_ser
			self.lidar_ser = None
			self.lidar_port = None
		if ser is None:
			return
		try:
			ser.close()
		except Exception:
			pass

	def read_sensor_data(self):
		if self.sensor_data_ser == None:
			return

		try:
			buffer_clear = False
			while self.sensor_data_ser.in_waiting > 0:
				buffer_clear = True
				sensor_readline = self.sensor_data_ser.readline()
				if len(sensor_readline) <= self.sensor_data_max_len:
					self.sensor_list.append(sensor_readline.decode('utf-8')[:-2])
				else:
					self.sensor_list.append(sensor_readline.decode('utf-8')[:self.sensor_data_max_len])
					self.sensor_list.append(sensor_readline.decode('utf-8')[self.sensor_data_max_len:-2])
			if buffer_clear:
				self.sensor_data = self.sensor_list.copy()
				self.sensor_list.clear()
				self.sensor_data_ser.reset_input_buffer()
		except Exception as e:
			print(f"[base_ctrl.read_sensor_data] error: {e}")

	def parse_lidar_frame(self, data):
		parsed = parse_ld19_frame(data)
		if parsed is None:
			return self.last_start_angle
		start_angle, _end_angle, distances, _intensities = parsed
		for i, distance in enumerate(distances):
			self.lidar_angles.append(np.radians(start_angle + i * 0.83333 + 180))
			self.lidar_distances.append(distance)
		return start_angle

	def lidar_data_recv(self):
		with self._lidar_lock:
			self._lidar_data_recv_locked()

	def _lidar_data_recv_locked(self):
		if self.lidar_ser is None or not getattr(self.lidar_ser, "is_open", False):
			now = time.time()
			if now - self._lidar_retry_at < 2.0:
				return
			self._lidar_retry_at = now
			if not self._open_lidar_locked():
				return
		try:
			deadline = time.monotonic() + 1.2
			got_wrap = False
			start_angle = self.last_start_angle
			while time.monotonic() < deadline:
				header = self.lidar_ser.read(1)
				if not header:
					break
				if header != b"\x54":
					continue
				rest = self.lidar_ser.read(LIDAR_FRAME_LEN - 1)
				if len(rest) != LIDAR_FRAME_LEN - 1:
					continue
				data = header + rest
				if data[1] != LIDAR_VERLEN:
					continue
				hex_data = list(data)
				start_angle = self.parse_lidar_frame(hex_data)
				if self.last_start_angle > start_angle:
					got_wrap = True
					break
				self.last_start_angle = start_angle
			if not self.lidar_angles:
				return
			self.last_start_angle = start_angle
			self.lidar_angles_show = self.lidar_angles.copy()
			self.lidar_distances_show = self.lidar_distances.copy()
			self.lidar_angles.clear()
			self.lidar_distances.clear()
			self.lidar_updated_at = time.time()
			self.lidar_last_error = None
			if not got_wrap:
				# Partial revolution is still usable on first spin-up.
				pass
		except Exception as e:
			print(f"[base_ctrl.lidar_data_recv] error: {e}")
			self.lidar_last_error = str(e)
			try:
				if self.lidar_ser is not None:
					self.lidar_ser.close()
			except Exception:
				pass
			self.lidar_ser = None
			self.lidar_port = None
			self._lidar_retry_at = 0.0

	def lidar_snapshot(self):
		with self._lidar_lock:
			angs = list(self.lidar_angles_show)
			dists = list(self.lidar_distances_show)
			port = self.lidar_port
			opened = bool(self.lidar_ser and getattr(self.lidar_ser, "is_open", False))
			updated_at = self.lidar_updated_at
			err = self.lidar_last_error
		points = []
		for ang, dist in zip(angs, dists):
			if dist is None or dist <= 0:
				continue
			points.append({
				"deg": round(float(np.degrees(ang)) % 360.0, 2),
				"mm": int(dist),
			})
		valid = [p["mm"] for p in points]
		nearest = min(points, key=lambda p: p["mm"]) if points else None
		bins = [None] * 36
		for p in points:
			idx = int(p["deg"] // 10) % 36
			if bins[idx] is None or p["mm"] < bins[idx]:
				bins[idx] = p["mm"]
		step = max(1, len(points) // 12) if points else 1
		return {
			"port": port,
			"open": opened,
			"points": len(angs),
			"valid_points": len(points),
			"min_mm": min(valid) if valid else None,
			"max_mm": max(valid) if valid else None,
			"nearest": nearest,
			"bins_10deg_mm": bins,
			"sample": points[::step][:12],
			"updated_at": updated_at,
			"error": err,
		}


class BaseController:

	def __init__(self, uart_dev_set, buad_set):
		self.uart_dev = uart_dev_set
		self.baud = buad_set
		self._ser_lock = threading.Lock()
		# True when Flask intentionally released UART for ROS (ugv_bringup).
		self.serial_released_for_ros = False
		try:
			self.ser = serial.Serial(uart_dev_set, buad_set, timeout=1)
		except Exception as e:
			print(f"[base_ctrl] Serial port {uart_dev_set} error: {e}")
			try:
				from app_log import app_log as olog
				olog.error('serial', f'Serial port {uart_dev_set} open failed',
				           port=uart_dev_set, baud=buad_set, error=str(e))
			except Exception:
				pass
			self.ser = None
		self.rl = ReadLine(self.ser)
		self.command_queue = queue.Queue()
		self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
		self.command_thread.start()

		self.base_light_status = 0
		self.head_light_status = 0

		self.data_buffer = None
		self.base_data = None

		self.use_lidar = bool(f['base_config']['use_lidar'])
		self.extra_sensor = bool(f['base_config']['extra_sensor'])
		if self.use_lidar:
			self.rl.open_lidar()
		if self.extra_sensor:
			self.rl.open_extra_sensor()
		# When False: drop wheel T:1/T:13 on serial (legacy). Full ROS mode also
		# releases the port via release_serial_for_ros().
		self.enable_motor_control = True
		self._chassis_bypass_types = {1, 13, "1", "13"}
		self._bypass_log_last = 0.0
		self._release_log_last = 0.0

	def serial_is_open(self):
		with self._ser_lock:
			return bool(self.ser and getattr(self.ser, 'is_open', False))

	def set_use_lidar(self, enabled):
		"""Open/close the USB lidar UART to match hangar `use_lidar`."""
		self.use_lidar = bool(enabled)
		if not self.use_lidar:
			self.rl.close_lidar()
			return True
		ok = bool(self.rl.open_lidar())
		if not ok:
			try:
				from app_log import app_log as olog
				olog.warn(
					'lidar',
					'use_lidar on but USB lidar did not open',
					error=getattr(self.rl, 'lidar_last_error', None),
					candidates=lidar_port_candidates(),
				)
			except Exception:
				pass
		return ok

	def set_extra_sensor(self, enabled):
		self.extra_sensor = bool(enabled)
		if not self.extra_sensor:
			self.rl.close_extra_sensor()
			return True
		return bool(self.rl.open_extra_sensor())

	def release_serial_for_ros(self):
		"""Close UART so ugv_bringup / ROS can own /dev/ttyAMA0 (or serial0)."""
		with self._ser_lock:
			if self.ser is not None:
				try:
					self.ser.close()
				except Exception as e:
					print(f"[base_ctrl] serial close: {e}")
				self.ser = None
			self.rl.s = None
			self.rl.buf = bytearray()
			self.serial_released_for_ros = True
		# Drain pending writes so they don't fire after reclaim
		try:
			while True:
				self.command_queue.get_nowait()
		except queue.Empty:
			pass
		print(f"[base_ctrl] Serial RELEASED for ROS ({self.uart_dev})")
		try:
			from app_log import app_log as olog
			olog.info(
				'serial',
				f'Serial released for ROS 2 ({self.uart_dev}) — ugv_bringup may open UART',
				port=self.uart_dev, owner='ros2',
			)
		except Exception:
			pass
		return True

	def claim_serial_for_flask(self):
		"""Re-open UART for Flask direct serial control."""
		with self._ser_lock:
			if self.ser is not None and getattr(self.ser, 'is_open', False):
				self.serial_released_for_ros = False
				return True
			try:
				self.ser = serial.Serial(self.uart_dev, self.baud, timeout=1)
				self.rl.s = self.ser
				self.rl.buf = bytearray()
				self.serial_released_for_ros = False
			except Exception as e:
				self.ser = None
				self.rl.s = None
				print(f"[base_ctrl] Serial reclaim failed {self.uart_dev}: {e}")
				try:
					from app_log import app_log as olog
					olog.error(
						'serial',
						f'Serial reclaim failed ({self.uart_dev}): {e}',
						port=self.uart_dev, error=str(e), owner='flask',
					)
				except Exception:
					pass
				return False
		print(f"[base_ctrl] Serial CLAIMED by Flask ({self.uart_dev})")
		try:
			from app_log import app_log as olog
			olog.info(
				'serial',
				f'Serial claimed by Flask ({self.uart_dev}) — direct path active',
				port=self.uart_dev, owner='flask',
			)
		except Exception:
			pass
		return True

	def feedback_data(self):
		try:
			if not self.rl.s:
				return None
			while self.rl.s.in_waiting > 0:
				self.data_buffer = json.loads(self.rl.readline().decode('utf-8'))
				if 'T' in self.data_buffer:
					self.base_data = self.data_buffer
					self.data_buffer = None
					if self.base_data["T"] == 1003:
						print(self.base_data)
						try:
							from app_log import app_log as olog
							bd = self.base_data
							olog.info(
								'esp_now',
								f'ESP-NOW T:1003 from {bd.get("mac", "?")}',
								T=1003, mac=bd.get('mac'),
								megs=str(bd.get('megs', ''))[:120],
							)
						except Exception:
							pass
						return self.base_data
			self.rl.clear_buffer()
			self.data_buffer = json.loads(self.rl.readline().decode('utf-8'))
			self.base_data = self.data_buffer
			return self.base_data
		except Exception as e:
			try:
				self.rl.clear_buffer()
			except Exception:
				pass
			# Quiet when port intentionally released for ROS
			if not self.serial_released_for_ros:
				print(f"[base_ctrl.feedback_data] error: {e}")


	def on_data_received(self):
		if self.ser:
			self.ser.reset_input_buffer()
		data_read = json.loads(self.rl.readline().decode('utf-8'))
		return data_read


	def send_command(self, data):
		# Last-line chassis guard: line-follow / timelapse / CLI `base -c`
		# bypass Flask routing. Zeros and gimbal still pass.
		try:
			from seek_nav import chassis_serial_allowed
			if isinstance(data, dict) and not chassis_serial_allowed(data):
				return
		except Exception:
			pass
		if self.serial_released_for_ros or not self.ser:
			now = time.time()
			if now - getattr(self, '_release_log_last', 0) > 8.0:
				self._release_log_last = now
				t_code = data.get('T') if isinstance(data, dict) else None
				print(
					f"[base_ctrl] Serial not owned by Flask (ROS mode) — drop serial cmd T={t_code}. "
					"If PTZ is dead: start rosbridge or switch UI Control to Direct serial."
				)
				try:
					from app_log import app_log as olog
					olog.warn(
						'serial',
						'Dropped serial cmd — UART released for ROS 2 '
						'(start rosbridge or switch Control to Direct)',
						T=t_code,
						owner='ros2',
						hint='UGV_CONTROL_MODE=direct or UI Control: Direct serial',
						throttle_s=8.0,
					)
				except Exception:
					pass
			return
		if not self.enable_motor_control and isinstance(data, dict):
			if data.get("T") in self._chassis_bypass_types:
				# Throttle: stick heartbeats would flood the ops log
				now = time.time()
				if now - getattr(self, '_bypass_log_last', 0) > 5.0:
					self._bypass_log_last = now
					print("[base_ctrl] Chassis command bypassed (enable_motor_control=False; gimbal T:133/141 still allowed)")
					try:
						from app_log import app_log as olog
						olog.warn(
							'chassis_bypass',
							'Chassis serial bypassed (ROS 2 mode owns wheels; PT free)',
							T=data.get('T'), enable_motor_control=False,
						)
					except Exception:
						pass
				return
		self.command_queue.put(data)


	def process_commands(self):
		while True:
			data = self.command_queue.get()
			with self._ser_lock:
				ser = self.ser
				if ser and getattr(ser, 'is_open', False):
					try:
						ser.write((json.dumps(data) + '\n').encode("utf-8"))
					except Exception as e:
						print(f"[base_ctrl.process_commands] write error: {e}")


	def base_json_ctrl(self, input_json):
		self.send_command(input_json)


	def gimbal_emergency_stop(self):
		data = {"T":0}
		self.send_command(data)


	def base_speed_ctrl(self, input_left, input_right):
		data = {"T":1,"L":input_left,"R":input_right}
		self.send_command(data)


	def gimbal_ctrl(self, input_x, input_y, input_speed, input_acceleration):
		data = {"T":133,"X":input_x,"Y":input_y,"SPD":input_speed,"ACC":input_acceleration}
		self.send_command(data)


	def gimbal_base_ctrl(self, input_x, input_y, input_speed):
		data = {"T":141,"X":input_x,"Y":input_y,"SPD":input_speed}
		self.send_command(data)


	def base_oled(self, input_line, input_text):
		data = {"T":3,"lineNum":input_line,"Text":input_text}
		self.send_command(data)


	def base_default_oled(self):
		data = {"T":-3}
		self.send_command(data)


	def bus_servo_id_set(self, old_id, new_id):
		# data = {"T":54,"old":old_id,"new":new_id}
		data = {"T":f['cmd_config']['cmd_set_servo_id'],"raw":old_id,"new":new_id}
		self.send_command(data)


	def bus_servo_torque_lock(self, input_id, input_status):
		# data = {"T":55,"id":input_id,"status":input_status}
		data = {"T":f['cmd_config']['cmd_servo_torque'],"id":input_id,"cmd":input_status}
		self.send_command(data)


	def bus_servo_mid_set(self, input_id):
		# data = {"T":58,"id":input_id}
		data = {"T":f['cmd_config']['cmd_set_servo_mid'],"id":input_id}
		self.send_command(data)


	def lights_ctrl(self, pwmA, pwmB):
		data = {"T":132,"IO4":pwmA,"IO5":pwmB}
		self.send_command(data)
		self.base_light_status = pwmA
		self.head_light_status = pwmB


	def base_lights_ctrl(self):
		if self.base_light_status != 0:
			self.base_light_status = 0
		else:
			self.base_light_status = 255
		self.lights_ctrl(self.base_light_status, self.head_light_status)

	def gimbal_dev_close(self):
		self.release_serial_for_ros()

	def breath_light(self, input_time):
		breath_start_time = time.time()
		while time.time() - breath_start_time < input_time:
			for i in range(0, 128, 10):
				self.lights_ctrl(i, 128-i)
				time.sleep(0.1)
			for i in range(0, 128, 10):
				self.lights_ctrl(128-i, i)
				time.sleep(0.1)
		self.lights_ctrl(0, 0)


if __name__ == '__main__':
	# RPi5
	base = BaseController('/dev/ttyAMA0', 115200)

	# RPi4B
	# base = BaseController('/dev/serial0', 115200)

	# breath light for 15s
	base.breath_light(15)

	# gimble ctrl, look forward
	#                x  y  spd acc
	base.gimbal_ctrl(0, 0, 10, 0)
    
    # x(-180 ~ 180)
	# x- look left
	# x+ look right

	# y(-30 ~ 90)
	# y- look down
	# y+ look up