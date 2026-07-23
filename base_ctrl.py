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

class ReadLine:
	def __init__(self, s):
		self.buf = bytearray()
		self.s = s

		self.sensor_data = []
		self.sensor_list = []
		try:
			self.sensor_data_ser = serial.Serial(glob.glob('/dev/ttyUSB*')[0], 115200)
			print("/dev/ttyUSB* connected succeed")
		except:
			self.sensor_data_ser = None
		self.sensor_data_max_len = 51

		try:
			self.lidar_ser = serial.Serial(glob.glob('/dev/ttyACM*')[0], 230400, timeout=1)
			print("/dev/ttyACM* connected succeed")
		except:
			self.lidar_ser = None
		self.ANGLE_PER_FRAME = 12
		self.HEADER = 0x54
		self.lidar_angles = []
		self.lidar_distances = []
		self.lidar_angles_show = []
		self.lidar_distances_show = []
		self.last_start_angle = 0

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
		# header = data[0]
		# verlen = data[1]
		# speed  = data[3] << 8 | data[2]
		start_angle = (data[5] << 8 | data[4]) * 0.01
		# print(start)
		# end_angle = (data[43] << 8 | data[42]) * 0.01
		for i in range(0, self.ANGLE_PER_FRAME):
			offset = 6 + i * 3
			distance = data[offset+1] << 8 | data[offset]
			confidence = data[offset+2]
			# lidar_angles.append(np.radians(start_angle + i * 0.167))
			self.lidar_angles.append(np.radians(start_angle + i * 0.83333 + 180))
			# lidar_angles.append(np.radians(start_angle + end_angle))
			self.lidar_distances.append(distance)
		# end_angle = (data[43] << 8 | data[42]) * 0.01
		# timestamp = data[45] << 8 | data[44]
		# crc = data[46]
		return start_angle

	def lidar_data_recv(self):
		if self.lidar_ser == None:
			return
		try:
			while True:
				self.header = self.lidar_ser.read(1)
				if self.header == b'\x54':
					# Read the rest of the data
					data = self.header + self.lidar_ser.read(46)
					hex_data = [int(hex(byte), 16) for byte in data]
					start_angle = self.parse_lidar_frame(hex_data)
					if self.last_start_angle > start_angle:
						break
					self.last_start_angle = start_angle
				else:
					self.lidar_ser.flushInput()

			self.last_start_angle = start_angle
			self.lidar_angles_show = self.lidar_angles.copy()
			self.lidar_distances_show = self.lidar_distances.copy()
			self.lidar_angles.clear()
			self.lidar_distances.clear()
		except Exception as e:
			print(f"[base_ctrl.lidar_data_recv] error: {e}")
			self.lidar_ser = serial.Serial(glob.glob('/dev/ttyACM*')[0], 230400, timeout=1)


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

		self.use_lidar = f['base_config']['use_lidar']
		self.extra_sensor = f['base_config']['extra_sensor']
		# When False: drop wheel T:1/T:13 on serial (legacy). Full ROS mode also
		# releases the port via release_serial_for_ros().
		self.enable_motor_control = True
		self._chassis_bypass_types = {1, 13, "1", "13"}
		self._bypass_log_last = 0.0
		self._release_log_last = 0.0

	def serial_is_open(self):
		with self._ser_lock:
			return bool(self.ser and getattr(self.ser, 'is_open', False))

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


	def _apply_chassis_inverts(self, data):
		"""Optionally flip wheel/linear signs for camera-forward alignment.

		UGV_INVERT_LINEAR=1  → negate T:1 L/R and T:13 X
		UGV_INVERT_ANGULAR=1 → negate T:13 Z (yaw)

		Use when stock "positive = forward" is opposite the camera-facing
		direction on a given chassis. Applied once at serial egress for all
		UI / AI / CLI paths that use base_json_ctrl.
		"""
		if not isinstance(data, dict):
			return data
		try:
			t = data.get('T')
		except Exception:
			return data
		inv_lin = (os.environ.get('UGV_INVERT_LINEAR') or '').strip().lower() in (
			'1', 'true', 'yes', 'on'
		)
		inv_ang = (os.environ.get('UGV_INVERT_ANGULAR') or '').strip().lower() in (
			'1', 'true', 'yes', 'on'
		)
		if not inv_lin and not inv_ang:
			return data
		out = dict(data)
		try:
			if t in (1, '1') and inv_lin:
				if 'L' in out:
					out['L'] = -float(out.get('L') or 0)
				if 'R' in out:
					out['R'] = -float(out.get('R') or 0)
			elif t in (13, '13'):
				if inv_lin and ('X' in out or 'x' in out):
					key = 'X' if 'X' in out else 'x'
					out[key] = -float(out.get(key) or 0)
				if inv_ang and ('Z' in out or 'z' in out):
					key = 'Z' if 'Z' in out else 'z'
					out[key] = -float(out.get(key) or 0)
		except (TypeError, ValueError):
			return data
		return out

	def send_command(self, data):
		if self.serial_released_for_ros or not self.ser:
			now = time.time()
			if now - getattr(self, '_release_log_last', 0) > 8.0:
				self._release_log_last = now
				print("[base_ctrl] Serial not owned by Flask (ROS mode) — drop serial cmd")
				try:
					from app_log import app_log as olog
					olog.warn(
						'serial',
						'Dropped serial cmd — UART released for ROS 2',
						T=(data.get('T') if isinstance(data, dict) else None),
						owner='ros2',
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
		# Align software "forward" with camera-forward when env invert is set
		data = self._apply_chassis_inverts(data)
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