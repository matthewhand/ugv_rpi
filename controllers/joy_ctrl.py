import pygame
import time
import json
import sys
import threading
import yaml, os
import serial
from serial import SerialException
import json
import queue
import logging
import math
import pyudev

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
        self.timeout = 0.1 
        self.frame_start = b'{'
        self.frame_end =  b"}\r\n"
        self.max_frame_length = 512
 
    def readline(self):
        start_time = time.time() 
        while True:
            i = max(1, min(self.max_frame_length, self.s.in_waiting))
            data = self.s.read(i)
            if data:
                self.buf.extend(data)

            end = self.buf.rfind(self.frame_end)

            if end >= 0:  
                start = self.buf.rfind(self.frame_start, 0, end)
                if start >= 0 and start < end:
                    r = self.buf[start:end + len(self.frame_end)] 
                    self.buf = self.buf[end + len(self.frame_end):]
                    return r
                elif start == -1:
                    continue

            if time.time() - start_time > self.timeout:
                break
 
    def clear_buffer(self):
        self.buf = bytearray()
        self.last_complete_line = bytearray()
        try:
            self.s.reset_input_buffer()
        except Exception as e:
            print(f"Error resetting input buffer: {e}")
 
    def has_last_complete_line(self):
        return len(self.last_complete_line) > 0
 
    def get_last_complete_line(self):
        return bytes(self.last_complete_line)
                
class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.log = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)  
        self.rl = ReadLine(self.ser)
        self.command_dict = {}
        self.lock = threading.Lock()
        self.running = True
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.read_fail_count = 0 
        self.read_fail_threshold = 5

        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
        
    def feedback_data(self):
        try:
            line = self.rl.readline()
            if not line:
                self.read_fail_count += 1
                if self.read_fail_count >= self.read_fail_threshold:
                    self.rl.clear_buffer()
                    self.read_fail_count = 0
                return

            line = line.decode('utf-8')
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            self.read_fail_count = 0  # 成功后清零
            return self.base_data

        except json.JSONDecodeError as e:
            self.log.error(f"JSON decode error: {e} with line: {line}")
            self.read_fail_count += 1
            if self.read_fail_count >= self.read_fail_threshold:
                self.rl.clear_buffer()
                self.read_fail_count = 0

        except Exception as e:
            self.log.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.read_fail_count += 1
            if self.read_fail_count >= self.read_fail_threshold:
                self.rl.clear_buffer()
                self.read_fail_count = 0

    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))
        return data_read
        
    def send_command(self, data: bytes):
        try:
            json_data = json.loads(data.decode())
            t = json_data.get("T")
            if t is not None:
                if str(t) == "3":
                    try:
                        # print(data)
                        self.ser.write(data)
                    except Exception as e:
                        print(f"Failed to immediately send T=3 data: {e}")
                    return
                with self.lock:
                    self.command_dict[str(t)] = data  
        except Exception as e:
            print(f"Failed to parse or store command: {e}")

    # Thread function to process and send commands from the queue
    def process_commands(self):
        while self.running:
            time.sleep(0.01)  
            with self.lock:
                if not self.command_dict:
                    continue
                for t_key, data in self.command_dict.items():
                    try:
                        # print(data)
                        self.ser.write(data)
                    except Exception as e:
                        print(f"Failed to send {t_key}: {e}")
                self.command_dict.clear()  

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/../config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

module_type = f['base_config']['module_type']

SHANWAN_Android_Gamepad = {
    "LEFT_STICK_X": 0,
    "LEFT_STICK_Y": 1,
    "RIGHT_STICK_X": 2,
    "RIGHT_STICK_Y": 3,
    "L2": 5,
    "R2": 4,
    "D_PAD_X": 6,
    "D_PAD_Y": 7,
    "A": 0,
    "B": 1,
    "X": 3,
    "Y": 4,
    "L1": 6,
    "R1": 7,
    "SELECT": 10,
    "START": 11,
    "HOME": 12,
    "LEFT_STICK_CLICK": 13,
    "RIGHT_STICK_CLICK": 14,
}

Xbox_360_Controller = {
    "LEFT_STICK_X": 0,
    "LEFT_STICK_Y": 1,
    "RIGHT_STICK_X": 3,
    "RIGHT_STICK_Y": 4,
    "L2": 2,
    "R2": 5,
    "D_PAD_X": 6,
    "D_PAD_Y": 7,
    "A": 0,
    "B": 1,
    "X": 2,
    "Y": 3,
    "L1": 4,
    "R1": 5,
    "SELECT": 6,
    "START": 7,
    "HOME": 8,
    "LEFT_STICK_CLICK": 9,
    "RIGHT_STICK_CLICK": 10,
}

def get_joystick_mapping(name):
    if name == "SHANWAN Android Gamepad":
        return SHANWAN_Android_Gamepad
    elif name == "Xbox 360 Controller":
        return Xbox_360_Controller
    else:
        print(f"Unknown joystick type: {name}, defaulting to Xbox mapping")
        return Xbox_360_Controller

class JoystickReader(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True

        pygame.init()
        pygame.joystick.init()

        self.joystick = None
        self.lock = threading.Lock()
        self.axes = []
        self.buttons = []
        self.hats = []

        self.running = True

        self.Joy_active = False
        self.joystick_name = None

        self.context = pyudev.Context()
        self.monitor = pyudev.Monitor.from_netlink(self.context)
        self.monitor.filter_by(subsystem='usb')

    def init_joystick(self):
        try:
            events = pygame.event.get()
            pygame.joystick.init()
        except pygame.error as e:
            print(f"Failed to (re)initialize joystick subsystem: {e}")
            self.Joy_active = False
            self.joystick = None
            return     

        joystick_names = self.get_joystick_names()
        if len(joystick_names) == 0:
            if self.Joy_active:
                print("Joystick disconnected")
            self.Joy_active = False
            self.joystick = None
            self.axes = []
            self.buttons = []
            self.hats = []
            self.joystick_name = None
            return
        
        if not self.Joy_active or self.joystick_name != joystick_names[0]:
            self.joystick_name = joystick_names[0]
            print(f"Joystick connected: {self.joystick_name}")
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.Joy_active = True
                self.axes = [0.0] * self.joystick.get_numaxes()
                self.buttons = [False] * self.joystick.get_numbuttons()
                self.hats = [(0, 0)] * self.joystick.get_numhats()
            except pygame.error as e:
                print(f"Failed to initialize joystick object: {e}")
                self.Joy_active = False
                self.joystick = None

    def get_joystick_names(self):

        joystick_names = []
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("No joystick found")
        else:
            for i in range(joystick_count):
                try:
                    joystick = pygame.joystick.Joystick(i)
                    if not joystick.get_init():
                        joystick.init()
                    joystick_names.append(joystick.get_name())
                except pygame.error as e:
                    print(f"Error initializing joystick {i}: {e}")
        return joystick_names

    def usb_event(self, action, device):
        if action in ('add', 'remove'):
            print(f"USB device {action} detected: {device}") 
            time.sleep(0.1)  
            self.init_joystick()

    def run(self):
        observer = pyudev.MonitorObserver(self.monitor, self.usb_event)
        observer.start()

        self.init_joystick()

        while self.running:
            if not self.Joy_active or self.joystick is None:
                time.sleep(0.1)
                continue

            try:
                if not pygame.joystick.get_init():
                    time.sleep(0.1)
                    continue

                pygame.event.pump()
                with self.lock:
                    for i in range(len(self.axes)):
                        self.axes[i] = self.joystick.get_axis(i)
                    for i in range(len(self.buttons)):
                        self.buttons[i] = self.joystick.get_button(i)
                    for i in range(len(self.hats)):
                        self.hats[i] = self.joystick.get_hat(i)
            except pygame.error as e:
                print(f"Joystick read error: {e}")
                self.init_joystick()

            time.sleep(0.01)

    def get_state(self):
        if not self.Joy_active:
            return [], [], []
        with self.lock:
            return self.axes[:], self.buttons[:], self.hats[:]

class JoyTeleop:
    def __init__(self, joystick_reader, serial_port):
        self.js_reader = joystick_reader
        self.base_controller = BaseController(serial_port, 115200)
        self.mapping = None   

        self.prev_l2_pressed = False
        self.prev_l1_pressed = False
        self.prev_r2_pressed = False
        self.prev_r1_pressed = False
        self.prev_led_next_gear_pressed = False
        self.prev_led_down_gear_pressed = False
        self.prev_leg_btn_pressed = False

        self.mode = 'turn'
        self.speed_gear = 0.3
        self.speed_level = [0.3, 0.66, 1.0]
        self.led_gear = 0.0
        self.led_level = [round(i / 25.0, 3) for i in range(0, 26)] 

        self.xspeed_limit = f['args_config']['max_speed']
        self.yspeed_limit = f['args_config']['max_speed']
        self.yaw_speed_limit = f['args_config']['max_turn_speed']
        self.zero_vel_count = 0
        self.zero_vel_limit = 5

        self.led_limit = 255.0
        self.last_led_sent_data = None
        self.y_press_start = None
        self.x_press_start = None
        self.repeat_delay = 0.5  
        self.repeat_interval = 0.1  
        self.last_repeat_time = 0

        self.last_leg_sent_data = None
        self.last_pt_sent_data = None

        self.ptPoseState = type('', (), {})()
        self.ptPoseState.x = 0.0
        self.ptPoseState.y = 0.0

        self.leg_limits = {
            0: { 
                "coxa": { "min": -1.134, "max": 0, "initial": -0.5235 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            },
            1: { 
                "coxa": { "min": -0.7853, "max": 0.7853, "initial": 0 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            },
            2: { 
                "coxa": { "min": 0, "max": 1.134, "initial": 0.5235 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            },
            3: { 
                "coxa": { "min": 0, "max": 1.134, "initial": 0.5235 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            },
            4: { 
                "coxa": { "min": -0.7853, "max": 0.7853, "initial": 0 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            },
            5: { 
                "coxa": { "min": -1.134, "max": 0, "initial": -0.5235 },
                "femur": { "min": 0, "max": 1.57, "initial": 0.5235 },
                "tibia": { "min": 0, "max": 3.1415, "initial": 2.619 }
            }
        }

        self.legJointState = type('', (), {})()
        self.legJointState.leg = 0

        self.leg_current_state = {}
        self.leg_initial_state = {}

        for leg, lim in self.leg_limits.items():
            self.leg_current_state[leg] = {
                "coxa":  lim["coxa"]["initial"],
                "femur": lim["femur"]["initial"],
                "tibia": lim["tibia"]["initial"],
            }

            self.leg_initial_state[leg] = self.leg_current_state[leg].copy()

    def next_gear(self, current, options):
        if current in options:
            idx = options.index(current)
            if idx < len(options) - 1:
                return options[idx + 1]
        return current 

    def down_gear(self, current, options):
        if current in options:
            idx = options.index(current)
            if idx > 0:
                return options[idx - 1]
        return current  

    def show_msg(self, type):
        if type=="speed":
            print(f"[Gear] Speed: {self.speed_gear}")
        if type=="led":
            print(f"[Gear] LED: {self.led_gear}")
        if type=="mode":
            print(f"MODE: {self.mode}")            
        if type=="led":
            print(f"LEG: {self.legJointState.leg}")  

    def send_cmd_vel(self, linear_x, linear_y, yaw_z):
        is_zero = (
            linear_x == 0.0 and
            linear_y == 0.0 and
            yaw_z == 0.0
        )

        if is_zero:
            self.zero_vel_count += 1
            if self.zero_vel_count > self.zero_vel_limit:
                return
        else:
            self.zero_vel_count = 0

        data = json.dumps({
            'T': f['cmd_config']['cmd_movition_ctrl'],
            'X': linear_x,
            'Y': linear_y,
            'Yaw': yaw_z
        }) + "\n"

        self.base_controller.send_command(data.encode())
        for leg, state in self.leg_initial_state.items():
            self.leg_current_state[leg]["coxa"]  = state["coxa"]
            self.leg_current_state[leg]["femur"] = state["femur"]
            self.leg_current_state[leg]["tibia"] = state["tibia"]

    def send_led_ctrl(self, led_value):
        IO4 = max(0, min(led_value, 255))
        IO5 = max(0, min(led_value, 255))     

        data = json.dumps({'T': f['cmd_config']['cmd_set_led_pwm'], "IO4": IO4, "IO5": IO5}) + "\n"
        if data == self.last_led_sent_data:
            return
        self.last_led_sent_data = data          
        self.base_controller.send_command(data.encode()) 

    def send_pt_joint_ctrl(self, ptPoseState):
        if module_type == 2: 
            x_degree = (180 * ptPoseState.x) / 3.1415926
            y_degree = (180 * ptPoseState.y) / 3.1415926

            data = json.dumps({'T': f['cmd_config']['cmd_gimbal_ctrl'], 'X': x_degree, 'Y': y_degree, "SPD": 0, "ACC": 0}) + "\n"
            if data == self.last_pt_sent_data:
                return
            self.last_pt_sent_data = data     
            self.base_controller.send_command(data.encode()) 

    def send_leg_joint_ctrl(self):
        leg_idx = self.legJointState.leg
        state = self.leg_current_state[leg_idx]

        data = json.dumps({
            'T': f['cmd_config']['cmd_leg_rad_ctrl'],
            'leg': leg_idx,
            'coxa': state["coxa"],
            'femur': state["femur"],
            'tibia': state["tibia"],
        }) + "\n"

        if data == self.last_leg_sent_data:
            return

        self.last_leg_sent_data = data
        self.base_controller.send_command(data.encode())

    def handle_events(self):
        if self.js_reader.Joy_active:
            axis, buttons, hats = self.js_reader.get_state()

            if not axis or not buttons or not hats:
                return

            joystick_names = self.js_reader.get_joystick_names()

            if len(joystick_names) != 0:
                joystick_name = joystick_names[0]
                self.mapping = get_joystick_mapping(joystick_name)    

            x = 0
            y = 0
            yaw = 0

            val = axis[self.mapping["L2"]]
            l2_pressed = (val != -1.0 and val != 0)
            if l2_pressed and not self.prev_l2_pressed:
                self.speed_gear = self.next_gear(self.speed_gear, self.speed_level)
                self.show_msg("speed")
            self.prev_l2_pressed = l2_pressed

            l1_pressed = buttons[self.mapping["L1"]] == 1
            if l1_pressed and not self.prev_l1_pressed:
                self.speed_gear = self.down_gear(self.speed_gear, self.speed_level)
                self.show_msg("speed")
            self.prev_l1_pressed = l1_pressed

            r1_pressed = buttons[self.mapping["R1"]] == 1
            if r1_pressed and not self.prev_r1_pressed:
                self.mode = 'turn' if self.mode == 'move' else 'move'
                self.show_msg("mode")
            self.prev_r1_pressed = r1_pressed
            
            hat_x = hats[0][0]

            if self.mode == 'turn':
                yaw = -hat_x * self.yaw_speed_limit * self.speed_gear
            else:  # move
                y = -hat_x * self.yspeed_limit * self.speed_gear

            x = hats[0][1] * self.xspeed_limit * self.speed_gear

            self.send_cmd_vel(
                max(-self.xspeed_limit, min(self.xspeed_limit, x)),
                max(-self.yspeed_limit, min(self.yspeed_limit, y)),
                max(-self.yaw_speed_limit, min(self.yaw_speed_limit, yaw)),
            )

            leg_btn_pressed = buttons[self.mapping["LEFT_STICK_CLICK"]] == 1
            if leg_btn_pressed and not self.prev_leg_btn_pressed:
                self.legJointState.leg = (self.legJointState.leg + 1) % 6
                self.show_msg("leg")
            self.prev_leg_btn_pressed = leg_btn_pressed

            joint_sensitivity = 0.01

            leg_idx = self.legJointState.leg
            leg_state = self.leg_current_state[leg_idx]
            leg_limit = self.leg_limits[leg_idx]

            change_joystick_left_x = axis[self.mapping["LEFT_STICK_X"]]
            change_joystick_left_y = axis[self.mapping["LEFT_STICK_Y"]]

            # ---- coxa (X 轴) ----
            if abs(change_joystick_left_x) > joint_sensitivity:
                leg_state["coxa"] -= change_joystick_left_x * joint_sensitivity
                leg_state["coxa"] = max(
                    leg_limit["coxa"]["min"],
                    min(leg_limit["coxa"]["max"], leg_state["coxa"])
                )

            # ---- femur / tibia (Y 轴) ----
            if abs(change_joystick_left_y) > joint_sensitivity:
                if r1_pressed:
                    # R1 + Y → tibia
                    leg_state["tibia"] += change_joystick_left_y * joint_sensitivity
                    leg_state["tibia"] = max(
                        leg_limit["tibia"]["min"],
                        min(leg_limit["tibia"]["max"], leg_state["tibia"])
                    )
                else:
                    # Y → femur
                    leg_state["femur"] += change_joystick_left_y * joint_sensitivity
                    leg_state["femur"] = max(
                        leg_limit["femur"]["min"],
                        min(leg_limit["femur"]["max"], leg_state["femur"])
                    )

            self.send_leg_joint_ctrl()

            led_next_gear_pressed = buttons[self.mapping["Y"]] == 1
            led_down_gear_pressed = buttons[self.mapping["X"]] == 1

            current_time = time.time()

            if led_next_gear_pressed:
                if not self.prev_led_next_gear_pressed:
                    self.led_gear = self.next_gear(self.led_gear, self.led_level)
                    self.show_msg("led")
                    self.y_press_start = current_time
                    self.last_repeat_time = current_time
                else:
                    if current_time - self.y_press_start >= self.repeat_delay:
                        if current_time - self.last_repeat_time >= self.repeat_interval:
                            self.led_gear = self.next_gear(self.led_gear, self.led_level)
                            self.show_msg("led")
                            self.last_repeat_time = current_time
            else:
                self.y_press_start = None

            if led_down_gear_pressed:
                if not self.prev_led_down_gear_pressed:
                    self.led_gear = self.down_gear(self.led_gear, self.led_level)
                    self.show_msg("led")
                    self.x_press_start = current_time
                    self.last_repeat_time = current_time
                else:
                    if current_time - self.x_press_start >= self.repeat_delay:
                        if current_time - self.last_repeat_time >= self.repeat_interval:
                            self.led_gear = self.down_gear(self.led_gear, gear_levels)
                            self.show_msg("led")
                            self.last_repeat_time = current_time
            else:
                self.x_press_start = None

            self.prev_led_next_gear_pressed = led_next_gear_pressed
            self.prev_led_down_gear_pressed = led_down_gear_pressed

            led_data = self.led_limit * self.led_gear
            self.send_led_ctrl(led_data)

            if module_type == 2:
                change_x = axis[self.mapping["RIGHT_STICK_X"]]
                change_y = axis[self.mapping["RIGHT_STICK_Y"]]

                if abs(change_x)>0.001: 
                    self.ptPoseState.x += 0.025 * change_x
                if abs(change_y)>0.001: 
                    self.ptPoseState.y -= 0.025 * change_y

                self.ptPoseState.x = max(-3.14, min(3.14, self.ptPoseState.x))
                self.ptPoseState.y = max(-0.523, min(1.57, self.ptPoseState.y))	

                if buttons[self.mapping["RIGHT_STICK_CLICK"]]:
                    self.ptPoseState.x = 0.0
                    self.ptPoseState.y = 0.0

                self.send_pt_joint_ctrl(self.ptPoseState)

def main():
    js_reader = JoystickReader()
    js_reader.start()
    
    serial_port = "/dev/ttyACM0"
    joy_ctrl = JoyTeleop(js_reader, serial_port)
    base_controller = BaseController(serial_port, 115200)

    try:
        while True:
            if js_reader.Joy_active:
                joy_ctrl.handle_events()  
            time.sleep(0.05)            

    except KeyboardInterrupt:
        print("exit")
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
