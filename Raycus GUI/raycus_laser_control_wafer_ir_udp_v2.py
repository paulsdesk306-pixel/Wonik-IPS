# %%
# Imports & configuration
import re
import socket
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import messagebox, ttk

try:
    import serial
except ImportError:
    serial = None

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError as exc:
    raise SystemExit("matplotlib이 필요합니다: pip install matplotlib") from exc

LASER_IP = "192.168.0.10"
UDP_PORT = 8098
UDP_TIMEOUT = 1.0
AUTO_REFRESH_MS = 1000

IR_PORTS = ("COM5", "COM3")
IR_SENSOR_NAMES = ("Wafer", "Outside Wafer")
# IR Sensor 1 = Wafer = COM5 / IR Sensor 2 = Outside Wafer = COM3
IR_BAUDRATE = 19200
IR_REQUEST_PACKET = bytes.fromhex("01 03 9C 42 00 02 4A 4F")
IR_RESPONSE_SIZE = 9
IR_POLL_INTERVAL = 0.05
IR_PLOT_WINDOW_SECONDS = 60.0
IR_PLOT_REFRESH_MS = 200
IR_HISTORY_MAXLEN = 4000

STATUS_TRANSLATION = {
    "空闲中": "대기 (Idle)",
    "出光中": "발진 중",
    "连续模式": "연속 모드 (CW)",
    "脉冲模式": "펄스 모드",
    "禁止": "비활성 (Disabled)",
    "使能": "활성 (Enabled)",
    "软件使能": "소프트웨어 제어",
    "硬件使能": "하드웨어 제어",
    "红光内控": "가이드 레이저 내부 제어",
    "红光外控": "가이드 레이저 외부 제어",
    "红光开": "가이드 레이저 ON",
    "红光关": "가이드 레이저 OFF",
    "激光器准备好": "READY",
    "激光器未准备好": "NOT READY",
    "激光器正常": "정상",
    "激光器异常": "이상",
    "正常": "정상",
}

STATUS_MARKERS = {
    "emission": "EMissON",
    "pulse_mode": "PluseEN",
    "gate": "GateEN",
    "modulation": "ModEN",
    "ad": "ADEN",
    "external_enable": "EXTEN",
    "wave": "WaveEN",
    "guide_control": "RedLedEXTEN",
    "guide_state": "RedOn",
    "ready": "Ready",
    "alarm": "Alarm",
}

ALARM_MARKERS = {
    "avg_power": "PowerMax",
    "emission": "LaserErr",
    "optical_power": "LPowerErr",
    "acdc": "ACDCErr",
    "boost": "BoostErr",
    "driver1": "Curdri1Err",
    "driver2": "Curdri2Err",
    "driver3": "Curdri3Err",
    "driver4": "Curdri4Err",
    "temperature": "Terr",
}

PARAMETER_FIELDS = [
    ("power", "Current Power"),
    ("frequency", "Current Frequency"),
    ("pulse_width", "Current Pulse Width"),
    ("temp1", "Internal Temp Sensor 1"),
    ("temp2", "Internal Temp Sensor 2"),
    ("mcu_temp", "Main Control Board Temp"),
    ("ir1", f"IR Sensor 1 - {IR_SENSOR_NAMES[0]} ({IR_PORTS[0]})"),
    ("ir2", f"IR Sensor 2 - {IR_SENSOR_NAMES[1]} ({IR_PORTS[1]})"),
    ("power_time", "Total Power-on Time"),
    ("laser_time", "Total Emission Time"),
]

STATUS_FIELDS = [
    ("emission", "Emission State"),
    ("pulse_mode", "Pulse / Continuous Mode"),
    ("gate", "Gate Mode"),
    ("modulation", "Modulation Mode"),
    ("ad", "External AD Mode"),
    ("external_enable", "External Laser Enable"),
    ("wave", "Waveform Mode"),
    ("guide_control", "Guide Laser Control"),
    ("guide_state", "Guide Laser State"),
    ("ready", "Laser Ready"),
    ("alarm", "Laser Alarm"),
]

ALARM_FIELDS = [
    ("avg_power", "Average Power"),
    ("emission", "Emission"),
    ("optical_power", "Optical Power"),
    ("acdc", "ACDC"),
    ("boost", "Boost Module"),
    ("driver1", "Current Driver Board 1"),
    ("driver2", "Current Driver Board 2"),
    ("driver3", "Current Driver Board 3"),
    ("driver4", "Current Driver Board 4"),
    ("temperature", "Temperature"),
]

TIME_FIELDS = [
    ("device_time", "Device Time"),
    ("device_date", "Device Date"),
    ("pc_time", "PC Current Time"),
    ("connected_since", "App Connected Since"),
    ("connection_duration", "Connection Duration"),
    ("last_poll", "Last Successful Communication"),
]


# %%
# Raycus UDP ASCII communication & control

# Packet Sender에서 확인한 것과 동일한 방식:
#   PC -> 192.168.0.10:8098 / UDP
#   ASCII command + CR(0x0D)
#   예: ABN\r, ABF\r, RRT?\r, STA\r
#
# 장비가 응답한 UDP packet은 요청에 사용한 PC source port로 돌아온다.

UDP_LOCK = threading.Lock()


def udp_command(command, timeout=UDP_TIMEOUT):
    """
    Raycus ASCII command를 UDP로 보내고 응답을 받는다.

    command에는 CR을 넣어도 되고 안 넣어도 된다.
    실제 송신 시 항상 마지막에 CR(0x0D)을 1개 붙인다.
    """
    command = str(command).strip('\r\n')
    payload = (command + '\r').encode('ascii')

    with UDP_LOCK:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            sock.sendto(payload, (LASER_IP, UDP_PORT))

            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout as exc:
                raise TimeoutError(
                    f"Raycus UDP 응답 없음: {LASER_IP}:{UDP_PORT}, command={command!r}"
                ) from exc

    response = data.decode('ascii', errors='replace').strip('\x00\r\n ')
    upper = response.upper()
    ok = not (
        upper.startswith('ERR')
        or upper in {'BCMD', 'COMMAND ERR!'}
        or 'COMMAND ERR' in upper
    )

    return {
        'ok': ok,
        'result': response,
        'body': response,          # 기존 GUI/코드 호환용
        'header': f"UDP {addr[0]}:{addr[1]}",
        'path': command,           # 기존 format 함수 호환용
        'command': command,
        'tx_ascii': command + r'\r',
        'tx_hex': payload.hex(' ').upper(),
        'rx_ascii': response,
        'rx_hex': data.hex(' ').upper(),
        'from_address': addr,
    }


def _local_result(message, ok=True, command='LOCAL'):
    """UDP ASCII 명령으로 직접 제공되지 않는 기능의 GUI 호환 결과."""
    return {
        'ok': ok,
        'result': message,
        'body': message,
        'header': 'LOCAL',
        'path': command,
        'command': command,
        'tx_ascii': '-',
        'tx_hex': '-',
        'rx_ascii': message,
        'rx_hex': '-',
        'from_address': None,
    }


def check_connection():
    try:
        response = udp_command('RRT?')
        return response['ok'] and bool(response['result'])
    except Exception:
        return False


def read_parameters():
    # HTTP의 LaserPara.shtml 대신 개별 ASCII read command를 사용한다.
    return {
        'power': udp_command('RCS'),
        'frequency': udp_command('RPRR'),
        'pulse_width': udp_command('RPW'),
        'laser_temp': udp_command('RCT'),
        'board_temp': udp_command('RBT'),
    }


def read_status():
    return udp_command('STA')


def read_alarm():
    # ASCII UDP protocol에서는 별도 Alarm page 대신 STA bit를 사용한다.
    return udp_command('STA')


def read_time():
    # 문서화된 ASCII UDP command table에는 RTC read command가 없다.
    return _local_result('N/A (RTC not exposed by ASCII UDP)', command='RTC')


def guide_laser_on():
    return udp_command('ABN')


def guide_laser_off():
    return udp_command('ABF')


def guide_external_on():
    return udp_command('EEABC')


def guide_external_off():
    return udp_command('DEABC')


def emission_on():
    return udp_command('EMON')


def emission_off():
    return udp_command('EMOFF')


def set_power(value):
    value = str(value).strip()
    if not value:
        raise ValueError('Power 값을 입력하세요.')
    return udp_command(f'SDC {value}')


def set_frequency(value):
    value = str(value).strip()
    if not value:
        raise ValueError('Frequency 값을 입력하세요.')
    return udp_command(f'SPRR {value}')


def set_pulse_width(value):
    value = str(value).strip()
    if not value:
        raise ValueError('Pulse Width 값을 입력하세요.')
    return udp_command(f'SPW {value}')


def pulse_mode():
    # 현재 확인된 RFL-A3000D-B ASCII command table에는
    # HTTP PulseMode.cgi와 1:1 대응하는 단독 QCW/CW 전환 명령이 없다.
    # SQCW는 power/frequency/pulse width를 함께 설정하는 별도 명령이므로
    # 여기서 임의로 호출하지 않는다.
    return _local_result(
        'Pulse/QCW mode switch is not exposed as a standalone ASCII UDP command',
        command='Pulse Mode',
    )


def continuous_mode():
    return _local_result(
        'Continuous/CW mode switch is not exposed as a standalone ASCII UDP command',
        command='Continuous Mode',
    )


def gate_on():
    return udp_command('EGM')


def gate_off():
    return udp_command('DGM')


def modulation_on():
    return udp_command('EMOD')


def modulation_off():
    return udp_command('DMOD')


def external_enable_on():
    return udp_command('ELE')


def external_enable_off():
    return udp_command('DLE')


def ad_mode_on():
    return udp_command('EEC')


def ad_mode_off():
    return udp_command('DEC')


# %%
# Raycus UDP parsing & structured reads

UNAVAILABLE = 'N/A (UDP readback unsupported by this firmware)'

# 이 장비에서는 제어 명령(ABN/ABF/SDC/SPRR/SPW/EMON/EMOFF 등)은
# ASCII UDP ACK가 정상적으로 돌아오지만, STA/RCS/RPRR/RPW/RCT/RBT 같은
# readback 명령은 실제 장비에서 응답하지 않는 것을 확인했다.
# 따라서 모니터는 읽기 명령을 반복 전송하지 않고, RRT?로 통신 생존 여부만 확인하며
# 아래 값들은 '장비가 ACK한 마지막 명령'을 shadow state로 표시한다.
# 실제 물리 상태/출력 상태를 의미한다고 가정하지 않는다.
laser_shadow = {
    'power': UNAVAILABLE,
    'frequency': UNAVAILABLE,
    'pulse_width': UNAVAILABLE,
    'emission': 'UNKNOWN (no UDP readback)',
    'pulse_mode': 'UNKNOWN (no UDP readback)',
    'gate': 'UNKNOWN (no UDP readback)',
    'modulation': 'UNKNOWN (no UDP readback)',
    'ad': 'UNKNOWN (no UDP readback)',
    'external_enable': 'UNKNOWN (no UDP readback)',
    'wave': 'UNKNOWN (no UDP readback)',
    'guide_control': 'UNKNOWN (no UDP readback)',
    'guide_state': 'UNKNOWN (no UDP readback)',
    'ready': 'UNKNOWN (no UDP readback)',
    'alarm': 'UNKNOWN (no UDP readback)',
}


def _set_shadow_from_ack(result):
    """성공적으로 ACK 받은 제어 명령만 shadow state에 반영한다."""
    if not isinstance(result, dict) or not result.get('ok'):
        return

    cmd = (result.get('command') or '').strip()
    response = (result.get('result') or '').strip()
    upper = cmd.upper()

    if upper == 'ABN':
        laser_shadow['guide_state'] = 'ON command ACK (actual state not read back)'
    elif upper == 'ABF':
        laser_shadow['guide_state'] = 'OFF command ACK (actual state not read back)'
    elif upper == 'EEABC':
        laser_shadow['guide_control'] = 'External guide command ACK'
    elif upper == 'DEABC':
        laser_shadow['guide_control'] = 'Internal guide command ACK'
    elif upper == 'EMON':
        laser_shadow['emission'] = 'ON command ACK (actual emission not read back)'
    elif upper == 'EMOFF':
        laser_shadow['emission'] = 'OFF command ACK (actual emission not read back)'
    elif upper == 'EGM':
        laser_shadow['gate'] = 'Enabled command ACK'
    elif upper == 'DGM':
        laser_shadow['gate'] = 'Disabled command ACK'
    elif upper == 'EMOD':
        laser_shadow['modulation'] = 'Enabled command ACK'
    elif upper == 'DMOD':
        laser_shadow['modulation'] = 'Disabled command ACK'
    elif upper == 'ELE':
        laser_shadow['external_enable'] = 'Enabled command ACK'
    elif upper == 'DLE':
        laser_shadow['external_enable'] = 'Disabled command ACK'
    elif upper == 'EEC':
        laser_shadow['ad'] = 'Enabled command ACK'
    elif upper == 'DEC':
        laser_shadow['ad'] = 'Disabled command ACK'
    elif upper.startswith('SDC '):
        value = extract_number(response.split(':', 1)[-1] if ':' in response else cmd[4:])
        laser_shadow['power'] = f'{value} % (last set ACK)'
    elif upper.startswith('SPRR '):
        value = extract_number(response.split(':', 1)[-1] if ':' in response else cmd[5:])
        laser_shadow['frequency'] = f'{value} Hz (last set ACK)'
    elif upper.startswith('SPW '):
        value = extract_number(response.split(':', 1)[-1] if ':' in response else cmd[4:])
        laser_shadow['pulse_width'] = f'{value} ms (last set ACK)'


def extract_number(value):
    match = re.search(r'[-+]?\d+(?:\.\d+)?', value or '')
    return match.group(0) if match else value


def with_unit(value, unit):
    if value in (None, '', '-'):
        return '-'
    if str(value).startswith('N/A'):
        return str(value)
    return f"{extract_number(str(value))} {unit}"


def seconds_display(value):
    if value in (None, '', '-'):
        return '-'
    if str(value).startswith('N/A'):
        return str(value)

    match = re.search(r'\d+', str(value))
    if not match:
        return str(value)

    total = int(match.group(0))
    hour, remainder = divmod(total, 3600)
    minute, second = divmod(remainder, 60)
    return f"{total} s  ({hour:02d}:{minute:02d}:{second:02d})"


def translate_value(value):
    return STATUS_TRANSLATION.get(value, value)


def _response_value(response, command=None):
    """RCS: 50.0 / RBT:36.6 같은 응답에서 값 부분만 꺼낸다."""
    if not response.get('ok'):
        raise RuntimeError(response.get('result') or f"{command or 'UDP'} command error")

    text = (response.get('result') or '').strip()
    if ':' in text:
        return text.split(':', 1)[1].strip()
    if '：' in text:
        return text.split('：', 1)[1].strip()
    return text


def _numeric_value(response, command=None):
    value = _response_value(response, command)
    match = re.search(r'[-+]?\d+(?:\.\d+)?', value)
    if not match:
        raise ValueError(f"숫자 응답 파싱 실패 ({command}): {response.get('result')!r}")
    return match.group(0)


def _read_sta_value(response=None):
    response = response or read_status()
    if not response.get('ok'):
        raise RuntimeError(response.get('result') or 'STA command error')

    text = response.get('result') or ''
    match = re.search(r'STA\s*[:：]\s*(\d+)', text, flags=re.IGNORECASE)
    if not match:
        # 일부 펌웨어가 값만 반환할 경우도 허용
        match = re.search(r'\b(\d+)\b', text)
    if not match:
        raise ValueError(f"STA 응답 파싱 실패: {text!r}")
    return int(match.group(1))


def _bit(value, index):
    return bool(value & (1 << index))


def _normal_alarm(is_alarm, alarm_text='이상', normal_text='정상'):
    return alarm_text if is_alarm else normal_text


def get_parameter_values():
    # 실제 장비의 ASCII UDP readback 명령(RCS/RPRR/RPW/RCT/RBT)이 응답하지 않으므로
    # ACK 받은 마지막 설정값만 표시한다. IR 센서는 별도 serial worker가 실시간 갱신한다.
    return {
        'power': laser_shadow['power'],
        'frequency': laser_shadow['frequency'],
        'pulse_width': laser_shadow['pulse_width'],
        'temp1': UNAVAILABLE,
        'temp2': UNAVAILABLE,
        'mcu_temp': UNAVAILABLE,
        'power_time': UNAVAILABLE,
        'laser_time': UNAVAILABLE,
    }


def get_status_values(status_value=None):
    # 중요: 아래 값은 실제 상태 readback이 아니라, 성공 ACK를 받은 마지막 명령 상태다.
    return {
        'emission': laser_shadow['emission'],
        'pulse_mode': laser_shadow['pulse_mode'],
        'gate': laser_shadow['gate'],
        'modulation': laser_shadow['modulation'],
        'ad': laser_shadow['ad'],
        'external_enable': laser_shadow['external_enable'],
        'wave': laser_shadow['wave'],
        'guide_control': laser_shadow['guide_control'],
        'guide_state': laser_shadow['guide_state'],
        'ready': laser_shadow['ready'],
        'alarm': laser_shadow['alarm'],
    }


def get_alarm_values(status_value=None):
    # 실제 alarm readback을 하지 못하는 상태에서 '정상'이라고 표시하면 위험하므로
    # 모든 항목을 명시적으로 UNKNOWN/N/A로 둔다.
    return {
        'avg_power': UNAVAILABLE,
        'emission': UNAVAILABLE,
        'optical_power': UNAVAILABLE,
        'acdc': UNAVAILABLE,
        'boost': UNAVAILABLE,
        'driver1': UNAVAILABLE,
        'driver2': UNAVAILABLE,
        'driver3': UNAVAILABLE,
        'driver4': UNAVAILABLE,
        'temperature': UNAVAILABLE,
    }


def get_time_values():
    return {
        'device_time': 'N/A (ASCII UDP)',
        'device_date': 'N/A (ASCII UDP)',
    }

def format_duration(seconds):
    seconds = max(0, int(seconds))
    hour, remainder = divmod(seconds, 3600)
    minute, second = divmod(remainder, 60)

    if hour >= 24:
        day, hour = divmod(hour, 24)
        return f"{day} d {hour:02d}:{minute:02d}:{second:02d}"
    return f"{hour:02d}:{minute:02d}:{second:02d}"


# %%
# Dual non-contact IR serial workers

class IRSerialWorker:
    def __init__(self, channel, port, sensor_name, temp_callback, log_callback):
        self.channel = channel
        self.port = port
        self.sensor_name = sensor_name
        self.temp_callback = temp_callback
        self.log_callback = log_callback
        self.stop_event = threading.Event()
        self.thread = None
        self.ser = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            name=f"IR{self.channel + 1}-{self.sensor_name.replace(' ', '_')}",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        if serial is None:
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] pyserial 없음: pip install pyserial"
            )
            return

        try:
            self.log_callback(f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 시도...")
            self.ser = serial.Serial(
                self.port,
                IR_BAUDRATE,
                timeout=0.08,
                write_timeout=0.08,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.log_callback(f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 성공")
        except Exception as exc:
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 실패: {exc}"
            )
            self._close_serial()
            return

        try:
            while not self.stop_event.is_set():
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(IR_REQUEST_PACKET)
                    self.ser.flush()

                    response = self.ser.read(IR_RESPONSE_SIZE)
                    if (
                        len(response) == IR_RESPONSE_SIZE
                        and response[0] == 0x01
                        and response[1] == 0x03
                    ):
                        raw_temp = int.from_bytes(
                            response[3:5],
                            byteorder="big",
                            signed=True,
                        )
                        self.temp_callback(self.channel, raw_temp / 10.0)
                except (serial.SerialException, OSError) as exc:
                    self.log_callback(
                        f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 통신 오류: {exc}"
                    )
                    break
                except Exception:
                    pass

                self.stop_event.wait(IR_POLL_INTERVAL)
        finally:
            self._close_serial()

    def _close_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def stop(self):
        self.stop_event.set()
        self._close_serial()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)


# %%
# GUI callbacks & monitor logic

root = None
output = None
parameter_vars = {}
status_vars = {}
alarm_vars = {}
time_vars = {}

connection_var = None
last_refresh_var = None
auto_refresh_var = None

power_entry = None
freq_entry = None
pulse_entry = None

refresh_in_progress = False
manual_refresh_pending = False
connected_since = None
last_successful_poll = None

# IR trend data is updated on the Tk main thread.
ir_start_time = time.monotonic()
ir_history = [
    deque(maxlen=IR_HISTORY_MAXLEN),
    deque(maxlen=IR_HISTORY_MAXLEN),
]
ir_plot_axis = None
ir_plot_lines = [None, None]
ir_plot_canvas = None


def log(text):
    if output is None:
        return
    output.insert(tk.END, text + "\n")
    output.see(tk.END)


def log_from_worker(text):
    if root is not None:
        root.after(0, log, text)


def update_ir_temperature(channel, temperature):
    if root is None:
        return

    def apply_value():
        key = f"ir{channel + 1}"
        if key in parameter_vars:
            parameter_vars[key].set(f"{temperature:.1f} °C")

        elapsed = time.monotonic() - ir_start_time
        ir_history[channel].append((elapsed, temperature))

    root.after(0, apply_value)


def update_ir_plots():
    if root is None or ir_plot_axis is None or ir_plot_canvas is None:
        return

    elapsed_now = time.monotonic() - ir_start_time
    cutoff = max(0.0, elapsed_now - IR_PLOT_WINDOW_SECONDS)
    visible_temperatures = []

    for channel in range(2):
        history = ir_history[channel]
        while history and history[0][0] < cutoff:
            history.popleft()

        line = ir_plot_lines[channel]
        if line is None:
            continue

        if history:
            times = [sample[0] for sample in history]
            temperatures = [sample[1] for sample in history]
            line.set_data(times, temperatures)
            visible_temperatures.extend(temperatures)
        else:
            line.set_data([], [])

    x_right = max(IR_PLOT_WINDOW_SECONDS, elapsed_now)
    ir_plot_axis.set_xlim(x_right - IR_PLOT_WINDOW_SECONDS, x_right)

    if visible_temperatures:
        temp_min = min(visible_temperatures)
        temp_max = max(visible_temperatures)
        span = temp_max - temp_min
        padding = max(1.0, span * 0.15)
        ir_plot_axis.set_ylim(temp_min - padding, temp_max + padding)

    ir_plot_canvas.draw_idle()
    root.after(IR_PLOT_REFRESH_MS, update_ir_plots)


def build_ir_plot(parent):
    global ir_plot_axis, ir_plot_canvas

    frame = ttk.Frame(parent, padding=6)
    frame.pack(fill="both", expand=True)

    figure = Figure(figsize=(8.0, 5.5), dpi=100)
    axis = figure.add_subplot(111)
    line1, = axis.plot([], [], linewidth=1.5, label=f"IR Sensor 1 - {IR_SENSOR_NAMES[0]} ({IR_PORTS[0]})")
    line2, = axis.plot([], [], linewidth=1.5, label=f"IR Sensor 2 - {IR_SENSOR_NAMES[1]} ({IR_PORTS[1]})")

    axis.set_title("Wafer / Outside-Wafer IR Temperature Trend")
    axis.set_xlabel("Elapsed Time (s)")
    axis.set_ylabel("Temperature (°C)")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left")
    axis.set_xlim(0, IR_PLOT_WINDOW_SECONDS)
    axis.set_ylim(20, 40)
    figure.tight_layout()

    canvas = FigureCanvasTkAgg(figure, master=frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    ir_plot_axis = axis
    ir_plot_lines[0] = line1
    ir_plot_lines[1] = line2
    ir_plot_canvas = canvas


def format_command_result(result):
    if not isinstance(result, dict):
        return str(result)
    return (
        f"UDP : {result.get('ok')}\n"
        f"TX : {result.get('tx_ascii', '-')}\n"
        f"TX HEX : {result.get('tx_hex', '-')}\n"
        f"RX : {result.get('rx_ascii', result.get('result'))}\n"
        f"From : {result.get('header', '-')}"
    )


def run_command(func, *args):
    def worker():
        try:
            result = func(*args)
            root.after(0, finish_command, result)
        except Exception as exc:
            root.after(0, command_error, exc)

    threading.Thread(target=worker, daemon=True).start()


def finish_command(result):
    _set_shadow_from_ack(result)
    log(format_command_result(result))
    request_monitor_refresh()


def command_error(exc):
    messagebox.showwarning("Command error", str(exc))
    log(f"ERROR: {exc}")


def update_connection():
    connection_var.set("CHECKING...")
    last_refresh_var.set("Checking connection...")

    def worker():
        connected = check_connection()
        root.after(0, finish_connection_check, connected)

    threading.Thread(target=worker, daemon=True).start()


def finish_connection_check(connected):
    if connected:
        connection_var.set("CONNECTED")
        log("Connection check: CONNECTED")
        request_monitor_refresh(manual=True)
    else:
        connection_var.set("DISCONNECTED")
        last_refresh_var.set("Connection check failed")
        log("Connection check: DISCONNECTED")


def apply_power():
    run_command(set_power, power_entry.get())


def apply_frequency():
    run_command(set_frequency, freq_entry.get())


def apply_pulse():
    run_command(set_pulse_width, pulse_entry.get())


def build_table(parent, fields, variables):
    frame = ttk.Frame(parent, padding=10)
    frame.pack(fill="both", expand=True)

    for row, (key, label) in enumerate(fields):
        ttk.Label(frame, text=label, width=30).grid(
            row=row,
            column=0,
            sticky="w",
            padx=(0, 10),
            pady=4,
        )
        ttk.Label(frame, textvariable=variables[key], width=32).grid(
            row=row,
            column=1,
            sticky="w",
            pady=4,
        )

    frame.columnconfigure(1, weight=1)


def monitor_worker():
    try:
        # 실제 장비에서 STA/RCS 계열 readback이 timeout이므로 반복 호출하지 않는다.
        # RRT?는 현재 장비에서 UDP request/response가 검증되어 있으므로
        # 통신 생존 여부 확인에만 사용한다.
        alive = udp_command('RRT?', timeout=UDP_TIMEOUT)
        if not alive.get('ok') or not alive.get('result'):
            raise RuntimeError('RRT? 응답이 유효하지 않습니다.')

        data = {
            'parameters': get_parameter_values(),
            'status': get_status_values(),
            'alarms': get_alarm_values(),
            'time': get_time_values(),
        }
        root.after(0, apply_monitor_data, data)
    except Exception as exc:
        root.after(0, apply_monitor_error, str(exc))

def request_monitor_refresh(manual=False):
    global refresh_in_progress, manual_refresh_pending

    if refresh_in_progress:
        if manual:
            manual_refresh_pending = True
            last_refresh_var.set("Refresh queued...")
        return False

    refresh_in_progress = True
    if manual:
        last_refresh_var.set("Refreshing...")
        log("Manual refresh started")

    threading.Thread(target=monitor_worker, daemon=True).start()
    return True


def apply_monitor_data(data):
    global refresh_in_progress, manual_refresh_pending
    global connected_since, last_successful_poll

    now = datetime.now()
    if connected_since is None:
        connected_since = now
    last_successful_poll = now

    for key, value in data["parameters"].items():
        parameter_vars[key].set(value)
    for key, value in data["status"].items():
        status_vars[key].set(value)
    for key, value in data["alarms"].items():
        alarm_vars[key].set(value)

    device_time = data["time"].get("device_time", "-")
    device_date = data["time"].get("device_date", "-")

    time_vars["device_time"].set(device_time)
    time_vars["device_date"].set(device_date)
    time_vars["pc_time"].set(now.strftime("%Y-%m-%d %H:%M:%S"))
    time_vars["connected_since"].set(
        connected_since.strftime("%Y-%m-%d %H:%M:%S")
    )
    time_vars["connection_duration"].set(
        format_duration((now - connected_since).total_seconds())
    )
    time_vars["last_poll"].set(
        last_successful_poll.strftime("%Y-%m-%d %H:%M:%S")
    )

    connection_var.set("CONNECTED")
    last_refresh_var.set(f"UDP alive {now.strftime('%H:%M:%S')}")
    refresh_in_progress = False

    if manual_refresh_pending:
        manual_refresh_pending = False
        root.after(10, lambda: request_monitor_refresh(manual=True))


def apply_monitor_error(message):
    global refresh_in_progress, manual_refresh_pending
    global connected_since, last_successful_poll

    connection_var.set("DISCONNECTED")
    last_refresh_var.set("ERROR")
    connected_since = None
    last_successful_poll = None
    refresh_in_progress = False

    time_vars["connected_since"].set("-")
    time_vars["connection_duration"].set("-")
    time_vars["last_poll"].set("-")
    log(f"MONITOR ERROR: {message}")

    if manual_refresh_pending:
        manual_refresh_pending = False
        root.after(100, lambda: request_monitor_refresh(manual=True))


def auto_refresh_loop():
    now = datetime.now()
    time_vars["pc_time"].set(now.strftime("%Y-%m-%d %H:%M:%S"))

    if connected_since is not None:
        time_vars["connection_duration"].set(
            format_duration((now - connected_since).total_seconds())
        )

    if auto_refresh_var.get():
        request_monitor_refresh()

    root.after(AUTO_REFRESH_MS, auto_refresh_loop)


# %%
# Build GUI & run

def build_gui():
    global root, output
    global parameter_vars, status_vars, alarm_vars, time_vars
    global connection_var, last_refresh_var, auto_refresh_var
    global power_entry, freq_entry, pulse_entry

    root = tk.Tk()
    root.title("Raycus Laser Control / Monitor - UDP v2")
    root.geometry("1320x920")
    root.minsize(1150, 760)

    parameter_vars = {
        key: tk.StringVar(master=root, value="-")
        for key, _ in PARAMETER_FIELDS
    }
    status_vars = {
        key: tk.StringVar(master=root, value="-")
        for key, _ in STATUS_FIELDS
    }
    alarm_vars = {
        key: tk.StringVar(master=root, value="-")
        for key, _ in ALARM_FIELDS
    }
    time_vars = {
        key: tk.StringVar(master=root, value="-")
        for key, _ in TIME_FIELDS
    }

    connection_frame = ttk.LabelFrame(root, text="Connection")
    connection_frame.pack(fill="x", padx=10, pady=(10, 5))

    ttk.Label(connection_frame, text="Laser IP:").pack(
        side="left", padx=(8, 4), pady=6
    )
    ttk.Label(connection_frame, text=f"{LASER_IP}:{UDP_PORT} / UDP").pack(
        side="left", padx=(0, 20)
    )

    connection_var = tk.StringVar(master=root, value="UNKNOWN")
    ttk.Label(connection_frame, textvariable=connection_var).pack(
        side="left", padx=10
    )
    ttk.Button(
        connection_frame,
        text="Check",
        command=update_connection,
    ).pack(side="left", padx=4)
    ttk.Button(
        connection_frame,
        text="Refresh Now",
        command=lambda: request_monitor_refresh(manual=True),
    ).pack(side="left", padx=4)

    auto_refresh_var = tk.BooleanVar(master=root, value=True)
    ttk.Checkbutton(
        connection_frame,
        text="Auto refresh (1 s)",
        variable=auto_refresh_var,
    ).pack(side="left", padx=12)

    ttk.Label(connection_frame, text="Device RTC:").pack(
        side="left", padx=(25, 4)
    )
    last_refresh_var = tk.StringVar(master=root, value="-")
    ttk.Label(connection_frame, textvariable=last_refresh_var).pack(side="left")

    body = ttk.Panedwindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=10, pady=5)

    control_panel = ttk.Frame(body, width=470)
    monitor_panel = ttk.Frame(body)
    body.add(control_panel, weight=0)
    body.add(monitor_panel, weight=1)

    guide_frame = ttk.LabelFrame(control_panel, text="Guide Laser")
    guide_frame.pack(fill="x", pady=4)
    guide_buttons = [
        ("Guide ON", guide_laser_on),
        ("Guide OFF", guide_laser_off),
        ("External Guide", guide_external_on),
        ("Internal Guide", guide_external_off),
    ]
    for text, func in guide_buttons:
        ttk.Button(
            guide_frame,
            text=text,
            command=lambda f=func: run_command(f),
        ).pack(side="left", padx=2, pady=5)

    parameter_frame = ttk.LabelFrame(control_panel, text="Laser Parameters")
    parameter_frame.pack(fill="x", pady=4)

    ttk.Label(parameter_frame, text="Power (%)").grid(
        row=0, column=0, sticky="w", padx=5, pady=4
    )
    power_entry = ttk.Entry(parameter_frame, width=12)
    power_entry.grid(row=0, column=1, padx=5, pady=4)
    ttk.Button(parameter_frame, text="SET", command=apply_power).grid(
        row=0, column=2, padx=5
    )

    ttk.Label(parameter_frame, text="Frequency (Hz)").grid(
        row=1, column=0, sticky="w", padx=5, pady=4
    )
    freq_entry = ttk.Entry(parameter_frame, width=12)
    freq_entry.grid(row=1, column=1, padx=5, pady=4)
    ttk.Button(parameter_frame, text="SET", command=apply_frequency).grid(
        row=1, column=2, padx=5
    )

    ttk.Label(parameter_frame, text="Pulse Width (ms)").grid(
        row=2, column=0, sticky="w", padx=5, pady=4
    )
    pulse_entry = ttk.Entry(parameter_frame, width=12)
    pulse_entry.grid(row=2, column=1, padx=5, pady=4)
    ttk.Button(parameter_frame, text="SET", command=apply_pulse).grid(
        row=2, column=2, padx=5
    )

    mode_frame = ttk.LabelFrame(control_panel, text="Operating Modes")
    mode_frame.pack(fill="x", pady=4)
    mode_buttons = [
        ("Pulse Mode", pulse_mode),
        ("Continuous", continuous_mode),
        ("Gate ON", gate_on),
        ("Gate OFF", gate_off),
        ("Modulation ON", modulation_on),
        ("Modulation OFF", modulation_off),
        ("External Enable", external_enable_on),
        ("External Disable", external_enable_off),
        ("AD Mode ON", ad_mode_on),
        ("AD Mode OFF", ad_mode_off),
    ]
    for index, (text, func) in enumerate(mode_buttons):
        ttk.Button(
            mode_frame,
            text=text,
            command=lambda f=func: run_command(f),
        ).grid(
            row=index // 2,
            column=index % 2,
            sticky="ew",
            padx=3,
            pady=3,
        )
    mode_frame.columnconfigure(0, weight=1)
    mode_frame.columnconfigure(1, weight=1)

    emission_frame = ttk.LabelFrame(control_panel, text="Main Laser Emission")
    emission_frame.pack(fill="x", pady=4)
    ttk.Button(
        emission_frame,
        text="EMISSION ON",
        command=lambda: run_command(emission_on),
    ).pack(side="left", padx=8, pady=6)
    ttk.Button(
        emission_frame,
        text="EMISSION OFF",
        command=lambda: run_command(emission_off),
    ).pack(side="left", padx=8, pady=6)

    notebook = ttk.Notebook(monitor_panel)
    notebook.pack(fill="both", expand=True)

    tabs = {
        "parameters": ttk.Frame(notebook),
        "ir_trend": ttk.Frame(notebook),
        "status": ttk.Frame(notebook),
        "alarms": ttk.Frame(notebook),
        "time": ttk.Frame(notebook),
    }
    notebook.add(tabs["parameters"], text="Realtime Parameters")
    notebook.add(tabs["ir_trend"], text="Wafer IR Trend")
    notebook.add(tabs["status"], text="Status")
    notebook.add(tabs["alarms"], text="Alarm")
    notebook.add(tabs["time"], text="Device Time")

    build_table(tabs["parameters"], PARAMETER_FIELDS, parameter_vars)
    build_ir_plot(tabs["ir_trend"])
    build_table(tabs["status"], STATUS_FIELDS, status_vars)
    build_table(tabs["alarms"], ALARM_FIELDS, alarm_vars)
    build_table(tabs["time"], TIME_FIELDS, time_vars)

    output_frame = ttk.LabelFrame(root, text="Communication Log")
    output_frame.pack(fill="both", padx=10, pady=(5, 10))
    output = tk.Text(output_frame, height=10)
    output.pack(fill="both", expand=True)


def main():
    build_gui()

    ir_workers = [
        IRSerialWorker(
            channel=index,
            port=port,
            sensor_name=IR_SENSOR_NAMES[index],
            temp_callback=update_ir_temperature,
            log_callback=log_from_worker,
        )
        for index, port in enumerate(IR_PORTS)
    ]

    for worker in ir_workers:
        worker.start()

    def on_close():
        for worker in ir_workers:
            worker.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    update_connection()
    root.after(200, update_ir_plots)
    root.after(500, auto_refresh_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
