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
from urllib.parse import urlencode

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
HTTP_PORT = 80
HTTP_TIMEOUT = 3.0
AUTO_REFRESH_MS = 1000

IR_PORTS = ("COM3", "COM5")
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
    ("ir1", f"Non-Contact IR Sensor CH1 ({IR_PORTS[0]})"),
    ("ir2", f"Non-Contact IR Sensor CH2 ({IR_PORTS[1]})"),
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
# Raycus HTTP communication & control

def http_get(path, params=None):
    if params:
        query = urlencode(params, encoding="gbk")
        path = f"{path}?{query}"

    request = (
        f"GET {path} HTTP/1.0\r\n"
        f"Host: {LASER_IP}\r\n"
        "Connection: close\r\n\r\n"
    ).encode("ascii")

    chunks = []
    with socket.create_connection((LASER_IP, HTTP_PORT), timeout=HTTP_TIMEOUT) as sock:
        sock.settimeout(HTTP_TIMEOUT)
        sock.sendall(request)
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.append(chunk)

    response = b"".join(chunks)
    header_raw, separator, body_raw = response.partition(b"\r\n\r\n")
    if not separator:
        body_raw = b""

    header = header_raw.decode("latin1", errors="replace")
    body = body_raw.decode("gbk", errors="replace")
    match = re.search(r"<!--#SetResult-->\s*([^<\r\n]+)", body)

    return {
        "ok": "200 OK" in header,
        "result": match.group(1).strip() if match else None,
        "body": body,
        "header": header,
        "path": path,
    }


def check_connection():
    try:
        return http_get("/")["ok"]
    except Exception:
        return False


def read_parameters():
    return http_get("/LaserPara.shtml")


def read_status():
    return http_get("/LaserStatus.shtml")


def read_alarm():
    return http_get("/LaserAlarm.shtml")


def read_time():
    return http_get("/LaserTime.shtml")


def guide_laser_on():
    return http_get("/GuideLaserON.cgi", {"Enable": "红光开"})


def guide_laser_off():
    return http_get("/GuideLaserON.cgi", {"Disable": "红光关"})


def guide_external_on():
    return http_get("/EXTGuideLaserEN.cgi", {"Enable": "红光外控"})


def guide_external_off():
    return http_get("/EXTGuideLaserEN.cgi", {"Disable": "红光内控"})


def emission_on():
    return http_get("/Emission.cgi", {"Enable": "出光"})


def emission_off():
    return http_get("/Emission.cgi", {"Disable": "关光"})


def set_power(value):
    return http_get(
        "/SetPower.cgi",
        {"Power": str(value), "button_SetPower": "Set"},
    )


def set_frequency(value):
    return http_get(
        "/SetFrequency.cgi",
        {"Frequency": str(value), "button_SetFreq": "Set"},
    )


def set_pulse_width(value):
    return http_get(
        "/SetPulseTime.cgi",
        {"PulseTime": str(value), "button_SetPulseTime": "Set"},
    )


def pulse_mode():
    result = http_get("/PulseMode.cgi", {"Enable": "脉冲模式"})
    if result.get("result") == "Command Err!":
        result["result"] = "Pulse Mode unsupported by this CW firmware"
    return result


def continuous_mode():
    try:
        status = read_status()
        current = extract_ssi(status.get("body", ""), "PluseEN", default="")
        if status.get("ok") and current == "连续模式":
            return {
                "ok": True,
                "result": "Already Continuous Mode (CW)",
                "body": status.get("body", ""),
                "header": status.get("header", ""),
                "path": "/PulseMode.cgi (not sent; already CW)",
            }
    except Exception:
        pass

    result = http_get("/PulseMode.cgi", {"Disable": "连续模式"})
    if result.get("result") == "Command Err!":
        result["result"] = "Continuous Mode is already fixed by this CW firmware"
    return result


def gate_on():
    return http_get("/GateMode.cgi", {"Enable": "GATE模式开"})


def gate_off():
    return http_get("/GateMode.cgi", {"Disable": "GATE模式关"})


def modulation_on():
    return http_get("/ModMode.cgi", {"Enable": "调制模式开"})


def modulation_off():
    return http_get("/ModMode.cgi", {"Disable": "调制模式关"})


def external_enable_on():
    return http_get("/ExtEN.cgi", {"Enable": "外部使能开"})


def external_enable_off():
    return http_get("/ExtEN.cgi", {"Disable": "外部使能关"})


def ad_mode_on():
    return http_get("/ADMODE.cgi", {"button_EEC": "AD模式使能"})


def ad_mode_off():
    return http_get("/ADMODE.cgi", {"button_DEC": "AD模式禁止"})


# %%
# Raycus parsing & structured reads

def extract_ssi(body, name, default="-"):
    match = re.search(
        rf"<!--#{re.escape(name)}-->\s*([^<\r\n]+)",
        body,
    )
    return match.group(1).strip() if match else default


def extract_number(value):
    match = re.search(r"[-+]?\d+(?:\.\d+)?", value or "")
    return match.group(0) if match else value


def with_unit(value, unit):
    if value in (None, "", "-"):
        return "-"
    return f"{extract_number(value)} {unit}"


def seconds_display(value):
    if value in (None, "", "-"):
        return "-"

    match = re.search(r"\d+", value)
    if not match:
        return value

    total = int(match.group(0))
    hour, remainder = divmod(total, 3600)
    minute, second = divmod(remainder, 60)
    return f"{total} s  ({hour:02d}:{minute:02d}:{second:02d})"


def translate_value(value):
    return STATUS_TRANSLATION.get(value, value)


def get_parameter_values():
    response = read_parameters()
    if not response["ok"]:
        raise RuntimeError("LaserPara.shtml HTTP error")

    body = response["body"]
    return {
        "power": with_unit(extract_ssi(body, "Powere"), "%"),
        "frequency": with_unit(extract_ssi(body, "Frequency"), "Hz"),
        "pulse_width": with_unit(extract_ssi(body, "PluseTime"), "ms"),
        "temp1": with_unit(extract_ssi(body, "T1"), "°C"),
        "temp2": with_unit(extract_ssi(body, "T2"), "°C"),
        "mcu_temp": with_unit(extract_ssi(body, "MCUT"), "°C"),
        "power_time": seconds_display(extract_ssi(body, "PowerTime")),
        "laser_time": seconds_display(extract_ssi(body, "LaserTime")),
    }


def get_status_values():
    response = read_status()
    if not response["ok"]:
        raise RuntimeError("LaserStatus.shtml HTTP error")

    body = response["body"]
    return {
        key: translate_value(extract_ssi(body, marker))
        for key, marker in STATUS_MARKERS.items()
    }


def get_alarm_values():
    response = read_alarm()
    if not response["ok"]:
        raise RuntimeError("LaserAlarm.shtml HTTP error")

    body = response["body"]
    return {
        key: translate_value(extract_ssi(body, marker))
        for key, marker in ALARM_MARKERS.items()
    }


def get_time_values():
    response = read_time()
    if not response["ok"]:
        raise RuntimeError("LaserTime.shtml HTTP error")

    body = response["body"]
    return {
        "device_time": extract_ssi(body, "hour"),
        "device_date": extract_ssi(body, "year"),
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
    def __init__(self, channel, port, temp_callback, log_callback):
        self.channel = channel
        self.port = port
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
            name=f"IR-CH{self.channel + 1}",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        if serial is None:
            self.log_callback(
                f"[IR CH{self.channel + 1}] pyserial 없음: pip install pyserial"
            )
            return

        try:
            self.log_callback(f"[IR CH{self.channel + 1}] {self.port} 연결 시도...")
            self.ser = serial.Serial(
                self.port,
                IR_BAUDRATE,
                timeout=0.08,
                write_timeout=0.08,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.log_callback(f"[IR CH{self.channel + 1}] {self.port} 연결 성공")
        except Exception as exc:
            self.log_callback(
                f"[IR CH{self.channel + 1}] {self.port} 연결 실패: {exc}"
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
                        f"[IR CH{self.channel + 1}] 통신 오류: {exc}"
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
ir_plot_axes = [None, None]
ir_plot_lines = [None, None]
ir_plot_canvases = [None, None]


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
    if root is None:
        return

    elapsed_now = time.monotonic() - ir_start_time
    cutoff = max(0.0, elapsed_now - IR_PLOT_WINDOW_SECONDS)

    for channel in range(2):
        history = ir_history[channel]
        while history and history[0][0] < cutoff:
            history.popleft()

        axis = ir_plot_axes[channel]
        line = ir_plot_lines[channel]
        canvas = ir_plot_canvases[channel]
        if axis is None or line is None or canvas is None:
            continue

        if history:
            times = [sample[0] for sample in history]
            temperatures = [sample[1] for sample in history]
            line.set_data(times, temperatures)

            temp_min = min(temperatures)
            temp_max = max(temperatures)
            span = temp_max - temp_min
            padding = max(1.0, span * 0.15)
            axis.set_ylim(temp_min - padding, temp_max + padding)
        else:
            line.set_data([], [])

        x_right = max(IR_PLOT_WINDOW_SECONDS, elapsed_now)
        axis.set_xlim(x_right - IR_PLOT_WINDOW_SECONDS, x_right)
        canvas.draw_idle()

    root.after(IR_PLOT_REFRESH_MS, update_ir_plots)


def build_ir_plot(parent, channel):
    frame = ttk.Frame(parent, padding=6)
    frame.pack(fill="both", expand=True)

    figure = Figure(figsize=(6.8, 3.0), dpi=100)
    axis = figure.add_subplot(111)
    line, = axis.plot([], [], linewidth=1.4)

    axis.set_title(
        f"IR Sensor {channel + 1} Temperature ({IR_PORTS[channel]})"
    )
    axis.set_xlabel("Elapsed Time (s)")
    axis.set_ylabel("Temperature (°C)")
    axis.grid(True, alpha=0.3)
    axis.set_xlim(0, IR_PLOT_WINDOW_SECONDS)
    axis.set_ylim(20, 40)
    figure.tight_layout()

    canvas = FigureCanvasTkAgg(figure, master=frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    ir_plot_axes[channel] = axis
    ir_plot_lines[channel] = line
    ir_plot_canvases[channel] = canvas


def format_command_result(result):
    if not isinstance(result, dict):
        return str(result)
    return (
        f"HTTP : {result.get('ok')}\n"
        f"Result : {result.get('result')}\n"
        f"Path : {result.get('path')}"
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
        data = {
            "parameters": get_parameter_values(),
            "status": get_status_values(),
            "alarms": get_alarm_values(),
            "time": get_time_values(),
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
    last_refresh_var.set(f"{device_date} {device_time}")
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
    root.title("Raycus Laser Control / Monitor")
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
    ttk.Label(connection_frame, text=LASER_IP).pack(
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
    notebook.add(tabs["ir_trend"], text="IR Temperature Trend")
    notebook.add(tabs["status"], text="Status")
    notebook.add(tabs["alarms"], text="Alarm")
    notebook.add(tabs["time"], text="Device Time")

    build_table(tabs["parameters"], PARAMETER_FIELDS, parameter_vars)
    build_ir_plot(tabs["ir_trend"], 0)
    build_ir_plot(tabs["ir_trend"], 1)
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
