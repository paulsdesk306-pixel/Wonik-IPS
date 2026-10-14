# %%
# Raycus Laser Control / Monitor - UDP
# Reference-UI revision
# - Keeps the original UDP ASCII control commands and dual IR sensor support.
# - Reworks the three requested GUI areas to match the supplied reference images:
#   01 CONNECTION & POLLING / 03 READBACK & STATUS / Command & error log.

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


# ============================================================
# Configuration
# ============================================================
DEFAULT_LASER_IP = "192.168.0.10"
DEFAULT_UDP_PORT = 8098
DEFAULT_READ_MS = 200       # one UDP read query every N ms
DEFAULT_TIMEOUT_MS = 700

IR_PORTS = ("COM5", "COM3")
IR_SENSOR_NAMES = ("Wafer", "Outside Wafer")
IR_BAUDRATE = 19200
IR_REQUEST_PACKET = bytes.fromhex("01 03 9C 42 00 02 4A 4F")
IR_RESPONSE_SIZE = 9
IR_POLL_INTERVAL = 0.05
IR_PLOT_WINDOW_SECONDS = 60.0
IR_PLOT_REFRESH_MS = 200
IR_HISTORY_MAXLEN = 4000

# One command per polling tick. A full cycle therefore takes
# len(POLL_COMMANDS) * Reads[ms].
POLL_COMMANDS = ("STA", "RCT", "RPRR", "RCS", "RPW", "RBT")

# STA bit definitions from the supplied Raycus Protocol Manual V2.0.
STA_BIT_LABELS = {
    0: "Reserved",
    1: "Over-temperature",
    2: "Laser enabled",
    3: "Reserved",
    4: "External AD ON",
    5: "Reserved",
    6: "Reserved",
    7: "Alarm exists",
    8: "Aiming beam ON",
    9: "Reserved",
    10: "Reserved",
    11: "Reserved",
    12: "Modulation ON",
    13: "Reserved",
    14: "Reserved",
    15: "Reserved",
    16: "Gate mode ON",
    17: "External control ON",
    18: "READY",
    19: "Reserved",
    20: "Reserved",
    21: "Reserved",
    22: "Wave mode ON",
    23: "Reserved",
    24: "Low temperature alarm",
    25: "Reserved",
    26: "Reserved",
    27: "External aiming beam",
    28: "Reserved",
    29: "Critical fault",
    30: "Reserved",
    31: "Reserved",
}


# ============================================================
# Persistent UDP client
# ============================================================
class RaycusUDPClient:
    """Single persistent UDP socket so the PC source port stays stable."""

    def __init__(self):
        self.sock = None
        self.ip = DEFAULT_LASER_IP
        self.port = DEFAULT_UDP_PORT
        self.timeout_s = DEFAULT_TIMEOUT_MS / 1000.0
        self.lock = threading.Lock()

    @property
    def is_open(self):
        return self.sock is not None

    @property
    def local_address(self):
        if self.sock is None:
            return None
        try:
            return self.sock.getsockname()
        except OSError:
            return None

    def open(self, ip, port, timeout_ms):
        self.close()
        self.ip = str(ip).strip()
        self.port = int(port)
        self.timeout_s = max(0.05, float(timeout_ms) / 1000.0)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout_s)
        # UDP connect() does not perform a TCP-style handshake.
        # It fixes the peer and keeps a stable local source port.
        sock.connect((self.ip, self.port))
        self.sock = sock
        return self.local_address

    def close(self):
        with self.lock:
            sock = self.sock
            self.sock = None
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    @staticmethod
    def _command_token(command):
        return str(command).strip().split()[0].upper()

    @classmethod
    def _matches_command(cls, command, response):
        upper = response.upper().lstrip()
        if upper.startswith("ERR") or "COMMAND ERR" in upper or upper == "BCMD":
            return True
        token = cls._command_token(command)
        if token == "RRT?":
            return upper.startswith("{") or "MAINCONTROL" in upper
        return upper.startswith(token)

    def command(self, command, timeout_ms=None):
        if self.sock is None:
            raise RuntimeError("Raycus UDP가 연결되지 않았습니다. 먼저 Connect를 누르세요.")

        command = str(command).strip("\r\n")
        payload = (command + "\r").encode("ascii")
        timeout_s = self.timeout_s if timeout_ms is None else max(0.05, float(timeout_ms) / 1000.0)

        with self.lock:
            if self.sock is None:
                raise RuntimeError("Raycus UDP가 연결되지 않았습니다.")

            self.sock.settimeout(timeout_s)

            # Flush delayed datagrams from a previous request.
            self.sock.setblocking(False)
            try:
                while True:
                    self.sock.recv(4096)
            except (BlockingIOError, OSError):
                pass
            finally:
                if self.sock is not None:
                    self.sock.setblocking(True)
                    self.sock.settimeout(timeout_s)

            t0 = time.perf_counter()
            self.sock.send(payload)
            deadline = time.perf_counter() + timeout_s

            while True:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Raycus UDP 응답 없음: {self.ip}:{self.port}, command={command!r}"
                    )
                self.sock.settimeout(remaining)
                try:
                    data = self.sock.recv(4096)
                except socket.timeout as exc:
                    raise TimeoutError(
                        f"Raycus UDP 응답 없음: {self.ip}:{self.port}, command={command!r}"
                    ) from exc

                response = data.decode("ascii", errors="replace").strip("\x00\r\n ")
                if not self._matches_command(command, response):
                    # Delayed response to another query; keep waiting within timeout.
                    continue
                break

            rtt_ms = (time.perf_counter() - t0) * 1000.0

        upper = response.upper()
        ok = not (
            upper.startswith("ERR")
            or upper == "BCMD"
            or "COMMAND ERR" in upper
        )

        local = self.local_address
        return {
            "ok": ok,
            "result": response,
            "command": command,
            "tx_ascii": command + r"\r",
            "tx_hex": payload.hex(" ").upper(),
            "rx_ascii": response,
            "rx_hex": data.hex(" ").upper(),
            "header": f"UDP {self.ip}:{self.port}",
            "local_address": local,
            "rtt_ms": rtt_ms,
        }


raycus = RaycusUDPClient()


# ============================================================
# Raycus command helpers
# ============================================================
def udp_command(command):
    return raycus.command(command, timeout_ms=get_timeout_ms())


def guide_laser_on():
    return udp_command("ABN")


def guide_laser_off():
    return udp_command("ABF")


def guide_external_on():
    return udp_command("EEABC")


def guide_external_off():
    return udp_command("DEABC")


def emission_on():
    return udp_command("EMON")


def emission_off():
    return udp_command("EMOFF")


def set_power(value):
    value = str(value).strip()
    if not value:
        raise ValueError("Power 값을 입력하세요.")
    float(value)
    return udp_command(f"SDC {value}")


def set_frequency(value):
    value = str(value).strip()
    if not value:
        raise ValueError("Frequency 값을 입력하세요.")
    float(value)
    return udp_command(f"SPRR {value}")


def set_pulse_width(value):
    value = str(value).strip()
    if not value:
        raise ValueError("Pulse Width 값을 입력하세요.")
    float(value)
    # This exact unit previously returned a valid SPW ACK.
    return udp_command(f"SPW {value}")


def gate_on():
    return udp_command("EGM")


def gate_off():
    return udp_command("DGM")


def modulation_on():
    return udp_command("EMOD")


def modulation_off():
    return udp_command("DMOD")


def hardware_emission_on():
    return udp_command("ELE")


def hardware_emission_off():
    return udp_command("DLE")


def external_control_on():
    return udp_command("EEC")


def external_control_off():
    return udp_command("DEC")


# ============================================================
# Parsing helpers
# ============================================================
def response_number(response, command):
    text = (response.get("result") or "").strip()
    if not response.get("ok"):
        raise RuntimeError(text or f"{command} error")
    # Keep only payload after ':' when present.
    if ":" in text:
        text = text.split(":", 1)[1]
    elif "：" in text:
        text = text.split("：", 1)[1]
    match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    if not match:
        raise ValueError(f"{command} 응답 숫자 파싱 실패: {response.get('result')!r}")
    return float(match.group(0))


def response_word(response, command):
    return int(round(response_number(response, command)))


def bit(word, index):
    return 1 if (word & (1 << index)) else 0


# ============================================================
# Dual non-contact IR serial workers
# ============================================================
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
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] "
                "pyserial 없음: pip install pyserial"
            )
            return

        try:
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 시도..."
            )
            self.ser = serial.Serial(
                self.port,
                IR_BAUDRATE,
                timeout=0.08,
                write_timeout=0.08,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 성공"
            )
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
                            response[3:5], byteorder="big", signed=True
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


# ============================================================
# GUI state
# ============================================================
root = None
output = None

ip_var = None
port_var = None
transport_var = None
reads_ms_var = None
timeout_ms_var = None
connection_var = None
polling_info_var = None
sta_rtt_var = None
local_port_var = None

power_entry = None
freq_entry = None
pulse_entry = None

readback_vars = {}
status_vars = {}
sta_bit_vars = {}
sta_mode_var = None
ir_current_vars = []

poll_in_progress = False
poll_index = 0
last_sta_poll_time = None
last_sta_period_ms = None
last_successful_comm = None
connected_since = None

latest_rprr_hz = None
latest_rpw_ms = None
latest_sta_value = None

emission_started_at = None
emission_elapsed_frozen = 0.0

# Avoid repeating identical polling errors in the log.
last_poll_error_by_command = {}

# IR trend data
ir_start_time = time.monotonic()
ir_history = [
    deque(maxlen=IR_HISTORY_MAXLEN),
    deque(maxlen=IR_HISTORY_MAXLEN),
]
ir_plot_axis = None
ir_plot_lines = [None, None]
ir_plot_canvas = None


# ============================================================
# Generic GUI helpers
# ============================================================
def get_timeout_ms():
    try:
        return max(50, int(float(timeout_ms_var.get())))
    except Exception:
        return DEFAULT_TIMEOUT_MS


def get_read_ms():
    try:
        return max(50, int(float(reads_ms_var.get())))
    except Exception:
        return DEFAULT_READ_MS


def log(text, level="INFO"):
    if output is None:
        return
    stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{stamp}] {level:<5} {text}\n"
    output.configure(state="normal")
    output.insert(tk.END, line)
    output.see(tk.END)
    output.configure(state="disabled")


def log_from_worker(text):
    if root is not None:
        root.after(0, lambda: log(text))


def format_command_result(result):
    local = result.get("local_address")
    local_text = f"{local[0]}:{local[1]}" if local else "-"
    return (
        f"TX {result.get('tx_ascii', '-')}  [{result.get('tx_hex', '-')}] | "
        f"RX {result.get('rx_ascii', '-')} | "
        f"RTT {result.get('rtt_ms', 0.0):.1f} ms | Local {local_text}"
    )


# ============================================================
# Connection / polling UI callbacks
# ============================================================
def connect_from_ui():
    try:
        ip = ip_var.get().strip()
        port = int(port_var.get())
        timeout = get_timeout_ms()
        if transport_var.get().upper() != "UDP":
            raise ValueError("현재 Raycus 코드는 UDP transport만 사용합니다.")
    except Exception as exc:
        messagebox.showwarning("Connection setting", str(exc))
        return

    connection_var.set("Connecting...")

    def worker():
        try:
            local = raycus.open(ip, port, timeout)
            try:
                result = raycus.command("RRT?", timeout_ms=timeout)
                verified = result.get("ok") and bool(result.get("result"))
                err = None
            except Exception as exc:
                # Keep UDP socket open. The user's laser has previously ACKed control
                # commands even when some read commands timed out.
                result = None
                verified = False
                err = exc
            root.after(0, lambda: finish_connect(local, verified, result, err))
        except Exception as exc:
            root.after(0, lambda: finish_connect(None, False, None, exc))

    threading.Thread(target=worker, daemon=True).start()


def finish_connect(local, verified, result, error):
    global connected_since, last_successful_comm

    if not raycus.is_open:
        connection_var.set("Disconnected")
        log(f"Connect failed: {error}", "ERROR")
        return

    connected_since = datetime.now()
    if verified:
        connection_var.set("Connected / RRT? OK")
        last_successful_comm = datetime.now()
        log("Connection check: CONNECTED")
        if result:
            log(format_command_result(result))
    else:
        connection_var.set("UDP socket open / RRT? no response")
        log(f"UDP socket opened, but RRT? did not answer: {error}", "WARN")

    if local:
        local_port_var.set(f"Local {local[0]}:{local[1]}")
    update_polling_info()


def disconnect_from_ui():
    global poll_in_progress, connected_since
    raycus.close()
    poll_in_progress = False
    connected_since = None
    connection_var.set("Disconnected")
    local_port_var.set("Local -")
    sta_rtt_var.set("STA RTT: -- ms")
    update_polling_info()
    log("Disconnected")


def update_polling_info():
    target = get_read_ms()
    sta_target = target * len(POLL_COMMANDS)
    actual = "--" if last_sta_period_ms is None else f"{last_sta_period_ms:.0f}"
    polling_info_var.set(
        f"STA target {sta_target} ms / actual {actual} ms | Reads target {target} ms/query"
    )


def poll_tick():
    global poll_in_progress, poll_index

    if raycus.is_open and not poll_in_progress:
        command = POLL_COMMANDS[poll_index % len(POLL_COMMANDS)]
        poll_index = (poll_index + 1) % len(POLL_COMMANDS)
        poll_in_progress = True
        threading.Thread(target=poll_worker, args=(command,), daemon=True).start()

    update_polling_info()
    root.after(get_read_ms(), poll_tick)


def poll_worker(command):
    try:
        result = raycus.command(command, timeout_ms=get_timeout_ms())
        root.after(0, lambda: finish_poll(command, result, None))
    except Exception as exc:
        root.after(0, lambda: finish_poll(command, None, exc))


def finish_poll(command, result, error):
    global poll_in_progress, last_successful_comm
    global last_sta_poll_time, last_sta_period_ms
    global latest_rprr_hz, latest_rpw_ms, latest_sta_value

    poll_in_progress = False

    if error is not None:
        msg = str(error)
        if last_poll_error_by_command.get(command) != msg:
            log(f"{command} readback: {msg}", "WARN")
            last_poll_error_by_command[command] = msg
        return

    last_poll_error_by_command.pop(command, None)
    last_successful_comm = datetime.now()

    try:
        if command == "RCT":
            value = response_number(result, "RCT")
            readback_vars["laser_temp"].set(f"{value:.1f} °C")

        elif command == "RPRR":
            value = response_number(result, "RPRR")
            latest_rprr_hz = value
            readback_vars["rep_rate"].set(f"{value:g} Hz")
            readback_vars["rep_freq"].set(f"{value / 1000.0:.3f} kHz")
            update_duty_display()

        elif command == "RCS":
            value = response_number(result, "RCS")
            readback_vars["current"].set(f"{value:g} %")

        elif command == "RPW":
            value = response_number(result, "RPW")
            latest_rpw_ms = value
            readback_vars["pulse_width"].set(f"{value:g} ms")
            update_duty_display()

        elif command == "RBT":
            value = response_number(result, "RBT")
            readback_vars["board_temp"].set(f"{value:.1f} °C")

        elif command == "STA":
            value = response_word(result, "STA")
            latest_sta_value = value
            readback_vars["sta"].set(f"{value}  (0x{value:08X})")
            update_sta_bits(value)
            update_decoded_status(value)

            now = time.monotonic()
            if last_sta_poll_time is not None:
                last_sta_period_ms = (now - last_sta_poll_time) * 1000.0
            last_sta_poll_time = now
            sta_rtt_var.set(f"STA RTT: {result.get('rtt_ms', 0.0):.1f} ms")

    except Exception as exc:
        log(f"{command} parse error: {exc}", "ERROR")


def update_duty_display():
    if latest_rprr_hz is None or latest_rpw_ms is None:
        readback_vars["duty"].set("--  (needs RPW + RPRR)")
        return
    duty = latest_rpw_ms * latest_rprr_hz / 10.0
    readback_vars["duty"].set(f"{duty:.2f} %")


# ============================================================
# Command callbacks
# ============================================================
def run_command(func, *args):
    def worker():
        try:
            result = func(*args)
            root.after(0, lambda: finish_command(result))
        except Exception as exc:
            root.after(0, lambda: command_error(exc))

    threading.Thread(target=worker, daemon=True).start()


def finish_command(result):
    global emission_started_at, emission_elapsed_frozen
    global latest_rprr_hz, latest_rpw_ms

    log(format_command_result(result), "OK" if result.get("ok") else "ERROR")
    command = (result.get("command") or "").upper()

    if result.get("ok"):
        try:
            if command == "EMON":
                emission_started_at = time.monotonic()
                emission_elapsed_frozen = 0.0
            elif command == "EMOFF":
                if emission_started_at is not None:
                    emission_elapsed_frozen = time.monotonic() - emission_started_at
                emission_started_at = None
            elif command.startswith("SDC"):
                value = response_number(result, "SDC")
                readback_vars["current"].set(f"{value:g} %  (ACK)")
            elif command.startswith("SPRR"):
                value = response_number(result, "SPRR")
                latest_rprr_hz = value
                readback_vars["rep_rate"].set(f"{value:g} Hz  (ACK)")
                readback_vars["rep_freq"].set(f"{value / 1000.0:.3f} kHz  (ACK)")
                update_duty_display()
            elif command.startswith("SPW"):
                value = response_number(result, "SPW")
                latest_rpw_ms = value
                readback_vars["pulse_width"].set(f"{value:g} ms  (ACK)")
                update_duty_display()
        except Exception:
            pass


def command_error(exc):
    log(str(exc), "ERROR")
    messagebox.showwarning("Command error", str(exc))


def apply_power():
    run_command(set_power, power_entry.get())


def apply_frequency():
    run_command(set_frequency, freq_entry.get())


def apply_pulse():
    run_command(set_pulse_width, pulse_entry.get())


# ============================================================
# STA display
# ============================================================
def update_sta_bits(value=None):
    if value is None:
        for idx in range(32):
            sta_bit_vars[idx].set(f"b{idx:02d}   -")
        return

    raw_only = sta_mode_var.get() == "Raw bits only"
    for idx in range(32):
        v = bit(value, idx)
        if raw_only:
            text = f"b{idx:02d}   {v}"
        else:
            label = STA_BIT_LABELS.get(idx, "Reserved")
            text = f"b{idx:02d}   {v}   {label}"
        sta_bit_vars[idx].set(text)


def update_decoded_status(sta):
    status_vars["emission"].set("ENABLED" if bit(sta, 2) else "DISABLED")
    status_vars["ad"].set("ON" if bit(sta, 4) else "OFF")
    status_vars["alarm"].set("ALARM" if bit(sta, 7) else "NORMAL")
    status_vars["guide_state"].set("ON" if bit(sta, 8) else "OFF")
    status_vars["modulation"].set("ON" if bit(sta, 12) else "OFF")
    status_vars["gate"].set("ON" if bit(sta, 16) else "OFF")
    status_vars["external_control"].set("ON" if bit(sta, 17) else "OFF")
    status_vars["ready"].set("READY" if bit(sta, 18) else "NOT READY")
    status_vars["wave"].set("ON" if bit(sta, 22) else "OFF")
    status_vars["guide_control"].set("EXTERNAL" if bit(sta, 27) else "INTERNAL")
    status_vars["critical"].set("FAULT" if bit(sta, 29) else "NORMAL")


# ============================================================
# IR GUI logic
# ============================================================
def update_ir_temperature(channel, temperature):
    if root is None:
        return

    def apply_value():
        if channel < len(ir_current_vars):
            ir_current_vars[channel].set(f"{temperature:.1f} °C")
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
        padding = max(1.0, (temp_max - temp_min) * 0.15)
        ir_plot_axis.set_ylim(temp_min - padding, temp_max + padding)

    ir_plot_canvas.draw_idle()
    root.after(IR_PLOT_REFRESH_MS, update_ir_plots)


def build_ir_plot(parent):
    global ir_plot_axis, ir_plot_canvas

    header = ttk.Frame(parent, padding=(8, 6))
    header.pack(fill="x")
    ttk.Label(header, text="Wafer:", style="ReadbackName.TLabel").pack(side="left")
    ttk.Label(header, textvariable=ir_current_vars[0], style="ReadbackValue.TLabel").pack(side="left", padx=(6, 24))
    ttk.Label(header, text="Outside Wafer:", style="ReadbackName.TLabel").pack(side="left")
    ttk.Label(header, textvariable=ir_current_vars[1], style="ReadbackValue.TLabel").pack(side="left", padx=(6, 0))

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


# ============================================================
# Time / session UI
# ============================================================
def update_fast_ui():
    if emission_started_at is not None:
        elapsed = time.monotonic() - emission_started_at
    else:
        elapsed = emission_elapsed_frozen
    readback_vars["elapsed"].set(f"{elapsed:.2f} s")

    # Session information is updated without additional network traffic.
    if connected_since is not None:
        status_vars["connected_since"].set(connected_since.strftime("%Y-%m-%d %H:%M:%S"))
    else:
        status_vars["connected_since"].set("-")
    if last_successful_comm is not None:
        status_vars["last_comm"].set(last_successful_comm.strftime("%Y-%m-%d %H:%M:%S"))
    else:
        status_vars["last_comm"].set("-")

    root.after(100, update_fast_ui)


# ============================================================
# GUI builders
# ============================================================
def build_connection_section(parent):
    ttk.Label(parent, text="01  CONNECTION & POLLING", style="Section.TLabel").pack(
        anchor="w", padx=12, pady=(8, 2)
    )
    frame = ttk.Frame(parent, relief="groove", borderwidth=1, padding=(12, 10))
    frame.pack(fill="x", padx=12, pady=(0, 6))

    row1 = ttk.Frame(frame)
    row1.pack(fill="x")

    ttk.Label(row1, text="IP selection").pack(side="left")
    ip_box = ttk.Combobox(row1, textvariable=ip_var, width=17, values=(DEFAULT_LASER_IP,))
    ip_box.pack(side="left", padx=(5, 14))

    ttk.Label(row1, text="Port").pack(side="left")
    ttk.Entry(row1, textvariable=port_var, width=7).pack(side="left", padx=(5, 14))

    ttk.Label(row1, text="Transport").pack(side="left")
    ttk.Combobox(
        row1,
        textvariable=transport_var,
        values=("UDP",),
        width=6,
        state="readonly",
    ).pack(side="left", padx=(5, 14))

    ttk.Label(row1, text="Reads [ms]").pack(side="left")
    ttk.Entry(row1, textvariable=reads_ms_var, width=6).pack(side="left", padx=(5, 14))

    ttk.Label(row1, text="Timeout [ms]").pack(side="left")
    ttk.Entry(row1, textvariable=timeout_ms_var, width=7).pack(side="left", padx=(5, 14))

    ttk.Button(row1, text="Connect", command=connect_from_ui, width=12).pack(side="left", padx=(0, 6))
    ttk.Button(row1, text="Disconnect", command=disconnect_from_ui, width=12).pack(side="left")

    row2 = ttk.Frame(frame)
    row2.pack(fill="x", pady=(12, 0))
    ttk.Label(row2, textvariable=connection_var, style="Connection.TLabel").pack(side="left")
    ttk.Label(row2, textvariable=local_port_var, style="Note.TLabel").pack(side="right")

    row3 = ttk.Frame(frame)
    row3.pack(fill="x", pady=(8, 0))
    ttk.Label(row3, textvariable=polling_info_var, style="Note.TLabel").pack(side="left")
    ttk.Label(row3, textvariable=sta_rtt_var, style="Note.TLabel").pack(side="right")


def build_control_panel(parent):
    ttk.Label(parent, text="02  LASER CONTROL", style="Section.TLabel").pack(
        anchor="w", pady=(0, 2)
    )
    frame = ttk.Frame(parent, relief="groove", borderwidth=1, padding=8)
    frame.pack(fill="both", expand=True)

    guide = ttk.LabelFrame(frame, text="Guide Laser", padding=6)
    guide.pack(fill="x", pady=(0, 6))
    for text, func in (
        ("Guide ON", guide_laser_on),
        ("Guide OFF", guide_laser_off),
        ("External Guide", guide_external_on),
        ("Internal Guide", guide_external_off),
    ):
        ttk.Button(guide, text=text, command=lambda f=func: run_command(f)).pack(
            side="left", padx=2, pady=2
        )

    params = ttk.LabelFrame(frame, text="Laser Parameters", padding=6)
    params.pack(fill="x", pady=6)

    ttk.Label(params, text="Power (%)").grid(row=0, column=0, sticky="w", padx=4, pady=4)
    power_entry.grid(in_=params, row=0, column=1, padx=4, pady=4)
    ttk.Button(params, text="SET", command=apply_power).grid(row=0, column=2, padx=4)

    ttk.Label(params, text="Frequency (Hz)").grid(row=1, column=0, sticky="w", padx=4, pady=4)
    freq_entry.grid(in_=params, row=1, column=1, padx=4, pady=4)
    ttk.Button(params, text="SET", command=apply_frequency).grid(row=1, column=2, padx=4)

    ttk.Label(params, text="Pulse Width (ms)").grid(row=2, column=0, sticky="w", padx=4, pady=4)
    pulse_entry.grid(in_=params, row=2, column=1, padx=4, pady=4)
    ttk.Button(params, text="SET", command=apply_pulse).grid(row=2, column=2, padx=4)

    modes = ttk.LabelFrame(frame, text="Operating Modes", padding=6)
    modes.pack(fill="x", pady=6)
    mode_buttons = (
        ("Gate ON", gate_on),
        ("Gate OFF", gate_off),
        ("Modulation ON", modulation_on),
        ("Modulation OFF", modulation_off),
        ("HW Emission Ctrl ON", hardware_emission_on),
        ("HW Emission Ctrl OFF", hardware_emission_off),
        ("External Control ON", external_control_on),
        ("External Control OFF", external_control_off),
    )
    for idx, (text, func) in enumerate(mode_buttons):
        ttk.Button(modes, text=text, command=lambda f=func: run_command(f)).grid(
            row=idx // 2, column=idx % 2, sticky="ew", padx=3, pady=3
        )
    modes.columnconfigure(0, weight=1)
    modes.columnconfigure(1, weight=1)

    emission = ttk.LabelFrame(frame, text="Main Laser Emission", padding=6)
    emission.pack(fill="x", pady=6)
    ttk.Button(emission, text="EMISSION ON", command=lambda: run_command(emission_on)).pack(
        side="left", padx=8, pady=4
    )
    ttk.Button(emission, text="EMISSION OFF", command=lambda: run_command(emission_off)).pack(
        side="left", padx=8, pady=4
    )

    note = ttk.LabelFrame(frame, text="Protocol", padding=6)
    note.pack(fill="x", pady=(6, 0))
    ttk.Label(
        note,
        text="UDP ASCII + CR (0x0D)  /  Raycus :8098\n"
             "Readback is round-robin to avoid flooding the device.",
        style="Note.TLabel",
        justify="left",
    ).pack(anchor="w")


def add_readback_row(parent, row, label, key, note):
    ttk.Label(parent, text=label, style="ReadbackName.TLabel").grid(
        row=row * 2, column=0, sticky="w", padx=(14, 10), pady=(8, 0)
    )
    ttk.Label(parent, textvariable=readback_vars[key], style="ReadbackValue.TLabel").grid(
        row=row * 2, column=1, sticky="w", padx=8, pady=(8, 0)
    )
    ttk.Label(parent, text=note, style="ReadbackNote.TLabel").grid(
        row=row * 2 + 1, column=0, columnspan=2, sticky="w", padx=(14, 10), pady=(0, 2)
    )


def build_readback_status(parent):
    ttk.Label(parent, text="03  READBACK & STATUS", style="Section.TLabel").pack(
        anchor="w", pady=(0, 2)
    )
    outer = ttk.Frame(parent, relief="groove", borderwidth=1, padding=4)
    outer.pack(fill="both", expand=True)

    top = ttk.Frame(outer)
    top.pack(fill="x")
    top.columnconfigure(1, weight=1)

    rows = (
        ("Laser temp", "laser_temp", "RCT / device"),
        ("Elapsed time", "elapsed", "Host time since EMON ack; not optical ON time"),
        ("Rep. Rate", "rep_rate", "RPRR / device setting [Hz]"),
        ("Current", "current", "RCS / current SETPOINT [%], not measured A"),
        ("Optical power", "optical_power", "Not exposed by the supplied ASCII command table"),
        ("Duty", "duty", "Computed from RPW × RPRR; not measured duty"),
        ("Repetition frequency", "rep_freq", "Same RPRR value, shown in kHz"),
        ("Pulse width", "pulse_width", "RPW / device setting [ms]"),
        ("Board temp", "board_temp", "RBT / controller board temperature"),
    )
    for idx, (label, key, note) in enumerate(rows):
        add_readback_row(top, idx, label, key, note)

    ttk.Separator(outer, orient="horizontal").pack(fill="x", padx=10, pady=8)

    sta_head = ttk.Frame(outer)
    sta_head.pack(fill="x", padx=10)
    ttk.Label(sta_head, text="STA:", style="ReadbackName.TLabel").pack(side="left")
    ttk.Label(sta_head, textvariable=readback_vars["sta"], style="ReadbackValue.TLabel").pack(
        side="left", padx=(8, 16)
    )
    ttk.Combobox(
        sta_head,
        textvariable=sta_mode_var,
        values=("Manual V2.0 mapping", "Raw bits only"),
        state="readonly",
        width=22,
    ).pack(side="left")

    ttk.Label(
        outer,
        text="Bit values are raw from STA; labels follow the supplied V2.0 manual.",
        style="ReadbackNote.TLabel",
    ).pack(anchor="w", padx=14, pady=(4, 5))

    bits_frame = ttk.Frame(outer, padding=(10, 0, 10, 8))
    bits_frame.pack(fill="both", expand=True)
    bits_frame.columnconfigure(0, weight=1)
    bits_frame.columnconfigure(1, weight=1)

    for idx in range(32):
        col = 0 if idx < 16 else 1
        row = idx if idx < 16 else idx - 16
        label = tk.Label(
            bits_frame,
            textvariable=sta_bit_vars[idx],
            anchor="w",
            font=("Consolas", 9),
            bg="#E6EBF1",
            fg="#355273",
            padx=8,
            pady=3,
            relief="flat",
        )
        label.grid(row=row, column=col, sticky="ew", padx=2, pady=1)


def build_status_decode_tab(parent):
    frame = ttk.Frame(parent, padding=10)
    frame.pack(fill="both", expand=True)
    fields = (
        ("emission", "Laser Enable"),
        ("ad", "External AD"),
        ("alarm", "Alarm"),
        ("guide_state", "Guide Beam"),
        ("modulation", "Modulation"),
        ("gate", "Gate"),
        ("external_control", "External Control"),
        ("ready", "Ready"),
        ("wave", "Wave Mode"),
        ("guide_control", "Guide Control"),
        ("critical", "Critical Fault"),
        ("connected_since", "Connected Since"),
        ("last_comm", "Last Successful Comm"),
    )
    for row, (key, label) in enumerate(fields):
        ttk.Label(frame, text=label, width=24).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Label(frame, textvariable=status_vars[key], style="ReadbackValue.TLabel").grid(
            row=row, column=1, sticky="w", pady=4
        )


def build_log_section(parent):
    ttk.Label(parent, text="Command / error log", style="Section.TLabel").pack(
        anchor="w", padx=12, pady=(4, 2)
    )
    frame = ttk.Frame(parent, relief="groove", borderwidth=1)
    frame.pack(fill="both", padx=12, pady=(0, 10))

    scrollbar = ttk.Scrollbar(frame, orient="vertical")
    scrollbar.pack(side="right", fill="y")

    global output
    output = tk.Text(
        frame,
        height=7,
        bg="#14273B",
        fg="#E7EEF7",
        insertbackground="white",
        selectbackground="#35597A",
        font=("Consolas", 9),
        relief="flat",
        wrap="word",
        yscrollcommand=scrollbar.set,
        padx=8,
        pady=6,
        state="disabled",
    )
    output.pack(side="left", fill="both", expand=True)
    scrollbar.configure(command=output.yview)


# ============================================================
# Main GUI
# ============================================================
def build_gui():
    global root
    global ip_var, port_var, transport_var, reads_ms_var, timeout_ms_var
    global connection_var, polling_info_var, sta_rtt_var, local_port_var
    global power_entry, freq_entry, pulse_entry
    global readback_vars, status_vars, sta_bit_vars, sta_mode_var, ir_current_vars

    root = tk.Tk()
    root.title("Raycus Laser Control / Monitor - UDP")
    root.geometry("1440x960")
    root.minsize(1180, 760)

    style = ttk.Style(root)
    try:
        style.theme_use("vista")
    except tk.TclError:
        pass
    style.configure("Section.TLabel", font=("Segoe UI", 10, "bold"))
    style.configure("Connection.TLabel", font=("Segoe UI", 9, "bold"), foreground="#173B67")
    style.configure("Note.TLabel", font=("Segoe UI", 8), foreground="#46627E")
    style.configure("ReadbackName.TLabel", font=("Segoe UI", 9, "bold"), foreground="#153A68")
    style.configure("ReadbackValue.TLabel", font=("Consolas", 10, "bold"), foreground="#003D79")
    style.configure("ReadbackNote.TLabel", font=("Segoe UI", 7), foreground="#5C7690")

    ip_var = tk.StringVar(root, DEFAULT_LASER_IP)
    port_var = tk.StringVar(root, str(DEFAULT_UDP_PORT))
    transport_var = tk.StringVar(root, "UDP")
    reads_ms_var = tk.StringVar(root, str(DEFAULT_READ_MS))
    timeout_ms_var = tk.StringVar(root, str(DEFAULT_TIMEOUT_MS))
    connection_var = tk.StringVar(root, "Disconnected")
    polling_info_var = tk.StringVar(root, "")
    sta_rtt_var = tk.StringVar(root, "STA RTT: -- ms")
    local_port_var = tk.StringVar(root, "Local -")

    readback_vars = {
        "laser_temp": tk.StringVar(root, "--"),
        "elapsed": tk.StringVar(root, "0.00 s"),
        "rep_rate": tk.StringVar(root, "--"),
        "current": tk.StringVar(root, "--"),
        "optical_power": tk.StringVar(root, "--"),
        "duty": tk.StringVar(root, "--  (needs RPW + RPRR)"),
        "rep_freq": tk.StringVar(root, "--"),
        "pulse_width": tk.StringVar(root, "--"),
        "board_temp": tk.StringVar(root, "--"),
        "sta": tk.StringVar(root, "--"),
    }

    status_keys = (
        "emission", "ad", "alarm", "guide_state", "modulation", "gate",
        "external_control", "ready", "wave", "guide_control", "critical",
        "connected_since", "last_comm",
    )
    status_vars = {key: tk.StringVar(root, "-") for key in status_keys}
    sta_bit_vars = {idx: tk.StringVar(root, f"b{idx:02d}   -") for idx in range(32)}
    sta_mode_var = tk.StringVar(root, "Manual V2.0 mapping")
    sta_mode_var.trace_add("write", lambda *_: update_sta_bits(latest_sta_value))
    ir_current_vars = [tk.StringVar(root, "--"), tk.StringVar(root, "--")]

    # Entries are created here and placed later with grid(in_=...).
    power_entry = ttk.Entry(root, width=12)
    freq_entry = ttk.Entry(root, width=12)
    pulse_entry = ttk.Entry(root, width=12)

    build_connection_section(root)

    body = ttk.Panedwindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=12, pady=4)

    control_host = ttk.Frame(body, width=430)
    monitor_host = ttk.Frame(body)
    body.add(control_host, weight=0)
    body.add(monitor_host, weight=1)

    build_control_panel(control_host)

    notebook = ttk.Notebook(monitor_host)
    notebook.pack(fill="both", expand=True)

    readback_tab = ttk.Frame(notebook)
    ir_tab = ttk.Frame(notebook)
    decoded_tab = ttk.Frame(notebook)
    notebook.add(readback_tab, text="Readback & Status")
    notebook.add(ir_tab, text="Wafer IR Trend")
    notebook.add(decoded_tab, text="Decoded STA")

    build_readback_status(readback_tab)
    build_ir_plot(ir_tab)
    build_status_decode_tab(decoded_tab)

    build_log_section(root)
    update_polling_info()


# ============================================================
# Run
# ============================================================
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
        raycus.close()
        for worker in ir_workers:
            worker.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, update_fast_ui)
    root.after(200, update_ir_plots)
    root.after(250, poll_tick)
    root.mainloop()


if __name__ == "__main__":
    main()
