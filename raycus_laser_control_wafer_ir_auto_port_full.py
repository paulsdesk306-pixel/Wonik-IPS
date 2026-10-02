# %%
# Raycus Laser Control / Monitor - UDP
# Unified fast UDP GUI - direct Duty Cycle input revision
# - Keeps the original UDP ASCII control commands and dual IR sensor support.
# - Reworks the three requested GUI areas to match the supplied reference images:
#   Unified controls, direct Duty Cycle entry, fast persistent UDP polling, and IR monitoring.
# - Optical power display is calculated from the RCS power setpoint using a 3 kW rated maximum.

import json
import os
import re
import socket
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

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
DEFAULT_READ_MS = 50        # target gap between read queries
DEFAULT_TIMEOUT_MS = 180     # short timeout keeps manual controls responsive

# Rated maximum optical output used for GUI estimate.
# IMPORTANT: this is a calculated value from the RCS setpoint, not a power-meter measurement.
MAX_OPTICAL_POWER_W = 3000.0

# IR 센서는 COM 번호를 하드코딩하지 않는다.
# 실행 중 Windows의 모든 COM 포트를 스캔하고, 실제 Modbus 응답 + CRC가
# 정상인 포트만 IR 센서로 판별한다.
IR_PORTS = [None, None]
IR_SENSOR_NAMES = ("Wafer", "Outside Wafer")
IR_BAUDRATE = 19200
IR_REQUEST_PACKET = bytes.fromhex("01 03 9C 42 00 02 4A 4F")
IR_RESPONSE_SIZE = 9
IR_POLL_INTERVAL = 0.05
IR_PLOT_WINDOW_SECONDS = 60.0
IR_PLOT_REFRESH_MS = 200
IR_HISTORY_MAXLEN = 4000

# IR 자동 검색/재연결
IR_PROBE_TIMEOUT = 0.15
IR_SCAN_INTERVAL = 2.0
IR_RECONNECT_DELAY = 0.5

# 최초 1회 역할을 정할 때 기존 장비 배선(COM5=Wafer, COM3=Outside)을
# 힌트로만 사용한다. 이후에는 USB serial number 또는 physical location을
# 저장해 COM 번호가 바뀌어도 같은 센서를 다시 찾는다.
IR_LEGACY_ROLE_HINTS = {
    "Wafer": "COM5",
    "Outside Wafer": "COM3",
}

_mapping_root = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "RaycusLaserControl"
IR_MAPPING_FILE = _mapping_root / "ir_sensor_mapping.json"

# One command per polling tick. A full cycle therefore takes
# len(POLL_COMMANDS) * Reads[ms].
POLL_COMMANDS = ("STA", "RCT", "RPRR", "RPW", "RCS", "RBT")

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


def set_duty_cycle(duty_value, frequency_hz):
    """Set Duty Cycle from the user-facing percentage field.

    The Raycus ASCII command table supplied with this project has no standalone
    "set duty" command. The device exposes SPRR (frequency) and SPW (pulse width).
    Therefore the GUI accepts Duty Cycle directly, while this helper performs the
    protocol-required SPW conversion internally and immediately verifies the
    result with RPRR/RPW readback. Pulse width is never exposed as a user input.
    """
    duty = float(str(duty_value).strip())
    frequency = float(str(frequency_hz).strip())
    if not (0.0 <= duty <= 100.0):
        raise ValueError("Duty Cycle은 0~100 % 범위로 입력하세요.")
    if frequency <= 0.0:
        raise ValueError("Duty Cycle 설정을 위해 Frequency는 0보다 커야 합니다.")

    # period_ms = 1000 / Hz; pulse_ms = period_ms * duty/100
    pulse_ms = duty * 10.0 / frequency

    # Always set frequency immediately before SPW. This prevents the laser from
    # evaluating SPW against an old repetition rate and returning a stale Duty.
    freq_ack = udp_command(f"SPRR {frequency:g}")
    if not freq_ack.get("ok"):
        return freq_ack

    pw_ack = udp_command(f"SPW {pulse_ms:.6f}")
    if not pw_ack.get("ok"):
        return pw_ack

    verified_frequency = None
    verified_pulse_ms = None
    verified_duty = None
    verify_errors = []

    # Best-effort verification. A firmware may ACK writes while timing out reads.
    try:
        rprr = udp_command("RPRR")
        verified_frequency = response_number(rprr, "RPRR")
    except Exception as exc:
        verify_errors.append(f"RPRR: {exc}")

    try:
        rpw = udp_command("RPW")
        verified_pulse_ms = response_number(rpw, "RPW")
    except Exception as exc:
        verify_errors.append(f"RPW: {exc}")

    if verified_frequency is not None and verified_pulse_ms is not None:
        verified_duty = verified_pulse_ms * verified_frequency / 10.0

    duty_from_ack = response_duty(pw_ack)
    if verified_duty is None and duty_from_ack is not None:
        verified_duty = duty_from_ack

    local = pw_ack.get("local_address")
    return {
        "ok": True,
        "result": pw_ack.get("result", ""),
        "command": f"DUTY {duty:g}",
        "tx_ascii": f"SPRR {frequency:g}\\r  ->  SPW {pulse_ms:.6f}\\r",
        "tx_hex": "(two UDP commands)",
        "rx_ascii": f"{freq_ack.get('rx_ascii', '')} | {pw_ack.get('rx_ascii', '')}",
        "rx_hex": "-",
        "header": pw_ack.get("header", f"UDP {raycus.ip}:{raycus.port}"),
        "local_address": local,
        "rtt_ms": float(freq_ack.get("rtt_ms", 0.0)) + float(pw_ack.get("rtt_ms", 0.0)),
        "requested_duty": duty,
        "requested_frequency": frequency,
        "internal_pulse_ms": pulse_ms,
        "verified_frequency": verified_frequency,
        "verified_pulse_ms": verified_pulse_ms,
        "verified_duty": verified_duty,
        "verify_errors": verify_errors,
    }


def _local_result(message, command="LOCAL"):
    return {
        "ok": True,
        "result": message,
        "command": command,
        "tx_ascii": "-",
        "tx_hex": "-",
        "rx_ascii": message,
        "rx_hex": "-",
        "rtt_ms": 0.0,
        "local_address": raycus.local_address,
        "from_address": None,
    }


def pulse_mode():
    return _local_result(
        "Pulse Mode button retained; no standalone verified ASCII UDP switch is assigned.",
        "Pulse Mode",
    )


def continuous_mode():
    return _local_result(
        "Continuous button retained; no standalone verified ASCII UDP switch is assigned.",
        "Continuous",
    )


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


# Original panel naming retained.
def external_enable_on():
    return hardware_emission_on()


def external_enable_off():
    return hardware_emission_off()


def ad_mode_on():
    return external_control_on()


def ad_mode_off():
    return external_control_off()


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


def response_duty(response):
    """Return Duty=xx% when the laser includes duty in an ACK."""
    text = (response.get("result") or "").replace("\r", " ")
    match = re.search(r"Duty\s*=\s*([-+]?\d+(?:\.\d+)?)\s*%", text, re.IGNORECASE)
    return float(match.group(1)) if match else None


def bit(word, index):
    return 1 if (word & (1 << index)) else 0


def update_power_readback(percent, suffix=""):
    """
    Update Current(setpoint %) and calculated Optical Power together.

    Optical Power is NOT measured optical output. It is an estimate based on
    the 3 kW rated maximum:
        optical_power_W = RCS_percent / 100 * 3000 W
    """
    percent = float(percent)
    optical_w = MAX_OPTICAL_POWER_W * percent / 100.0

    readback_vars["current"].set(f"{percent:g} %{suffix}")

    # Display unit automatically:
    #   < 1000 W  -> W
    #   >= 1000 W -> kW
    # Trailing zeros are removed, with up to 2 decimal places.
    def fmt_value(value):
        return f"{value:.2f}".rstrip("0").rstrip(".")

    if optical_w >= 1000.0:
        optical_text = f"{fmt_value(optical_w / 1000.0)} kW"
    else:
        optical_text = f"{fmt_value(optical_w)} W"

    readback_vars["optical_power"].set(f"{optical_text}{suffix}")

    return optical_w


def refresh_duty_from_device_readback(source="READBACK"):
    """Update Duty Cycle only when both device RPRR and RPW are known."""
    global latest_duty_pct
    if latest_rprr_hz is None or latest_rpw_ms is None:
        return None
    duty = latest_rpw_ms * latest_rprr_hz / 10.0
    latest_duty_pct = duty
    if readback_vars and "duty" in readback_vars:
        readback_vars["duty"].set(f"{duty:g} %")
    return duty


# ============================================================
# Dual non-contact IR serial workers - automatic COM discovery
# ============================================================
def modbus_crc16(data: bytes) -> int:
    """CRC-16/MODBUS. 반환값의 low byte가 wire에서 먼저 전송된다."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def valid_ir_response(response: bytes) -> bool:
    """IR 센서의 9-byte Modbus 응답을 header + byte count + CRC까지 검증한다."""
    if len(response) != IR_RESPONSE_SIZE:
        return False
    if response[0] != 0x01 or response[1] != 0x03 or response[2] != 0x04:
        return False
    received_crc = response[-2] | (response[-1] << 8)
    return received_crc == modbus_crc16(response[:-2])


def port_stable_key(info) -> str:
    """
    COM 번호 대신 가능한 한 안정적인 장치 식별자를 만든다.
    우선순위: USB serial number > USB physical location > HWID > COM 번호.
    """
    vid = f"{info.vid:04X}" if info.vid is not None else "----"
    pid = f"{info.pid:04X}" if info.pid is not None else "----"
    serial_no = (info.serial_number or "").strip()
    location = (getattr(info, "location", None) or "").strip()
    hwid = (info.hwid or "").strip()

    if serial_no:
        return f"SN:{vid}:{pid}:{serial_no}"
    if location:
        return f"LOC:{vid}:{pid}:{location}"
    if hwid:
        return f"HWID:{hwid}"
    return f"DEV:{info.device}"


def port_identity_text(info) -> str:
    serial_no = (info.serial_number or "").strip()
    location = (getattr(info, "location", None) or "").strip()
    if serial_no:
        return f"serial={serial_no}"
    if location:
        return f"location={location}"
    return "no stable USB serial/location"


def load_ir_mapping():
    try:
        if IR_MAPPING_FILE.exists():
            data = json.loads(IR_MAPPING_FILE.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
    except Exception:
        pass
    return {}


def save_ir_mapping(mapping):
    try:
        IR_MAPPING_FILE.parent.mkdir(parents=True, exist_ok=True)
        IR_MAPPING_FILE.write_text(
            json.dumps(mapping, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception:
        # Mapping 저장 실패가 센서 통신 자체를 막으면 안 된다.
        pass


def probe_ir_port(info):
    """한 COM 포트에 실제 IR 요청을 보내 보고 정상 센서인지 판별한다."""
    if serial is None:
        return None

    ser = None
    try:
        ser = serial.Serial(
            info.device,
            IR_BAUDRATE,
            timeout=IR_PROBE_TIMEOUT,
            write_timeout=IR_PROBE_TIMEOUT,
        )
        # USB-UART가 열린 직후 안정화될 시간을 조금 준다.
        time.sleep(0.03)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(IR_REQUEST_PACKET)
        ser.flush()
        response = ser.read(IR_RESPONSE_SIZE)
        if not valid_ir_response(response):
            return None

        raw_temp = int.from_bytes(response[3:5], byteorder="big", signed=True)
        return {
            "device": info.device,
            "key": port_stable_key(info),
            "identity": port_identity_text(info),
            "description": info.description or "",
            "temperature": raw_temp / 10.0,
            "vid": info.vid,
            "pid": info.pid,
            "serial_number": info.serial_number,
            "location": getattr(info, "location", None),
        }
    except Exception:
        return None
    finally:
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass


def discover_ir_sensors(exclude_ports=None):
    """현재 존재하는 COM 포트 중 실제 IR 센서로 응답하는 장치만 반환한다."""
    if serial is None or list_ports is None:
        return []

    excluded = set(exclude_ports or ())
    found = []
    for info in list_ports.comports():
        if info.device in excluded:
            continue
        sensor = probe_ir_port(info)
        if sensor is not None:
            found.append(sensor)

    def sort_key(sensor):
        match = re.search(r"(\d+)$", sensor["device"], re.IGNORECASE)
        com_num = int(match.group(1)) if match else 99999
        return (sensor["key"], com_num)

    found.sort(key=sort_key)
    return found


def choose_ir_assignments(candidates, missing_channels, mapping):
    """
    발견한 센서들을 Wafer/Outside 역할에 매칭한다.
    1) 저장된 stable key
    2) 최초 부트스트랩용 기존 COM 힌트
    3) 남은 센서를 안정 정렬 순서로 배정
    """
    available = list(candidates)
    assignments = {}

    # 1. Persisted stable identity
    for channel in list(missing_channels):
        role = IR_SENSOR_NAMES[channel]
        wanted_key = mapping.get(role)
        if not wanted_key:
            continue
        match = next((s for s in available if s["key"] == wanted_key), None)
        if match is not None:
            assignments[channel] = match
            available.remove(match)

    # 2. Existing wiring hint, only useful before a stable mapping exists.
    for channel in missing_channels:
        if channel in assignments:
            continue
        role = IR_SENSOR_NAMES[channel]
        hint = IR_LEGACY_ROLE_HINTS.get(role)
        match = next((s for s in available if s["device"].upper() == str(hint).upper()), None)
        if match is not None:
            assignments[channel] = match
            available.remove(match)

    # 3. Deterministic fallback.
    for channel in missing_channels:
        if channel in assignments or not available:
            continue
        assignments[channel] = available.pop(0)

    return assignments


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

    def is_alive(self):
        return bool(self.thread and self.thread.is_alive())

    def _run(self):
        if serial is None:
            self.log_callback("pyserial 없음: pip install pyserial")
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
                    if valid_ir_response(response):
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


class IRSensorManager:
    """
    IR 센서 포트를 자동 검색하고, 연결이 끊기거나 COM 번호가 바뀌면 재탐색한다.
    두 동일 센서를 구분할 때 USB serial number가 있으면 가장 안정적이고,
    serial number가 없으면 USB physical location을 이용한다.
    """
    def __init__(self, temp_callback, log_callback, assignment_callback):
        self.temp_callback = temp_callback
        self.log_callback = log_callback
        self.assignment_callback = assignment_callback
        self.stop_event = threading.Event()
        self.thread = None
        self.workers = [None, None]
        self.sensor_meta = [None, None]
        self.mapping = load_ir_mapping()
        self._lock = threading.Lock()

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="IR-AutoDiscovery", daemon=True)
        self.thread.start()

    def _cleanup_dead_workers(self):
        for channel in range(2):
            worker = self.workers[channel]
            if worker is not None and not worker.is_alive():
                try:
                    worker.stop()
                except Exception:
                    pass
                self.workers[channel] = None
                self.sensor_meta[channel] = None
                IR_PORTS[channel] = None
                self.assignment_callback(channel, None, None)

    def _run(self):
        if serial is None or list_ports is None:
            self.log_callback("IR 자동 검색 불가: pyserial을 설치하세요 (pip install pyserial)")
            return

        self.log_callback("IR 센서 자동 검색 시작: COM 번호 하드코딩 없이 실제 Modbus 응답으로 탐색")

        while not self.stop_event.is_set():
            self._cleanup_dead_workers()
            missing = [i for i, worker in enumerate(self.workers) if worker is None]

            if missing:
                active_ports = {
                    worker.port
                    for worker in self.workers
                    if worker is not None and worker.is_alive()
                }
                candidates = discover_ir_sensors(exclude_ports=active_ports)
                assignments = choose_ir_assignments(candidates, missing, self.mapping)

                for channel in missing:
                    sensor = assignments.get(channel)
                    if sensor is None:
                        continue

                    role = IR_SENSOR_NAMES[channel]
                    self.mapping[role] = sensor["key"]
                    save_ir_mapping(self.mapping)
                    IR_PORTS[channel] = sensor["device"]
                    self.sensor_meta[channel] = sensor
                    self.assignment_callback(channel, sensor["device"], sensor)
                    self.log_callback(
                        f"[IR 자동매칭] {role} -> {sensor['device']} "
                        f"({sensor['identity']}, probe={sensor['temperature']:.1f} °C)"
                    )

                    worker = IRSerialWorker(
                        channel=channel,
                        port=sensor["device"],
                        sensor_name=role,
                        temp_callback=self.temp_callback,
                        log_callback=self.log_callback,
                    )
                    self.workers[channel] = worker
                    worker.start()

            self.stop_event.wait(IR_SCAN_INTERVAL)

    def stop(self):
        self.stop_event.set()
        for worker in self.workers:
            if worker is not None:
                worker.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)


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
duty_entry = None
power_set_var = None
freq_set_var = None
duty_set_var = None

# Manual control commands take priority over background polling.
manual_command_pending = threading.Event()

readback_vars = {}
status_vars = {}
sta_bit_vars = {}
sta_mode_var = None
ir_current_vars = []
ir_port_vars = []

poll_in_progress = False
poll_index = 0
last_sta_poll_time = None
last_sta_period_ms = None
last_successful_comm = None
connected_since = None

latest_rprr_hz = None
latest_rpw_ms = None
latest_duty_pct = None
latest_requested_duty_pct = None
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
        return max(80, int(float(timeout_ms_var.get())))
    except Exception:
        return DEFAULT_TIMEOUT_MS


def get_read_ms():
    try:
        return max(30, int(float(reads_ms_var.get())))
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

    # Do not start a background read while the user is issuing a control command.
    # This prevents a slow read timeout from making button presses feel laggy.
    if (
        raycus.is_open
        and not poll_in_progress
        and not manual_command_pending.is_set()
    ):
        command = POLL_COMMANDS[poll_index % len(POLL_COMMANDS)]
        poll_index = (poll_index + 1) % len(POLL_COMMANDS)
        poll_in_progress = True
        threading.Thread(target=poll_worker, args=(command,), daemon=True).start()

    update_polling_info()
    root.after(get_read_ms(), poll_tick)


def poll_worker(command):
    try:
        # Readback gets a shorter timeout than manual controls so an unsupported
        # read command cannot freeze control responsiveness.
        read_timeout = min(get_timeout_ms(), 180)
        result = raycus.command(command, timeout_ms=read_timeout)
        root.after(0, lambda: finish_poll(command, result, None))
    except Exception as exc:
        root.after(0, lambda: finish_poll(command, None, exc))


def finish_poll(command, result, error):
    global poll_in_progress, last_successful_comm
    global last_sta_poll_time, last_sta_period_ms
    global latest_rprr_hz, latest_rpw_ms, latest_duty_pct, latest_sta_value

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
            refresh_duty_from_device_readback("READBACK")

        elif command == "RPW":
            # RPW is used only for device verification of Duty Cycle.
            # It is intentionally not exposed as a user-facing parameter.
            latest_rpw_ms = response_number(result, "RPW")
            refresh_duty_from_device_readback("READBACK")

        elif command == "RCS":
            value = response_number(result, "RCS")
            update_power_readback(value)

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


# ============================================================
# Command callbacks
# ============================================================
def run_command(func, *args):
    # Give button-triggered commands priority over background readback.
    manual_command_pending.set()

    def worker():
        try:
            result = func(*args)
            root.after(0, lambda: finish_command(result))
        except Exception as exc:
            root.after(0, lambda: command_error(exc))
        finally:
            manual_command_pending.clear()

    threading.Thread(target=worker, daemon=True).start()


def finish_command(result, log_result=True):
    global emission_started_at, emission_elapsed_frozen
    global latest_rprr_hz, latest_rpw_ms, latest_duty_pct, latest_requested_duty_pct

    if log_result:
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
                update_power_readback(value)
            elif command.startswith("SPRR"):
                value = response_number(result, "SPRR")
                latest_rprr_hz = value
                readback_vars["rep_rate"].set(f"{value:g} Hz  (ACK)")
                readback_vars["rep_freq"].set(f"{value / 1000.0:.3f} kHz  (ACK)")
                # Do not overwrite Duty with a stale Duty field returned by SPRR.
                # Duty is updated only by the dedicated DUTY operation or RPRR+RPW readback.
                refresh_duty_from_device_readback("READBACK")
            elif command.startswith("SPW"):
                duty = response_duty(result)
                if duty is not None:
                    latest_duty_pct = duty
                    readback_vars["duty"].set(f"{duty:g} %  (ACK)")
            elif command.startswith("DUTY"):
                requested = result.get("requested_duty")
                latest_requested_duty_pct = requested
                verified_frequency = result.get("verified_frequency")
                verified_pulse = result.get("verified_pulse_ms")
                verified_duty = result.get("verified_duty")
                if verified_frequency is not None:
                    latest_rprr_hz = verified_frequency
                    readback_vars["rep_rate"].set(f"{verified_frequency:g} Hz")
                    readback_vars["rep_freq"].set(f"{verified_frequency / 1000.0:.3f} kHz")
                if verified_pulse is not None:
                    latest_rpw_ms = verified_pulse
                if verified_duty is not None:
                    latest_duty_pct = verified_duty
                    readback_vars["duty"].set(f"{verified_duty:g} %  (VERIFIED)")
                elif requested is not None:
                    readback_vars["duty"].set(f"{requested:g} %  (REQUESTED; readback unavailable)")
                errors = result.get("verify_errors") or []
                if errors:
                    log("Duty verification: " + " | ".join(errors), "WARN")
        except Exception:
            pass


def command_error(exc):
    log(str(exc), "ERROR")
    messagebox.showwarning("Command error", str(exc))


def _validate_parameter_inputs(require_all=False):
    values = {}
    for name, var in (("power", power_set_var), ("frequency", freq_set_var), ("duty", duty_set_var)):
        text = var.get().strip()
        if not text:
            if require_all:
                raise ValueError(f"{name} 값을 입력하세요.")
            continue
        value = float(text)
        if value < 0:
            raise ValueError(f"{name} 값은 0 이상이어야 합니다.")
        values[name] = value

    if "power" in values and values["power"] > 100:
        raise ValueError("Power는 0~100 % 범위로 입력하세요.")
    if "duty" in values and values["duty"] > 100:
        raise ValueError("Duty Cycle은 0~100 % 범위로 입력하세요.")
    if "frequency" in values and values["frequency"] <= 0:
        raise ValueError("Frequency는 0보다 커야 합니다.")
    return values


def apply_power():
    _validate_parameter_inputs()
    run_command(set_power, power_set_var.get())


def apply_frequency():
    _validate_parameter_inputs()
    run_command(set_frequency, freq_set_var.get())


def apply_duty():
    try:
        values = _validate_parameter_inputs()
        if "duty" not in values:
            raise ValueError("Duty Cycle 값을 입력하세요.")
        if "frequency" not in values:
            raise ValueError("Duty Cycle 설정을 위해 Frequency를 먼저 입력하세요.")
    except Exception as exc:
        command_error(exc)
        return
    run_command(set_duty_cycle, f"{values['duty']:g}", f"{values['frequency']:g}")


def apply_all_parameters():
    try:
        values = _validate_parameter_inputs(require_all=True)
    except Exception as exc:
        command_error(exc)
        return

    manual_command_pending.set()

    def worker():
        try:
            commands = (
                ("Power", set_power, (f"{values['power']:g}",)),
                # set_duty_cycle sets SPRR first, then SPW, then verifies RPRR/RPW.
                ("Frequency + Duty Cycle", set_duty_cycle, (f"{values['duty']:g}", f"{values['frequency']:g}")),
            )
            results = []
            for label, func, args in commands:
                result = func(*args)
                results.append((label, result))
                if not result.get("ok"):
                    break
            root.after(0, lambda: finish_parameter_bundle(results))
        except Exception as exc:
            root.after(0, lambda: command_error(exc))
        finally:
            manual_command_pending.clear()

    threading.Thread(target=worker, daemon=True).start()


def finish_parameter_bundle(results):
    for label, result in results:
        log(f"{label}: {format_command_result(result)}", "OK" if result.get("ok") else "ERROR")
        finish_command(result, log_result=False)


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
def update_ir_assignment(channel, port, sensor_meta=None):
    """IR 자동 탐색 결과를 GUI 라벨/그래프 legend에 반영한다."""
    if root is None:
        return

    def apply_assignment():
        display_port = port or "AUTO SEARCH"
        if channel < len(ir_port_vars):
            ir_port_vars[channel].set(display_port)
        if port is None and channel < len(ir_current_vars):
            ir_current_vars[channel].set("--")

        line = ir_plot_lines[channel] if channel < len(ir_plot_lines) else None
        if line is not None:
            line.set_label(
                f"IR Sensor {channel + 1} - {IR_SENSOR_NAMES[channel]} ({display_port})"
            )
            if ir_plot_axis is not None:
                ir_plot_axis.legend(loc="upper left")
            if ir_plot_canvas is not None:
                ir_plot_canvas.draw_idle()

    root.after(0, apply_assignment)


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
    ttk.Label(header, text="Wafer", style="ReadbackName.TLabel").pack(side="left")
    ttk.Label(header, textvariable=ir_port_vars[0], style="Unit.TLabel").pack(side="left", padx=(5, 8))
    ttk.Label(header, textvariable=ir_current_vars[0], style="ReadbackValue.TLabel").pack(side="left", padx=(0, 24))
    ttk.Label(header, text="Outside Wafer", style="ReadbackName.TLabel").pack(side="left")
    ttk.Label(header, textvariable=ir_port_vars[1], style="Unit.TLabel").pack(side="left", padx=(5, 8))
    ttk.Label(header, textvariable=ir_current_vars[1], style="ReadbackValue.TLabel").pack(side="left", padx=(0, 0))

    frame = ttk.Frame(parent, padding=6)
    frame.pack(fill="both", expand=True)

    figure = Figure(figsize=(8.0, 5.5), dpi=100)
    axis = figure.add_subplot(111)
    line1, = axis.plot([], [], linewidth=1.5, label=f"IR Sensor 1 - {IR_SENSOR_NAMES[0]} (AUTO SEARCH)")
    line2, = axis.plot([], [], linewidth=1.5, label=f"IR Sensor 2 - {IR_SENSOR_NAMES[1]} (AUTO SEARCH)")
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
    global power_entry, freq_entry, duty_entry

    ttk.Label(parent, text="02  LASER CONTROL", style="Section.TLabel").pack(
        anchor="w", pady=(0, 2)
    )
    frame = ttk.Frame(parent, relief="groove", borderwidth=1, padding=10)
    frame.pack(fill="both", expand=True)

    guide = ttk.LabelFrame(frame, text="Guide Laser", padding=8)
    guide.pack(fill="x", pady=(0, 8))
    guide.columnconfigure((0, 1), weight=1)
    for idx, (text, func) in enumerate((
        ("Guide ON", guide_laser_on),
        ("Guide OFF", guide_laser_off),
        ("External Guide", guide_external_on),
        ("Internal Guide", guide_external_off),
    )):
        ttk.Button(guide, text=text, command=lambda f=func: run_command(f)).grid(
            row=idx // 2, column=idx % 2, sticky="ew", padx=3, pady=3
        )

    params = ttk.LabelFrame(frame, text="Laser Parameters", padding=8)
    params.pack(fill="x", pady=8)
    params.columnconfigure(1, weight=1)

    ttk.Label(params, text="Power (%)", style="ControlName.TLabel").grid(
        row=0, column=0, sticky="w", padx=(4, 10), pady=5
    )
    power_entry = ttk.Entry(params, textvariable=power_set_var, width=14)
    power_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=5)
    ttk.Button(params, text="SET", command=apply_power, width=9).grid(row=0, column=2, padx=4)

    ttk.Label(params, text="Frequency (Hz)", style="ControlName.TLabel").grid(
        row=1, column=0, sticky="w", padx=(4, 10), pady=5
    )
    freq_entry = ttk.Entry(params, textvariable=freq_set_var, width=14)
    freq_entry.grid(row=1, column=1, sticky="ew", padx=4, pady=5)
    ttk.Button(params, text="SET", command=apply_frequency, width=9).grid(row=1, column=2, padx=4)

    ttk.Label(params, text="Duty Cycle (%)", style="ControlName.TLabel").grid(
        row=2, column=0, sticky="w", padx=(4, 10), pady=5
    )
    duty_entry = ttk.Entry(params, textvariable=duty_set_var, width=14)
    duty_entry.grid(row=2, column=1, sticky="ew", padx=4, pady=5)
    ttk.Button(params, text="SET", command=apply_duty, width=9).grid(row=2, column=2, padx=4)

    ttk.Button(params, text="SET ALL PARAMETERS", command=apply_all_parameters).grid(
        row=3, column=0, columnspan=3, sticky="ew", padx=4, pady=(8, 3)
    )

    modes = ttk.LabelFrame(frame, text="Operating Modes", padding=8)
    modes.pack(fill="x", pady=8)
    modes.columnconfigure((0, 1), weight=1)
    mode_buttons = (
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
    )
    for idx, (text, func) in enumerate(mode_buttons):
        ttk.Button(modes, text=text, command=lambda f=func: run_command(f)).grid(
            row=idx // 2, column=idx % 2, sticky="ew", padx=3, pady=3
        )

    emission = ttk.LabelFrame(frame, text="Main Laser Emission", padding=8)
    emission.pack(fill="x", pady=8)
    emission.columnconfigure((0, 1), weight=1)
    ttk.Button(emission, text="EMISSION ON", command=lambda: run_command(emission_on)).grid(
        row=0, column=0, sticky="ew", padx=4, pady=3
    )
    ttk.Button(emission, text="EMISSION OFF", command=lambda: run_command(emission_off)).grid(
        row=0, column=1, sticky="ew", padx=4, pady=3
    )

    note = ttk.LabelFrame(frame, text="Protocol", padding=8)
    note.pack(fill="x", pady=(8, 0))
    ttk.Label(
        note,
        text=(
            "UDP ASCII + CR (0x0D)  /  Raycus :8098\n"
            "Manual controls have priority; readback runs sequentially in the background."
        ),
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
        ("Optical power", "optical_power", "Calculated from RCS setpoint using 3 kW rated max; not measured power"),
        ("Duty Cycle", "duty", "Duty verified from device readback after setting"),
        ("Repetition frequency", "rep_freq", "Same RPRR value, shown in kHz"),
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
    global power_entry, freq_entry, duty_entry
    global power_set_var, freq_set_var, duty_set_var
    global readback_vars, status_vars, sta_bit_vars, sta_mode_var, ir_current_vars, ir_port_vars

    root = tk.Tk()
    root.title("Raycus Laser Control / Monitor - UDP")
    root.geometry("1480x980")
    root.minsize(1220, 800)

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
    style.configure("ControlName.TLabel", font=("Segoe UI", 9, "bold"), foreground="#173B67")
    style.configure("Unit.TLabel", font=("Segoe UI", 8), foreground="#46627E")

    ip_var = tk.StringVar(root, DEFAULT_LASER_IP)
    port_var = tk.StringVar(root, str(DEFAULT_UDP_PORT))
    transport_var = tk.StringVar(root, "UDP")
    reads_ms_var = tk.StringVar(root, str(DEFAULT_READ_MS))
    timeout_ms_var = tk.StringVar(root, str(DEFAULT_TIMEOUT_MS))
    connection_var = tk.StringVar(root, "Disconnected")
    polling_info_var = tk.StringVar(root, "")
    sta_rtt_var = tk.StringVar(root, "STA RTT: -- ms")
    local_port_var = tk.StringVar(root, "Local -")

    # User-facing set-point inputs. Duty Cycle is entered directly.
    power_set_var = tk.StringVar(root, "10")
    freq_set_var = tk.StringVar(root, "100")
    duty_set_var = tk.StringVar(root, "50")

    readback_vars = {
        "laser_temp": tk.StringVar(root, "--"),
        "elapsed": tk.StringVar(root, "0.00 s"),
        "rep_rate": tk.StringVar(root, "--"),
        "current": tk.StringVar(root, "--"),
        "optical_power": tk.StringVar(root, "--"),
        "duty": tk.StringVar(root, "--"),
        "rep_freq": tk.StringVar(root, "--"),
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
    ir_port_vars = [tk.StringVar(root, "AUTO SEARCH"), tk.StringVar(root, "AUTO SEARCH")]

    build_connection_section(root)

    body = ttk.Panedwindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=12, pady=4)

    control_host = ttk.Frame(body, width=460)
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

    ir_manager = IRSensorManager(
        temp_callback=update_ir_temperature,
        log_callback=log_from_worker,
        assignment_callback=update_ir_assignment,
    )
    ir_manager.start()

    def on_close():
        raycus.close()
        ir_manager.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, update_fast_ui)
    root.after(200, update_ir_plots)
    root.after(250, poll_tick)
    root.mainloop()


if __name__ == "__main__":
    main()
