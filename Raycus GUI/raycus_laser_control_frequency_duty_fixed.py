# %%
# Raycus Laser Control / Monitor - UDP
# Unified fast UDP GUI - direct Duty Cycle input revision
# - Keeps the original UDP ASCII control commands and dual IR sensor support.
# - Reworks the three requested GUI areas to match the supplied reference images:
#   Unified controls, direct Duty Cycle entry, fast persistent UDP polling, and IR monitoring.
# - Optical power display is calculated from the RCS power setpoint using a 3 kW rated maximum.

import csv
import json
import math
import numpy as np
import os
import re
import socket
import threading
import tempfile
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

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
IR_REGISTER_ADDRESS = 0x9C42
IR_REGISTER_COUNT = 2
IR_SLAVE_ID = 1
# 실제 기존 코드에서 정상 동작한 요청 프레임을 그대로 사용한다.
# COM 번호만 실행 시점에 자동으로 찾아낸다.
IR_KNOWN_REQUEST = bytes.fromhex("01 03 9C 42 00 02 4A 4F")
IR_RESPONSE_SIZE = 9
IR_POLL_INTERVAL = 0.05
IR_PLOT_WINDOW_SECONDS = 60.0
IR_PLOT_REFRESH_MS = 200
IR_HISTORY_MAXLEN = 4000

# IR 자동 검색/재연결. 0.10 s는 일부 USB-Serial에서 탐색 시 너무 짧을 수 있어
# 실제 센서 탐색은 최대 0.30 s, 3회 재시도한다.
IR_PROBE_TIMEOUT = 0.30
IR_PROBE_ATTEMPTS = 3
IR_OPEN_SETTLE = 0.15
IR_SCAN_INTERVAL = 2.0
IR_RECONNECT_DELAY = 0.5

# 현재 확인된 배선에서 첫 자동 학습 시 사용할 힌트.
# 사용자가 알려준 현재 상태: COM5 쪽이 Outside, COM8 쪽이 Wafer.
# 한 번 identity를 저장한 뒤에는 COM 번호가 바뀌어도 이 값은 사용하지 않는다.
IR_BOOTSTRAP_ROLE_HINTS = {
    "Wafer": "COM8",
    "Outside Wafer": "COM5",
}

_mapping_root = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "RaycusLaserControl"
IR_MAPPING_FILE = _mapping_root / "ir_sensor_mapping.json"
# v3: Modbus ID를 장치 identity로 쓰지 않고 USB serial/location/HWID만 사용.
IR_ROLE_MAPPING_VERSION = 3

# 20CH thermocouple (TC) acquisition.
# Protocol taken from the supplied v8.2 logger:
#   COM4 / 115200 baud / send b"s" to start / b"q" to stop.
# The GUI port remains editable if Windows assigns another COM number.
TC_DEFAULT_PORT = "COM4"
TC_BAUDRATE = 115200
TC_SERIAL_TIMEOUT = 0.02
TC_WRITE_TIMEOUT = 0.02
TC_CHANNEL_COUNT = 20
TC_PLOT_WINDOW_SECONDS = 60.0
TC_PLOT_REFRESH_MS = 200
TC_HEATMAP_REFRESH_MS = 300
TC_DATA_PROCESS_MS = 50
TC_RENDER_MAX_POINTS = 450
TC_HISTORY_MAXLEN = 12000

# TC wafer-layout mapping borrowed from the supplied v8.2 logger.
# sensor_id -> TC channel index (0-based)
TC_SENSOR_MAPPING = {
    1:0, 2:1, 3:2, 4:3, 5:4, 6:5, 7:6, 8:7, 9:8, 10:9,
    11:17, 12:12, 13:13, 14:14, 15:15, 16:16, 17:19,
}

def _generate_tc_wafer_coords():
    coords = {9: (0.0, 0.0)}
    r_in, r_out = 0.40, 0.85
    angles = [90, 45, 0, -45, -90, -135, 180, 135]
    outer_sensors = [1, 2, 11, 15, 17, 16, 7, 3]
    for sensor_id, angle in zip(outer_sensors, angles):
        rad = math.radians(angle)
        coords[sensor_id] = (r_out * math.cos(rad), r_out * math.sin(rad))
    inner_sensors = [4, 5, 10, 12, 14, 13, 8, 6]
    for sensor_id, angle in zip(inner_sensors, angles):
        rad = math.radians(angle)
        coords[sensor_id] = (r_in * math.cos(rad), r_in * math.sin(rad))
    return coords

TC_SENSOR_COORDS = _generate_tc_wafer_coords()
TC_HEATMAP_GRID = 100

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
    frequency = float(value)
    if frequency <= 0.0:
        raise ValueError("Frequency는 0보다 커야 합니다.")

    # 단독 Rep Rate 변경이 현재 Pulse Width와 충돌하면 현재 Duty를 유지하며 함께 변경.
    current_frequency = response_number(udp_command("RPRR"), "RPRR")
    current_pulse_ms = response_number(udp_command("RPW"), "RPW")
    if current_pulse_ms * frequency / 10.0 > 100.0:
        current_duty = current_pulse_ms * current_frequency / 10.0
        return set_duty_cycle(current_duty, frequency)

    return udp_command(f"SPRR {value}")


def set_duty_cycle(duty_value, frequency_hz):
    """Set Duty Cycle via SPRR/SPW while avoiding invalid intermediate Duty."""
    duty = float(str(duty_value).strip())
    frequency = float(str(frequency_hz).strip())
    if not (0.0 <= duty <= 100.0):
        raise ValueError("Duty Cycle은 0~100 % 범위로 입력하세요.")
    if frequency <= 0.0:
        raise ValueError("Duty Cycle 설정을 위해 Frequency는 0보다 커야 합니다.")

    # period_ms = 1000 / Hz; pulse_ms = period_ms * duty/100
    pulse_ms = duty * 10.0 / frequency

    # 높아지는 Rep Rate에서 기존 Pulse Width 때문에 Duty > 100%가 되는 문제 방지.
    # 반대로 Rep Rate가 낮아질 때는 주파수를 먼저 줄여 PW 증가 시 과도한 Duty를 피함.
    current_frequency = response_number(udp_command("RPRR"), "RPRR")
    pw_command = f"SPW {pulse_ms:.6f}"
    freq_command = f"SPRR {frequency:g}"

    if frequency > current_frequency:
        command_order = (pw_command, freq_command)
    else:
        command_order = (freq_command, pw_command)

    acknowledgements = {}
    for command in command_order:
        ack = udp_command(command)
        if not ack.get("ok"):
            return ack
        acknowledgements[command.split()[0]] = ack

    freq_ack = acknowledgements["SPRR"]
    pw_ack = acknowledgements["SPW"]

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
        "tx_ascii": "  ->  ".join(command + r"\r" for command in command_order),
        "tx_hex": "(two UDP commands)",
        "rx_ascii": " | ".join(acknowledgements[command.split()[0]].get("rx_ascii", "") for command in command_order),
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
# Dual non-contact IR serial workers - robust automatic COM discovery
# ============================================================
def modbus_crc16(data: bytes) -> int:
    """CRC-16/MODBUS. Wire order is low-byte first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_ir_request(slave_id: int = IR_SLAVE_ID) -> bytes:
    """Build the IR request. ID=1 reproduces the previously verified packet exactly."""
    body = bytes([
        slave_id & 0xFF,
        0x03,
        (IR_REGISTER_ADDRESS >> 8) & 0xFF,
        IR_REGISTER_ADDRESS & 0xFF,
        (IR_REGISTER_COUNT >> 8) & 0xFF,
        IR_REGISTER_COUNT & 0xFF,
    ])
    crc = modbus_crc16(body)
    return body + crc.to_bytes(2, byteorder="little")


# Startup self-check: this MUST equal the packet that worked in the original fixed-COM code.
assert build_ir_request(IR_SLAVE_ID) == IR_KNOWN_REQUEST


def _find_valid_ir_frame(data: bytes, expected_slave_id=IR_SLAVE_ID):
    """
    Find a valid 9-byte Modbus frame inside a possibly noisy serial read.
    Returns (frame, crc_ok) or (None, False).

    Primary acceptance requires CRC. A header-valid 9-byte frame is retained as a
    compatibility fallback because the original working code only checked ID/function.
    """
    if not data:
        return None, False

    header_candidate = None
    for i in range(max(0, len(data) - IR_RESPONSE_SIZE + 1)):
        frame = data[i:i + IR_RESPONSE_SIZE]
        if len(frame) != IR_RESPONSE_SIZE:
            continue
        if frame[0] != expected_slave_id or frame[1] != 0x03 or frame[2] != 0x04:
            continue
        if header_candidate is None:
            header_candidate = frame
        received_crc = frame[-2] | (frame[-1] << 8)
        if received_crc == modbus_crc16(frame[:-2]):
            return frame, True

    # Compatibility fallback for the exact 9-byte response shape used by the old code.
    if header_candidate is not None:
        return header_candidate, False
    return None, False


def valid_ir_response(response: bytes, expected_slave_id=IR_SLAVE_ID) -> bool:
    frame, _ = _find_valid_ir_frame(response, expected_slave_id)
    return frame is not None


def _read_serial_window(ser, timeout_s):
    """Collect bytes until a full response arrives or timeout expires."""
    deadline = time.monotonic() + max(0.05, float(timeout_s))
    buf = bytearray()
    while time.monotonic() < deadline:
        try:
            waiting = int(getattr(ser, "in_waiting", 0) or 0)
        except Exception:
            waiting = 0
        chunk = ser.read(waiting if waiting > 0 else 1)
        if chunk:
            buf.extend(chunk)
            frame, crc_ok = _find_valid_ir_frame(bytes(buf), IR_SLAVE_ID)
            if frame is not None:
                return frame, crc_ok, bytes(buf)
        else:
            time.sleep(0.005)
    frame, crc_ok = _find_valid_ir_frame(bytes(buf), IR_SLAVE_ID)
    return frame, crc_ok, bytes(buf)


def _safe_text(value):
    return (value or "").strip()


def _vid_pid(info):
    vid = f"{info.vid:04X}" if info.vid is not None else "----"
    pid = f"{info.pid:04X}" if info.pid is not None else "----"
    return vid, pid


def _serial_from_hwid(hwid: str):
    match = re.search(r"(?:^|\s)SER=([^\s]+)", hwid or "", flags=re.IGNORECASE)
    return match.group(1).strip() if match else ""


def collect_port_identity(info):
    """Collect Windows/pyserial identity fields. COM number itself is not identity."""
    vid, pid = _vid_pid(info)
    hwid = _safe_text(info.hwid)
    serial_no = _safe_text(info.serial_number) or _serial_from_hwid(hwid)
    location = _safe_text(getattr(info, "location", None))
    return {
        "device": info.device,
        "vid": vid,
        "pid": pid,
        "serial_number": serial_no,
        "location": location,
        "hwid": hwid,
        "description": _safe_text(info.description),
        "manufacturer": _safe_text(getattr(info, "manufacturer", None)),
        "product": _safe_text(getattr(info, "product", None)),
        "interface": _safe_text(getattr(info, "interface", None)),
    }


def _normalized_hwid(meta):
    hwid = meta.get("hwid", "")
    # Remove only volatile LOCATION token; preserve serial/instance data when present.
    return re.sub(r"\s+LOCATION=[^\s]+", "", hwid, flags=re.IGNORECASE).strip()


def finalize_sensor_identities(found):
    """
    Strong identity priority:
      1) unique USB serial number
      2) unique USB physical location
      3) unique PNP/HWID
      4) current COM as fallback only

    Modbus slave ID is deliberately NOT used as a device identity: both current
    sensors answer slave ID 1, so using ID=1 would randomly swap Wafer/Outside.
    """
    from collections import Counter

    serial_counts = Counter(
        (s["vid"], s["pid"], s["serial_number"])
        for s in found if s.get("serial_number")
    )
    location_counts = Counter(
        (s["vid"], s["pid"], s["location"])
        for s in found if s.get("location")
    )
    hwid_counts = Counter(_normalized_hwid(s) for s in found if _normalized_hwid(s))

    for sensor in found:
        serial_no = sensor.get("serial_number", "")
        location = sensor.get("location", "")
        normalized_hwid = _normalized_hwid(sensor)
        vid, pid = sensor.get("vid", "----"), sensor.get("pid", "----")

        if serial_no and serial_counts[(vid, pid, serial_no)] == 1:
            sensor["key"] = f"SER:{vid}:{pid}:{serial_no}"
            sensor["identity_source"] = "USB serial"
            sensor["identity"] = f"USB serial={serial_no}"
            sensor["stable"] = True
        elif location and location_counts[(vid, pid, location)] == 1:
            sensor["key"] = f"LOC:{vid}:{pid}:{location}"
            sensor["identity_source"] = "USB location"
            sensor["identity"] = f"USB location={location}"
            sensor["stable"] = True
        elif normalized_hwid and hwid_counts[normalized_hwid] == 1:
            sensor["key"] = f"HWID:{normalized_hwid}"
            sensor["identity_source"] = "PNP/HWID"
            sensor["identity"] = f"HWID={normalized_hwid}"
            sensor["stable"] = True
        else:
            sensor["key"] = f"DEV:{sensor['device']}"
            sensor["identity_source"] = "COM fallback"
            sensor["identity"] = f"COM fallback={sensor['device']}"
            sensor["stable"] = False
        sensor["slave_id"] = IR_SLAVE_ID
    return found


def load_ir_mapping():
    """Load saved role mapping, dropping old ambiguous Modbus-ID-only mappings."""
    data = {"__role_mapping_version__": IR_ROLE_MAPPING_VERSION}
    try:
        if IR_MAPPING_FILE.exists():
            loaded = json.loads(IR_MAPPING_FILE.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                for role in IR_SENSOR_NAMES:
                    rec = loaded.get(role)
                    if isinstance(rec, dict):
                        # Keep only mappings with a real distinguishing field.
                        if rec.get("serial_number") or rec.get("location") or rec.get("hwid"):
                            data[role] = rec
                data["__role_mapping_version__"] = IR_ROLE_MAPPING_VERSION
                save_ir_mapping(data)
    except Exception:
        pass
    return data


def save_ir_mapping(mapping):
    try:
        IR_MAPPING_FILE.parent.mkdir(parents=True, exist_ok=True)
        IR_MAPPING_FILE.write_text(
            json.dumps(mapping, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except Exception:
        pass


def sensor_mapping_record(sensor):
    return {
        "key": sensor.get("key"),
        "vid": sensor.get("vid"),
        "pid": sensor.get("pid"),
        "serial_number": sensor.get("serial_number"),
        "location": sensor.get("location"),
        "hwid": sensor.get("hwid"),
        "identity_source": sensor.get("identity_source"),
    }


def mapping_matches(sensor, saved) -> int:
    if not isinstance(saved, dict):
        return 0

    s_serial = sensor.get("serial_number")
    m_serial = saved.get("serial_number")
    if s_serial and m_serial and s_serial == m_serial:
        if sensor.get("vid") == saved.get("vid") and sensor.get("pid") == saved.get("pid"):
            return 1000
        return 950

    s_loc = sensor.get("location")
    m_loc = saved.get("location")
    if s_loc and m_loc and s_loc == m_loc:
        if sensor.get("vid") == saved.get("vid") and sensor.get("pid") == saved.get("pid"):
            return 800
        return 750

    s_hwid = _normalized_hwid(sensor)
    m_hwid = re.sub(r"\s+LOCATION=[^\s]+", "", saved.get("hwid", ""), flags=re.IGNORECASE).strip()
    if s_hwid and m_hwid and s_hwid == m_hwid:
        return 650

    if sensor.get("key") and sensor.get("key") == saved.get("key"):
        return 600
    return 0


def _open_ir_serial(device, timeout):
    """Open a serial port with explicit 19200-8-N-1 settings."""
    return serial.Serial(
        port=device,
        baudrate=IR_BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=min(0.05, timeout),
        write_timeout=timeout,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )


def probe_ir_port(info, detail=False):
    """
    Probe one CURRENT COM port with the exact packet that previously worked.
    Returns (sensor_meta, diagnostic_text).
    """
    if serial is None:
        return None, "pyserial unavailable"

    meta = collect_port_identity(info)
    ser = None
    last_raw = b""
    try:
        ser = _open_ir_serial(info.device, IR_PROBE_TIMEOUT)
        time.sleep(IR_OPEN_SETTLE)

        for attempt in range(1, IR_PROBE_ATTEMPTS + 1):
            try:
                ser.reset_input_buffer()
                # Do not reset_output_buffer here; some USB bridges behave poorly when
                # repeatedly purged immediately after opening.
                ser.write(IR_KNOWN_REQUEST)
                ser.flush()
                frame, crc_ok, raw = _read_serial_window(ser, IR_PROBE_TIMEOUT)
                last_raw = raw
                if frame is None:
                    time.sleep(0.03)
                    continue

                raw_temp = int.from_bytes(frame[3:5], byteorder="big", signed=True)
                meta.update({
                    "slave_id": IR_SLAVE_ID,
                    "temperature": raw_temp / 10.0,
                    "probe_packet": IR_KNOWN_REQUEST.hex(" ").upper(),
                    "crc_ok": crc_ok,
                })
                return meta, (
                    f"OK attempt={attempt}, temp={raw_temp/10.0:.1f}C, "
                    f"CRC={'OK' if crc_ok else 'header-only fallback'}"
                )
            except Exception as exc:
                last_error = str(exc)
                time.sleep(0.03)
        raw_text = last_raw.hex(" ").upper() if last_raw else "<no bytes>"
        return None, f"no valid response; RX={raw_text}"
    except Exception as exc:
        return None, f"open failed: {exc}"
    finally:
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass


def discover_ir_sensors(exclude_ports=None, diagnostics=False):
    """Scan CURRENT Windows COM list and probe each port with the known IR frame."""
    if serial is None or list_ports is None:
        return [], ["pyserial unavailable"]

    excluded = {str(x).upper() for x in (exclude_ports or ())}
    found = []
    diag = []
    infos = list(list_ports.comports())
    if not infos:
        return [], ["Windows reports no COM ports"]

    for info in infos:
        dev = str(info.device)
        if dev.upper() in excluded:
            continue
        sensor, message = probe_ir_port(info, detail=diagnostics)
        meta = collect_port_identity(info)
        desc = meta.get("description") or "serial device"
        diag.append(f"{dev} [{desc}] -> {message}")
        if sensor is not None:
            found.append(sensor)

    finalize_sensor_identities(found)

    def com_number(sensor):
        m = re.search(r"(\d+)$", sensor.get("device", ""), re.IGNORECASE)
        return int(m.group(1)) if m else 99999

    found.sort(key=com_number)
    return found, diag


def choose_ir_assignments(candidates, missing_channels, mapping):
    available = list(candidates)
    assignments = {}
    reasons = {}

    # 1) previously learned strong USB identity
    for channel in list(missing_channels):
        role = IR_SENSOR_NAMES[channel]
        saved = mapping.get(role)
        if not saved:
            continue
        scored = [(mapping_matches(sensor, saved), sensor) for sensor in available]
        scored = [(score, sensor) for score, sensor in scored if score > 0]
        if scored:
            score, sensor = max(scored, key=lambda item: item[0])
            assignments[channel] = sensor
            reasons[channel] = f"saved USB identity score={score}"
            available.remove(sensor)

    # 2) first bootstrap only: current known physical roles (COM8 Wafer, COM5 Outside).
    for channel in missing_channels:
        if channel in assignments:
            continue
        role = IR_SENSOR_NAMES[channel]
        hint = IR_BOOTSTRAP_ROLE_HINTS.get(role)
        match = next((s for s in available if s["device"].upper() == hint.upper()), None) if hint else None
        if match is not None:
            assignments[channel] = match
            reasons[channel] = f"first bootstrap hint {hint}"
            available.remove(match)

    # 3) if only one candidate remains, assign it to the only missing role.
    unresolved = [c for c in missing_channels if c not in assignments]
    if len(unresolved) == 1 and len(available) == 1:
        channel = unresolved[0]
        assignments[channel] = available.pop(0)
        reasons[channel] = "remaining detected sensor"

    # 4) deterministic fallback. User can press Swap once; the strong identities are then saved.
    for channel in missing_channels:
        if channel in assignments or not available:
            continue
        assignments[channel] = available.pop(0)
        reasons[channel] = "COM-order fallback; use Swap once if roles are reversed"

    return assignments, reasons


class IRSerialWorker:
    def __init__(self, channel, port, slave_id, sensor_name, temp_callback, log_callback):
        self.channel = channel
        self.port = port
        self.slave_id = int(slave_id)
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
            self.ser = _open_ir_serial(self.port, 0.12)
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 성공"
            )
        except Exception as exc:
            self.log_callback(
                f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 연결 실패: {exc}"
            )
            self._close_serial()
            return

        bad_count = 0
        try:
            while not self.stop_event.is_set():
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(IR_KNOWN_REQUEST)
                    self.ser.flush()
                    frame, crc_ok, raw = _read_serial_window(self.ser, 0.12)
                    if frame is not None:
                        bad_count = 0
                        raw_temp = int.from_bytes(frame[3:5], byteorder="big", signed=True)
                        self.temp_callback(self.channel, raw_temp / 10.0)
                    else:
                        bad_count += 1
                        if bad_count >= 10:
                            raise serial.SerialException("연속 응답 실패 - COM 재검색")
                except (serial.SerialException, OSError) as exc:
                    self.log_callback(
                        f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 통신 오류: {exc}"
                    )
                    break
                except Exception as exc:
                    bad_count += 1
                    if bad_count >= 10:
                        self.log_callback(
                            f"[IR Sensor {self.channel + 1} - {self.sensor_name} | {self.port}] 반복 오류: {exc}"
                        )
                        break
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
    """Discover current COM ports, learn USB identity, and reconnect automatically."""
    def __init__(self, temp_callback, log_callback, assignment_callback):
        self.temp_callback = temp_callback
        self.log_callback = log_callback
        self.assignment_callback = assignment_callback
        self.stop_event = threading.Event()
        self.thread = None
        self.workers = [None, None]
        self.sensor_meta = [None, None]
        self.mapping = load_ir_mapping()
        self._lock = threading.RLock()
        self.rescan_event = threading.Event()
        self._scan_no = 0
        self._last_diag_signature = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="IR-AutoDiscovery", daemon=True)
        self.thread.start()

    def _stop_channel(self, channel):
        worker = self.workers[channel]
        if worker is not None:
            try:
                worker.stop()
            except Exception:
                pass
        self.workers[channel] = None
        self.sensor_meta[channel] = None
        IR_PORTS[channel] = None
        self.assignment_callback(channel, None, None)

    def _cleanup_dead_workers(self):
        for channel in range(2):
            worker = self.workers[channel]
            if worker is not None and not worker.is_alive():
                self._stop_channel(channel)

    def request_rescan(self):
        with self._lock:
            for channel in range(2):
                self._stop_channel(channel)
        self._last_diag_signature = None
        self.rescan_event.set()
        self.log_callback("[IR] 강제 재검색 요청")

    def swap_roles(self):
        """Swap saved identities AND live channels, then reconnect."""
        with self._lock:
            a = self.mapping.get(IR_SENSOR_NAMES[0])
            b = self.mapping.get(IR_SENSOR_NAMES[1])
            self.mapping[IR_SENSOR_NAMES[0]] = b
            self.mapping[IR_SENSOR_NAMES[1]] = a
            save_ir_mapping(self.mapping)
            for channel in range(2):
                self._stop_channel(channel)
        self.rescan_event.set()
        self.log_callback("[IR] Wafer / Outside 역할을 교환하고 identity 매핑을 저장했습니다.")

    def clear_mapping(self):
        with self._lock:
            self.mapping = {"__role_mapping_version__": IR_ROLE_MAPPING_VERSION}
            save_ir_mapping(self.mapping)
            for channel in range(2):
                self._stop_channel(channel)
        self._last_diag_signature = None
        self.rescan_event.set()
        self.log_callback("[IR] 저장된 identity 매핑 초기화 -> 현재 COM 전체 재검색")

    def _log_diagnostics_if_changed(self, diag, found):
        signature = (tuple(diag), tuple((s.get("device"), s.get("identity")) for s in found))
        if signature == self._last_diag_signature:
            return
        self._last_diag_signature = signature
        for line in diag:
            self.log_callback(f"[IR scan] {line}")
        if not found:
            self.log_callback(
                "[IR scan] 센서 응답을 찾지 못했습니다. 다른 프로그램이 COM 포트를 사용 중인지도 확인하세요."
            )

    def _run(self):
        if serial is None or list_ports is None:
            self.log_callback("IR 자동 검색 불가: pyserial 설치 필요 (pip install pyserial)")
            return

        self.log_callback(
            "IR 자동 검색 시작: 현재 Windows COM 목록 -> 실제 센서 요청 01 03 9C 42 00 02 4A 4F -> 응답 확인"
        )
        self.log_callback(
            "[IR 역할] Sensor 1 = Wafer / Sensor 2 = Outside Wafer; COM 번호가 바뀌면 USB identity로 다시 매칭"
        )

        while not self.stop_event.is_set():
            with self._lock:
                self._cleanup_dead_workers()
                missing = [i for i, worker in enumerate(self.workers) if worker is None]
                active_ports = {
                    worker.port
                    for worker in self.workers
                    if worker is not None and worker.is_alive()
                }

            if missing:
                self._scan_no += 1
                ir_excluded_ports = set(active_ports)
                tc_port_now = globals().get("tc_active_port") or TC_DEFAULT_PORT
                if tc_port_now:
                    ir_excluded_ports.add(tc_port_now)
                candidates, diag = discover_ir_sensors(
                    exclude_ports=ir_excluded_ports,
                    diagnostics=True,
                )
                self._log_diagnostics_if_changed(diag, candidates)
                assignments, reasons = choose_ir_assignments(candidates, missing, self.mapping)

                with self._lock:
                    for channel in missing:
                        sensor = assignments.get(channel)
                        if sensor is None or self.workers[channel] is not None:
                            continue

                        role = IR_SENSOR_NAMES[channel]
                        self.mapping[role] = sensor_mapping_record(sensor)
                        save_ir_mapping(self.mapping)
                        IR_PORTS[channel] = sensor["device"]
                        self.sensor_meta[channel] = sensor
                        self.assignment_callback(channel, sensor["device"], sensor)

                        reason = reasons.get(channel, "auto")
                        self.log_callback(
                            f"[IR 자동매칭] {role} -> {sensor['device']} / {sensor['identity']} / "
                            f"{reason} / probe={sensor['temperature']:.1f} °C"
                        )
                        if not sensor.get("stable"):
                            self.log_callback(
                                f"[IR 주의] {role}: USB serial/location/HWID가 고유하지 않습니다. "
                                "역할이 반대면 Swap을 한 번 누르면 현재 매핑을 저장합니다."
                            )

                        worker = IRSerialWorker(
                            channel=channel,
                            port=sensor["device"],
                            slave_id=IR_SLAVE_ID,
                            sensor_name=role,
                            temp_callback=self.temp_callback,
                            log_callback=self.log_callback,
                        )
                        self.workers[channel] = worker
                        worker.start()

            self.rescan_event.wait(IR_SCAN_INTERVAL)
            self.rescan_event.clear()

    def stop(self):
        self.stop_event.set()
        self.rescan_event.set()
        with self._lock:
            for worker in self.workers:
                if worker is not None:
                    worker.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)


# ============================================================
# 20CH thermocouple serial acquisition
# ============================================================
class TC20CHWorker:
    """
    20-channel thermocouple receiver translated from the supplied v8.2 engine.

    Device protocol:
      - serial: 115200 baud
      - send b's' after opening to start streaming
      - each data line: tick_us, CH1, ... CH20
      - "OPEN", empty fields, and 0.0 are treated as NaN
      - send b'q' before closing
    """

    def __init__(self, port, data_callback, log_callback, state_callback=None):
        self.port = port
        self.data_callback = data_callback
        self.log_callback = log_callback
        self.state_callback = state_callback
        self.stop_event = threading.Event()
        self.thread = None
        self.ser = None
        self.start_tick = None

    def is_alive(self):
        return bool(self.thread and self.thread.is_alive())

    def start(self):
        if self.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            name=f"TC20CH-{self.port}",
            daemon=True,
        )
        self.thread.start()

    def _set_state(self, state):
        if self.state_callback:
            try:
                self.state_callback(state)
            except Exception:
                pass

    def _open(self):
        if serial is None:
            raise RuntimeError("pyserial이 없습니다: pip install pyserial")

        self.log_callback(f"[TC 20CH] 포트 [{self.port}] 오픈 시도...")
        self.ser = serial.Serial(
            self.port,
            TC_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TC_SERIAL_TIMEOUT,
            write_timeout=TC_WRITE_TIMEOUT,
        )
        time.sleep(0.03)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(b"s")
        self.ser.flush()
        self.start_tick = None
        self.log_callback(f"[TC 20CH] 포트 [{self.port}] 연결 성공 / 측정 시작")
        self._set_state(f"RUNNING / {self.port}")

    def _run(self):
        line_buffer = ""
        try:
            self._set_state(f"CONNECTING / {self.port}")
            self._open()

            while not self.stop_event.is_set():
                if self.ser is None or not self.ser.is_open:
                    break

                try:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        raw = self.ser.read(waiting).decode("utf-8", errors="ignore")
                        line_buffer += raw

                        while "\n" in line_buffer:
                            line, line_buffer = line_buffer.split("\n", 1)
                            line = line.strip()
                            if not line or "Measurement" in line:
                                continue

                            parts = line.split(",")
                            if len(parts) < TC_CHANNEL_COUNT + 1:
                                continue

                            try:
                                tick = int(parts[0].strip())
                            except Exception:
                                continue

                            if self.start_tick is None:
                                self.start_tick = tick

                            rel = tick - self.start_tick
                            if rel < 0:
                                rel += 2**32
                            ts = rel / 1_000_000.0

                            temps = []
                            for i in range(1, TC_CHANNEL_COUNT + 1):
                                value_text = parts[i].strip()
                                if not value_text or value_text.upper() == "OPEN":
                                    temps.append(math.nan)
                                    continue
                                try:
                                    value = float(value_text)
                                except Exception:
                                    temps.append(math.nan)
                                    continue
                                temps.append(math.nan if value == 0.0 else value)

                            try:
                                self.data_callback(temps, ts)
                            except Exception:
                                pass

                except (serial.SerialException, OSError) as exc:
                    self.log_callback(f"[TC 20CH] 통신 오류 [{self.port}]: {exc}")
                    self._set_state("DISCONNECTED")
                    break
                except Exception as exc:
                    self.log_callback(f"[TC 20CH] 파싱 오류: {exc}")

                self.stop_event.wait(0.001)

        except Exception as exc:
            self.log_callback(f"[TC 20CH] 포트 [{self.port}] 연결 실패: {exc}")
            self._set_state("ERROR")
        finally:
            self._close()

    def _close(self):
        try:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(b"q")
                    self.ser.flush()
                except Exception:
                    pass
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        if self.stop_event.is_set():
            self._set_state("STOPPED")

    def stop(self):
        self.stop_event.set()
        try:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(b"q")
                    self.ser.flush()
                except Exception:
                    pass
        except Exception:
            pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self._close()


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
ir_identity_vars = []
ir_manager_ref = None

# TC 20CH state
tc_port_var = None
tc_status_var = None
tc_port_combo = None
tc_current_vars = []
tc_worker_ref = None
tc_active_port = None
tc_latest_values = [math.nan] * TC_CHANNEL_COUNT
tc_latest_device_time = None
tc_start_monotonic = time.monotonic()
tc_history = [deque(maxlen=TC_HISTORY_MAXLEN) for _ in range(TC_CHANNEL_COUNT)]
tc_plot_axis = None
tc_plot_lines = []
tc_plot_canvas = None
tc_map_axis = None
tc_map_canvas = None
tc_map_image = None
tc_map_annotations = {}
tc_map_cbar = None
tc_map_unused_var = None
tc_heatmap_sensor_ids = []
tc_heatmap_weights = None
tc_heatmap_mask = None
tc_heatmap_extent = (-1.05, 1.05, -1.05, 1.05)
tc_sample_queue = deque(maxlen=6000)
tc_sample_lock = threading.Lock()
tc_tab_ref = None
ir_tab_ref = None

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
# Export-only recording (independent of the existing plot buffers)
# ============================================================
export_temp_dir = None
export_journals = {}
export_last_values = {}
export_recording_error = None
export_busy = False
export_wall_start = time.time()
export_monotonic_start = time.monotonic()

EXPORT_HEADERS = {
    "values": ("timestamp", "section", "key", "value"),
    "ir": ("timestamp", "elapsed_s", "sensor", "role", "port", "identity", "temperature_C"),
    "tc": ("timestamp", "session_started_at", "elapsed_s", "device_time", "port")
          + tuple(f"CH{i + 1:02d}_C" for i in range(TC_CHANNEL_COUNT)),
}
EXPORT_NOTES = {
    "timestamps": "Local ISO 8601 time including UTC offset; elapsed_s uses the original acquisition clock.",
    "history": "CSV contains samples/value changes recorded since this program started, including earlier TC sessions. Temporary records are removed when the program exits; use Save before closing.",
    "graphs": "PNG trend images show the recent plot window; CSV retains the complete recorded history. Heatmap is the current interpolated estimate, not additional sensor measurements.",
    "missing": "Blank TC numeric cells mean OPEN/missing/non-finite. A CSV with only a header means no samples were received.",
    "power": "Current is the RCS setpoint (%); optical power is calculated using 3000 W rated maximum, not measured optical power.",
    "elapsed": "Laser elapsed time is host time since EMON acknowledgement, not measured optical ON time.",
}


def _export_timestamp(monotonic_time=None):
    if monotonic_time is None:
        return datetime.now().astimezone().isoformat(timespec="milliseconds")
    epoch = export_wall_start + monotonic_time - export_monotonic_start
    return datetime.fromtimestamp(epoch).astimezone().isoformat(timespec="milliseconds")


def _export_number(value):
    if value is None:
        return ""
    try:
        return float(value) if math.isfinite(float(value)) else ""
    except (TypeError, ValueError):
        return str(value)


def _append_export_record(name, row):
    global export_recording_error
    if export_recording_error is not None:
        return
    try:
        journal = export_journals.get(name)
        if journal is not None:
            journal["writer"].writerow(row)
    except Exception as exc:
        # A disk failure must not interrupt laser control or sensor acquisition.
        export_recording_error = str(exc)
        log(f"SAVE recording error: {exc}", level="ERROR")


def _record_export_value(section, key, var):
    value = var.get()
    identity = (section, str(key))
    if export_last_values.get(identity) == value:
        return
    export_last_values[identity] = value
    _append_export_record("values", (_export_timestamp(), section, key, value))


def initialize_export_recording():
    global export_temp_dir, export_recording_error
    try:
        export_temp_dir = tempfile.TemporaryDirectory(prefix="raycus_export_")
        for name, header in EXPORT_HEADERS.items():
            path = Path(export_temp_dir.name) / f"{name}.csv"
            handle = path.open("w", encoding="utf-8-sig", newline="")
            writer = csv.writer(handle)
            writer.writerow(header)
            export_journals[name] = {"path": path, "handle": handle, "writer": writer}
        connection_fields = {
            "state": connection_var, "ip": ip_var, "port": port_var,
            "transport": transport_var, "reads_ms": reads_ms_var,
            "timeout_ms": timeout_ms_var, "local_address": local_port_var,
            "polling": polling_info_var, "sta_rtt": sta_rtt_var,
        }
        for section, variables in (
            ("readback", readback_vars), ("status", status_vars),
            ("sta_bits", sta_bit_vars), ("connection", connection_fields),
            ("tc_state", {"state": tc_status_var, "selected_port": tc_port_var}),
        ):
            for key, var in variables.items():
                _record_export_value(section, key, var)
                var.trace_add("write", lambda *_, s=section, k=key, v=var: _record_export_value(s, k, v))
    except Exception as exc:
        export_recording_error = str(exc)
        log(f"SAVE recording unavailable: {exc}", level="ERROR")


def close_export_recording():
    for journal in export_journals.values():
        try:
            journal["handle"].close()
        except Exception:
            pass
    if export_temp_dir is not None and not export_busy:
        export_temp_dir.cleanup()


def _capture_export_snapshot():
    """Read Tk and live buffers only on the GUI thread; saving uses immutable copies."""
    if export_recording_error is not None:
        raise RuntimeError(f"저장용 기록 오류: {export_recording_error}")
    boundaries = {}
    for name, journal in export_journals.items():
        journal["handle"].flush()
        boundaries[name] = (str(journal["path"]), journal["path"].stat().st_size)
    now = time.monotonic()
    snapshot = {
        "saved_at": _export_timestamp(),
        "notes": dict(EXPORT_NOTES),
        "connection": {
            "state": connection_var.get(), "ip": ip_var.get(), "port": port_var.get(),
            "transport": transport_var.get(), "reads_ms": reads_ms_var.get(),
            "timeout_ms": timeout_ms_var.get(), "local_address": local_port_var.get(),
            "polling": polling_info_var.get(), "sta_rtt": sta_rtt_var.get(),
        },
        "parameter_inputs": {"power_percent": power_set_var.get(), "frequency_Hz": freq_set_var.get(), "duty_percent": duty_set_var.get()},
        "readback": {k: v.get() for k, v in readback_vars.items()},
        "status": {k: v.get() for k, v in status_vars.items()},
        "sta": {
            "raw_word": latest_sta_value, "display_mode": sta_mode_var.get(),
            "bits": {f"b{i:02d}": {"value": bit(latest_sta_value, i) if latest_sta_value is not None else None, "label": STA_BIT_LABELS[i]} for i in range(32)},
        },
        "ir": [{"sensor": i + 1, "role": IR_SENSOR_NAMES[i], "port": ir_port_vars[i].get(), "identity": ir_identity_vars[i].get(), "current": ir_current_vars[i].get()} for i in range(2)],
        "tc": {
            "state": tc_status_var.get(), "selected_port": tc_port_var.get(),
            "active_port": tc_active_port, "device_time": _export_number(tc_latest_device_time),
            "channels": {f"CH{i + 1:02d}": {"current": tc_current_vars[i].get(), "temperature_C": _export_number(tc_latest_values[i])} for i in range(TC_CHANNEL_COUNT)},
            "sensor_layout": {f"S{s}": {"channel": TC_SENSOR_MAPPING[s] + 1, "x": xy[0], "y": xy[1]} for s, xy in TC_SENSOR_COORDS.items()},
            "heatmap_color_limits_C": list(tc_map_image.get_clim()) if tc_map_image is not None else [0.0, 500.0],
        },
    }
    plots = {
        "ir": [list(h) for h in ir_history], "tc": [list(h) for h in tc_history],
        "ir_elapsed": max(IR_PLOT_WINDOW_SECONDS, now - ir_start_time),
        "tc_elapsed": max(TC_PLOT_WINDOW_SECONDS, now - tc_start_monotonic),
        "heatmap": _compute_tc_heatmap_fast().copy(),
        "log": output.get("1.0", "end-1c") if output is not None else "",
    }
    return snapshot, boundaries, plots


def _write_export_csv(path, header, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def _copy_export_prefix(source, size, destination):
    # Copy exactly the save-time prefix; acquisition may keep appending meanwhile.
    with open(source, "rb") as src, destination.open("wb") as dst:
        remaining = size
        while remaining:
            chunk = src.read(min(1024 * 1024, remaining))
            if not chunk:
                raise IOError("저장용 기록 파일이 예상보다 짧습니다.")
            dst.write(chunk)
            remaining -= len(chunk)


def _save_export_trend(path, histories, labels, right, window, title):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(10, 6), dpi=150)
    FigureCanvasAgg(fig)
    axis = fig.add_subplot(111)
    for history, label in zip(histories, labels):
        points = [p for p in history if p[0] >= right - window]
        if len(points) > TC_RENDER_MAX_POINTS:
            points = points[::math.ceil(len(points) / TC_RENDER_MAX_POINTS)] + points[-1:]
        axis.plot([p[0] for p in points], [p[1] for p in points], linewidth=1.0, label=label)
    axis.set(title=title, xlabel="Elapsed Time (s)", ylabel="Temperature (°C)", xlim=(right - window, right))
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left", fontsize=7, ncol=5 if len(labels) > 2 else 1)
    fig.tight_layout()
    fig.savefig(path)


def _save_export_heatmap(folder, snapshot, grid):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.patches import Circle
    from matplotlib.patheffects import withStroke
    fig = Figure(figsize=(7, 7), dpi=150)
    FigureCanvasAgg(fig)
    axis = fig.add_subplot(111)
    axis.set(title="TC Wafer Heatmap / Sensor Positions", aspect="equal", xlim=(-1.08, 1.08), ylim=(-1.08, 1.08))
    axis.axis("off")
    axis.add_patch(Circle((0, 0), 1, fill=False, color="black"))
    axis.add_patch(Circle((0, 0), 0.60, fill=False, color="gray", linestyle="--", alpha=0.45))
    axis.add_patch(Circle((0, 0), 0.20, fill=False, color="gray", linestyle=":", alpha=0.45))
    low, high = snapshot["tc"]["heatmap_color_limits_C"]
    image = axis.imshow(grid, extent=tc_heatmap_extent, origin="lower", interpolation="bilinear", cmap="turbo", vmin=low, vmax=high)
    layout_rows = []
    for sensor, xy in TC_SENSOR_COORDS.items():
        channel = TC_SENSOR_MAPPING[sensor] + 1
        value = snapshot["tc"]["channels"][f"CH{channel:02d}"]["temperature_C"]
        text = "OPEN" if value == "" else f"{value:.1f}°C"
        axis.plot(*xy, "o", color="white", markeredgecolor="black", markersize=6)
        axis.text(xy[0], xy[1] + 0.06, f"S{sensor}\nCH{channel:02d}\n{text}", ha="center", va="bottom", fontsize=8, color="white", path_effects=[withStroke(linewidth=1.5, foreground="black")])
        layout_rows.append((sensor, channel, xy[0], xy[1], value))
    fig.colorbar(image, ax=axis, fraction=0.046, pad=0.03).set_label("Temperature (°C)")
    fig.savefig(folder / "tc_heatmap.png", bbox_inches="tight")
    _write_export_csv(folder / "tc_sensor_layout.csv", ("sensor", "channel", "x_normalized", "y_normalized", "temperature_C"), layout_rows)
    xs = np.linspace(tc_heatmap_extent[0], tc_heatmap_extent[1], grid.shape[1])
    ys = np.linspace(tc_heatmap_extent[2], tc_heatmap_extent[3], grid.shape[0])
    _write_export_csv(folder / "tc_heatmap_grid.csv", ("x_normalized", "y_normalized", "temperature_C"), ((x, y, _export_number(grid[j, i])) for j, y in enumerate(ys) for i, x in enumerate(xs)))


def _flatten_export_values(value, prefix=""):
    if isinstance(value, dict):
        for key, item in value.items():
            yield from _flatten_export_values(item, f"{prefix}.{key}" if prefix else str(key))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _flatten_export_values(item, f"{prefix}[{index}]")
    else:
        yield prefix, value


def _write_export_bundle(folder, scope, key, snapshot, boundaries, plots):
    selected = {"saved_at": snapshot["saved_at"], "notes": snapshot["notes"]}
    sections = {
        "all": ("connection", "parameter_inputs", "readback", "status", "sta", "ir", "tc"),
        "readback": ("connection", "parameter_inputs", "readback", "sta"),
        "status": ("status", "sta"), "sta": ("sta",),
        "ir": ("ir",), "tc": ("tc",), "heatmap": ("tc",), "log": ("connection",),
    }
    for section in sections[scope]:
        selected[section] = snapshot[section]
    if key is not None:
        if scope in ("readback", "status"):
            selected = {"saved_at": snapshot["saved_at"], "notes": snapshot["notes"], scope: {key: snapshot[scope][key]}}
        elif scope == "ir":
            selected["ir"] = [snapshot["ir"][key]]
        elif scope == "tc":
            selected["tc"] = dict(snapshot["tc"], channels={f"CH{key + 1:02d}": snapshot["tc"]["channels"][f"CH{key + 1:02d}"]})
        elif scope == "sta":
            selected["sta"] = dict(snapshot["sta"], bits={f"b{key:02d}": snapshot["sta"]["bits"][f"b{key:02d}"]})
    (folder / "snapshot.json").write_text(json.dumps(selected, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    _write_export_csv(folder / "current_values.csv", ("field", "value"), _flatten_export_values({k: v for k, v in selected.items() if k != "notes"}))

    journals = []
    if scope in ("all", "readback", "status", "sta"):
        journals.append("values")
    if scope in ("all", "ir"):
        journals.append("ir")
    if scope in ("all", "tc"):
        journals.append("tc")
    for name in journals:
        source, size = boundaries[name]
        path = folder / f"{name}_history.csv"
        _copy_export_prefix(source, size, path)
        if scope == "all" or (name in ("ir", "tc") and key is None):
            continue
        filtered = path.with_suffix(".filtered.csv")
        with path.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if name == "values":
                allowed = {"readback", "connection", "sta_bits"} if scope == "readback" else {"status", "sta_bits"} if scope == "status" else {"sta_bits"}
                rows = (tuple(row[h] for h in EXPORT_HEADERS[name]) for row in reader if ((row["section"] in allowed or (scope == "sta" and row["section"] == "readback" and row["key"] == "sta")) and (key is None or (row["section"] == ("sta_bits" if scope == "sta" else scope) and row["key"] == str(key)))))
                _write_export_csv(filtered, EXPORT_HEADERS[name], rows)
            elif name == "ir":
                _write_export_csv(filtered, EXPORT_HEADERS[name], (tuple(row[h] for h in EXPORT_HEADERS[name]) for row in reader if row["sensor"] == str(key + 1)))
            else:
                header = EXPORT_HEADERS[name][:5] + (f"CH{key + 1:02d}_C",)
                _write_export_csv(filtered, header, (tuple(row[h] for h in header) for row in reader))
        filtered.replace(path)

    if scope in ("all", "ir"):
        indices = range(2) if key is None else (key,)
        _save_export_trend(folder / "ir_trend.png", [plots["ir"][i] for i in indices], [f"IR {i + 1} - {IR_SENSOR_NAMES[i]}" for i in indices], plots["ir_elapsed"], IR_PLOT_WINDOW_SECONDS, "Wafer / Outside-Wafer IR Temperature Trend")
    if scope in ("all", "tc"):
        indices = range(TC_CHANNEL_COUNT) if key is None else (key,)
        _save_export_trend(folder / "tc_trend.png", [plots["tc"][i] for i in indices], [f"CH{i + 1:02d}" for i in indices], plots["tc_elapsed"], TC_PLOT_WINDOW_SECONDS, "Thermocouple Temperature Trend")
    if scope in ("all", "heatmap") or (scope == "tc" and key is None):
        _save_export_heatmap(folder, snapshot, plots["heatmap"])
    if scope in ("all", "log"):
        (folder / "command_error_log.txt").write_text(plots["log"], encoding="utf-8-sig")


def save_monitor_data(scope="all", key=None):
    global export_busy
    if export_busy:
        messagebox.showinfo("Save", "이전 저장 작업이 진행 중입니다.")
        return
    destination = filedialog.askdirectory(parent=root, title="저장할 폴더 선택")
    if not destination:
        return
    try:
        snapshot, boundaries, plots = _capture_export_snapshot()
        suffix = f"_b{key:02d}" if scope == "sta" and key is not None else f"_{key + 1:02d}" if isinstance(key, int) else f"_{key}" if key is not None else ""
        folder = Path(destination) / f"Raycus_{scope}{suffix}_{datetime.now():%Y%m%d_%H%M%S_%f}"
        folder.mkdir()
    except Exception as exc:
        messagebox.showerror("Save", str(exc))
        return
    export_busy = True
    log(f"SAVE started: {folder}")

    def finish(error):
        global export_busy
        export_busy = False
        if error is not None:
            log(f"SAVE ERROR: {error} | partial folder: {folder}", level="ERROR")
            messagebox.showerror("Save failed", f"저장 실패: {error}\n일부 파일만 생성되었을 수 있습니다.\n{folder}")
        else:
            log(f"SAVE completed: {folder}")
            messagebox.showinfo("Saved", f"저장 완료\n{folder}")

    def worker():
        error = None
        try:
            _write_export_bundle(folder, scope, key, snapshot, boundaries, plots)
        except Exception as exc:
            error = str(exc)
        try:
            root.after(0, lambda: finish(error))
        except (RuntimeError, tk.TclError):
            pass

    # Separate export figures and fixed journal boundaries keep acquisition running.
    threading.Thread(target=worker, daemon=False).start()


def add_export_button(parent, scope, key=None, text="Save", **grid_options):
    button = ttk.Button(parent, text=text, command=lambda: save_monitor_data(scope, key))
    if grid_options:
        button.grid(**grid_options)
    return button


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
    """Reflect current COM port + stable identity in GUI and plot legend."""
    if root is None:
        return

    def apply_assignment():
        display_port = port or "AUTO SEARCH"
        if channel < len(ir_port_vars):
            ir_port_vars[channel].set(display_port)
        if channel < len(ir_identity_vars):
            if sensor_meta:
                slave_id = sensor_meta.get("slave_id", "?")
                ident = sensor_meta.get("identity", "identity unknown")
                ir_identity_vars[channel].set(f"ID={slave_id} | {ident}")
            else:
                ir_identity_vars[channel].set("Searching...")
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


def swap_ir_roles_from_ui():
    if ir_manager_ref is not None:
        threading.Thread(target=ir_manager_ref.swap_roles, daemon=True).start()


def rescan_ir_sensors_from_ui():
    if ir_manager_ref is not None:
        threading.Thread(target=ir_manager_ref.request_rescan, daemon=True).start()


def clear_ir_mapping_from_ui():
    if ir_manager_ref is None:
        return
    if messagebox.askyesno(
        "IR mapping reset",
        "저장된 Wafer / Outside 센서 식별 매핑을 지우고 다시 탐색할까요?",
    ):
        threading.Thread(target=ir_manager_ref.clear_mapping, daemon=True).start()


def update_ir_temperature(channel, temperature):
    if root is None:
        return

    def apply_value():
        if channel < len(ir_current_vars):
            ir_current_vars[channel].set(f"{temperature:.1f} °C")
        elapsed = time.monotonic() - ir_start_time
        ir_history[channel].append((elapsed, temperature))
        _append_export_record("ir", (
            _export_timestamp(), elapsed, channel + 1, IR_SENSOR_NAMES[channel],
            ir_port_vars[channel].get(), ir_identity_vars[channel].get(), _export_number(temperature),
        ))

    root.after(0, apply_value)


def update_ir_plots():
    if root is None or ir_plot_axis is None or ir_plot_canvas is None:
        return

    if not _tab_is_visible(ir_tab_ref):
        root.after(IR_PLOT_REFRESH_MS, update_ir_plots)
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

    identity_box = ttk.LabelFrame(parent, text="IR Sensor Auto Identity", padding=6)
    identity_box.pack(fill="x", padx=6, pady=(6, 2))
    identity_box.columnconfigure(1, weight=1)

    for row, role in enumerate(IR_SENSOR_NAMES):
        ttk.Label(identity_box, text=role, style="ReadbackName.TLabel", width=15).grid(
            row=row, column=0, sticky="w", padx=(4, 6), pady=2
        )
        ttk.Label(identity_box, textvariable=ir_port_vars[row], style="ReadbackValue.TLabel", width=10).grid(
            row=row, column=1, sticky="w", padx=(0, 8), pady=2
        )
        ttk.Label(identity_box, textvariable=ir_current_vars[row], style="ReadbackValue.TLabel", width=12).grid(
            row=row, column=2, sticky="w", padx=(0, 8), pady=2
        )
        ttk.Label(identity_box, textvariable=ir_identity_vars[row], style="Unit.TLabel").grid(
            row=row, column=3, sticky="w", padx=(0, 8), pady=2
        )

    button_bar = ttk.Frame(identity_box)
    button_bar.grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=(5, 1))
    ttk.Button(button_bar, text="Rescan IR", command=rescan_ir_sensors_from_ui).pack(side="left", padx=(0, 5))
    ttk.Button(button_bar, text="Swap Wafer / Outside", command=swap_ir_roles_from_ui).pack(side="left", padx=5)
    ttk.Button(button_bar, text="Forget Mapping", command=clear_ir_mapping_from_ui).pack(side="left", padx=5)
    add_export_button(button_bar, "ir", text="Save IR").pack(side="left", padx=(16, 5))

    ttk.Label(
        identity_box,
        text=(
            "Auto identity: actual sensor response → USB serial → USB physical location → PNP/HWID. "
            "COM number may change and is used only as a first-learning/fallback hint."
        ),
        style="Note.TLabel",
    ).grid(row=3, column=0, columnspan=4, sticky="w", padx=4, pady=(3, 1))

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
# TC 20CH GUI logic
# ============================================================
def _list_serial_ports():
    if list_ports is None:
        return []
    try:
        return [info.device for info in list_ports.comports()]
    except Exception:
        return []


def refresh_tc_port_list():
    ports = _list_serial_ports()
    if tc_port_combo is not None:
        tc_port_combo.configure(values=ports)

    if tc_port_var is not None:
        current = tc_port_var.get().strip()
        if not current:
            if TC_DEFAULT_PORT in ports:
                tc_port_var.set(TC_DEFAULT_PORT)
            elif ports:
                tc_port_var.set(ports[0])


def update_tc_state(state):
    if root is not None:
        root.after(
            0,
            lambda: tc_status_var.set(state) if tc_status_var is not None else None,
        )


def update_tc_data(temps, device_ts):
    """Receive TC packets from worker thread without touching Tk on every packet.

    Raw samples are queued and the Tk thread drains them in batches. This preserves
    the measurement history while preventing hundreds of Tk callbacks per second.
    """
    if len(temps) != TC_CHANNEL_COUNT:
        return
    sample = (time.monotonic(), list(temps), device_ts)
    with tc_sample_lock:
        tc_sample_queue.append(sample)


def process_tc_sample_queue():
    """Batch pending TC packets and update Tk labels only once per GUI tick."""
    global tc_latest_values, tc_latest_device_time

    if root is None:
        return

    pending = []
    with tc_sample_lock:
        while tc_sample_queue:
            pending.append(tc_sample_queue.popleft())

    if pending:
        for sample_time, temps, device_ts in pending:
            elapsed = max(0.0, sample_time - tc_start_monotonic)
            _append_export_record("tc", (
                _export_timestamp(sample_time), _export_timestamp(tc_start_monotonic),
                elapsed, _export_number(device_ts), tc_active_port or tc_port_var.get(),
                *(_export_number(value) for value in temps),
            ))
            for idx, value in enumerate(temps):
                if value is not None and not math.isnan(value):
                    tc_history[idx].append((elapsed, value))

        _, latest_temps, latest_device_ts = pending[-1]
        tc_latest_values = list(latest_temps)
        tc_latest_device_time = latest_device_ts

        # Tk StringVars are intentionally updated once per batch, not per serial packet.
        for idx, value in enumerate(latest_temps):
            if idx >= len(tc_current_vars):
                continue
            if value is None or math.isnan(value):
                tc_current_vars[idx].set('OPEN')
            else:
                tc_current_vars[idx].set(f'{value:.1f} °C')

    root.after(TC_DATA_PROCESS_MS, process_tc_sample_queue)


def start_tc_from_ui():
    global tc_worker_ref, tc_active_port, tc_start_monotonic

    if serial is None:
        messagebox.showwarning("TC 20CH", "pyserial이 없습니다. pip install pyserial")
        return

    port = tc_port_var.get().strip() if tc_port_var is not None else TC_DEFAULT_PORT
    if not port:
        messagebox.showwarning("TC 20CH", "TC 포트를 선택하세요.")
        return

    if tc_worker_ref is not None and tc_worker_ref.is_alive():
        if tc_active_port == port:
            return
        tc_worker_ref.stop()

    tc_active_port = port
    tc_start_monotonic = time.monotonic()
    with tc_sample_lock:
        tc_sample_queue.clear()
    for history in tc_history:
        history.clear()
    for var in tc_current_vars:
        var.set("--")

    tc_worker_ref = TC20CHWorker(
        port=port,
        data_callback=update_tc_data,
        log_callback=log_from_worker,
        state_callback=update_tc_state,
    )
    tc_worker_ref.start()


def stop_tc_from_ui():
    global tc_worker_ref, tc_active_port
    worker = tc_worker_ref
    tc_worker_ref = None
    if worker is not None:
        worker.stop()
    tc_active_port = None
    if tc_status_var is not None:
        tc_status_var.set("STOPPED")


def _tab_is_visible(tab_widget):
    if tab_widget is None:
        return True
    try:
        return bool(tab_widget.winfo_ismapped())
    except Exception:
        return True


def update_tc_plot():
    if root is None or tc_plot_axis is None or tc_plot_canvas is None:
        return

    # Hidden Matplotlib tabs are not redrawn. Data acquisition/history continues.
    if not _tab_is_visible(tc_tab_ref):
        root.after(TC_PLOT_REFRESH_MS, update_tc_plot)
        return

    elapsed_now = time.monotonic() - tc_start_monotonic
    cutoff = max(0.0, elapsed_now - TC_PLOT_WINDOW_SECONDS)
    visible_min = None
    visible_max = None

    for idx in range(TC_CHANNEL_COUNT):
        history = tc_history[idx]
        while history and history[0][0] < cutoff:
            history.popleft()

        if idx >= len(tc_plot_lines):
            continue

        if not history:
            tc_plot_lines[idx].set_data([], [])
            continue

        # Rendering all raw points from 20 channels is expensive. Keep every sample
        # in history, but draw at most TC_RENDER_MAX_POINTS per channel.
        n = len(history)
        step = max(1, math.ceil(n / TC_RENDER_MAX_POINTS))
        samples = list(history)[::step]
        if samples[-1] != history[-1]:
            samples.append(history[-1])

        xs = [sample[0] for sample in samples]
        ys = [sample[1] for sample in samples]
        tc_plot_lines[idx].set_data(xs, ys)

        local_min = min(ys)
        local_max = max(ys)
        visible_min = local_min if visible_min is None else min(visible_min, local_min)
        visible_max = local_max if visible_max is None else max(visible_max, local_max)

    x_right = max(TC_PLOT_WINDOW_SECONDS, elapsed_now)
    tc_plot_axis.set_xlim(x_right - TC_PLOT_WINDOW_SECONDS, x_right)

    if visible_min is not None and visible_max is not None:
        span = visible_max - visible_min
        pad = max(2.0, span * 0.10)
        tc_plot_axis.set_ylim(visible_min - pad, visible_max + pad)

    tc_plot_canvas.draw_idle()
    root.after(TC_PLOT_REFRESH_MS, update_tc_plot)


def initialize_tc_heatmap_cache():
    """Precompute all geometry-dependent interpolation weights once."""
    global tc_heatmap_sensor_ids, tc_heatmap_weights, tc_heatmap_mask

    xs = np.linspace(tc_heatmap_extent[0], tc_heatmap_extent[1], TC_HEATMAP_GRID)
    ys = np.linspace(tc_heatmap_extent[2], tc_heatmap_extent[3], TC_HEATMAP_GRID)
    grid_x, grid_y = np.meshgrid(xs, ys)
    tc_heatmap_mask = (grid_x ** 2 + grid_y ** 2) > 1.0

    sensor_ids = []
    weights = []
    for sensor_id, (x, y) in TC_SENSOR_COORDS.items():
        ch_index = TC_SENSOR_MAPPING.get(sensor_id)
        if ch_index is None:
            continue
        distance2 = (grid_x - x) ** 2 + (grid_y - y) ** 2
        weight = 1.0 / np.maximum(distance2, 1e-6)
        weights.append(weight.astype(np.float32))
        sensor_ids.append(sensor_id)

    tc_heatmap_sensor_ids = sensor_ids
    tc_heatmap_weights = np.stack(weights, axis=0) if weights else None


def _compute_tc_heatmap_fast():
    if tc_heatmap_weights is None:
        initialize_tc_heatmap_cache()
    if tc_heatmap_weights is None:
        return np.full((TC_HEATMAP_GRID, TC_HEATMAP_GRID), np.nan, dtype=float)

    values = []
    valid = []
    for i, sensor_id in enumerate(tc_heatmap_sensor_ids):
        ch_index = TC_SENSOR_MAPPING.get(sensor_id)
        value = tc_latest_values[ch_index] if ch_index is not None else math.nan
        if value is not None and not math.isnan(value):
            valid.append(i)
            values.append(float(value))

    if not valid:
        return np.full((TC_HEATMAP_GRID, TC_HEATMAP_GRID), np.nan, dtype=float)

    w = tc_heatmap_weights[valid]
    vals = np.asarray(values, dtype=np.float32)[:, None, None]
    denom = np.sum(w, axis=0)
    grid_z = np.sum(w * vals, axis=0) / np.maximum(denom, 1e-12)
    grid_z = grid_z.astype(float, copy=False)
    grid_z[tc_heatmap_mask] = np.nan
    return grid_z


def update_tc_heatmap():
    if root is None or tc_map_image is None or tc_map_canvas is None:
        return

    if not _tab_is_visible(tc_tab_ref):
        root.after(TC_HEATMAP_REFRESH_MS, update_tc_heatmap)
        return

    grid_z = _compute_tc_heatmap_fast()
    tc_map_image.set_data(grid_z)

    # Fixed heatmap color scale: 0~500 °C.
    # No dynamic colorbar rescaling -> lighter GUI rendering.
    tc_map_image.set_clim(0.0, 400.0)

    for sensor_id in TC_SENSOR_COORDS:
        ch_index = TC_SENSOR_MAPPING.get(sensor_id)
        value = math.nan
        if ch_index is not None and ch_index < len(tc_latest_values):
            value = tc_latest_values[ch_index]
        txt = tc_map_annotations.get(sensor_id)
        if txt is None:
            continue
        ch_text = ch_index + 1 if ch_index is not None else 0
        if value is None or math.isnan(value):
            txt.set_text(f'S{sensor_id}\nCH{ch_text:02d}\nOPEN')
        else:
            txt.set_text(f'S{sensor_id}\nCH{ch_text:02d}\n{value:.1f}°C')

    mapped_channels = set(TC_SENSOR_MAPPING.values())
    pieces = []
    for idx in range(TC_CHANNEL_COUNT):
        if idx in mapped_channels:
            continue
        value = tc_latest_values[idx]
        if value is None or math.isnan(value):
            pieces.append(f'CH{idx + 1:02d}=OPEN')
        else:
            pieces.append(f'CH{idx + 1:02d}={value:.1f}°C')
    if tc_map_unused_var is not None:
        tc_map_unused_var.set('Unused / off-wafer: ' + (' | '.join(pieces) if pieces else '-'))

    tc_map_canvas.draw_idle()
    root.after(TC_HEATMAP_REFRESH_MS, update_tc_heatmap)


def build_tc_plot(parent):
    global tc_plot_axis, tc_plot_lines, tc_plot_canvas, tc_port_combo
    global tc_map_axis, tc_map_canvas, tc_map_image, tc_map_annotations, tc_map_cbar, tc_map_unused_var

    controls = ttk.LabelFrame(parent, text="20CH Thermocouple Acquisition", padding=8)
    controls.pack(fill="x", padx=6, pady=(6, 2))

    ttk.Label(controls, text="Port", style="ControlName.TLabel").pack(side="left")
    tc_port_combo = ttk.Combobox(
        controls,
        textvariable=tc_port_var,
        width=10,
        values=_list_serial_ports(),
    )
    tc_port_combo.pack(side="left", padx=(5, 8))

    ttk.Button(controls, text="Refresh Ports", command=refresh_tc_port_list).pack(
        side="left", padx=4
    )
    ttk.Button(controls, text="Start TC", command=start_tc_from_ui).pack(
        side="left", padx=4
    )
    ttk.Button(controls, text="Stop TC", command=stop_tc_from_ui).pack(
        side="left", padx=4
    )

    ttk.Label(
        controls,
        textvariable=tc_status_var,
        style="Connection.TLabel",
    ).pack(side="left", padx=(16, 6))

    ttk.Label(
        controls,
        text="115200 baud / start='s' / stop='q' / tick + CH1~CH20",
        style="Note.TLabel",
    ).pack(side="right", padx=4)

    save_bar = ttk.Frame(parent)
    save_bar.pack(fill="x", padx=12, pady=(2, 0))
    add_export_button(save_bar, "tc", text="Save TC").pack(side="left", padx=(0, 6))
    add_export_button(save_bar, "heatmap", text="Save Heatmap").pack(side="left", padx=6)

    values = ttk.LabelFrame(parent, text="Live TC Values", padding=6)
    values.pack(fill="x", padx=6, pady=4)

    for idx in range(TC_CHANNEL_COUNT):
        block = idx // 5
        row = idx % 5
        base_col = block * 2
        ttk.Label(
            values,
            text=f"TC CH{idx + 1:02d}",
            style="ReadbackName.TLabel",
            width=9,
        ).grid(row=row, column=base_col, sticky="w", padx=(6, 3), pady=2)
        ttk.Label(
            values,
            textvariable=tc_current_vars[idx],
            style="ReadbackValue.TLabel",
            width=11,
        ).grid(row=row, column=base_col + 1, sticky="w", padx=(0, 12), pady=2)

    ttk.Label(
        values,
        text="Wafer heatmap uses the supplied v8.2 TC sensor layout. Labels show sensor position (S#) and mapped TC channel.",
        style="Note.TLabel",
    ).grid(row=5, column=0, columnspan=8, sticky='w', padx=6, pady=(6, 0))

    plot_host = ttk.Panedwindow(parent, orient='horizontal')
    plot_host.pack(fill="both", expand=True, padx=6, pady=6)

    trend_host = ttk.Frame(plot_host, padding=2)
    map_host = ttk.Frame(plot_host, padding=2)
    plot_host.add(trend_host, weight=3)
    plot_host.add(map_host, weight=2)

    fig = Figure(figsize=(7.4, 5.0), dpi=100)
    axis = fig.add_subplot(111)
    lines = []
    for idx in range(TC_CHANNEL_COUNT):
        line, = axis.plot([], [], linewidth=1.0, label=f"CH{idx + 1}")
        lines.append(line)

    axis.set_title("20CH Thermocouple Temperature Trend")
    axis.set_xlabel("Elapsed Time (s)")
    axis.set_ylabel("Temperature (°C)")
    axis.grid(True, alpha=0.3)
    axis.set_xlim(0, TC_PLOT_WINDOW_SECONDS)
    axis.set_ylim(20, 150)
    axis.legend(loc="upper left", fontsize=7, ncol=5)
    fig.tight_layout()

    canvas = FigureCanvasTkAgg(fig, master=trend_host)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    map_fig = Figure(figsize=(5.2, 5.0), dpi=100)
    map_fig.subplots_adjust(top=0.86)
    map_axis = map_fig.add_subplot(111)
    map_axis.set_title('TC Wafer Heatmap / Sensor Positions', y=1.08, pad=12)
    map_axis.set_aspect('equal')
    map_axis.set_xlim(-1.08, 1.08)
    map_axis.set_ylim(-1.08, 1.08)
    map_axis.axis('off')
    map_axis.add_artist(plt_circle := __import__('matplotlib.pyplot').pyplot.Circle((0, 0), 1.0, color='black', fill=False, linewidth=1.3))
    map_axis.add_artist(__import__('matplotlib.pyplot').pyplot.Circle((0, 0), 0.60, color='gray', fill=False, linestyle='--', alpha=0.45))
    map_axis.add_artist(__import__('matplotlib.pyplot').pyplot.Circle((0, 0), 0.20, color='gray', fill=False, linestyle=':', alpha=0.45))

    tc_map_annotations = {}
    for sensor_id, (x, y) in TC_SENSOR_COORDS.items():
        ch_index = TC_SENSOR_MAPPING.get(sensor_id)
        map_axis.plot(x, y, 'o', color='white', markersize=6, markeredgecolor='black', zorder=3)
        label = f'S{sensor_id}\nCH{ch_index + 1:02d}' if ch_index is not None else f'S{sensor_id}'
        ann = map_axis.text(
            x, y + 0.06, label,
            ha='center', va='bottom', fontsize=7, color='white', zorder=4,
            path_effects=[]
        )
        tc_map_annotations[sensor_id] = ann

    # Set white text with black outline only if matplotlib.patheffects is available via import side effect.
    try:
        import matplotlib.patheffects as PathEffects
        for ann in tc_map_annotations.values():
            ann.set_path_effects([PathEffects.withStroke(linewidth=1.5, foreground='black')])
    except Exception:
        pass

    initialize_tc_heatmap_cache()
    initial_grid = np.full((TC_HEATMAP_GRID, TC_HEATMAP_GRID), np.nan, dtype=float)
    img = map_axis.imshow(
        initial_grid,
        extent=tc_heatmap_extent,
        origin='lower',
        interpolation='bilinear',
        cmap='turbo',
        vmin=0.0,
        vmax=500.0,
        zorder=0,
    )
    cbar = map_fig.colorbar(img, ax=map_axis, fraction=0.046, pad=0.03)
    cbar.set_label('Temperature (°C)')
    cbar.set_ticks([0, 100, 200, 300, 400, 500])

    unused_var = tk.StringVar(parent, 'Unused / off-wafer: CH11=OPEN | CH18=OPEN | CH19=OPEN')
    ttk.Label(map_host, textvariable=unused_var, style='Note.TLabel', anchor='w').pack(fill='x', padx=2, pady=(0, 4))

    map_canvas = FigureCanvasTkAgg(map_fig, master=map_host)
    map_canvas.get_tk_widget().pack(fill='both', expand=True)

    tc_plot_axis = axis
    tc_plot_lines = lines
    tc_plot_canvas = canvas
    tc_map_axis = map_axis
    tc_map_canvas = map_canvas
    tc_map_image = img
    tc_map_cbar = cbar
    tc_map_unused_var = unused_var



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
    title_bar = ttk.Frame(parent)
    title_bar.pack(fill="x")
    ttk.Label(title_bar, text="03  READBACK & STATUS", style="Section.TLabel").pack(
        side="left", anchor="w", pady=(0, 2)
    )
    add_export_button(title_bar, "readback", text="Save Readback").pack(side="right", pady=(0, 2))
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
    add_export_button(frame, "status", text="Save Decode").grid(row=len(fields), column=0, columnspan=2, sticky="w", pady=(16, 0))


def build_log_section(parent):
    title_bar = ttk.Frame(parent)
    title_bar.pack(fill="x", padx=12, pady=(4, 2))
    ttk.Label(title_bar, text="Command / error log", style="Section.TLabel").pack(
        side="left", anchor="w"
    )
    add_export_button(title_bar, "log", text="Save Log").pack(side="right")
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
    global tc_tab_ref, ir_tab_ref
    global root
    global ip_var, port_var, transport_var, reads_ms_var, timeout_ms_var
    global connection_var, polling_info_var, sta_rtt_var, local_port_var
    global power_entry, freq_entry, duty_entry
    global power_set_var, freq_set_var, duty_set_var
    global readback_vars, status_vars, sta_bit_vars, sta_mode_var, ir_current_vars, ir_port_vars, ir_identity_vars
    global tc_port_var, tc_status_var, tc_current_vars

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
    ir_identity_vars = [tk.StringVar(root, "Searching..."), tk.StringVar(root, "Searching...")]

    tc_port_var = tk.StringVar(root, TC_DEFAULT_PORT)
    tc_status_var = tk.StringVar(root, "STOPPED")
    tc_current_vars = [tk.StringVar(root, "--") for _ in range(TC_CHANNEL_COUNT)]

    build_connection_section(root)

    body = ttk.Panedwindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=12, pady=4)

    control_host = ttk.Frame(body, width=460)
    monitor_host = ttk.Frame(body)
    body.add(control_host, weight=0)
    body.add(monitor_host, weight=1)

    build_control_panel(control_host)

    save_bar = ttk.Frame(monitor_host)
    save_bar.pack(fill="x", pady=(0, 4))
    add_export_button(save_bar, "all", text="Save All").pack(side="right")

    notebook = ttk.Notebook(monitor_host)
    notebook.pack(fill="both", expand=True)

    readback_tab = ttk.Frame(notebook)
    ir_tab = ttk.Frame(notebook)
    tc_tab = ttk.Frame(notebook)
    decoded_tab = ttk.Frame(notebook)
    ir_tab_ref = ir_tab
    tc_tab_ref = tc_tab
    notebook.add(readback_tab, text="Readback & Status")
    notebook.add(ir_tab, text="Wafer IR Trend")
    notebook.add(tc_tab, text="TC 20CH")
    notebook.add(decoded_tab, text="Decoded STA")

    build_readback_status(readback_tab)
    build_ir_plot(ir_tab)
    build_tc_plot(tc_tab)
    build_status_decode_tab(decoded_tab)

    build_log_section(root)
    update_polling_info()
    initialize_export_recording()


# ============================================================
# Run
# ============================================================
def main():
    global ir_manager_ref
    build_gui()

    # Start TC first so COM4 (or the selected TC port) is not probed by the IR scanner.
    refresh_tc_port_list()
    root.after(150, start_tc_from_ui)

    ir_manager = IRSensorManager(
        temp_callback=update_ir_temperature,
        log_callback=log_from_worker,
        assignment_callback=update_ir_assignment,
    )
    ir_manager_ref = ir_manager
    root.after(450, ir_manager.start)

    def on_close():
        raycus.close()
        stop_tc_from_ui()
        ir_manager.stop()
        close_export_recording()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, process_tc_sample_queue)
    root.after(100, update_fast_ui)
    root.after(200, update_ir_plots)
    root.after(250, update_tc_plot)
    root.after(300, update_tc_heatmap)
    root.after(250, poll_tick)
    root.mainloop()


if __name__ == "__main__":
    main()
