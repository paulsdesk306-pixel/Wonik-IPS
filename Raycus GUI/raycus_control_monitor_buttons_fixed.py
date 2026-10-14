# %%
import socket
import re
import threading
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox
from urllib.parse import urlencode

# ============================================================
# Raycus configuration
# ============================================================
LASER_IP = "192.168.0.10"
HTTP_PORT = 80
TIMEOUT = 3.0
AUTO_REFRESH_MS = 1000

# ============================================================
# HTTP communication
# ============================================================
def http_get(path, params=None):
    if params:
        query = urlencode(params, encoding="gbk")
        path = f"{path}?{query}"

    request = (
        f"GET {path} HTTP/1.0\r\n"
        f"Host: {LASER_IP}\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("ascii")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(TIMEOUT)
        sock.connect((LASER_IP, HTTP_PORT))
        sock.sendall(request)

        response = b""
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            response += chunk

    if b"\r\n\r\n" in response:
        header_raw, body_raw = response.split(b"\r\n\r\n", 1)
    else:
        header_raw = response
        body_raw = b""

    header = header_raw.decode("latin1", errors="replace")
    body = body_raw.decode("gbk", errors="replace")
    http_ok = "200 OK" in header

    match = re.search(r'<!--#SetResult-->\s*([^<\r\n]+)', body)
    result = match.group(1).strip() if match else None

    return {
        "ok": http_ok,
        "result": result,
        "body": body,
        "header": header,
        "path": path,
    }

# ============================================================
# Connection / raw page reads
# ============================================================
def check_connection():
    try:
        return http_get("/")["ok"]
    except Exception:
        return False


def read_main():
    return http_get("/index.shtml")


def read_parameters():
    return http_get("/LaserPara.shtml")


def read_status():
    return http_get("/LaserStatus.shtml")


def read_alarm():
    return http_get("/LaserAlarm.shtml")


def read_time():
    return http_get("/LaserTime.shtml")

# ============================================================
# Control functions - existing interface
# ============================================================
def guide_laser_on():
    return http_get("/GuideLaserON.cgi", {"Enable": "红光开"})


def guide_laser_off():
    return http_get("/GuideLaserON.cgi", {"Disable": "红光关"})


def guide_external_on():
    return http_get("/EXTGuideLaserEN.cgi", {"Enable": "红光外控"})


def guide_external_off():
    return http_get("/EXTGuideLaserEN.cgi", {"Disable": "红光内控"})


def emission_off():
    return http_get("/Emission.cgi", {"Disable": "关光"})


def emission_on():
    return http_get("/Emission.cgi", {"Enable": "出光"})


def set_power(value):
    # Browser form includes both the value field and submit-button field.
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
    """
    The embedded web page exposes /PulseMode.cgi, but this RFL-C30000M-CE
    (continuous-wave model) rejects the Pulse Mode request with "Command Err!".
    Keep the button for visibility, but report the device/firmware limitation
    instead of treating it as a network/HTTP failure.
    """
    r = http_get("/PulseMode.cgi", {"Enable": "脉冲模式"})
    if r.get("result") == "Command Err!":
        r["result"] = "Pulse Mode unsupported by this CW firmware"
    return r


def continuous_mode():
    """
    The status page already reports this unit as Continuous Mode (CW).
    Do not resend the generic PulseMode.cgi Disable command when the laser is
    already in Continuous Mode, because this firmware returns "Command Err!".
    """
    try:
        r = read_status()
        if r.get("ok"):
            current = extract_ssi(r.get("body", ""), "PluseEN", default="")
            if current == "连续模式":
                return {
                    "ok": True,
                    "result": "Already Continuous Mode (CW)",
                    "body": r.get("body", ""),
                    "header": r.get("header", ""),
                    "path": "/PulseMode.cgi (not sent; already CW)",
                }
    except Exception:
        pass

    r = http_get("/PulseMode.cgi", {"Disable": "连续模式"})
    if r.get("result") == "Command Err!":
        r["result"] = "Continuous Mode is already fixed by this CW firmware"
    return r


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

# ============================================================
# SSI value parser
# Raycus/lwIP pages contain values such as:
#   <!--#Ready-->激光器准备好
#   <!--#Powere-->10%
# ============================================================
def extract_ssi(body, name, default="-"):
    pattern = rf'<!--#{re.escape(name)}-->\s*([^<\r\n]+)'
    m = re.search(pattern, body)
    return m.group(1).strip() if m else default


def extract_number(value):
    m = re.search(r'[-+]?\d+(?:\.\d+)?', value or "")
    return m.group(0) if m else value


def with_unit(value, unit):
    if value in (None, "", "-"):
        return "-"
    n = extract_number(value)
    return f"{n} {unit}"


def seconds_display(value):
    if value in (None, "", "-"):
        return "-"
    m = re.search(r'\d+', value)
    if not m:
        return value
    total = int(m.group(0))
    h, rem = divmod(total, 3600)
    minute, sec = divmod(rem, 60)
    return f"{total} s  ({h:02d}:{minute:02d}:{sec:02d})"


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


def translate_value(value):
    return STATUS_TRANSLATION.get(value, value)

# ============================================================
# Structured live reads
# ============================================================
def get_parameter_values():
    r = read_parameters()
    if not r["ok"]:
        raise RuntimeError("LaserPara.shtml HTTP error")
    b = r["body"]
    return {
        "power": with_unit(extract_ssi(b, "Powere"), "%"),
        "frequency": with_unit(extract_ssi(b, "Frequency"), "Hz"),
        "pulse_width": with_unit(extract_ssi(b, "PluseTime"), "ms"),
        "temp1": with_unit(extract_ssi(b, "T1"), "°C"),
        "temp2": with_unit(extract_ssi(b, "T2"), "°C"),
        "mcu_temp": with_unit(extract_ssi(b, "MCUT"), "°C"),
        "power_time": seconds_display(extract_ssi(b, "PowerTime")),
        "laser_time": seconds_display(extract_ssi(b, "LaserTime")),
    }


def get_status_values():
    r = read_status()
    if not r["ok"]:
        raise RuntimeError("LaserStatus.shtml HTTP error")
    b = r["body"]
    names = {
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
    return {k: translate_value(extract_ssi(b, marker)) for k, marker in names.items()}


def get_alarm_values():
    r = read_alarm()
    if not r["ok"]:
        raise RuntimeError("LaserAlarm.shtml HTTP error")
    b = r["body"]
    names = {
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
    return {k: translate_value(extract_ssi(b, marker)) for k, marker in names.items()}


def get_time_values():
    """
    Raycus 장치 내부 RTC(Real-Time Clock) 값을 읽는다.
    이 값은 PC 시간이 아니라 레이저 장치 자체의 날짜/시간이다.
    """
    r = read_time()
    if not r["ok"]:
        raise RuntimeError("LaserTime.shtml HTTP error")
    b = r["body"]
    return {
        "device_time": extract_ssi(b, "hour"),
        "device_date": extract_ssi(b, "year"),
    }


def format_duration(seconds):
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    if h >= 24:
        d, h = divmod(h, 24)
        return f"{d} d {h:02d}:{m:02d}:{s:02d}"
    return f"{h:02d}:{m:02d}:{s:02d}"

# ============================================================
# GUI helpers
# ============================================================
def log(text):
    output.insert(tk.END, text + "\n")
    output.see(tk.END)


def run_command(func, *args):
    try:
        result = func(*args)

        if isinstance(result, dict):
            result_text = result.get("result")

            # 통신 로그에는 실제 응답값과 요청 경로만 표시.
            # "Status : RESPONSE RECEIVED" 같은 중복 상태 문구는 표시하지 않음.
            text = (
                f"HTTP : {result.get('ok')}\n"
                f"Result : {result_text}\n"
                f"Path : {result.get('path')}"
            )
        else:
            text = str(result)

        log(text)

        # command 수행 뒤 오른쪽 모니터 값은 즉시 다시 읽음
        request_monitor_refresh()

    except Exception as e:
        messagebox.showwarning("Command error", str(e))
        log(f"ERROR: {e}")


def update_connection():
    """
    Check 버튼:
    - 버튼 클릭 즉시 CHECKING... 표시
    - HTTP 연결 확인
    - 결과를 Connection 라벨과 로그에 표시
    - 연결 성공 시 수동 Refresh도 한 번 요청
    """
    connection_var.set("CHECKING...")
    last_refresh_var.set("Checking connection...")
    root.update_idletasks()

    try:
        connected = check_connection()

        if connected:
            connection_var.set("CONNECTED")
            log("Connection check: CONNECTED")
            request_monitor_refresh(manual=True)
        else:
            connection_var.set("DISCONNECTED")
            last_refresh_var.set("Connection check failed")
            log("Connection check: DISCONNECTED")

    except Exception as e:
        connection_var.set("DISCONNECTED")
        last_refresh_var.set("Connection check error")
        log(f"Connection check ERROR: {e}")


def apply_power():
    run_command(set_power, power_entry.get())


def apply_frequency():
    run_command(set_frequency, freq_entry.get())


def apply_pulse():
    run_command(set_pulse_width, pulse_entry.get())

# ============================================================
# Monitoring GUI data
# ============================================================
PARAMETER_FIELDS = [
    ("power", "Current Power"),
    ("frequency", "Current Frequency"),
    ("pulse_width", "Current Pulse Width"),
    ("temp1", "Internal Temp Sensor 1"),
    ("temp2", "Internal Temp Sensor 2"),
    ("mcu_temp", "Main Control Board Temp"),
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

# Tkinter 변수(StringVar/BooleanVar)는 반드시 Tk() 생성 이후에 만들어야 함.
root = tk.Tk()

parameter_vars = {k: tk.StringVar(master=root, value="-") for k, _ in PARAMETER_FIELDS}
status_vars = {k: tk.StringVar(master=root, value="-") for k, _ in STATUS_FIELDS}
alarm_vars = {k: tk.StringVar(master=root, value="-") for k, _ in ALARM_FIELDS}
time_vars = {k: tk.StringVar(master=root, value="-") for k, _ in TIME_FIELDS}

refresh_lock = threading.Lock()
refresh_in_progress = False
manual_refresh_pending = False

# 논리적 연결 세션 정보
# 주의: HTTP/1.0 요청은 매번 TCP 연결 -> 요청/응답 -> 연결 종료 방식이다.
# 아래 시간은 "하나의 TCP 소켓이 계속 연결된 시간"이 아니라,
# 이 GUI가 장치와 정상 통신을 시작한 뒤 유지 중인 논리적 연결 시간이다.
connected_since = None
last_successful_poll = None


def build_table(parent, fields, variables):
    frame = ttk.Frame(parent, padding=10)
    frame.pack(fill="both", expand=True)
    for row, (key, label) in enumerate(fields):
        ttk.Label(frame, text=label, width=28).grid(
            row=row, column=0, sticky="w", padx=(0, 10), pady=4
        )
        ttk.Label(frame, textvariable=variables[key], width=32).grid(
            row=row, column=1, sticky="w", pady=4
        )
    frame.columnconfigure(1, weight=1)


def apply_monitor_data(data):
    global refresh_in_progress, connected_since, last_successful_poll
    global manual_refresh_pending

    try:
        now = datetime.now()

        # 첫 정상 통신 시각을 연결 시작 시각으로 기록
        if connected_since is None:
            connected_since = now

        last_successful_poll = now

        for k, v in data["parameters"].items():
            parameter_vars[k].set(v)

        for k, v in data["status"].items():
            status_vars[k].set(v)

        for k, v in data["alarms"].items():
            alarm_vars[k].set(v)

        # 장치 RTC 값
        time_vars["device_time"].set(data["time"].get("device_time", "-"))
        time_vars["device_date"].set(data["time"].get("device_date", "-"))

        # PC / GUI 세션 값
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

        # 상단에는 장치 RTC를 표시
        last_refresh_var.set(
            f'{data["time"].get("device_date", "-")} '
            f'{data["time"].get("device_time", "-")}'
        )

        connection_var.set("CONNECTED")

    finally:
        refresh_in_progress = False

        # Auto-refresh가 진행 중일 때 사용자가 Refresh Now를 눌렀다면
        # 현재 작업이 끝난 직후 수동 refresh를 한 번 더 실행한다.
        if manual_refresh_pending:
            manual_refresh_pending = False
            root.after(10, lambda: request_monitor_refresh(manual=True))


def apply_monitor_error(message):
    global refresh_in_progress, connected_since, last_successful_poll
    global manual_refresh_pending

    connection_var.set("DISCONNECTED")
    last_refresh_var.set("ERROR")

    # 다음 정상 통신 때 새로운 세션으로 시작
    connected_since = None
    last_successful_poll = None

    time_vars["connected_since"].set("-")
    time_vars["connection_duration"].set("-")
    time_vars["last_poll"].set("-")

    refresh_in_progress = False
    log(f"MONITOR ERROR: {message}")

    # 수동 refresh 요청이 대기 중이었다면 오류 후에도 한 번 재시도
    if manual_refresh_pending:
        manual_refresh_pending = False
        root.after(100, lambda: request_monitor_refresh(manual=True))


def monitor_worker():
    try:
        data = {
            "parameters": get_parameter_values(),
            "status": get_status_values(),
            "alarms": get_alarm_values(),
            "time": get_time_values(),
        }
        root.after(0, apply_monitor_data, data)
    except Exception as e:
        root.after(0, apply_monitor_error, str(e))


def request_monitor_refresh(manual=False):
    """
    모니터 새로고침 요청.

    manual=False:
        Auto refresh용. 이미 polling 중이면 그냥 건너뜀.

    manual=True:
        Refresh Now 버튼용.
        이미 polling 중이면 요청을 버리지 않고 'pending'으로 저장해서
        현재 polling이 끝난 직후 반드시 한 번 더 refresh한다.
    """
    global refresh_in_progress, manual_refresh_pending

    if refresh_in_progress:
        if manual:
            manual_refresh_pending = True
            last_refresh_var.set("Refresh queued...")
            log("Manual refresh queued")
        return False

    refresh_in_progress = True

    if manual:
        last_refresh_var.set("Refreshing...")
        log("Manual refresh started")

    threading.Thread(
        target=monitor_worker,
        daemon=True
    ).start()

    return True


def auto_refresh_loop():
    now = datetime.now()

    # PC 시간은 네트워크 상태와 무관하게 계속 표시
    if "pc_time" in time_vars:
        time_vars["pc_time"].set(now.strftime("%Y-%m-%d %H:%M:%S"))

    # 논리적 연결 세션 경과시간
    if connected_since is not None:
        time_vars["connection_duration"].set(
            format_duration((now - connected_since).total_seconds())
        )

    if auto_refresh_var.get():
        request_monitor_refresh()

    root.after(AUTO_REFRESH_MS, auto_refresh_loop)

# ============================================================
# GUI
# ============================================================
root.title("Raycus Laser Control / Monitor")
root.geometry("1320x920")
root.minsize(1150, 760)

# ------------------ Connection ------------------
connection_frame = ttk.LabelFrame(root, text="Connection")
connection_frame.pack(fill="x", padx=10, pady=(10, 5))

ttk.Label(connection_frame, text="Laser IP:").pack(side="left", padx=(8, 4), pady=6)
ttk.Label(connection_frame, text=LASER_IP).pack(side="left", padx=(0, 20))
connection_var = tk.StringVar(master=root, value="UNKNOWN")
ttk.Label(connection_frame, textvariable=connection_var).pack(side="left", padx=10)
ttk.Button(
    connection_frame,
    text="Check",
    command=update_connection
).pack(side="left", padx=4)

ttk.Button(
    connection_frame,
    text="Refresh Now",
    command=lambda: request_monitor_refresh(manual=True)
).pack(side="left", padx=4)
auto_refresh_var = tk.BooleanVar(master=root, value=True)
ttk.Checkbutton(connection_frame, text="Auto refresh (1 s)", variable=auto_refresh_var).pack(side="left", padx=12)
ttk.Label(connection_frame, text="Device RTC:").pack(side="left", padx=(25, 4))
last_refresh_var = tk.StringVar(master=root, value="-")
ttk.Label(connection_frame, textvariable=last_refresh_var).pack(side="left")

# ------------------ Main body ------------------
body = ttk.Panedwindow(root, orient="horizontal")
body.pack(fill="both", expand=True, padx=10, pady=5)

control_panel = ttk.Frame(body, width=470)
monitor_panel = ttk.Frame(body)
body.add(control_panel, weight=0)
body.add(monitor_panel, weight=1)

# ------------------ Left: Controls ------------------
# Read / Monitor buttons removed: the same information is already shown
# continuously in the live monitor tabs on the right.
guide_frame = ttk.LabelFrame(control_panel, text="Guide Laser")
guide_frame.pack(fill="x", pady=4)
for text, func in [
    ("Guide ON", guide_laser_on),
    ("Guide OFF", guide_laser_off),
    ("External Guide", guide_external_on),
    ("Internal Guide", guide_external_off),
]:
    ttk.Button(guide_frame, text=text, command=lambda f=func: run_command(f)).pack(
        side="left", padx=2, pady=5
    )

parameter_frame = ttk.LabelFrame(control_panel, text="Laser Parameters")
parameter_frame.pack(fill="x", pady=4)

ttk.Label(parameter_frame, text="Power (%)").grid(row=0, column=0, sticky="w", padx=5, pady=4)
power_entry = ttk.Entry(parameter_frame, width=12)
power_entry.grid(row=0, column=1, padx=5, pady=4)
ttk.Button(parameter_frame, text="SET", command=apply_power).grid(row=0, column=2, padx=5)

ttk.Label(parameter_frame, text="Frequency (Hz)").grid(row=1, column=0, sticky="w", padx=5, pady=4)
freq_entry = ttk.Entry(parameter_frame, width=12)
freq_entry.grid(row=1, column=1, padx=5, pady=4)
ttk.Button(parameter_frame, text="SET", command=apply_frequency).grid(row=1, column=2, padx=5)

ttk.Label(parameter_frame, text="Pulse Width (ms)").grid(row=2, column=0, sticky="w", padx=5, pady=4)
pulse_entry = ttk.Entry(parameter_frame, width=12)
pulse_entry.grid(row=2, column=1, padx=5, pady=4)
ttk.Button(parameter_frame, text="SET", command=apply_pulse).grid(row=2, column=2, padx=5)

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
for i, (text, func) in enumerate(mode_buttons):
    ttk.Button(mode_frame, text=text, command=lambda f=func: run_command(f)).grid(
        row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3
    )
mode_frame.columnconfigure(0, weight=1)
mode_frame.columnconfigure(1, weight=1)

emission_frame = ttk.LabelFrame(control_panel, text="Main Laser Emission")
emission_frame.pack(fill="x", pady=4)
ttk.Button(emission_frame, text="EMISSION ON", command=lambda: run_command(emission_on)).pack(
    side="left", padx=8, pady=6
)
ttk.Button(emission_frame, text="EMISSION OFF", command=lambda: run_command(emission_off)).pack(
    side="left", padx=8, pady=6
)

# ------------------ Right: Live Monitor Notebook ------------------
notebook = ttk.Notebook(monitor_panel)
notebook.pack(fill="both", expand=True)

param_tab = ttk.Frame(notebook)
status_tab = ttk.Frame(notebook)
alarm_tab = ttk.Frame(notebook)
time_tab = ttk.Frame(notebook)

notebook.add(param_tab, text="Realtime Parameters")
notebook.add(status_tab, text="Status")
notebook.add(alarm_tab, text="Alarm")
notebook.add(time_tab, text="Device Time")

build_table(param_tab, PARAMETER_FIELDS, parameter_vars)
build_table(status_tab, STATUS_FIELDS, status_vars)
build_table(alarm_tab, ALARM_FIELDS, alarm_vars)
build_table(time_tab, TIME_FIELDS, time_vars)

# ------------------ Console ------------------
output_frame = ttk.LabelFrame(root, text="Communication Log")
output_frame.pack(fill="both", padx=10, pady=(5, 10))
output = tk.Text(output_frame, height=10)
output.pack(fill="both", expand=True)

# ------------------ Start ------------------
update_connection()
root.after(500, auto_refresh_loop)
root.mainloop()
# %%
