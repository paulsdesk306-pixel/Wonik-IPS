# %%
import socket
import re
import tkinter as tk
from tkinter import ttk, messagebox
from urllib.parse import urlencode


# ============================================================
# Raycus configuration
# ============================================================

LASER_IP = "192.168.0.10"
HTTP_PORT = 80
TIMEOUT = 3.0


# ============================================================
# HTTP communication
# ============================================================

def http_get(path, params=None):

    if params:
        query = urlencode(
            params,
            encoding="gbk"
        )
        path = f"{path}?{query}"

    request = (
        f"GET {path} HTTP/1.0\r\n"
        f"Host: {LASER_IP}\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("ascii")

    with socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    ) as sock:

        sock.settimeout(TIMEOUT)

        sock.connect(
            (LASER_IP, HTTP_PORT)
        )

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

        header_raw, body_raw = response.split(
            b"\r\n\r\n",
            1
        )

    else:

        header_raw = response
        body_raw = b""

    header = header_raw.decode(
        "latin1",
        errors="replace"
    )

    body = body_raw.decode(
        "gbk",
        errors="replace"
    )

    http_ok = "200 OK" in header

    match = re.search(
        r'<!--#SetResult-->\s*([^<\r\n]+)',
        body
    )

    result = (
        match.group(1).strip()
        if match else None
    )

    return {
        "ok": http_ok,
        "result": result,
        "body": body,
        "header": header,
        "path": path
    }


# ============================================================
# Connection
# ============================================================

def check_connection():

    try:
        r = http_get("/")

        return r["ok"]

    except Exception:
        return False


# ============================================================
# READ FUNCTIONS
# ============================================================

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
# GUIDE LASER
# ============================================================

def guide_laser_on():

    return http_get(
        "/GuideLaserON.cgi",
        {
            "Enable": "红光开"
        }
    )


def guide_laser_off():

    return http_get(
        "/GuideLaserON.cgi",
        {
            "Disable": "红光关"
        }
    )


def guide_external_on():

    return http_get(
        "/EXTGuideLaserEN.cgi",
        {
            "Enable": "红光外控"
        }
    )


def guide_external_off():

    return http_get(
        "/EXTGuideLaserEN.cgi",
        {
            "Disable": "红光内控"
        }
    )


# ============================================================
# SAFE STOP
# ============================================================

def emission_off():

    return http_get(
        "/Emission.cgi",
        {
            "Disable": "关光"
        }
    )


# ============================================================
# HAZARDOUS CONTROL PLACEHOLDERS
#
# GUI와 함수는 모두 존재하지만 실제 송신은 하지 않는다.
# 제조사 승인 제어층 / PLC interlock을 통해 연결할 위치.
# ============================================================

def emission_on():

    return http_get(
            "/Emission.cgi",
            {
                "Enable": "出光"
            }
        )


def set_power(value):
    return http_get(
        "/SetPower.cgi",
        {
            "Power": str(value)
        }
    )

def set_frequency(value):

    return http_get(
        "/SetFrequency.cgi",
        {
            "Frequency": str(value)
        }
    )


def set_pulse_width(value):

    return http_get(
            "/SetPulseTime.cgi",
            {
                "PulseTime": str(value)
            }
        )


def pulse_mode():

    return http_get(
            "/PulseMode.cgi",
            {
                "Enable": "脉冲模式"
            }
        )


def continuous_mode():

    return http_get(
            "/PulseMode.cgi",
            {
                "Disable": "连续模式"
            }
        )


def gate_on():

    return http_get(
            "/GateMode.cgi",
            {
                "Enable": "GATE模式开"
            }
        )


def gate_off():

    return http_get(
            "/GateMode.cgi",
            {
                "Disable": "GATE模式关"
            }
        )


def modulation_on():

    return http_get(
            "/ModMode.cgi",
            {
                "Enable": "调制模式开"
            }
        )


def modulation_off():

    return http_get(
            "/ModMode.cgi",
            {
                "Disable": "调制模式关"
            }
        )


def external_enable_on():

    return http_get(
            "/ExtEN.cgi",
            {
                "Enable": "外部使能开"
            }
        )


def external_enable_off():

    return http_get(
            "/ExtEN.cgi",
            {
                "Disable": "外部使能关"
            }
        )
    


def ad_mode_on():

    return http_get(
            "/ADMODE.cgi",
            {
                "button_EEC": "AD模式使能"
            }
        )


def ad_mode_off():

    return http_get(
            "/ADMODE.cgi",
            {
                "button_DEC": "AD模式禁止"
            }
        )


# ============================================================
# GUI helpers
# ============================================================

def run_command(func, *args):

    try:

        result = func(*args)

        if isinstance(result, dict):

            text = (
                f"HTTP : {result.get('ok')}\n"
                f"Result : {result.get('result')}\n"
                f"Path : {result.get('path')}"
            )

        else:
            text = str(result)

        log(text)

    except Exception as e:

        messagebox.showwarning(
            "Command blocked / error",
            str(e)
        )

        log(
            f"ERROR: {e}"
        )


def log(text):

    output.insert(
        tk.END,
        text + "\n"
    )

    output.see(
        tk.END
    )


def update_connection():

    connected = check_connection()

    if connected:

        connection_label.config(
            text="CONNECTED"
        )

    else:

        connection_label.config(
            text="DISCONNECTED"
        )


def apply_power():

    value = power_entry.get()

    run_command(
        set_power,
        value
    )


def apply_frequency():

    value = freq_entry.get()

    run_command(
        set_frequency,
        value
    )


def apply_pulse():

    value = pulse_entry.get()

    run_command(
        set_pulse_width,
        value
    )


# ============================================================
# GUI
# ============================================================

root = tk.Tk()

root.title(
    "Raycus Laser Control"
)

root.geometry(
    "1000x760"
)


# ============================================================
# Connection frame
# ============================================================

connection_frame = ttk.LabelFrame(
    root,
    text="Connection"
)

connection_frame.pack(
    fill="x",
    padx=10,
    pady=10
)


ttk.Label(
    connection_frame,
    text="Laser IP:"
).pack(
    side="left",
    padx=5
)


ttk.Label(
    connection_frame,
    text=LASER_IP
).pack(
    side="left"
)


connection_label = ttk.Label(
    connection_frame,
    text="UNKNOWN"
)

connection_label.pack(
    side="left",
    padx=20
)


ttk.Button(
    connection_frame,
    text="Check",
    command=update_connection
).pack(
    side="left"
)


# ============================================================
# Read functions
# ============================================================

read_frame = ttk.LabelFrame(
    root,
    text="Read / Monitor"
)

read_frame.pack(
    fill="x",
    padx=10,
    pady=5
)


ttk.Button(
    read_frame,
    text="Main Page",
    command=lambda:
        run_command(read_main)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    read_frame,
    text="Parameters",
    command=lambda:
        run_command(read_parameters)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    read_frame,
    text="Status",
    command=lambda:
        run_command(read_status)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    read_frame,
    text="Alarm",
    command=lambda:
        run_command(read_alarm)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    read_frame,
    text="Time",
    command=lambda:
        run_command(read_time)
).pack(
    side="left",
    padx=3
)


# ============================================================
# Guide laser
# ============================================================

guide_frame = ttk.LabelFrame(
    root,
    text="Guide Laser"
)

guide_frame.pack(
    fill="x",
    padx=10,
    pady=5
)


ttk.Button(
    guide_frame,
    text="Guide ON",
    command=lambda:
        run_command(guide_laser_on)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    guide_frame,
    text="Guide OFF",
    command=lambda:
        run_command(guide_laser_off)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    guide_frame,
    text="External Guide Control",
    command=lambda:
        run_command(guide_external_on)
).pack(
    side="left",
    padx=3
)


ttk.Button(
    guide_frame,
    text="Internal Guide Control",
    command=lambda:
        run_command(guide_external_off)
).pack(
    side="left",
    padx=3
)


# ============================================================
# Laser parameters
# ============================================================

parameter_frame = ttk.LabelFrame(
    root,
    text="Laser Parameters"
)

parameter_frame.pack(
    fill="x",
    padx=10,
    pady=5
)


ttk.Label(
    parameter_frame,
    text="Power"
).grid(
    row=0,
    column=0,
    padx=5
)


power_entry = ttk.Entry(
    parameter_frame,
    width=10
)

power_entry.grid(
    row=0,
    column=1
)


ttk.Button(
    parameter_frame,
    text="SET",
    command=apply_power
).grid(
    row=0,
    column=2,
    padx=5
)


ttk.Label(
    parameter_frame,
    text="Frequency"
).grid(
    row=1,
    column=0,
    padx=5
)


freq_entry = ttk.Entry(
    parameter_frame,
    width=10
)

freq_entry.grid(
    row=1,
    column=1
)


ttk.Button(
    parameter_frame,
    text="SET",
    command=apply_frequency
).grid(
    row=1,
    column=2,
    padx=5
)


ttk.Label(
    parameter_frame,
    text="Pulse Width"
).grid(
    row=2,
    column=0,
    padx=5
)


pulse_entry = ttk.Entry(
    parameter_frame,
    width=10
)

pulse_entry.grid(
    row=2,
    column=1
)


ttk.Button(
    parameter_frame,
    text="SET",
    command=apply_pulse
).grid(
    row=2,
    column=2,
    padx=5
)


# ============================================================
# Operating modes
# ============================================================

mode_frame = ttk.LabelFrame(
    root,
    text="Operating Modes"
)

mode_frame.pack(
    fill="x",
    padx=10,
    pady=5
)


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


for text, func in mode_buttons:

    ttk.Button(
        mode_frame,
        text=text,
        command=lambda f=func:
            run_command(f)
    ).pack(
        side="left",
        padx=2,
        pady=4
    )


# ============================================================
# Emission
# ============================================================

emission_frame = ttk.LabelFrame(
    root,
    text="Main Laser Emission"
)

emission_frame.pack(
    fill="x",
    padx=10,
    pady=10
)


ttk.Button(
    emission_frame,
    text="EMISSION ON",
    command=lambda:
        run_command(emission_on)
).pack(
    side="left",
    padx=10
)


ttk.Button(
    emission_frame,
    text="EMISSION OFF",
    command=lambda:
        run_command(emission_off)
).pack(
    side="left",
    padx=10
)


# ============================================================
# Output console
# ============================================================

output_frame = ttk.LabelFrame(
    root,
    text="Communication Log"
)

output_frame.pack(
    fill="both",
    expand=True,
    padx=10,
    pady=10
)


output = tk.Text(
    output_frame,
    height=15
)

output.pack(
    fill="both",
    expand=True
)


# ============================================================
# Initial check
# ============================================================

update_connection()


root.mainloop()
# %%
