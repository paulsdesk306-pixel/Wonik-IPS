# %%
# ============================================================
# RAYCUS TCP/HTTP CONTROL
# ============================================================
#
# 실제 확인된 네트워크:
#
#   Laser IP : 192.168.0.10
#   TCP      : 80
#   HTTP     : lwIP/1.3.1
#
# 발견된 CGI:
#
#   /SetPower.cgi
#   /SetFrequency.cgi
#   /SetPulseTime.cgi
#   /EXTGuideLaserEN.cgi
#   /GuideLaserON.cgi
#   /PulseMode.cgi
#   /GateMode.cgi
#   /ModMode.cgi
#   /ExtEN.cgi
#   /ADMODE.cgi
#   /Emission.cgi
#
# 실제 동작 확인:
#
#   Guide Laser ON  -> ABN -> 실제 ON 확인
#   Guide Laser OFF -> ABF -> 실제 OFF 확인
#
# ============================================================

import socket
import re
from urllib.parse import urlencode


# ============================================================
# 기본 설정
# ============================================================

LASER_IP = "192.168.0.10"
HTTP_PORT = 80
TIMEOUT = 3.0


# ============================================================
# HTTP 통신 함수
# ============================================================

def _http_get(path, params=None, send=True):
    """
    Raycus 내부 HTTP 서버에 GET 요청.

    send=False:
        실제 장비로 보내지 않고
        어떤 HTTP 요청이 만들어지는지만 출력.

    send=True:
        실제 TCP 80으로 요청 전송.
    """

    if params:

        query = urlencode(
            params,
            encoding="gbk",
            errors="strict"
        )

        request_path = f"{path}?{query}"

    else:

        request_path = path


    print()
    print("=" * 75)
    print("RAYCUS HTTP COMMAND")
    print("=" * 75)

    print("Target :", f"{LASER_IP}:{HTTP_PORT}")
    print("GET    :", request_path)


    # --------------------------------------------------------
    # Preview만 하는 경우
    # --------------------------------------------------------

    if not send:

        print("SEND   : BLOCKED / PREVIEW ONLY")

        return {
            "sent": False,
            "path": request_path,
            "http_ok": None,
            "set_result": None,
        }


    # --------------------------------------------------------
    # 실제 HTTP request 생성
    # --------------------------------------------------------

    request = (
        f"GET {request_path} HTTP/1.0\r\n"
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

        print("TCP    : CONNECTED")

        sock.sendall(
            request
        )

        response = b""

        while True:

            try:
                chunk = sock.recv(4096)

            except socket.timeout:
                break

            if not chunk:
                break

            response += chunk


    # ========================================================
    # HTTP header / body
    # ========================================================

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


    http_ok = (
        "200 OK" in header
    )


    # ========================================================
    # Raycus CGI 실행 결과 추출
    #
    # 예:
    #
    # <!--#SetResult-->ABN
    #
    # ========================================================

    match = re.search(
        r'<!--#SetResult-->\s*([^<\r\n]+)',
        body
    )

    if match:

        set_result = match.group(1).strip()

    else:

        set_result = None


    print(
        "HTTP   :",
        "OK" if http_ok else "FAILED"
    )

    print(
        "RESULT :",
        set_result
    )


    return {
        "sent": True,
        "path": request_path,
        "http_ok": http_ok,
        "set_result": set_result,
        "header": header,
        "body": body,
    }


# ============================================================
# TCP 연결 확인
# ============================================================

def check_connection():

    try:

        result = _http_get(
            "/"
        )

        if result["http_ok"]:

            print()
            print("RAYCUS : CONNECTED")

            return True

    except Exception as e:

        print(
            "Connection Error:",
            e
        )


    return False


# ============================================================
# 읽기 전용 페이지
# ============================================================

def read_main_page():

    return _http_get(
        "/index.shtml"
    )


def read_laser_parameters():

    return _http_get(
        "/LaserPara.shtml"
    )


def read_laser_status():

    return _http_get(
        "/LaserStatus.shtml"
    )


def read_laser_alarm():

    return _http_get(
        "/LaserAlarm.shtml"
    )


def read_laser_time():

    return _http_get(
        "/LaserTime.shtml"
    )


# ============================================================
# GUIDE LASER
# ============================================================

def guide_laser_on():
    """
    실제 검증 완료.

    HTTP CGI
        ↓

    ABN
        ↓

    Aiming Beam ON
    """

    result = _http_get(
        "/GuideLaserON.cgi",
        {
            "Enable": "红光开"
        }
    )


    if result["set_result"] == "ABN":

        print(
            "Guide Laser ON : VERIFIED"
        )

    else:

        print(
            "Guide Laser ON : CHECK RESULT"
        )


    return result


def guide_laser_off():
    """
    ABF = Aiming Beam OFF
    """

    result = _http_get(
        "/GuideLaserON.cgi",
        {
            "Disable": "红光关"
        }
    )


    if result["set_result"] == "ABF":

        print(
            "Guide Laser OFF : VERIFIED"
        )

    else:

        print(
            "Guide Laser OFF : CHECK RESULT"
        )


    return result


# ============================================================
# GUIDE LASER EXTERNAL / INTERNAL CONTROL
# ============================================================

def guide_external_control_enable():

    return _http_get(
        "/EXTGuideLaserEN.cgi",
        {
            "Enable": "红光外控"
        }
    )


def guide_external_control_disable():

    return _http_get(
        "/EXTGuideLaserEN.cgi",
        {
            "Disable": "红光内控"
        }
    )


# ============================================================
# EMISSION OFF
#
# 정지 방향만 실제 실행 가능
# ============================================================

def emission_off():

    result = _http_get(
        "/Emission.cgi",
        {
            "Disable": "关光"
        }
    )

    print(
        "Emission OFF :",
        result["set_result"]
    )

    return result


# ============================================================
# 아래부터는 주 레이저 동작 조건에 직접 영향을 줌.
#
# 실제 HTTP 양식은 HTML에서 확인됐지만
# 장비로 전송하지 않고 PREVIEW만 출력.
# ============================================================


# ============================================================
# POWER
# ============================================================

def preview_set_power(value):
    """
    HTML form 확인값:

    GET /SetPower.cgi

    Power=<value>
    button_SetPower=Set
    """

    return _http_get(
        "/SetPower.cgi",
        {
            "Power": value,
            "button_SetPower": "Set",
        },
        send=False
    )


# ============================================================
# FREQUENCY
# ============================================================

def preview_set_frequency(value):
    """
    GET /SetFrequency.cgi

    Frequency=<value>
    button_SetFreq=Set
    """

    return _http_get(
        "/SetFrequency.cgi",
        {
            "Frequency": value,
            "button_SetFreq": "Set",
        },
        send=False
    )


# ============================================================
# PULSE WIDTH
# ============================================================

def preview_set_pulse_width(value):
    """
    GET /SetPulseTime.cgi

    PulseTime=<value>
    button_SetPulseTime=Set
    """

    return _http_get(
        "/SetPulseTime.cgi",
        {
            "PulseTime": value,
            "button_SetPulseTime": "Set",
        },
        send=False
    )


# ============================================================
# PULSE / CONTINUOUS MODE
# ============================================================

def preview_pulse_mode_enable():

    return _http_get(
        "/PulseMode.cgi",
        {
            "Enable": "脉冲模式"
        },
        send=False
    )


def preview_continuous_mode():

    return _http_get(
        "/PulseMode.cgi",
        {
            "Disable": "连续模式"
        },
        send=False
    )


# ============================================================
# GATE MODE
# ============================================================

def preview_gate_mode_on():

    return _http_get(
        "/GateMode.cgi",
        {
            "Enable": "GATE模式开"
        },
        send=False
    )


def preview_gate_mode_off():

    return _http_get(
        "/GateMode.cgi",
        {
            "Disable": "GATE模式关"
        },
        send=False
    )


# ============================================================
# MODULATION MODE
# ============================================================

def preview_modulation_on():

    return _http_get(
        "/ModMode.cgi",
        {
            "Enable": "调制模式开"
        },
        send=False
    )


def preview_modulation_off():

    return _http_get(
        "/ModMode.cgi",
        {
            "Disable": "调制模式关"
        },
        send=False
    )


# ============================================================
# EXTERNAL ENABLE
# ============================================================

def preview_external_enable_on():

    return _http_get(
        "/ExtEN.cgi",
        {
            "Enable": "外部使能开"
        },
        send=False
    )


def preview_external_enable_off():

    return _http_get(
        "/ExtEN.cgi",
        {
            "Disable": "外部使能关"
        },
        send=False
    )


# ============================================================
# AD MODE
# ============================================================

def preview_ad_mode_enable():

    return _http_get(
        "/ADMODE.cgi",
        {
            "button_EEC": "AD模式使能"
        },
        send=False
    )


def preview_ad_mode_disable():

    return _http_get(
        "/ADMODE.cgi",
        {
            "button_DEC": "AD模式禁止"
        },
        send=False
    )


# ============================================================
# EMISSION ON
# ============================================================

def preview_emission_on():
    """
    HTML에서 Endpoint 및 form 존재 확인.

    주 레이저 발진을 시작할 수 있는 기능이므로
    실제 전송하지 않음.
    """

    return _http_get(
        "/Emission.cgi",
        {
            "Enable": "出光"
        },
        send=False
    )


# ============================================================
# 명령 목록 출력
# ============================================================

def help_commands():

    print()
    print("=" * 75)
    print("RAYCUS COMMAND LIST")
    print("=" * 75)

    print()
    print("[ CONNECTION ]")
    print("check_connection()")

    print()
    print("[ READ ]")
    print("read_main_page()")
    print("read_laser_parameters()")
    print("read_laser_status()")
    print("read_laser_alarm()")
    print("read_laser_time()")

    print()
    print("[ GUIDE LASER - ACTIVE ]")
    print("guide_laser_on()")
    print("guide_laser_off()")
    print("guide_external_control_enable()")
    print("guide_external_control_disable()")

    print()
    print("[ EMISSION STOP - ACTIVE ]")
    print("emission_off()")

    print()
    print("[ PREVIEW ONLY ]")
    print("preview_set_power(value)")
    print("preview_set_frequency(value)")
    print("preview_set_pulse_width(value)")
    print("preview_pulse_mode_enable()")
    print("preview_continuous_mode()")
    print("preview_gate_mode_on()")
    print("preview_gate_mode_off()")
    print("preview_modulation_on()")
    print("preview_modulation_off()")
    print("preview_external_enable_on()")
    print("preview_external_enable_off()")
    print("preview_ad_mode_enable()")
    print("preview_ad_mode_disable()")
    print("preview_emission_on()")


# ============================================================
# START
# ============================================================

print()
print("#" * 75)
print("RAYCUS TCP/HTTP CONTROL")
print("#" * 75)


if check_connection():

    help_commands()
# %%
guide_laser_on()
guide_laser_off()
guide_external_control_enable()
preview_emission_on()

# %%
