# %%
import socket
from urllib.parse import urlencode


LASER_IP = "192.168.0.10"
HTTP_PORT = 80
TIMEOUT = 3.0


def http_get(path, params=None):
    """
    Raycus 내부 HTTP 서버에 TCP로 직접 GET 요청을 보낸다.

    params에 중국어가 포함되어 있으므로
    현재 웹페이지 charset인 GBK 기준으로 URL encoding한다.
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


    request = (
        f"GET {request_path} HTTP/1.0\r\n"
        f"Host: {LASER_IP}\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("ascii")


    print("=" * 70)
    print("TCP/HTTP COMMAND")
    print("=" * 70)

    print("Target :", (LASER_IP, HTTP_PORT))
    print("GET    :", request_path)


    with socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    ) as sock:

        sock.settimeout(TIMEOUT)

        # TCP 연결
        sock.connect(
            (LASER_IP, HTTP_PORT)
        )

        print(
            "TCP    : CONNECTED"
        )


        # HTTP 요청
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


    # --------------------------------------------------------
    # HTTP header / body 분리
    # --------------------------------------------------------

    if b"\r\n\r\n" in response:

        header, body = response.split(
            b"\r\n\r\n",
            1
        )

    else:

        header = response
        body = b""


    print()
    print("HTTP RESPONSE HEADER")
    print("-" * 70)

    print(
        header.decode(
            "latin1",
            errors="replace"
        )
    )


    print()
    print(
        "Response body length:",
        len(body)
    )


    return response


# ============================================================
# Guide Laser OFF
# ============================================================

def guide_laser_off():

    return http_get(
        "/GuideLaserON.cgi",
        {
            "Disable": "红光关"
        }
    )


# ============================================================
# Guide Laser ON
# ============================================================

def guide_laser_on():

    return http_get(
        "/GuideLaserON.cgi",
        {
            "Enable": "红光开"
        }
    )
# %%
guide_laser_on()
# %%
import socket
import re
from urllib.parse import urlencode

LASER_IP = "192.168.0.10"
HTTP_PORT = 80
TIMEOUT = 3.0


def raycus_http_command(path, params):
    query = urlencode(
        params,
        encoding="gbk",
        errors="strict"
    )

    request_path = f"{path}?{query}"

    request = (
        f"GET {request_path} HTTP/1.0\r\n"
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

    # HTTP header / body 분리
    if b"\r\n\r\n" in response:
        header_raw, body_raw = response.split(b"\r\n\r\n", 1)
    else:
        header_raw = response
        body_raw = b""

    header = header_raw.decode("latin1", errors="replace")
    body = body_raw.decode("gbk", errors="replace")

    # HTTP 상태 확인
    http_ok = "200 OK" in header

    # CGI 실행 결과 추출
    match = re.search(
        r'<!--#SetResult-->\s*([A-Z0-9:_\-\s]+)',
        body
    )

    set_result = None

    if match:
        set_result = match.group(1).strip()

    print("=" * 60)
    print("RAYCUS HTTP COMMAND")
    print("=" * 60)
    print("Request   :", request_path)
    print("HTTP OK   :", http_ok)
    print("SetResult :", set_result)

    return {
        "http_ok": http_ok,
        "set_result": set_result,
        "header": header,
        "body": body,
    }


def guide_laser_on():
    result = raycus_http_command(
        "/GuideLaserON.cgi",
        {
            "Enable": "红光开"
        }
    )

    if result["set_result"] == "ABN":
        print("Guide Laser ON command: VERIFIED")
    else:
        print("Guide Laser ON command: NOT VERIFIED")

    return result


def guide_laser_off():
    result = raycus_http_command(
        "/GuideLaserON.cgi",
        {
            "Disable": "红光关"
        }
    )

    if result["set_result"] == "ABF":
        print("Guide Laser OFF command: VERIFIED")
    else:
        print("Guide Laser OFF command: NOT VERIFIED")

    return result
# %%
guide_laser_on()
# %%
guide_laser_off()
# %%
