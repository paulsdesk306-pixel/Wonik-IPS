#%%
# ============================================================
# Raycus Fiber Laser - UDP 8098 Status Monitoring
#
# 확인된 실제 통신 흐름
# ------------------------------------------------------------
# 1) Discovery
#    PC 192.168.0.222:50000
#       -> UDP Broadcast 255.255.255.255:8098
#       -> "RRT?\r"
#
#    Laser 192.168.0.10:8098
#       -> JSON 장비 정보 응답
#
# 2) Status Polling
#    PC 192.168.0.222:63971
#       -> Laser 192.168.0.10:8098
#       -> 195-byte ASCII-HEX frame 전송
#
#    Laser
#       -> 363-byte ASCII-HEX response
#
# 3) Response
#       ASCII HEX
#          ↓
#       Binary decode
#          ↓
#       Header / Length / CRC 검증
#          ↓
#       21개의 8-byte status record 파싱
#
#
# 주의
# ------------------------------------------------------------
# - 현재 코드는 "상태 조회"만 수행한다.
# - 레이저 발진 / 출력 변경 명령은 포함하지 않는다.
# - FiberLaserManagement.exe가 켜져 있으면
#   UDP 50000 또는 다른 포트를 점유할 수 있으므로 종료 후 실행.
# ============================================================


import socket
import time
import json
import struct


# ============================================================
# 1. 네트워크 설정
# ============================================================

# 레이저와 연결되어 있는 PC Ethernet IPv4 주소
PC_IP = "192.168.0.222"

# Raycus Laser IP
LASER_IP = "192.168.0.10"

# 실제 FiberLaserManagement 패킷에서 확인된 Laser UDP port
LASER_PORT = 8098


# Discovery 시 공식 프로그램에서 사용한 PC source port
DISCOVERY_LOCAL_PORT = 50000

# Status polling 시 캡처에서 사용한 PC source port
STATUS_LOCAL_PORT = 63971


# UDP 응답 최대 대기시간 [sec]
SOCKET_TIMEOUT = 3.0

# Discovery 응답 후 Status Polling 전까지 대기시간
# 실제 캡처에서는 약 5초 뒤 polling이 시작되었음.
DISCOVERY_WAIT = 5.1


# ============================================================
# 2. FiberLaserManagement에서 실제 조회한 21개 항목
# ============================================================
#
# 각 항목은 4-byte descriptor.
#
# 예:
#
#     06 30 00 AE
#              ^^
#              parameter ID처럼 동작
#
# 첫 번째 byte(06/04/07/00)의 정확한 공식 의미는
# 현재 자료만으로 확정하지 않음.
#
# ============================================================

STATUS_DESCRIPTORS = [
    "063000AE",
    "063000A1",
    "063000A4",
    "063000A2",
    "063000A3",
    "04300016",
    "06300070",
    "06300071",
    "06300072",
    "04300088",
    "04300087",
    "07300081",
    "07300085",
    "07300080",
    "0430009C",
    "0430009D",
    "0430009E",
    "0430009F",
    "00300086",
    "06300017",
    "063000AD",
]


# ============================================================
# 3. CRC-16/MODBUS 계산
# ============================================================
#
# 캡처된 Request / Response를 분석한 결과
# CRC-16/MODBUS 계산값과 packet 내부 CRC가 일치함.
#
# 예:
# Request CRC = FEAE
#
# ============================================================

def crc16_modbus(data: bytes) -> int:
    """
    CRC-16/MODBUS 계산.

    Parameters
    ----------
    data : bytes
        CRC 계산 대상

    Returns
    -------
    int
        16-bit CRC
    """

    crc = 0xFFFF

    for byte in data:

        crc ^= byte

        for _ in range(8):

            # LSB가 1인 경우
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001

            # LSB가 0인 경우
            else:
                crc >>= 1

    return crc & 0xFFFF


# ============================================================
# 4. Laser Discovery
# ============================================================
#
# 실제 FiberLaserManagement 통신:
#
# PC:50000
#      |
#      | UDP Broadcast
#      | RRT?\r
#      v
# 255.255.255.255:8098
#
#
# 정상 응답 예:
#
# {
#   "MaincontrolPCBType":"50010066",
#   "MaincontrolPCBVersion":"VA0",
#   "MaincontrolSoftVersion":"4.04",
#   "LaserType":"CS399B",
#   ...
# }
#
# ============================================================

def discover_laser():

    print()
    print("=" * 70)
    print("STEP 1 : RAYCUS LASER DISCOVERY")
    print("=" * 70)

    # UDP socket 생성
    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    ) as sock:

        # ----------------------------------------------------
        # Broadcast 송신 허용
        # ----------------------------------------------------

        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_BROADCAST,
            1
        )


        # ----------------------------------------------------
        # PC 측 UDP port = 50000
        # ----------------------------------------------------
        #
        # FiberLaserManagement.exe가 실행 중이면
        # 이 port를 이미 사용 중일 수 있음.
        #
        # 그 경우 WinError 10048 발생.
        # ----------------------------------------------------

        try:

            sock.bind(
                (
                    PC_IP,
                    DISCOVERY_LOCAL_PORT
                )
            )

        except OSError as e:

            raise RuntimeError(
                "\n"
                "UDP Discovery Port 50000을 사용할 수 없습니다.\n"
                "\n"
                "FiberLaserManagement.exe가 실행 중인지 확인하세요.\n"
                "프로그램을 완전히 종료한 뒤 다시 실행하세요.\n"
                "\n"
                f"원래 오류:\n{e}"
            )


        # 응답 대기시간
        sock.settimeout(
            SOCKET_TIMEOUT
        )


        # ----------------------------------------------------
        # Discovery command
        # ----------------------------------------------------
        #
        # ASCII:
        #
        # R R T ? CR
        #
        # HEX:
        #
        # 52 52 54 3F 0D
        #
        # ----------------------------------------------------

        command = b"RRT?\r"


        print(
            "Local socket :",
            sock.getsockname()
        )

        print(
            "TX command   :",
            command
        )

        print(
            "TX HEX       :",
            command.hex(" ").upper()
        )


        # ----------------------------------------------------
        # Broadcast 송신
        # ----------------------------------------------------

        sock.sendto(
            command,
            (
                "255.255.255.255",
                LASER_PORT
            )
        )


        # ----------------------------------------------------
        # Laser 응답 수신
        # ----------------------------------------------------

        try:

            data, addr = sock.recvfrom(
                4096
            )

        except socket.timeout:

            raise TimeoutError(
                "\nRaycus Discovery 응답이 없습니다.\n"
                "Laser IP / Ethernet 연결 / Laser 상태를 확인하세요."
            )


    # --------------------------------------------------------
    # ASCII Decode
    # --------------------------------------------------------

    text = data.decode(
        "ascii",
        errors="replace"
    )


    print()
    print(
        "RX from      :",
        addr
    )

    print(
        "RX length    :",
        len(data)
    )

    print(
        "RX text      :",
        text
    )


    # --------------------------------------------------------
    # JSON Decode
    # --------------------------------------------------------

    try:

        info = json.loads(
            text
        )

    except json.JSONDecodeError:

        # 혹시 JSON 형식이 아닌 응답이 오더라도
        # raw 형태로 저장
        info = {
            "raw": text
        }


    return info


# ============================================================
# 5. Status Request Frame 생성
# ============================================================
#
# 내부 Binary Frame:
#
#
# FE FE FE 68
# FF FF
# 31
# 00
# 00 54
# [84-byte DATA]
# FE AE
# 55
#
#
# 구조:
#
# FEFEFE68     Header
# FFFF         field (정확한 공식 명칭 미확정)
# 31           Request control code
# 00           field (정확한 공식 명칭 미확정)
# 0054         DATA length = 84 bytes
# DATA         21 × 4 byte = 84 bytes
# FEAE         CRC
# 55           End
#
#
# 중요한 점:
#
# 이 97-byte binary frame 자체를 UDP로 보내는 게 아님.
#
#
# Binary
#   ↓
# HEX 문자열
#   ↓
# ASCII encoding
#   ↓
# CR 추가
#
#
# 따라서:
#
# 97 binary bytes
#     ↓
# 194 ASCII HEX characters
#     +
# CR 1 byte
#
# = UDP Payload 195 bytes
#
# ============================================================

def build_status_request():

    print()
    print("=" * 70)
    print("BUILD STATUS REQUEST")
    print("=" * 70)


    # --------------------------------------------------------
    # 21개의 4-byte descriptor를 하나의 binary body로 결합
    # --------------------------------------------------------

    body = b"".join(
        bytes.fromhex(item)
        for item in STATUS_DESCRIPTORS
    )


    print(
        "Descriptor count :",
        len(STATUS_DESCRIPTORS)
    )

    print(
        "Body length      :",
        len(body),
        "bytes"
    )


    # 21 × 4 = 84
    if len(body) != 84:

        raise ValueError(
            f"Status Request Body 길이 오류: {len(body)} bytes"
        )


    # --------------------------------------------------------
    # DATA length
    # --------------------------------------------------------
    #
    # decimal 84
    # = hexadecimal 0x0054
    #
    # big endian:
    #
    # 00 54
    #
    # --------------------------------------------------------

    data_length = len(body).to_bytes(
        2,
        byteorder="big"
    )


    # --------------------------------------------------------
    # CRC 계산 대상
    # --------------------------------------------------------
    #
    # 실제 packet 분석 결과:
    #
    # FE FE FE 68
    #
    # 는 CRC 계산에서 제외.
    #
    # FF FF부터 DATA 마지막까지 CRC 계산.
    #
    # --------------------------------------------------------

    crc_target = (
        b"\xFF\xFF"
        + b"\x31"
        + b"\x00"
        + data_length
        + body
    )


    # --------------------------------------------------------
    # CRC 계산
    # --------------------------------------------------------

    crc_value = crc16_modbus(
        crc_target
    )


    # 계산된 16-bit integer → 2-byte big endian
    #
    # 0xFEAE
    #
    # →
    #
    # FE AE

    crc_bytes = crc_value.to_bytes(
        2,
        byteorder="big"
    )


    # --------------------------------------------------------
    # Binary Frame 완성
    # --------------------------------------------------------

    raw_frame = (
        b"\xFE\xFE\xFE\x68"
        + crc_target
        + crc_bytes
        + b"\x55"
    )


    print(
        "Internal frame  :",
        len(raw_frame),
        "bytes"
    )

    print(
        "CRC             :",
        f"0x{crc_value:04X}"
    )


    # --------------------------------------------------------
    # 캡처 원본과 비교 검증
    # --------------------------------------------------------

    if len(raw_frame) != 97:

        raise ValueError(
            f"Internal frame length 오류: {len(raw_frame)}"
        )


    # 성공했던 캡처 Request의 CRC = FEAE

    if crc_value != 0xFEAE:

        raise ValueError(
            f"Request CRC 오류. "
            f"Expected=FEAE / Actual={crc_value:04X}"
        )


    # --------------------------------------------------------
    # Binary → 대문자 HEX 문자열
    # --------------------------------------------------------

    frame_hex = raw_frame.hex().upper()


    # --------------------------------------------------------
    # HEX 문자열 → ASCII
    #
    # 마지막에 CR 추가
    # --------------------------------------------------------

    packet = (
        frame_hex.encode("ascii")
        + b"\r"
    )


    print(
        "UDP payload     :",
        len(packet),
        "bytes"
    )


    # 실제 Wireshark capture:
    #
    # UDP payload = 195 bytes

    if len(packet) != 195:

        raise ValueError(
            f"UDP Payload length 오류: {len(packet)}"
        )


    print()
    print(
        "TX Frame:"
    )

    print(
        packet.decode("ascii")
    )


    return packet


# ============================================================
# 6. Status Polling
# ============================================================
#
# PC
# 192.168.0.222:63971
#
#      ↓ UDP
#
# Laser
# 192.168.0.10:8098
#
#
# Request
# = 195 bytes
#
# Response
# = 363 bytes
#
# ============================================================

def request_status(packet: bytes):

    print()
    print("=" * 70)
    print("STEP 2 : STATUS POLLING")
    print("=" * 70)


    # UDP socket
    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    ) as sock:


        # ----------------------------------------------------
        # 캡처에서 사용했던 Local Port 63971에 bind
        # ----------------------------------------------------

        try:

            sock.bind(
                (
                    PC_IP,
                    STATUS_LOCAL_PORT
                )
            )

        except OSError as e:

            raise RuntimeError(
                "\n"
                "UDP Status Port 63971을 사용할 수 없습니다.\n"
                "\n"
                "FiberLaserManagement.exe가 실행 중인지 확인하세요.\n"
                "\n"
                f"원래 오류:\n{e}"
            )


        sock.settimeout(
            SOCKET_TIMEOUT
        )


        print(
            "Local socket :",
            sock.getsockname()
        )

        print(
            "Remote       :",
            (LASER_IP, LASER_PORT)
        )

        print(
            "TX length    :",
            len(packet)
        )


        # ----------------------------------------------------
        # Laser로 packet 전송
        # ----------------------------------------------------

        sock.sendto(
            packet,
            (
                LASER_IP,
                LASER_PORT
            )
        )


        # ----------------------------------------------------
        # Laser 응답
        # ----------------------------------------------------

        try:

            data, addr = sock.recvfrom(
                4096
            )

        except socket.timeout:

            raise TimeoutError(
                "\nRaycus Status Polling 응답이 없습니다."
            )


    print()
    print(
        "RX from      :",
        addr
    )

    print(
        "RX length    :",
        len(data),
        "bytes"
    )


    return data


# ============================================================
# 7. Status Response Parser
# ============================================================
#
# 실제 UDP Response:
#
# ASCII HEX 문자열 + CR
#
#
# 예:
#
# FEFEFE68FFFFB10000A8....F6C255\r
#
#
# 처리:
#
# ASCII
#   ↓
# bytes.fromhex()
#   ↓
# Binary Frame
#
#
# Response Binary Frame:
#
# FE FE FE 68
# FF FF
# B1
# 00
# 00 A8
# [168-byte DATA]
# [2-byte CRC]
# 55
#
#
# DATA:
#
# 21 × 8 byte
#
#
# 각 record:
#
# 06 30 00 AE | 41 C8 00 00
# ------------   -----------
# descriptor        value
#
# ============================================================

def parse_status_response(data: bytes):

    print()
    print("=" * 70)
    print("STEP 3 : RESPONSE PARSING")
    print("=" * 70)


    # --------------------------------------------------------
    # 마지막 CR 제거
    # --------------------------------------------------------

    if data.endswith(b"\r"):

        data = data[:-1]


    # --------------------------------------------------------
    # ASCII bytes → Python string
    # --------------------------------------------------------

    text = data.decode(
        "ascii"
    )


    # --------------------------------------------------------
    # HEX text → 실제 binary
    # --------------------------------------------------------

    try:

        raw = bytes.fromhex(
            text
        )

    except ValueError as e:

        raise ValueError(
            f"Response HEX Decode 실패: {e}"
        )


    print(
        "ASCII length   :",
        len(data)
    )

    print(
        "Binary length  :",
        len(raw)
    )


    # --------------------------------------------------------
    # Header 검사
    # --------------------------------------------------------

    expected_header = bytes.fromhex(
        "FEFEFE68"
    )


    if raw[:4] != expected_header:

        raise ValueError(
            "잘못된 Raycus Response Header"
        )


    # --------------------------------------------------------
    # End byte 검사
    # --------------------------------------------------------

    if raw[-1] != 0x55:

        raise ValueError(
            "Frame End Byte가 0x55가 아닙니다."
        )


    # --------------------------------------------------------
    # Header field 분석
    # --------------------------------------------------------

    # Request = 0x31
    # Response = 0xB1

    control = raw[6]


    # 아직 정확한 의미를 확정하지 않은 field
    unknown_field = raw[7]


    # 2-byte big-endian DATA length
    data_length = int.from_bytes(
        raw[8:10],
        byteorder="big"
    )


    print()
    print(
        f"Control        : 0x{control:02X}"
    )

    print(
        f"Unknown field  : 0x{unknown_field:02X}"
    )

    print(
        f"Data length    : {data_length} bytes"
    )


    if control != 0xB1:

        print(
            "WARNING:"
            " 예상 Response Control 값 0xB1과 다릅니다."
        )


    # --------------------------------------------------------
    # 전체 Frame Length 검증
    # --------------------------------------------------------
    #
    # 10 bytes
    # =
    # Header ~ Length field까지
    #
    # +
    #
    # DATA
    #
    # +
    #
    # CRC 2 bytes
    #
    # +
    #
    # End 1 byte
    #
    # --------------------------------------------------------

    expected_frame_length = (
        10
        + data_length
        + 2
        + 1
    )


    if len(raw) != expected_frame_length:

        raise ValueError(
            "\nFrame Length 오류\n"
            f"Actual   = {len(raw)}\n"
            f"Expected = {expected_frame_length}"
        )


    print(
        "Frame length   : OK"
    )


    # --------------------------------------------------------
    # CRC 검증
    # --------------------------------------------------------

    # packet 안에 들어있는 CRC
    received_crc = int.from_bytes(
        raw[-3:-1],
        byteorder="big"
    )


    # 실제 확인된 CRC 계산 범위:
    #
    # raw[4]
    #
    # 즉 FF FF부터
    #
    # CRC 직전까지

    calculated_crc = crc16_modbus(
        raw[4:-3]
    )


    print(
        f"Received CRC   : 0x{received_crc:04X}"
    )

    print(
        f"Calculated CRC : 0x{calculated_crc:04X}"
    )


    if received_crc != calculated_crc:

        raise ValueError(
            "\nCRC CHECK FAILED"
        )


    print(
        "CRC check      : OK"
    )


    # --------------------------------------------------------
    # DATA 영역 추출
    # --------------------------------------------------------

    body = raw[
        10:
        10 + data_length
    ]


    # --------------------------------------------------------
    # Response record는 8 bytes 단위
    # --------------------------------------------------------

    if len(body) % 8 != 0:

        raise ValueError(
            "Response Body가 8-byte record 구조가 아닙니다."
        )


    record_count = (
        len(body) // 8
    )


    print(
        "Record count   :",
        record_count
    )


    # 현재 캡처는 21개
    if record_count != 21:

        print(
            "WARNING:"
            f" 예상 record 수 21개와 다릅니다: {record_count}"
        )


    records = []


    # --------------------------------------------------------
    # 8-byte씩 Record 분리
    # --------------------------------------------------------

    for offset in range(
        0,
        len(body),
        8
    ):

        record = body[
            offset:
            offset + 8
        ]


        # ----------------------------------------------------
        # Record 구조
        #
        # 06 30 00 AE 41 C8 00 00
        #
        # 앞 4 bytes:
        #     06 30 00 AE
        #     Descriptor
        #
        # 뒤 4 bytes:
        #     41 C8 00 00
        #     Value
        #
        # ----------------------------------------------------

        descriptor = record[:4]

        value_bytes = record[4:8]


        # Descriptor 첫 byte
        type_code = record[0]


        # Descriptor 마지막 byte를
        # Parameter ID로 취급
        parameter_id = record[3]


        # ----------------------------------------------------
        # Value를 unsigned 32-bit integer로 해석
        # ----------------------------------------------------

        value_uint32 = int.from_bytes(
            value_bytes,
            byteorder="big",
            signed=False
        )


        # ----------------------------------------------------
        # Value를 IEEE754 Float32로도 해석
        #
        # 아직 각 ID별 실제 data type을
        # 완전히 확정하지 않았기 때문에
        # Integer / Float 둘 다 계산함.
        # ----------------------------------------------------

        value_float32 = struct.unpack(
            ">f",
            value_bytes
        )[0]


        records.append(
            {
                "index":
                    offset // 8 + 1,

                "descriptor":
                    descriptor.hex().upper(),

                "type_code":
                    f"{type_code:02X}",

                "id":
                    f"{parameter_id:02X}",

                "hex":
                    value_bytes.hex().upper(),

                "uint32":
                    value_uint32,

                "float32":
                    value_float32,
            }
        )


    return records


# ============================================================
# 8. 결과 출력
# ============================================================
#
# 아직 모든 ID의 실제 물리적 의미가 확정되지 않았기 때문에
#
# ID
# HEX
# UINT32
# FLOAT32
#
# 를 함께 출력함.
#
# ============================================================

def print_status_records(records):

    print()
    print("=" * 100)
    print("RAYCUS STATUS RECORDS")
    print("=" * 100)


    print(
        f"{'NO':<5}"
        f"{'DESCRIPTOR':<14}"
        f"{'TYPE':<7}"
        f"{'ID':<7}"
        f"{'VALUE HEX':<14}"
        f"{'UINT32':>15}"
        f"{'FLOAT32':>22}"
    )


    print("-" * 100)


    for r in records:

        print(
            f"{r['index']:<5}"
            f"{r['descriptor']:<14}"
            f"{r['type_code']:<7}"
            f"{r['id']:<7}"
            f"{r['hex']:<14}"
            f"{r['uint32']:>15}"
            f"{r['float32']:>22.6f}"
        )


# ============================================================
# 9. MAIN
# ============================================================

def main():

    print()
    print("#" * 70)
    print("RAYCUS UDP STATUS MONITOR")
    print("#" * 70)


    # ========================================================
    # STEP 1
    # Laser Discovery
    # ========================================================

    laser_info = discover_laser()


    print()
    print("=" * 70)
    print("LASER INFORMATION")
    print("=" * 70)


    for key, value in laser_info.items():

        print(
            f"{key:<28}: {value}"
        )


    # ========================================================
    # Discovery와 Status Polling 사이 잠시 대기
    # ========================================================

    print()
    print(
        f"Waiting {DISCOVERY_WAIT} seconds..."
    )


    time.sleep(
        DISCOVERY_WAIT
    )


    # ========================================================
    # STEP 2
    # Status Request 생성
    # ========================================================

    packet = build_status_request()


    # ========================================================
    # STEP 3
    # Status Polling
    # ========================================================

    response = request_status(
        packet
    )


    # ========================================================
    # STEP 4
    # Response Parsing
    # ========================================================

    records = parse_status_response(
        response
    )


    # ========================================================
    # STEP 5
    # 결과 표시
    # ========================================================

    print_status_records(
        records
    )


    print()
    print("=" * 70)
    print("DONE")
    print("=" * 70)


# ============================================================
# 프로그램 시작
# ============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print()
        print("사용자가 프로그램을 중지했습니다.")

    except Exception as e:

        print()
        print("=" * 70)
        print("ERROR")
        print("=" * 70)

        print(
            type(e).__name__
            + ":",
            e
        )
# %%
