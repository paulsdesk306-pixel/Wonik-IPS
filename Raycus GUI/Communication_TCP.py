#%%
import socket

PC_IP = "192.168.0.222"
LASER_PORT = 8098

# UDP socket 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Broadcast 전송 허용
sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_BROADCAST,
    1
)

# 실제 Raycus 프로그램이 사용한 PC측 discovery port
sock.bind((PC_IP, 50000))

# 3초 동안 응답 기다림
sock.settimeout(3)

# Raycus 장비 검색 명령
cmd = b"RRT?\r"

print("TX:", cmd)

# 네트워크에 Raycus 장비 검색 packet 전송
sock.sendto(
    cmd,
    ("255.255.255.255", LASER_PORT)
)

try:
    data, addr = sock.recvfrom(4096)

    print("연결 성공")
    print("Laser:", addr)
    print("Response:")
    print(data.decode("ascii"))

except socket.timeout:
    print("Laser 응답 없음")

finally:
    sock.close()
# %%
EXPECTED_IP = "192.168.0.10"
EXPECTED_SN = "250900020"

def check_connection():
    try:
        info = discover_laser()

        ip_ok = info.get("IP") == EXPECTED_IP
        sn_ok = info.get("SN") == EXPECTED_SN

        if ip_ok and sn_ok:
            print("CONNECTED")
            print("Laser:", info.get("LaserType"))
            print("SN   :", info.get("SN"))
            return True

        print("다른 Raycus 장비가 응답했습니다.")
        return False

    except Exception as e:
        print("DISCONNECTED:", e)
        return False
# %%
import socket
from concurrent.futures import ThreadPoolExecutor

IP = "192.168.0.10"

def check_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.15)
        if s.connect_ex((IP, port)) == 0:
            return port
    return None

print("TCP port scan start...")

with ThreadPoolExecutor(max_workers=100) as executor:
    results = executor.map(check_port, range(1, 65536))

open_ports = [p for p in results if p is not None]

print("OPEN TCP PORTS:", open_ports)
# %%
