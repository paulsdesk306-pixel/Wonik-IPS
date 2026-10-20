#%%
import socket

LASER_IP = "192.168.0.10"
LASER_PORT = 10001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(3)

try:
    command = b"RCT\r"   # 안전한 온도 조회 명령

    print("TX:", command)
    print("TX HEX:", command.hex(" "))

    sock.sendto(command, (LASER_IP, LASER_PORT))

    data, addr = sock.recvfrom(4096)

    print("FROM:", addr)
    print("RX RAW:", data)
    print("RX TEXT:", data.decode("ascii", errors="replace"))
    print("RX HEX:", data.hex(" "))

except socket.timeout:
    print("응답 없음")

finally:
    sock.close()