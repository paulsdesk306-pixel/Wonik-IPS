#%%
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
import socket

IP = "192.168.0.10"
PORT = 80

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(3)

    print("TCP connecting...")
    s.connect((IP, PORT))

    print("TCP CONNECTED:", s.getpeername())

    # HTTP GET 요청
    request = (
        "GET / HTTP/1.1\r\n"
        f"Host: {IP}\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("ascii")

    s.sendall(request)

    response = b""

    while True:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            break

        if not chunk:
            break

        response += chunk

print("\n--- RESPONSE ---")
print(response.decode("utf-8", errors="replace"))
# %%
