#%%
import socket

LASER_IP = "192.168.0.10"
LASER_PORT = 8098

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)
sock.connect((LASER_IP, LASER_PORT))

command = "RCT"

payload = (command + "\r").encode("ascii")
print("TX =", payload)

sock.send(payload)

try:
    data = sock.recv(4096)
    print("RX bytes =", data)
    text = data.decode("ascii")
    print("RX text = ", text)

except socket.timeout:
    print("No response")
# %%
print(payload.hex(" "))
# %%
