#%%
import socket

LASER_IP = "192.168.0.10"
LASER_PORT = 8098

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.settimeout(1.0)

sock.connect((LASER_IP, LASER_PORT))

print("UDP socket created")
print("local address =", sock.getsockname())
# %%
