#%%
import matplotlib.pyplot as plt
import serial
import time
import serial.tools.list_ports
import numpy as np
import socket
import re

ports = serial.tools.list_ports.comports()

for port in ports:
    print(port.device, port.description)

#%%
# 시리얼 통신 설정
PORT = "COM3"
BAUDRATE = 19200
INTERVAL = 1.0
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

#%%
# IR 센서 온도 읽기
def read_temperature(ser):
    request = bytes.fromhex("01 03 9C 42 00 02 4A 4F")

    ser.reset_input_buffer()
    ser.write(request)

    response = ser.read(9)

    if len(response) != 9:
        raise TimeoutError("IR 센서 응답 없음")

    if response[:3] != bytes.fromhex("01 03 04"):
        raise ValueError(f"비정상 응답: {response.hex(' ')}")

    temp1 = int.from_bytes(response[3:5], "big", signed=True) / 10.0

    return temp1

print(read_temperature(ser))

#%%
# ================================
# Raycus UDP 설정
# ================================
LASER_IP = "192.168.0.10"
LASER_PORT = 8098

# True면 EMON 자동 실행
AUTO_EMISSION_ON = True

laser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
laser.settimeout(0.3)
laser.connect((LASER_IP, LASER_PORT))

def udp_command(command):
    laser.send((command + "\r").encode("ascii"))

    while True:
        response = laser.recv(4096).decode(
            "ascii", errors="replace"
        ).strip("\x00\r\n ")

        if response.upper().startswith(("ERR", "BCMD")):
            raise RuntimeError(response)

        if response.upper().startswith(command.split()[0].upper()):
            return response

def read_number_from_response(text):
    match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    if match is None:
        raise ValueError(f"숫자 파싱 실패: {text}")
    return float(match.group())

def read_frequency():
    response = udp_command("RPRR")
    return read_number_from_response(response)

def set_duty_cycle(duty, frequency):
    # duty[%], frequency[Hz]
    pulse_ms = duty * 10.0 / frequency

    # Frequency 설정
    udp_command(f"SPRR {frequency:g}")

    # Pulse width 설정 -> 내부적으로 duty 반영
    udp_command(f"SPW {pulse_ms:.6f}")

#%%
# 1. PI 제어기 클래스 정의
class PIController:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, pv):
        error = setpoint - pv

        # P (비례항)
        p_out = self.kp * error

        # I (적분항)
        self.integral += error * self.dt
        i_out = self.ki * self.integral

        # 전체 제어 입력 계산
        control_signal = p_out + i_out + 40

        # 이전 에러 업데이트
        self.prev_error = error

        return control_signal

#%%
time_history = []
temp_history = []
dt_history = []
pid_gain = []
set_point = []

# PI 이득(Gain) 설정
kp = 0.65
ki = 0.07

# dt 설정
dt = 0

# Setpoint 설정
setpoint = 200

pid = PIController(kp, ki, dt)

#%%
# ================================
# 레이저 주파수 읽기
# ================================
frequency = read_frequency()
print("Laser Frequency =", frequency, "Hz")

if AUTO_EMISSION_ON:
    print("EMISSION ON")
    print(udp_command("EMON"))

#%%
# ================================
# 실시간 그래프 설정
# ================================
plt.ion()

# 온도 그래프
fig1, ax1 = plt.subplots(figsize=(10, 5))
line_temp, = ax1.plot([], [], 'b-', label='PV (현재값)')
line_set, = ax1.plot([], [], 'r--', label='Setpoint (목표값)')

ax1.set_title('PI Temperature Control')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Temperature [C]')
ax1.set_xlim(0, 60)
ax1.set_ylim(0, 250)
ax1.legend()
ax1.grid(True)

# Duty Cycle 그래프
fig2, ax2 = plt.subplots(figsize=(10, 5))
line_duty, = ax2.plot([], [], 'g-', label='Duty Cycle')

ax2.set_title('Duty Cycle vs Time')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Duty Cycle [%]')
ax2.set_xlim(0, 60)
ax2.set_ylim(0, 105)
ax2.legend()
ax2.grid(True)

plt.show(block=False)

last_plot_time = time.monotonic()

#%%
start_time = time.monotonic()
previous_time = start_time
elapsed_time = 0

try:
    while elapsed_time < 60:

        # 1. IR 온도 측정
        current_temp = read_temperature(ser)

        # 2. 현재 시간
        current_time = time.monotonic()

        # 3. 시간 계산
        elapsed_time = current_time - start_time
        dt = current_time - previous_time

        if elapsed_time >= 60:
            break

        # PI 계산
        pid.dt = dt
        control_signal = np.clip(pid.update(setpoint, current_temp), 0, 100)

        # ================================
        # 실제 레이저에 Duty Cycle 전송
        # ================================
        set_duty_cycle(control_signal, frequency)

        # 4. 데이터 저장
        time_history.append(elapsed_time)
        temp_history.append(current_temp)
        dt_history.append(dt)
        pid_gain.append(control_signal)
        set_point.append(setpoint)

        # 실시간 그래프 갱신
        if time.monotonic() - last_plot_time >= 0.2:
            line_temp.set_data(time_history, temp_history)
            line_set.set_data(time_history, set_point)
            line_duty.set_data(time_history, pid_gain)

            fig1.canvas.draw_idle()
            fig2.canvas.draw_idle()
            plt.pause(0.001)

            last_plot_time = time.monotonic()

        # 5. 이전 시간 갱신
        previous_time = current_time

except (KeyboardInterrupt, Exception) as e:
    print("측정/제어 중단:", e)

finally:
    try:
        print("EMOFF")
        print(udp_command("EMOFF"))
    except Exception as e:
        print("EMOFF 실패:", e)

    try:
        laser.close()
    except:
        pass

    try:
        ser.close()
    except:
        pass

#%%
# 마지막 그래프 반영
line_temp.set_data(time_history, temp_history)
line_set.set_data(time_history, set_point)
line_duty.set_data(time_history, pid_gain)

fig1.canvas.draw_idle()
fig2.canvas.draw_idle()

plt.ioff()
plt.show()

#%%
print("측정 데이터 개수:", len(time_history))

if len(dt_history) > 0:
    print("평균 dt:", np.mean(dt_history))
    print("평균 측정 주파수:", 1 / np.mean(dt_history))