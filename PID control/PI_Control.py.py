#%%
import matplotlib.pyplot as plt
import serial
import time
import serial.tools.list_ports

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
import time
import numpy as np

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

start_time = time.monotonic()
previous_time = start_time  # 추가 1: 이전 시간 초기화
elapsed_time = 0

while elapsed_time < 60:

    # 1. IR 온도 측정
    current_temp = read_temperature(ser)

    # 2. 현재 시간
    current_time = time.monotonic()

    # 3. 시간 계산
    elapsed_time = current_time - start_time
    dt = current_time - previous_time

    # PI 계산
    pid.dt = dt  # 추가 2: PI 제어기에 실제 dt 반영
    control_signal = np.clip(pid.update(setpoint, current_temp), 0, 100)

    # 4. 데이터 저장
    time_history.append(elapsed_time)
    temp_history.append(current_temp)
    dt_history.append(dt)
    pid_gain.append(control_signal)
    set_point.append(setpoint)

    # 5. 이전 시간 갱신
    previous_time = current_time

#%%
# 4. 결과 그래프 출력
plt.figure(figsize=(10, 5))
plt.plot(time_history, set_point, 'r--', label='Setpoint (목표값)')
plt.plot(time_history, temp_history, 'b-', label='PV (현재값)')
plt.plot(time_history, pid_gain, label='PI Output')
plt.title('PI Control Simulation')
plt.xlabel('Time [s]')
plt.ylabel('Value')
plt.legend()
plt.grid(True)
plt.show()
#%%