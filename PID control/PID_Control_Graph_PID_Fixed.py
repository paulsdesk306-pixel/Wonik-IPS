#%%
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import serial
import time
import serial.tools.list_ports
import numpy as np
import socket
import re
import csv
from datetime import datetime
from pathlib import Path

# 한글 폰트 설정 (Windows)
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False

ports = serial.tools.list_ports.comports()

for port in ports:
    print(port.device, port.description)

#%%
# 시리얼 통신 설정
PORT = "COM8"
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
AUTO_EMISSION_ON = False

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
celcius_200 = 0

# PID 제어기 클래스 정의
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = None

    def reset(self, setpoint, pv):
        # START 시 STOP 상태에서 누적된 적분값을 남기지 않음
        self.integral = 0.0
        self.prev_error = setpoint - pv

    def update(self, setpoint, pv, integrate=True):
        error = setpoint - pv

        # P (비례항)
        p_out = self.kp * error

        # D (미분항): 첫 측정 시 미분 킥 방지
        if self.prev_error is None or self.dt <= 0:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / self.dt
        d_out = self.kd * derivative

        # I (적분항): 출력이 포화되는 방향으로는 적분하지 않음 (Anti-windup)
        if integrate:
            candidate_integral = self.integral + error * self.dt
            candidate_output = (
                p_out + self.ki * candidate_integral
                + d_out + celcius_200
            )
            windup_high = (candidate_output > DUTY_MAX and error > 0)
            windup_low = (candidate_output < DUTY_MIN and error < 0)
            if not (windup_high or windup_low):
                self.integral = candidate_integral

        # 전체 제어 입력 계산
        control_signal = (
            p_out + self.ki * self.integral
            + d_out + celcius_200
        )
        self.prev_error = error
        return control_signal

#%%
time_history = []
temp_history = []
dt_history = []
pid_gain = []          # PID 계산 Duty
applied_duty = []      # 실제 장비에 보낸 Duty 명령 (STOP 시 0으로 기록)
set_point = []

# PID 이득(Gain) 설정
kp = 3.0
ki = 0.3
kd = 0.1

# dt 설정
dt = 0

# Setpoint 설정
setpoint = 200

# Duty Cycle 범위 (레이저가 실제 허용하는 최소 Duty는 별도 확인 필요)
DUTY_MIN = 3.0
DUTY_MAX = 100.0

pid = PIDController(kp, ki, kd, dt)

#%%
# ================================
# 레이저 주파수 읽기
# ================================
frequency = read_frequency()
print("Laser Frequency =", frequency, "Hz")

# PID 계산값을 실제 레이저에 적용할지 여부
apply_control = False
last_applied_duty = 0.0
off_acknowledged = True  # EMOFF 성공 여부 (출력 차단 실측을 의미하지 않음)

if AUTO_EMISSION_ON:
    print("EMISSION ON")
    print(udp_command("EMON"))
    apply_control = True
    off_acknowledged = False

#%%
# ================================
# 실시간 그래프 설정
# ================================
plt.ion()

# 두 그래프를 하나의 Figure에 표시
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9))
plt.subplots_adjust(top=0.90, hspace=0.35)

# 온도 그래프
line_temp, = ax1.plot([], [], 'b-', label='PV (현재값)')
line_set, = ax1.plot([], [], 'r--', label='Setpoint (목표값)')

ax1.set_title('PID Temperature Control')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Temperature [C]')
ax1.set_xlim(0, 10)
ax1.set_ylim(0, max(300, np.ceil((setpoint + 20) / 50) * 50))
ax1.legend()
ax1.grid(True)

# Duty Cycle 그래프
line_duty, = ax2.plot([], [], 'g-', label='Commanded Duty (전송 명령)')
line_pid, = ax2.plot([], [], '--', color='gray', alpha=0.8,
                     label='PID Duty (계산값)')

ax2.set_title('Duty Cycle vs Time')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Duty Cycle [%]')
ax2.set_xlim(0, 10)
ax2.set_ylim(0, 105)
ax2.legend()
ax2.grid(True)

# ================================
# START / STOP 버튼 추가
# ================================
def start_laser(event):
    global apply_control, last_applied_duty, off_acknowledged

    if apply_control:
        return

    try:
        if len(temp_history) == 0:
            print("아직 측정된 온도가 없습니다.")
            return

        # PID 적분 및 이전 오차 초기화
        pid.reset(setpoint, temp_history[-1])

        # 가장 최근 온도로 Duty 재계산 (START 전 적분값 사용하지 않음)
        starting_duty = float(np.clip(
            pid.update(setpoint, temp_history[-1], integrate=False),
            DUTY_MIN, DUTY_MAX
        ))
        set_duty_cycle(starting_duty, frequency)

        print("EMISSION ON")
        off_acknowledged = False
        print(udp_command("EMON"))
        last_applied_duty = starting_duty
        apply_control = True

    except Exception as e:
        apply_control = False
        print("START 실패:", e)
        try:
            print("EMOFF:", udp_command("EMOFF"))
            off_acknowledged = True
            last_applied_duty = 0.0
        except Exception as off_error:
            off_acknowledged = False
            print("EMOFF 실패 (장비 상태 직접 확인 필요):", off_error)


def stop_laser(event):
    global apply_control, last_applied_duty, off_acknowledged

    apply_control = False
    off_acknowledged = False

    try:
        print("EMOFF")
        print(udp_command("EMOFF"))
        off_acknowledged = True
        last_applied_duty = 0.0

    except Exception as e:
        last_applied_duty = float("nan")
        print("STOP 실패 (장비 상태 직접 확인 필요):", e)

# ================================
# CSV 저장 버튼 추가
# ================================
def save_csv(event):
    if len(time_history) == 0:
        print("저장할 측정 데이터가 없습니다.")
        return

    try:
        file_name = f"PID_data_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        file_path = Path(__file__).resolve().parent / file_name

        with open(file_path, "w", newline="", encoding="utf-8-sig") as file:
            writer = csv.writer(file)
            writer.writerow([
                "Time [s]", "Temperature [C]", "dt [s]",
                "Duty Cycle [%]", "Setpoint [C]",
                "Applied Duty Command [%]"
            ])
            writer.writerows(zip(
                time_history, temp_history, dt_history,
                pid_gain, set_point, applied_duty
            ))

        print(f"CSV 저장 완료: {file_path} ({len(time_history)}개 측정값)")
    except Exception as e:
        print("CSV 저장 실패:", e)

# SAVE CSV 버튼
save_ax = fig1.add_axes([0.43, 0.935, 0.15, 0.045])
save_button = Button(save_ax, 'SAVE CSV')
save_button.on_clicked(save_csv)

# START 버튼
start_ax = fig1.add_axes([0.62, 0.935, 0.15, 0.045])
start_button = Button(start_ax, 'START')
start_button.on_clicked(start_laser)

# STOP 버튼
stop_ax = fig1.add_axes([0.81, 0.935, 0.15, 0.045])
stop_button = Button(stop_ax, 'STOP')
stop_button.on_clicked(stop_laser)

plt.show(block=False)

last_plot_time = time.monotonic()
last_gui_time = time.monotonic()

#%%
start_time = time.monotonic()
previous_time = start_time
elapsed_time = 0
next_measurement_time = start_time

try:
    while plt.fignum_exists(fig1.number):
        now = time.monotonic()

        # 측정 대기 중에도 START / STOP 버튼 이벤트 처리
        if now < next_measurement_time:
            if now - last_gui_time >= 0.05:
                fig1.canvas.flush_events()
                last_gui_time = now
            time.sleep(0.01)
            continue

        # 1. IR 온도 측정
        current_temp = read_temperature(ser)

        # 2. 현재 시간
        current_time = time.monotonic()

        # 3. 시간 계산
        elapsed_time = current_time - start_time
        dt = current_time - previous_time

        # PID 계산: STOP 상태에서는 적분항을 누적하지 않음
        pid.dt = dt
        control_signal = float(np.clip(
            pid.update(setpoint, current_temp, integrate=apply_control),
            DUTY_MIN, DUTY_MAX
        ))

        # ================================
        # 실제 레이저에 Duty Cycle 전송
        # START 상태일 때만 적용
        # ================================
        if apply_control:
            set_duty_cycle(control_signal, frequency)
            last_applied_duty = control_signal
        else:
            last_applied_duty = 0.0 if off_acknowledged else float("nan")

        # 4. 데이터 저장
        time_history.append(elapsed_time)
        temp_history.append(current_temp)
        dt_history.append(dt)
        pid_gain.append(control_signal)
        applied_duty.append(last_applied_duty)
        set_point.append(setpoint)

        # ================================
        # 실시간 그래프 갱신 (최적화)
        # ================================
        now = time.monotonic()

        if now - last_plot_time >= 0.5:
            # 그래프 표시 데이터만 줄임 (CSV용 원본 데이터는 유지)
            step = max(1, (len(time_history) + 1499) // 1500)

            line_temp.set_data(time_history[::step], temp_history[::step])
            line_set.set_data(time_history[::step], set_point[::step])
            line_duty.set_data(time_history[::step], applied_duty[::step])
            line_pid.set_data(time_history[::step], pid_gain[::step])

            # 시간축 10초 단위로 자동 확장
            xmax = max(10, np.ceil((elapsed_time + 1) / 10) * 10)
            if xmax > ax1.get_xlim()[1]:
                ax1.set_xlim(0, xmax)
                ax2.set_xlim(0, xmax)

            # 온도가 250°C를 넘어도 그래프가 잘리지 않도록 Y축 자동 확장
            ymax = max(300, np.ceil((max(max(temp_history), setpoint) + 20) / 50) * 50)
            if ymax > ax1.get_ylim()[1]:
                ax1.set_ylim(0, ymax)

            fig1.canvas.draw_idle()
            last_plot_time = now

        # START / STOP 버튼 이벤트 처리
        if now - last_gui_time >= 0.05:
            fig1.canvas.flush_events()
            last_gui_time = now

        # 5. 이전 시간 갱신
        previous_time = current_time

        # 다음 측정 주기
        next_measurement_time += INTERVAL
        if next_measurement_time < time.monotonic():
            next_measurement_time = time.monotonic() + INTERVAL

except (KeyboardInterrupt, Exception) as e:
    print("측정/제어 중단:", e)

finally:
    try:
        print("EMOFF")
        print(udp_command("EMOFF"))
    except Exception as e:
        print("EMOFF 실패 (장비 상태 직접 확인 필요):", e)

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
if len(time_history) > 0:
    step = max(1, (len(time_history) + 1499) // 1500)
    line_temp.set_data(time_history[::step], temp_history[::step])
    line_set.set_data(time_history[::step], set_point[::step])
    line_duty.set_data(time_history[::step], applied_duty[::step])
    line_pid.set_data(time_history[::step], pid_gain[::step])

fig1.canvas.draw_idle()
plt.ioff()
plt.show()

#%%
print("측정 데이터 개수:", len(time_history))
if len(dt_history) > 0:
    print("평균 dt:", np.mean(dt_history))
    print("평균 측정 주파수:", 1 / np.mean(dt_history))
# %%
