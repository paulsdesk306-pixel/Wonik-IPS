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
import math
import threading
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
PORT = "COM8"                     # Wafer 중심 IR (PID 피드백)
IR_OUTSIDE_PORT = "COM5"         # Outside Wafer 외곽 IR (표시/저장 전용)
BAUDRATE = 19200
INTERVAL = 1.0
IR_OUTSIDE_MAX_AGE_S = 3.0      # 오래된 외곽 IR 값은 결측 처리
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

_ir_outside_lock = threading.Lock()
_ir_outside_stop_event = threading.Event()
_ir_outside_latest_temp = math.nan
_ir_outside_latest_time = None

# ================================
# TC 20CH 센서 설정 (기존 TC 로거 프로토콜)
# COM4 / 115200 baud / b's' 시작 / b'q' 종료
# ================================
TC_PORT = "COM4"
TC_BAUDRATE = 115200
TC_CHANNEL_COUNT = 20
TC_GRAPH_CHANNEL = 9     # CH09: 웨이퍼 중앙. PID에는 사용하지 않음
TC_MAX_AGE_S = 3.0      # 오래된 TC 데이터는 결측(NaN) 처리

# 기존 TC 17점 wafer-layout 배치에서 가장 바깥쪽 링에 속한 8개 측정점.
# TC sensor ID와 장비가 보내는 CH 번호는 서로 다르므로 실제 매핑을 적용한다.
TC_SENSOR_MAPPING = {
    1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10,
    11: 18, 12: 13, 13: 14, 14: 15, 15: 16, 16: 17, 17: 20,
}  # 1-based TC channel
TC_OUTER_SENSOR_IDS = (1, 2, 11, 15, 17, 16, 7, 3)
TC_OUTER_CHANNELS = tuple(TC_SENSOR_MAPPING[sensor_id]
                          for sensor_id in TC_OUTER_SENSOR_IDS)

if not 1 <= TC_GRAPH_CHANNEL <= TC_CHANNEL_COUNT:
    raise ValueError("TC_GRAPH_CHANNEL은 1~20이어야 합니다.")


def calculate_tc_outside_average(tc_values):
    """외곽 8점 유효 채널 평균. OPEN/0/NaN은 제외, 유효 개수도 반환."""
    valid = [float(tc_values[ch - 1]) for ch in TC_OUTER_CHANNELS
             if math.isfinite(tc_values[ch - 1])]
    if not valid:
        return math.nan, 0
    return sum(valid) / len(valid), len(valid)

# TC는 백그라운드 스레드에서 수집하여 IR/PID 1초 루프를 방해하지 않음
_tc_lock = threading.Lock()
_tc_stop_event = threading.Event()
_tc_latest_values = (math.nan,) * TC_CHANNEL_COUNT
_tc_latest_time = None


def parse_tc_line(text):
    """TC 프레임: tick_us, CH01, ..., CH20. 불완전한 프레임은 무시."""
    parts = text.strip().split(",")
    if len(parts) < TC_CHANNEL_COUNT + 1:
        return None
    try:
        int(parts[0].strip())  # 타임스탬프가 없는 헤더 등의 데이터는 무시
    except ValueError:
        return None

    values = []
    for field in parts[1:TC_CHANNEL_COUNT + 1]:
        field = field.strip()
        if not field or field.upper() == "OPEN":
            values.append(math.nan)
            continue
        try:
            value = float(field)
        except ValueError:
            value = math.nan
        # 기존 TC 로거와 동일: 0.0은 미연결/결측 값으로 처리
        if value == 0.0 or not math.isfinite(value):
            value = math.nan
        values.append(value)
    return tuple(values)


def tc_acquisition_worker():
    """독립 시리얼 수집 스레드. TC 실패가 PID 명령을 차단하지 않음."""
    global _tc_latest_values, _tc_latest_time
    tc_ser = None
    try:
        tc_ser = serial.Serial(
            TC_PORT, TC_BAUDRATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05, write_timeout=0.1,
        )
        tc_ser.reset_input_buffer()
        tc_ser.write(b"s")
        tc_ser.flush()
        print(f"TC 20CH 측정 시작: {TC_PORT} / {TC_BAUDRATE} baud")

        line_buffer = ""
        while not _tc_stop_event.is_set():
            chunk = tc_ser.read(tc_ser.in_waiting or 1)
            if not chunk:
                continue
            line_buffer += chunk.decode("utf-8", errors="ignore")

            while "\n" in line_buffer:
                line, line_buffer = line_buffer.split("\n", 1)
                parsed = parse_tc_line(line)
                if parsed is not None:
                    with _tc_lock:
                        _tc_latest_values = parsed
                        _tc_latest_time = time.monotonic()

            # 비정상 연속 데이터로 인한 메모리 증가 방지
            if len(line_buffer) > 8192:
                line_buffer = ""

    except (serial.SerialException, OSError, ValueError) as e:
        print(f"TC 센서 통신 실패 ({TC_PORT}): {e}; TC 데이터는 결측으로 기록됩니다.")
    finally:
        if tc_ser is not None:
            try:
                if tc_ser.is_open:
                    tc_ser.write(b"q")
                    tc_ser.close()
            except (serial.SerialException, OSError):
                pass
        print("TC 수집 종료")


def get_tc_snapshot():
    """한 IR 샘플에 대응하는 TC 최신 20채널과 수신 지연시간을 반환."""
    with _tc_lock:
        values = _tc_latest_values
        timestamp = _tc_latest_time
    if timestamp is None:
        return (math.nan,) * TC_CHANNEL_COUNT, math.nan
    age = max(0.0, time.monotonic() - timestamp)
    if age > TC_MAX_AGE_S:
        return (math.nan,) * TC_CHANNEL_COUNT, math.nan
    return values, age


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

# ================================
# 외곽 IR 센서: PID 샘플링 루프와 독립적으로 COM5 수집
# ================================
def ir_outside_acquisition_worker():
    """Wafer outside IR 측정. 오류 시 재시도하며 PID 루프는 건드리지 않음."""
    global _ir_outside_latest_temp, _ir_outside_latest_time
    outside_ser = None
    last_error_log = 0.0
    try:
        while not _ir_outside_stop_event.is_set():
            try:
                if outside_ser is None:
                    outside_ser = serial.Serial(IR_OUTSIDE_PORT, BAUDRATE, timeout=0.5)
                    print(f"Outside IR 측정 시작: {IR_OUTSIDE_PORT} / {BAUDRATE} baud")
                outside_temp = read_temperature(outside_ser)
                with _ir_outside_lock:
                    _ir_outside_latest_temp = outside_temp
                    _ir_outside_latest_time = time.monotonic()
            except (serial.SerialException, OSError, TimeoutError, ValueError) as e:
                if time.monotonic() - last_error_log >= 10.0:
                    print(f"Outside IR 측정 오류 ({IR_OUTSIDE_PORT}): {e}")
                    last_error_log = time.monotonic()
                if outside_ser is not None:
                    try:
                        outside_ser.close()
                    except (OSError, serial.SerialException):
                        pass
                    outside_ser = None
                _ir_outside_stop_event.wait(2.0)
                continue
            _ir_outside_stop_event.wait(INTERVAL)
    finally:
        if outside_ser is not None:
            try:
                outside_ser.close()
            except (OSError, serial.SerialException):
                pass
        print("Outside IR 수집 종료")


def get_ir_outside_snapshot():
    """최신 외곽 IR 온도. 측정 지연/연결 끊김 시 NaN 반환."""
    with _ir_outside_lock:
        temp = _ir_outside_latest_temp
        timestamp = _ir_outside_latest_time
    if timestamp is None or time.monotonic() - timestamp > IR_OUTSIDE_MAX_AGE_S:
        return math.nan
    return temp


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
temp_history = []           # IR 중심 (기존 PID 피드백)
ir_outside_history = []     # IR 외곽 (표시/저장 전용)
tc_temp_history = []       # TC 중심 CH09
tc_outside_history = []    # TC 바깥 링 8개 채널 유효값 평균
tc_outside_valid_count_history = []  # 평균에 실제 반영된 채널 수
tc_offset_history = []     # IR 중심 - TC 중심 [C]
tc_outside_offset_history = []  # IR 외곽 - TC 외곽 평균 [C]
tc_age_history = []        # TC 마지막 수신 데이터의 경과 시간
tc_all_history = []        # 매 IR 측정 시점의 TC CH01~CH20 전체
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
setpoint = 250

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

_tc_thread = threading.Thread(target=tc_acquisition_worker,
                              name="TC20CH-Reader", daemon=True)
_tc_thread.start()
_ir_outside_thread = threading.Thread(target=ir_outside_acquisition_worker,
                                      name="IR-Outside-Reader", daemon=True)
_ir_outside_thread.start()

#%%
# ================================
# 실시간 그래프 설정
# ================================
plt.ion()

# 두 그래프를 하나의 Figure에 표시
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9))
plt.subplots_adjust(top=0.90, hspace=0.35)

# 온도 그래프: IR 2개 + TC 2개(중심·외곽 평균) + 목표값
line_temp, = ax1.plot([], [], 'b-', label='IR Center / PID PV (COM8)')
line_ir_outside, = ax1.plot([], [], '-', color='deepskyblue',
                            label='IR Outside (COM5)')
line_tc, = ax1.plot([], [], '--', color='darkorange',
                    label=f'TC Center CH{TC_GRAPH_CHANNEL:02d}')
line_tc_outside, = ax1.plot([], [], '--', color='purple',
                            label=f'TC Outside Mean ({len(TC_OUTER_CHANNELS)} CH)')
line_set, = ax1.plot([], [], 'r:', label='Setpoint (목표값)')
measurement_text = ax1.text(0.01, 0.98,
                            'IR center/outside: -- / --\nTC center/outside: -- / --',
                            transform=ax1.transAxes, va='top', ha='left',
                            fontsize=9, bbox={'facecolor': 'white',
                                              'alpha': 0.85, 'edgecolor': 'none'})

ax1.set_title('PID Temperature Control (2 IR + 2 TC)')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Temperature [C]')
ax1.set_xlim(0, 10)
ax1.set_ylim(0, max(300, np.ceil((setpoint + 20) / 50) * 50))
ax1.legend(loc='upper right', fontsize=8, ncol=2)
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
                "Time [s]", "Temperature [C]", "dt [s]",  # Temperature: IR Center
                "Duty Cycle [%]", "Setpoint [C]",
                "Applied Duty Command [%]",
                f"TC CH{TC_GRAPH_CHANNEL:02d} [C]",  # TC Center
                "IR - TC [C]", "TC Sample Age [s]",  # Center offset
                "IR Outside [C]", "TC Outside Mean [C]",
                "IR Outside - TC Outside [C]",
                "TC Outside Valid CH Count",
                *[f"TC CH{i:02d} [C]" for i in range(1, TC_CHANNEL_COUNT + 1)],
            ])
            # TC 데이터가 없거나 오래되었으면 빈 칸으로 저장; 0°C로 오인하지 않음
            def clean_csv(value):
                return "" if isinstance(value, (float, np.floating)) and not math.isfinite(value) else value

            for row in zip(
                time_history, temp_history, dt_history,
                pid_gain, set_point, applied_duty,
                tc_temp_history, tc_offset_history, tc_age_history,
                ir_outside_history, tc_outside_history,
                tc_outside_offset_history, tc_outside_valid_count_history,
                tc_all_history,
            ):
                writer.writerow([clean_csv(v) for v in (*row[:-1], *row[-1])])

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

        # 4. 데이터 저장: 기존 PID에는 중심 IR만 사용, 나머지 3개는 비교용
        ir_outside_temp = get_ir_outside_snapshot()
        tc_values, tc_age = get_tc_snapshot()
        tc_selected = tc_values[TC_GRAPH_CHANNEL - 1]  # 중심 CH09
        tc_outside_temp, tc_outside_count = calculate_tc_outside_average(tc_values)
        tc_offset = (current_temp - tc_selected
                     if math.isfinite(tc_selected) else math.nan)
        tc_outside_offset = (
            ir_outside_temp - tc_outside_temp
            if math.isfinite(ir_outside_temp) and math.isfinite(tc_outside_temp)
            else math.nan
        )

        time_history.append(elapsed_time)
        temp_history.append(current_temp)
        ir_outside_history.append(ir_outside_temp)
        tc_temp_history.append(tc_selected)
        tc_outside_history.append(tc_outside_temp)
        tc_outside_valid_count_history.append(tc_outside_count)
        tc_offset_history.append(tc_offset)
        tc_outside_offset_history.append(tc_outside_offset)
        tc_age_history.append(tc_age)
        tc_all_history.append(tc_values)
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
            line_ir_outside.set_data(time_history[::step], ir_outside_history[::step])
            line_tc.set_data(time_history[::step], tc_temp_history[::step])
            line_tc_outside.set_data(time_history[::step], tc_outside_history[::step])
            line_set.set_data(time_history[::step], set_point[::step])

            def temp_label(value):
                return f"{value:.1f}°C" if math.isfinite(value) else "--"

            measurement_text.set_text(
                f"IR Center: {temp_label(current_temp)} | "
                f"IR Outside: {temp_label(ir_outside_temp)}\n"
                f"TC Center CH{TC_GRAPH_CHANNEL:02d}: {temp_label(tc_selected)} | "
                f"TC Outside Mean: {temp_label(tc_outside_temp)} "
                f"({tc_outside_count}/{len(TC_OUTER_CHANNELS)} CH)"
            )
            line_duty.set_data(time_history[::step], applied_duty[::step])
            line_pid.set_data(time_history[::step], pid_gain[::step])

            # 시간축 10초 단위로 자동 확장
            xmax = max(10, np.ceil((elapsed_time + 1) / 10) * 10)
            if xmax > ax1.get_xlim()[1]:
                ax1.set_xlim(0, xmax)
                ax2.set_xlim(0, xmax)

            # 온도가 250°C를 넘어도 그래프가 잘리지 않도록 Y축 자동 확장
            valid_extra = [x for series in (ir_outside_history,
                                            tc_temp_history, tc_outside_history)
                           for x in series if math.isfinite(x)]
            upper_temp = max(max(temp_history), setpoint,
                             max(valid_extra) if valid_extra else setpoint)
            ymax = max(300, np.ceil((upper_temp + 20) / 50) * 50)
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

    _tc_stop_event.set()
    _tc_thread.join(timeout=1.0)
    _ir_outside_stop_event.set()
    _ir_outside_thread.join(timeout=1.0)

    try:
        ser.close()
    except:
        pass

#%%
# 마지막 그래프 반영
if len(time_history) > 0:
    step = max(1, (len(time_history) + 1499) // 1500)
    line_temp.set_data(time_history[::step], temp_history[::step])
    line_ir_outside.set_data(time_history[::step], ir_outside_history[::step])
    line_tc.set_data(time_history[::step], tc_temp_history[::step])
    line_tc_outside.set_data(time_history[::step], tc_outside_history[::step])
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
