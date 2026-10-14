# %%
import serial
import time

# 시리얼 통신 설정
PORT = "COM8"
BAUDRATE = 19200
INTERVAL = 1.0

print(bytes.fromhex("01 03 9C 42 00 02 4A 4F"))
print(serial.Serial(PORT, BAUDRATE, timeout=1).reset_input_buffer())
print(serial.Serial(PORT, BAUDRATE, timeout=1).read(9))
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
    temp2 = int.from_bytes(response[5:7], "big", signed=True) / 10.0

    return temp1, temp2


# %%
# 실시간 온도 측정
try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print("IR 센서 연결 성공")
        print("온도 측정 시작 (1초 간격)\n")

        while True:
            start_time = time.monotonic()

            temp1, temp2 = read_temperature(ser)
            current_temp = temp1

            print(f"온도 1: {temp1:.1f} °C | 온도 2: {temp2:.1f} °C")

            elapsed = time.monotonic() - start_time
            time.sleep(max(0, INTERVAL - elapsed))

except KeyboardInterrupt:
    print("\n온도 측정 종료")

except Exception as e:
    print(f"오류: {e}")

# %%