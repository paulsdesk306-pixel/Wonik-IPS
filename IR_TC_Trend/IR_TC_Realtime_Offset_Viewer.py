# -*- coding: utf-8 -*-
"""IR(2 CH) vs TC(20 CH) 실시간 온도/Offset 비교기.

기존 Raycus 통합 GUI에서 쓰인 센서 프로토콜만 별도로 구현합니다.
레이저 UDP, EMON/EMOFF, Duty/Power 제어 기능은 없습니다.

실행: python IR_TC_Realtime_Offset_Viewer.py
데모: python IR_TC_Realtime_Offset_Viewer.py --demo
필수: pip install pyserial matplotlib
"""

from __future__ import annotations

import argparse
import csv
import math
import queue
import random
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# 실제 기존 코드에서 사용한 센서 사양
IR_BAUD = 19200
IR_REQUEST = bytes.fromhex("01 03 9C 42 00 02 4A 4F")
TC_BAUD = 115200
TC_COUNT = 20
PAIR_INTERVAL_S = 1.0  # Offset를 1초당 1회 기록해 TC 고속 데이터 과중복 방지
STALE_S = 1.0          # 센서 최신값이 이보다 오래되면 유효하지 않음
PAIR_SKEW_S = 0.6      # IR/TC 수신 시각 차이 제한
VIEW_WINDOW_S = 120.0
HISTORY_CAPACITY = 2400


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def parse_ir_frame(data: bytes):
    """(first register temperature, crc_verified) or None.

    레거시 코드와 동일하게 헤더가 맞으면 CRC 오류도 알림과 함께 받을 수 있음.
    실제 Offset 계산은 GUI의 CRC fallback 허용 여부로 결정.
    """
    fallback = None
    for pos in range(max(0, len(data) - 8)):
        frame = data[pos:pos + 9]
        if len(frame) != 9 or frame[:3] != b"\x01\x03\x04":
            continue
        temp = int.from_bytes(frame[3:5], "big", signed=True) / 10.0
        if not (-50 <= temp <= 1000):
            continue
        crc_ok = modbus_crc(frame[:7]) == int.from_bytes(frame[7:9], "little")
        if crc_ok:
            return temp, True
        fallback = (temp, False)
    return fallback


def parse_tc_line(line: str):
    """tick_us, [CH1..CH20] 또는 None. OPEN, 0, 빈 칸은 invalid."""
    if not line or "Measurement" in line:
        return None
    parts = line.strip().split(",")
    if len(parts) < TC_COUNT + 1:
        return None
    try:
        tick_us = int(parts[0].strip())
    except ValueError:
        return None
    temps = []
    for token in parts[1:TC_COUNT + 1]:
        token = token.strip()
        if not token or token.upper() == "OPEN":
            temps.append(math.nan)
            continue
        try:
            val = float(token)
        except ValueError:
            val = math.nan
        temps.append(val if math.isfinite(val) and val != 0.0 else math.nan)
    return tick_us, temps


def safe_mean(values):
    return sum(values) / len(values) if values else math.nan


def safe_std(values):
    if len(values) < 2:
        return math.nan
    mean = safe_mean(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


def fmt(value, precision=1):
    return f"{value:.{precision}f}" if value is not None and math.isfinite(value) else "--"


class IRWorker(threading.Thread):
    def __init__(self, index, port, events, stop_evt):
        super().__init__(daemon=True, name=f"IR{index+1}:{port}")
        self.index, self.port, self.events, self.stop_evt = index, port, events, stop_evt

    def send(self, item):
        try:
            self.events.put_nowait(item)
        except queue.Full:
            pass

    def run(self):
        try:
            with serial.Serial(self.port, IR_BAUD, timeout=0.025, write_timeout=0.2) as ser:
                time.sleep(0.10)
                self.send(("status", f"IR {self.index+1} / {self.port} 연결됨"))
                failures = 0
                while not self.stop_evt.is_set():
                    try:
                        ser.reset_input_buffer()
                        ser.write(IR_REQUEST)
                        ser.flush()
                        deadline = time.monotonic() + 0.24
                        buf = bytearray()
                        result = None
                        while time.monotonic() < deadline and not self.stop_evt.is_set():
                            chunk = ser.read(max(1, min(64, ser.in_waiting)))
                            if chunk:
                                buf.extend(chunk)
                                result = parse_ir_frame(bytes(buf))
                                if result is not None and result[1]:
                                    break
                            if len(buf) > 300:
                                buf = buf[-50:]
                        # CRC 가 틀린 첫 프레임도 진단용으로 전달
                        if result is None:
                            result = parse_ir_frame(bytes(buf))
                        if result is None:
                            failures += 1
                            if failures in (5, 20):
                                self.send(("status", f"IR {self.index+1}: 응답 없음 ({failures}회 연속)"))
                        else:
                            failures = 0
                            temp, verified = result
                            self.send(("ir", self.index, time.monotonic(), temp, verified))
                    except (OSError, serial.SerialException) as exc:
                        self.send(("status", f"IR {self.index+1} 통신 오류: {exc}"))
                        break
                    self.stop_evt.wait(0.13)
        except Exception as exc:
            self.send(("status", f"IR {self.index+1} / {self.port} 연결 실패: {exc}"))
        finally:
            self.send(("disconnected", "ir", self.index))


class TCWorker(threading.Thread):
    def __init__(self, port, events, stop_evt):
        super().__init__(daemon=True, name=f"TC20:{port}")
        self.port, self.events, self.stop_evt = port, events, stop_evt

    def send(self, item):
        try:
            self.events.put_nowait(item)
        except queue.Full:
            pass

    def run(self):
        ser = None
        try:
            ser = serial.Serial(self.port, TC_BAUD, timeout=0.03, write_timeout=0.2)
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(b"s")  # 기존 TC 장비 스트리밍 시작 명령
            ser.flush()
            self.send(("status", f"TC / {self.port} 연결됨, s 명령 전송"))
            buf = ""
            while not self.stop_evt.is_set():
                chunk = ser.read(max(1, min(4096, ser.in_waiting)))
                if not chunk:
                    continue
                buf += chunk.decode("utf-8", errors="ignore")
                if len(buf) > 30000:
                    buf = buf[-10000:]
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    data = parse_tc_line(line.strip())
                    if data is not None:
                        tick_us, values = data
                        self.send(("tc", time.monotonic(), tick_us, values))
        except Exception as exc:
            self.send(("status", f"TC / {self.port} 연결/수신 실패: {exc}"))
        finally:
            if ser is not None:
                try:
                    if ser.is_open:
                        ser.write(b"q")  # 기존 장비 스트리밍 정지 명령
                        ser.flush()
                        ser.close()
                except Exception:
                    pass
            self.send(("disconnected", "tc", 0))


class OffsetViewer:
    def __init__(self, root: tk.Tk, demo=False):
        self.root = root
        self.demo = demo
        self.root.title("IR - TC Real-time Offset Analyzer" + (" [DEMO - SIMULATED]" if demo else ""))
        self.root.geometry("1510x920")
        self.root.minsize(1150, 740)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.events = queue.Queue(maxsize=12000)
        self.stop_evt = threading.Event()
        self.threads = []
        self.running = False
        self.started_at = time.monotonic()
        self.last_pair_at = 0.0
        self.last_draw = 0.0
        self.last_demo = 0.0
        self.selected_ir = tk.StringVar(value="Wafer IR")
        self.selected_ch = tk.StringVar(value="CH09")  # 기존 TC 레이아웃에서 중심 S9 = CH09
        self.window_sec = tk.StringVar(value="30")
        self.allow_crc_fallback = tk.BooleanVar(value=False)
        self.port_wafer = tk.StringVar(value="COM8")
        self.port_outer = tk.StringVar(value="COM5")
        self.port_tc = tk.StringVar(value="COM4")
        self.status_text = tk.StringVar(value="미연결" if not demo else "SIMULATED DATA / NO HARDWARE")
        self.ir_readout = [tk.StringVar(value="-- °C"), tk.StringVar(value="-- °C")]
        self.tc_readout = tk.StringVar(value="-- °C")
        self.delta_readout = tk.StringVar(value="-- °C")
        self.avg_readout = tk.StringVar(value="-- °C")
        self.std_readout = tk.StringVar(value="-- °C")
        self.n_readout = tk.StringVar(value="N = 0")
        self.crc_labels = [tk.StringVar(value="NO DATA"), tk.StringVar(value="NO DATA")]
        self.latest_ir = [None, None]  # (host_monotonic, temperature, crc_good)
        self.latest_tc = None  # (host_monotonic, device_tick_us, [20 temperatures])
        self.records = []
        self.offsets = [[deque(maxlen=HISTORY_CAPACITY) for _ in range(TC_COUNT)] for _ in range(2)]
        self.chart_samples = deque(maxlen=HISTORY_CAPACITY)
        self._build_gui()
        self.refresh_ports()
        self.root.after(80, self._pump)
        if self.demo:
            self.start_demo()

    def _build_gui(self):
        base = ttk.Frame(self.root, padding=9)
        base.pack(fill=tk.BOTH, expand=True)
        top = ttk.LabelFrame(base, text="센서 연결  |  이 프로그램은 측정 전용이며 레이저 제어를 하지 않습니다", padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="IR Wafer").grid(row=0, column=0, sticky="w", padx=4)
        self.wafer_combo = ttk.Combobox(top, textvariable=self.port_wafer, width=12)
        self.wafer_combo.grid(row=0, column=1, padx=4)
        ttk.Label(top, text="IR Outside").grid(row=0, column=2, sticky="w", padx=4)
        self.outer_combo = ttk.Combobox(top, textvariable=self.port_outer, width=12)
        self.outer_combo.grid(row=0, column=3, padx=4)
        ttk.Label(top, text="TC 20CH").grid(row=0, column=4, sticky="w", padx=4)
        self.tc_combo = ttk.Combobox(top, textvariable=self.port_tc, width=12)
        self.tc_combo.grid(row=0, column=5, padx=4)
        ttk.Button(top, text="포트 새로고침", command=self.refresh_ports).grid(row=0, column=6, padx=5)
        ttk.Button(top, text="IR 자동검색", command=self.auto_find_ir).grid(row=0, column=7, padx=5)
        self.connect_btn = ttk.Button(top, text="CONNECT", command=self.connect)
        self.connect_btn.grid(row=0, column=8, padx=5)
        self.disconnect_btn = ttk.Button(top, text="DISCONNECT", command=self.disconnect)
        self.disconnect_btn.grid(row=0, column=9, padx=5)
        top.grid_columnconfigure(10, weight=1)
        ttk.Label(top, textvariable=self.status_text, foreground="#2b6387").grid(row=1, column=0, columnspan=10, sticky="w", pady=(6,0))

        card_frame = ttk.Frame(base, padding=(0, 7))
        card_frame.pack(fill=tk.X)
        cards = [
            ("WAFER IR", self.ir_readout[0]), ("OUTSIDE IR", self.ir_readout[1]),
            ("SELECTED TC", self.tc_readout), ("INSTANT OFFSET", self.delta_readout),
            ("MEAN OFFSET", self.avg_readout), ("OFFSET STD", self.std_readout),
        ]
        for col, (name, var) in enumerate(cards):
            f = ttk.LabelFrame(card_frame, text=name, padding=7)
            f.grid(row=0, column=col, sticky="nsew", padx=3)
            ttk.Label(f, textvariable=var, font=("Arial", 17, "bold")).pack()
            card_frame.columnconfigure(col, weight=1)

        control = ttk.Frame(base)
        control.pack(fill=tk.X, pady=(2, 5))
        ttk.Label(control, text="Offset 비교: IR").pack(side=tk.LEFT, padx=4)
        irselect = ttk.Combobox(control, textvariable=self.selected_ir, state="readonly", width=16,
                                values=["Wafer IR", "Outside IR"])
        irselect.pack(side=tk.LEFT, padx=3)
        irselect.bind("<<ComboboxSelected>>", lambda e: self.update_view())
        ttk.Label(control, text="TC").pack(side=tk.LEFT, padx=(12,3))
        chselect = ttk.Combobox(control, textvariable=self.selected_ch, state="readonly", width=9,
                                values=[f"CH{i+1:02d}" for i in range(TC_COUNT)])
        chselect.pack(side=tk.LEFT, padx=3)
        chselect.bind("<<ComboboxSelected>>", lambda e: self._change_channel())
        ttk.Label(control, text="평균 구간").pack(side=tk.LEFT, padx=(12,3))
        win = ttk.Combobox(control, textvariable=self.window_sec, state="readonly", width=6,
                           values=["10", "30", "60", "120"])
        win.pack(side=tk.LEFT, padx=3)
        win.bind("<<ComboboxSelected>>", lambda e: self.update_view())
        ttk.Label(control, text="초").pack(side=tk.LEFT)
        ttk.Checkbutton(control, text="CRC 오류 IR 데이터 허용 (비권장)", variable=self.allow_crc_fallback,
                        command=self.update_view).pack(side=tk.LEFT, padx=(12, 3))
        ttk.Label(control, textvariable=self.n_readout).pack(side=tk.LEFT, padx=7)
        ttk.Button(control, text="CLEAR DATA", command=self.clear_data).pack(side=tk.RIGHT, padx=4)
        ttk.Button(control, text="SAVE CSV", command=self.save_csv).pack(side=tk.RIGHT, padx=4)

        split = ttk.Panedwindow(base, orient=tk.HORIZONTAL)
        split.pack(fill=tk.BOTH, expand=True)
        plots = ttk.LabelFrame(split, text="온도 + Offset 실시간 Trend", padding=5)
        readings = ttk.LabelFrame(split, text="TC 20CH - 두 IR 센서와의 순간 Offset", padding=6)
        split.add(plots, weight=3)
        split.add(readings, weight=2)
        fig = Figure(figsize=(8.1, 6.4), dpi=100)
        self.temp_ax, self.offset_ax = fig.subplots(2,1,sharex=True,gridspec_kw={"height_ratios":[1.25,1]})
        fig.subplots_adjust(left=0.10, right=0.97, bottom=0.09, top=0.96, hspace=0.25)
        (self.line_wafer,) = self.temp_ax.plot([], [], color="#cc3434", lw=1.5, label="Wafer IR")
        (self.line_outer,) = self.temp_ax.plot([], [], color="#d39a22", lw=1.5, label="Outside IR")
        (self.line_tc,) = self.temp_ax.plot([], [], color="#287ac0", lw=1.5, label="Selected TC")
        (self.line_offset,) = self.offset_ax.plot([], [], color="#6943ae", lw=1.4, label="IR - TC")
        (self.line_mean,) = self.offset_ax.plot([], [], color="#117b62", lw=1.6, linestyle="--", label="Rolling mean")
        self.temp_ax.set_ylabel("Temperature (°C)")
        self.offset_ax.set_ylabel("Offset (°C)")
        self.offset_ax.set_xlabel("Elapsed time (s)")
        self.temp_ax.legend(loc="upper left", fontsize=8)
        self.offset_ax.legend(loc="upper left", fontsize=8)
        for ax in [self.temp_ax, self.offset_ax]:
            ax.grid(alpha=0.25)
            ax.set_xlim(0, 120)
        self.canvas = FigureCanvasTkAgg(fig, master=plots)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.tc_table = ttk.Treeview(readings, columns=("ch","temp","w","o"), show="headings", height=20)
        for col, label, width in [
            ("ch","TC",65), ("temp","Temp (°C)",94), ("w","Wafer - TC",112), ("o","Outside - TC",116)
        ]:
            self.tc_table.heading(col,text=label)
            self.tc_table.column(col,width=width,stretch=True,anchor=tk.CENTER)
        for i in range(TC_COUNT):
            self.tc_table.insert("", tk.END, iid=f"ch{i}", values=(f"CH{i+1:02d}","--","--","--"))
        self.tc_table.selection_set("ch8")
        self.tc_table.bind("<<TreeviewSelect>>", self._select_tree_row)
        scroll = ttk.Scrollbar(readings, command=self.tc_table.yview)
        self.tc_table.configure(yscrollcommand=scroll.set)
        self.tc_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        foot = ttk.LabelFrame(base, text="측정 상태 / 로그", padding=6)
        foot.pack(fill=tk.X, pady=(6,0))
        info = ttk.Frame(foot)
        info.pack(fill=tk.X)
        ttk.Label(info, text="IR CRC 상태:").pack(side=tk.LEFT)
        ttk.Label(info, textvariable=self.crc_labels[0]).pack(side=tk.LEFT, padx=(5,25))
        ttk.Label(info, text="Outside CRC:").pack(side=tk.LEFT)
        ttk.Label(info, textvariable=self.crc_labels[1]).pack(side=tk.LEFT, padx=5)
        ttk.Label(info, text="Offset = IR - TC   |   보정 TC→IR: TC + Offset   |   비교는 같은 위치·안정 구간 권장",
                  foreground="#555555").pack(side=tk.RIGHT)
        self.log_box = tk.Text(foot, height=4, state="disabled", wrap=tk.WORD, font=("Consolas", 9))
        self.log_box.pack(fill=tk.X, pady=(5,0))
        self.log("IR Wafer: COM8 / Outside: COM5 / TC: COM4 는 기존 장비 예시값입니다. COM은 현재 환경에 맞게 확인하세요.")
        self.log("TC CH09는 기존 장비의 가운데 TC 센서(S9) 매핑에 해당합니다. 실제 설치 위치를 반드시 확인하세요.")

    def log(self, text):
        timestamp = datetime.now().strftime("%H:%M:%S")
        if not hasattr(self, "log_box"):
            return
        self.log_box.configure(state="normal")
        self.log_box.insert(tk.END, f"[{timestamp}] {text}\n")
        lines = int(self.log_box.index("end-1c").split(".")[0])
        if lines > 180:
            self.log_box.delete("1.0", "45.0")
        self.log_box.see(tk.END)
        self.log_box.configure(state="disabled")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()] if list_ports else []
        for combo in (self.wafer_combo, self.outer_combo, self.tc_combo):
            combo["values"] = [""] + ports
        self.log("현재 COM 포트: " + (", ".join(ports) if ports else "검색된 포트 없음"))

    def auto_find_ir(self):
        if self.running:
            messagebox.showinfo("IR 자동검색", "측정 중에는 검색할 수 없습니다. DISCONNECT 후 사용하세요.")
            return
        if serial is None:
            messagebox.showerror("오류", "pip install pyserial 이 필요합니다.")
            return
        self.status_text.set("IR 센서 검색 중...")
        exclude = self.port_tc.get().strip().upper()
        self.log("IR 자동검색: TC 포트는 제외하고 IR 프로토콜로 응답 여부를 확인합니다.")
        threading.Thread(target=self._probe_ports, args=(exclude,), daemon=True).start()

    def _probe_ports(self, exclude):
        found = []
        for p in list_ports.comports():
            if p.device.upper() == exclude:
                continue
            try:
                with serial.Serial(p.device, IR_BAUD, timeout=0.03, write_timeout=0.2) as ser:
                    time.sleep(0.1)
                    ser.reset_input_buffer()
                    ser.write(IR_REQUEST)
                    ser.flush()
                    deadline = time.monotonic() + 0.32
                    buf = bytearray()
                    result = None
                    while time.monotonic() < deadline:
                        buf.extend(ser.read(max(1, min(64, ser.in_waiting))))
                        result = parse_ir_frame(bytes(buf))
                        if result is not None and result[1]:
                            break
                    if result:
                        found.append((p.device, result[0], result[1]))
            except Exception:
                continue
        self.events.put(("scan", found))

    def connect(self):
        if self.running:
            return
        if self.demo:
            self.start_demo()
            return
        if serial is None:
            messagebox.showerror("pyserial 누락", "pip install pyserial")
            return
        ports = [self.port_wafer.get().strip(), self.port_outer.get().strip(), self.port_tc.get().strip()]
        chosen = [p.upper() for p in ports if p]
        if not chosen:
            messagebox.showwarning("포트 미지정", "IR 또는 TC 포트를 하나 이상 입력하세요.")
            return
        if len(set(chosen)) != len(chosen):
            messagebox.showerror("포트 중복", "IR 두 개와 TC는 각각 다른 COM 포트를 사용해야 합니다.")
            return
        self.stop_evt = threading.Event()
        self.running = True
        self.clear_data()
        for index in range(2):
            if ports[index]:
                self.threads.append(IRWorker(index, ports[index], self.events, self.stop_evt))
        if ports[2]:
            self.threads.append(TCWorker(ports[2], self.events, self.stop_evt))
        for worker in self.threads:
            worker.start()
        self.status_text.set("센서 연결 중...")
        self.log("CONNECT: 선택된 IR/TC 포트의 측정을 시작합니다.")

    def disconnect(self):
        self.running = False
        self.stop_evt.set()
        self.status_text.set("DISCONNECTED")
        self.log("DISCONNECT: 센서 수신 종료 요청 (TC에 q 명령 전송)")
        # COM 포트를 닫기 전에 즉시 재연결하면 포트 중복 점유가 생길 수 있음.
        for thread in self.threads:
            thread.join(timeout=0.35)
        self.threads.clear()
        self.latest_ir = [None, None]
        self.latest_tc = None

    def start_demo(self):
        if self.running:
            return
        self.clear_data()
        self.running = True
        self.stop_evt = threading.Event()
        self.status_text.set("SIMULATED DATA - 실제 센서값 아님")
        self.log("DEMO: 가상 IR / TC 온도로 화면·Offset 계산 테스트 중 (실제 장비 연결 없음)")

    def _simulate(self, now):
        if not self.running or not self.demo or now - self.last_demo < 0.22:
            return
        self.last_demo = now
        t = now - self.started_at
        # 30초간 승온 후 안정화와 작은 노이즈를 모델링
        wafer = 25.0 + 170.0 * (1.0 - math.exp(-t / 16))
        center_tc = wafer - 3.2 + random.gauss(0, 0.20)
        outside_tc = wafer * 0.92 + 3 + random.gauss(0, 0.2)
        vals = [outside_tc + (i - 9) * 0.25 + random.gauss(0, 0.35) for i in range(20)]
        vals[8] = center_tc  # CH09
        self.events.put(("tc", now, int(t * 1e6), vals))
        self.events.put(("ir", 0, now, wafer + random.gauss(0, 0.13), True))
        self.events.put(("ir", 1, now, outside_tc + 1.2 + random.gauss(0, 0.13), True))

    def _pump(self):
        now = time.monotonic()
        self._simulate(now)
        received = 0
        while received < 4500:
            try:
                evt = self.events.get_nowait()
            except queue.Empty:
                break
            received += 1
            kind = evt[0]
            if kind == "ir" and self.running:
                _, index, timestamp, value, good = evt
                self.latest_ir[index] = (timestamp, value, good)
            elif kind == "tc" and self.running:
                _, timestamp, device_tick, values = evt
                self.latest_tc = (timestamp, device_tick, values)
            elif kind == "status":
                self.log(evt[1])
            elif kind == "disconnected":
                self.log(f"수신기 종료: {evt[1]} {evt[2]}")
            elif kind == "scan":
                self._handle_scan(evt[1])
        if self.running and now - self.last_pair_at >= PAIR_INTERVAL_S:
            self.last_pair_at = now
            self._collect_pair(now)
        if now - self.last_draw > 0.45:
            self.last_draw = now
            self.update_view()
        self.root.after(80, self._pump)

    def _handle_scan(self, found):
        if not found:
            self.log("IR 검색 실패: 유효한 응답을 받은 COM 포트가 없습니다.")
            self.status_text.set("IR 자동검색 실패")
            return
        text = ", ".join(f"{port}: {temp:.1f}°C {'CRC OK' if ok else 'CRC BAD'}"
                         for port, temp, ok in found)
        self.log(f"검색된 IR: {text}")
        names = [record[0] for record in found]
        # Wafer / Outside 역할은 COM 및 온도만으로 항상 확정할 수 없음.
        # 기존 배선 힌트 COM8=Wafer, COM5=Outside를 우선 적용.
        prefer_w = self.port_wafer.get().strip()
        prefer_o = self.port_outer.get().strip()
        wafer = prefer_w if prefer_w in names else ("COM8" if "COM8" in names else names[0])
        outside = prefer_o if prefer_o in names and prefer_o != wafer else next((p for p in names if p != wafer), "")
        self.port_wafer.set(wafer)
        self.port_outer.set(outside)
        self.status_text.set("IR 검색 완료 / Wafer·Outside 물리 배선 확인 필요")
        self.log(f"IR 배정: Wafer={wafer}, Outside={outside or '(없음)'}. 자동검색은 실제 센서 위치까지 판별하지 않습니다.")

    def _usable_ir(self, index, now):
        reading = self.latest_ir[index]
        if reading is None:
            return None
        ts, temp, crc_good = reading
        if now - ts > STALE_S or not math.isfinite(temp):
            return None
        if not crc_good and not self.allow_crc_fallback.get():
            return None
        return reading

    def _usable_tc(self, now):
        return self.latest_tc if self.latest_tc is not None and now - self.latest_tc[0] <= STALE_S else None

    def _collect_pair(self, now):
        tc = self._usable_tc(now)
        ir = [self._usable_ir(i, now) for i in range(2)]
        tc_vals = tc[2] if tc else [math.nan] * TC_COUNT
        ir_temps = [ir[i][1] if ir[i] else math.nan for i in range(2)]
        deltas = [[math.nan] * TC_COUNT for _ in range(2)]
        for i in range(2):
            if ir[i] is None or tc is None or abs(ir[i][0] - tc[0]) > PAIR_SKEW_S:
                continue
            for ch, val in enumerate(tc_vals):
                if math.isfinite(val):
                    delta = ir[i][1] - val
                    deltas[i][ch] = delta
                    self.offsets[i][ch].append((now, delta))
        if not any(math.isfinite(v) for group in deltas for v in group):
            return
        self.chart_samples.append((now, ir_temps, list(tc_vals), deltas))
        row = [datetime.now().isoformat(timespec="milliseconds"), round(now-self.started_at,3),
               self.port_wafer.get() if not self.demo else "DEMO",
               self.port_outer.get() if not self.demo else "DEMO",
               self.port_tc.get() if not self.demo else "DEMO",
               fmt(ir_temps[0],3) if math.isfinite(ir_temps[0]) else "",
               fmt(ir_temps[1],3) if math.isfinite(ir_temps[1]) else ""]
        row += [fmt(x,3) if math.isfinite(x) else "" for x in tc_vals]
        for i in range(2):
            row += [fmt(x,3) if math.isfinite(x) else "" for x in deltas[i]]
        self.records.append(row)

    def _offset_stats(self, ir_idx, ch_idx, now):
        window = float(self.window_sec.get())
        v = [delta for ts, delta in self.offsets[ir_idx][ch_idx] if now - ts <= window]
        return safe_mean(v), safe_std(v), len(v)

    def _change_channel(self):
        expected = f"ch{int(self.selected_ch.get()[2:]) - 1}"
        if self.tc_table.selection() != (expected,):
            self.tc_table.selection_set(expected)
            self.tc_table.see(expected)
        self.update_view()

    def _select_tree_row(self, _evt):
        selection = self.tc_table.selection()
        if selection and selection[0].startswith("ch"):
            ch = int(selection[0][2:])
            expected = f"CH{ch+1:02d}"
            if self.selected_ch.get() != expected:
                self.selected_ch.set(expected)
                self.update_view()

    def update_view(self):
        now = time.monotonic()
        if self.running and not self.demo:
            states = []
            for i, name in enumerate(("Wafer", "Outside")):
                port = self.port_wafer.get().strip() if i == 0 else self.port_outer.get().strip()
                if port:
                    states.append(f"{name}: {'LIVE' if self._usable_ir(i,now) else 'WAIT/STALE'}")
            if self.port_tc.get().strip():
                states.append(f"TC: {'LIVE' if self._usable_tc(now) else 'WAIT/STALE'}")
            self.status_text.set(" / ".join(states) if states else "NO ACTIVE PORT")
        for i in range(2):
            reading = self._usable_ir(i, now)
            self.ir_readout[i].set(f"{fmt(reading[1])} °C")
            original = self.latest_ir[i]
            if original is None or now - original[0] > STALE_S:
                self.crc_labels[i].set("STALE / NO DATA")
            else:
                self.crc_labels[i].set("CRC OK" if original[2] else "CRC ERROR (excluded)" if not self.allow_crc_fallback.get() else "CRC ERROR (allowed)")
        tc = self._usable_tc(now)
        ch = int(self.selected_ch.get()[2:])-1
        ir_idx = 0 if self.selected_ir.get() == "Wafer IR" else 1
        selected_tc = tc[2][ch] if tc else math.nan
        self.tc_readout.set(f"{fmt(selected_tc)} °C")
        ir_reading = self._usable_ir(ir_idx, now)
        if tc and ir_reading and math.isfinite(selected_tc) and abs(ir_reading[0]-tc[0]) <= PAIR_SKEW_S:
            offset = ir_reading[1] - selected_tc
        else:
            offset = math.nan
        self.delta_readout.set(f"{fmt(offset,2)} °C")
        avg, std, n = self._offset_stats(ir_idx,ch,now)
        self.avg_readout.set(f"{fmt(avg,2)} °C")
        self.std_readout.set(f"{fmt(std,2)} °C")
        self.n_readout.set(f"N = {n}")
        for i in range(TC_COUNT):
            temp = tc[2][i] if tc else math.nan
            row = [f"CH{i+1:02d}",fmt(temp)]
            for idx in range(2):
                rd = self._usable_ir(idx,now)
                if rd and tc and math.isfinite(temp) and abs(rd[0]-tc[0]) <= PAIR_SKEW_S:
                    row.append(fmt(rd[1] - temp,2))
                else:
                    row.append("--")
            self.tc_table.item(f"ch{i}",values=row)
        self._update_charts(now,ir_idx,ch)

    def _update_charts(self, now, ir_idx, ch):
        cutoff = now - VIEW_WINDOW_S
        samples = [r for r in self.chart_samples if r[0] >= cutoff]
        x = [t-self.started_at for t, _, _, _ in samples]
        if samples:
            wafer = [r[1][0] for r in samples]
            outside = [r[1][1] for r in samples]
            selected_tc = [r[2][ch] for r in samples]
            delta = [r[3][ir_idx][ch] for r in samples]
            rolling = []
            for index, d in enumerate(delta):
                start = samples[index][0] - float(self.window_sec.get())
                vals = [delta[j] for j in range(index+1) if samples[j][0] >= start and math.isfinite(delta[j])]
                rolling.append(safe_mean(vals))
        else:
            wafer, outside, selected_tc, delta, rolling = [], [], [], [], []
        for line, vals in [(self.line_wafer,wafer),(self.line_outer,outside),(self.line_tc,selected_tc),
                           (self.line_offset,delta),(self.line_mean,rolling)]:
            line.set_data(x,vals)
        self.line_tc.set_label(self.selected_ch.get())
        self.line_offset.set_label(self.selected_ir.get() + " - " + self.selected_ch.get())
        x_right = max(VIEW_WINDOW_S, now - self.started_at)
        for ax in (self.temp_ax,self.offset_ax):
            ax.set_xlim(max(0,x_right - VIEW_WINDOW_S),x_right)
        for ax,series,default_limits in [
            (self.temp_ax,[wafer,outside,selected_tc],(0,300)),
            (self.offset_ax,[delta,rolling],(-10,10))
        ]:
            valid = [v for seq in series for v in seq if math.isfinite(v)]
            if valid:
                low,high=min(valid),max(valid)
                pad=max(2.0, (high-low)*0.13)
                ax.set_ylim(low-pad,high+pad)
            else:
                ax.set_ylim(*default_limits)
        self.canvas.draw_idle()

    def clear_data(self):
        self.started_at = time.monotonic()
        self.last_pair_at = 0.0
        self.latest_ir = [None,None]
        self.latest_tc = None
        self.records.clear()
        self.chart_samples.clear()
        for sensor in self.offsets:
            for history in sensor:
                history.clear()
        if hasattr(self,"log_box"):
            self.log("데이터 기록 및 Offset 통계를 초기화했습니다.")

    def save_csv(self):
        if not self.records:
            messagebox.showinfo("저장할 데이터 없음", "아직 유효한 IR/TC 동시 측정 데이터가 없습니다.")
            return
        path = filedialog.asksaveasfilename(
            title="IR/TC Offset 데이터 저장", defaultextension=".csv",
            initialfile=f"IR_TC_Offset_{datetime.now():%Y%m%d_%H%M%S}.csv",
            filetypes=[("CSV 파일","*.csv")]
        )
        if not path:
            return
        header = ["local_timestamp","elapsed_s","wafer_ir_port","outside_ir_port","tc_port","wafer_ir_C","outside_ir_C"]
        header += [f"TC_CH{i+1:02d}_C" for i in range(TC_COUNT)]
        header += [f"WAFER_minus_TC_CH{i+1:02d}_C" for i in range(TC_COUNT)]
        header += [f"OUTSIDE_minus_TC_CH{i+1:02d}_C" for i in range(TC_COUNT)]
        with open(path,"w",newline="",encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.records)
        summary_path = Path(path).with_name(Path(path).stem + "_summary.csv")
        now = time.monotonic()
        with summary_path.open("w",newline="",encoding="utf-8-sig") as f:
            writer=csv.writer(f)
            writer.writerow(["IR_source","TC_channel","rolling_window_s","mean_offset_IR_minus_TC_C","std_offset_C","N"])
            for ir_idx, source in enumerate(("Wafer","Outside")):
                for ch in range(TC_COUNT):
                    mean,std,n=self._offset_stats(ir_idx,ch,now)
                    writer.writerow([source,f"CH{ch+1:02d}",self.window_sec.get(),
                                     round(mean,4) if math.isfinite(mean) else "",
                                     round(std,4) if math.isfinite(std) else "",n])
        self.log(f"CSV 저장: {path} ({len(self.records)}행) / 요약: {summary_path}")
        messagebox.showinfo("저장 완료",f"원본: {path}\n요약: {summary_path}")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="IR 2채널과 TC 20채널 실시간 Offset 분석기")
    parser.add_argument("--demo",action="store_true",help="실제 센서 없이 가상 신호로 화면 테스트")
    args = parser.parse_args()
    root = tk.Tk()
    OffsetViewer(root,demo=args.demo)
    root.mainloop()


if __name__ == "__main__":
    main()
