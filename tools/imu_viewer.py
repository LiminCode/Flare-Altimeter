"""Live 3-D attitude + laser range viewer for the flare-altimeter bring-up.

Reads the CSV stream produced by src/main.cpp and shows three things at once:
the module's body axes rotated into the world frame with the laser beam drawn
along body -Z, the history of the tilt angle theta, and the history of slant
range d against the vertical height h = d * cos(theta) derived from it.

    t_ms,qw,qi,qj,qk,roll,pitch,yaw,tilt,acc,dist_cm,strength,h_cm,corr

dist_cm and h_cm are -1 when the laser has no valid echo, which the viewer draws
as a gap rather than a value -- a missing reading must never look like a small one.

The quaternion already has the stored mounting reference divided out by the
firmware, so "level" on screen means the pose that was calibrated with the reset
button, not the chip's factory frame. `corr` reports whether the firmware is
applying the cos(theta) correction; the toggle button shows that reported state
rather than its own idea of it.

Usage:
    python imu_viewer.py --port COM4
    python imu_viewer.py --demo             # synthetic data, no hardware
    python imu_viewer.py --selftest out.png # render one frame headless
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

CSV_FIELDS = 15
ACC_NAMES = ("unreliable", "low", "medium", "high")
# Index is the firmware's AlertStage enum, and must stay in step with
# lib/alert/alert_engine.h -- the indices are just integers on the wire, so a
# rename on one side shows up as confidently mislabelled output on the other
# rather than as an error. That is how these went stale: the four-band ladder was
# replaced by step beeps, the enum was renamed, and the display kept cheerfully
# printing "CRITICAL" for what the firmware now calls FLARE NOW.
# 中文：索引要和 lib/alert/alert_engine.h 保持同步。枚举在串口上只是几个整数，
#       一边改名另一边不会报错，只会理直气壮地显示错误的名字 ——
#       这几个标签就是这么过期的：分档模型换成步进响声、枚举改了名，
#       而显示这边还在把固件所说的 FLARE NOW 印成 "CRITICAL"。
STAGE_NAMES = ("silent", "far", "closing", "near", "FLARE NOW")
STAGE_COLORS = ("#8a8a8a", "#1a7f37", "#b8860b", "#e07000", "#d62728")
TRAIL_SECONDS = 10.0
EXPECTED_HZ = 50
TILT_GATE_DEG = 40.0      # beyond this the beam points sideways; cos-correction is void
PLOT_MIN_CEILING_CM = 600.0   # never zoom in tighter than this
# Fixed rungs rather than a continuous fit. Autoscaling to the exact peak makes the
# axis twitch on every frame as the rolling window drops old samples, and a twitching
# axis is unreadable precisely while something is happening. Snapping to a ladder
# means the scale changes a handful of times during a descent instead of constantly.
# 中文：用固定的几档而不是连续自适应。按精确峰值缩放会让坐标轴每一帧都在抖 ——
#       滚动窗口每丢掉一个旧样本，上限就变一点 —— 而正在发生事情的时候，
#       一个抖动的坐标轴恰恰是最没法看的。吸附到几档上，
#       一次下降里刻度只变几次，而不是一直变。
PLOT_CEILING_LADDER_CM = (600.0, 1000.0, 1500.0, 2000.0, 3000.0, 4000.0)
PLOT_LOG_FLOOR_CM = 8.0       # a log axis cannot show 0; the sensor floors near 10 cm

# Display state the widgets mutate and the draw code reads.
VIEW = {"yscale": "auto"}     # auto | log | fixed
YSCALE_MODES = ("auto", "log", "fixed")

# The firmware's own configuration, parsed from its `s` status output rather than
# assumed here. Copies of firmware constants go stale silently -- the stage names
# in this file did exactly that, and a wrong number in a UI is worse than no
# number because it is believed. Empty until the board answers.
# 中文：固件的配置从它 `s` 命令的输出里解析出来，而不是在这边写死一份。
#       在上位机复制固件常量会悄悄过期 —— 这个文件里的档位名就是这么错的 ——
#       而界面上一个错的数字比没有数字更糟，因为它会被相信。
FW = {}

# TF02-Pro: 0.1-40 m at 90% reflectivity, 13.5 m worst case over grass. A test
# starting above this is exercising a regime the sensor cannot reach.
SENSOR_MAX_CM = 4000.0


@dataclass
class Sample:
    t: float          # seconds since the board booted
    quat: tuple       # (w, x, y, z)
    rpy: tuple        # degrees
    tilt: float       # degrees from vertical
    acc: int          # 0..3 fusion accuracy
    dist: float       # slant range, cm; NaN when the laser has no valid echo
    strength: int     # laser return strength
    height: float     # d * cos(tilt), cm; NaN when dist is NaN
    corr: bool        # is the firmware applying the cos(tilt) correction?
    stage: int        # firmware AlertStage: 0 silent .. 4 critical


class SampleBuffer:
    """Latest sample plus a bounded history, safe across threads."""

    def __init__(self, maxlen: int):
        self._lock = threading.Lock()
        self._latest: Sample | None = None
        self._history: deque[Sample] = deque(maxlen=maxlen)
        # The display history is a rolling window, deliberately -- the plots show
        # the last few seconds. A replay check needs the WHOLE run, and a 9.4 s
        # descent against a 10 s window is close enough that the opening samples
        # could be evicted before the check reads them. That would fail the
        # "reached 20 m" assertion for a reason that has nothing to do with the
        # firmware. So recording is separate and unbounded, and off by default.
        # 中文：显示用的历史是滚动窗口，这是故意的 —— 图只画最近几秒。
        #       但回放检查需要整段数据，而 9.4 秒的下降对上 10 秒的窗口，
        #       开头那几帧完全可能在检查读到之前就被挤出去了。
        #       那样「到过 20 米」这条断言会因为一个和固件毫无关系的原因失败。
        #       所以录制是另一份、不设上限、默认关闭。
        self._record: list[Sample] | None = None
        # Beep events echoed by the firmware, as (host_time, height_cm, hz).
        # Height cannot show what the ear receives: the design rests on beep RATE
        # carrying descent rate, and a rate can only be checked by counting real
        # beeps against a clock.
        self._beeps: list[tuple[float, float, int]] | None = None
        self.dropped = 0
        self.received = 0

    def start_recording(self) -> None:
        with self._lock:
            self._record = []
            self._beeps = []

    def recording(self) -> list[Sample]:
        with self._lock:
            return list(self._record) if self._record is not None else []

    def push_beep(self, height_cm: float, hz: int) -> None:
        with self._lock:
            if self._beeps is not None:
                self._beeps.append((time.monotonic(), height_cm, hz))

    def beeps(self) -> list[tuple[float, float, int]]:
        with self._lock:
            return list(self._beeps) if self._beeps is not None else []

    def push(self, s: Sample) -> None:
        with self._lock:
            self._latest = s
            self._history.append(s)
            if self._record is not None:
                self._record.append(s)
            self.received += 1

    def snapshot(self):
        with self._lock:
            return self._latest, list(self._history)


def parse_line(line: str) -> Sample | None:
    """Parse one CSV row. Returns None for banner text and partial lines."""
    parts = line.strip().split(",")
    if len(parts) != CSV_FIELDS:
        return None
    try:
        t_ms = int(parts[0])
        qw, qi, qj, qk = (float(v) for v in parts[1:5])
        roll, pitch, yaw, tilt = (float(v) for v in parts[5:9])
        acc = int(parts[9])
        dist = float(parts[10])
        strength = int(parts[11])
        height = float(parts[12])
        corr = parts[13] == "1"
        stage = int(parts[14])
    except ValueError:
        return None                      # header row or garbled frame

    # The firmware sends -1 for "no valid echo". Turning that into NaN here means
    # every plot and readout downstream shows a gap instead of a number, with no
    # special-casing: matplotlib skips NaN, and NaN comparisons are all False.
    # 中文：固件用 -1 表示「这一帧没有有效回波」。在入口处就转成 NaN，
    #       后面所有画图和判断就都不需要再特判 —— matplotlib 会自动断线，
    #       而 NaN 参与任何比较都是 False，不会被误判成「高度很低」。
    if dist < 0:
        dist = height = float("nan")
    return Sample(t_ms / 1000.0, (qw, qi, qj, qk), (roll, pitch, yaw), tilt, acc,
                  dist, strength, height, corr, stage)


def parse_fw_config(line: str) -> None:
    """Pull the firmware's alert parameters out of its `s` status line.

        beep every 50 cm, 700->3500 Hz over 1300..200 cm; continuous below
        200 cm; auto-stop 2 s under 100 cm

    Asking the board rather than duplicating its constants here is the whole
    point: a copy is a thing that can silently disagree, and the display has
    already been caught doing exactly that with the stage names.
    中文：向板子要参数，而不是在这里再抄一份。副本是会悄悄对不上的东西 ——
          这个界面已经在档位名上被抓到过一次了。
    """
    m = re.search(r"beep every ([\d.]+) cm, (\d+)->(\d+) Hz over ([\d.]+)\.\.([\d.]+) cm",
                  line)
    if not m:
        return
    FW["beep_step_cm"] = float(m.group(1))
    FW["pitch_low_hz"] = int(m.group(2))
    FW["pitch_high_hz"] = int(m.group(3))
    FW["pitch_top_cm"] = float(m.group(4))
    FW["changeover_cm"] = float(m.group(5))
    print(f"[viewer] firmware: beep/{FW['beep_step_cm']:.0f}cm, pitch "
          f"{FW['pitch_low_hz']}-{FW['pitch_high_hz']}Hz over "
          f"{FW['pitch_top_cm']:.0f}..{FW['changeover_cm']:.0f}cm", file=sys.stderr)


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Body->world rotation matrix. Columns are the body axes in world coords."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------- data sources

def serial_reader(port: str, baud: int, buf: SampleBuffer, stop: threading.Event,
                  commands: "queue.Queue[bytes]"):
    """Background thread: put the board in CSV mode and consume the stream.

    Buttons in the GUI thread hand single-character firmware commands over via
    `commands`; only this thread ever touches the port. A pyserial port is not
    thread-safe, and a write racing a readline can interleave mid-frame.

    中文：GUI 线程按下按钮时不直接写串口，而是把命令投进队列，由本线程发出去。
    pyserial 的端口对象不是线程安全的，写操作和 readline 撞在一起会插进半帧中间，
    结果是偶发的解析失败 —— 这种 bug 复现率低、极难查，用队列从结构上避免掉。
    """
    import serial                                    # imported late so --demo works without pyserial

    # Open with DTR and RTS held low. On the CH340 (COM4) these are ordinary
    # modem lines and the state does not matter, but on the S3's native
    # USB-Serial-JTAG port they are reset and boot-mode requests wired straight
    # into the chip -- pyserial's default of asserting both on open drops the
    # board out of the running application, and the stream stays silent forever.
    #
    # 中文说明：
    #   pyserial 默认在 open() 时把 DTR 和 RTS 都拉有效。走 CH340（COM4）时这只是
    #   普通的调制解调器控制线，拉不拉都无所谓；但走 ESP32-S3 自带的
    #   USB-Serial-JTAG（COM3）时，这两根线被硬件直接解释成「复位」和「进下载模式」
    #   的请求。默认行为等于一打开端口就把芯片从正在运行的应用里踢出去，
    #   于是端口能开、能写、就是永远读不到任何数据 —— 排查这个症状花了很久，
    #   因为它看起来完全像是固件不出声，而不像是上位机把板子搞停了。
    #   所以这里必须先建对象、把 dtr/rts 设成 False，再 open()，
    #   不能用 serial.Serial(port, baud) 这种「构造即打开」的写法。
    def open_port():
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 1
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    # Survive the board going away. From the battery stage on, unplugging USB is
    # routine, and on the XIAO the port itself disappears with the cable. The
    # first version let the reader thread die on the error, which left the window
    # up, frozen on the last frame, looking alive. Now it waits for the port to
    # come back and re-sends the handshake, because a re-plugged board has
    # rebooted into the human view and will not stream CSV until asked.
    # 中文说明（为什么要自动重连）：
    #   从接电池这一步开始，拔插 USB 是家常便饭；XIAO 的串口会随着线一起消失。
    #   最早的写法遇到串口错误时读线程直接退出，窗口还开着、停在最后一帧，
    #   看起来像还活着 —— 这比直接报错更糟。现在改为等端口回来并重新握手：
    #   重新插上的板子已经重启回到了人类可读视图，不再次发 'c' 它不会输出 CSV。
    first_open = True
    while not stop.is_set():
        try:
            ser = open_port()
        except Exception as exc:
            if first_open:
                # A busy port is the single most common way to start this tool
                # wrong. Say so plainly and stop, rather than retrying forever
                # against a port another program holds.
                # 中文：启动时端口被占是最常见的用法错误。直接说清楚并退出，
                #       而不是对着一个被别的程序占着的端口无限重试。
                print(f"[viewer] cannot open {port}: {exc}", file=sys.stderr)
                print("[viewer] something else holds the port -- another viewer"
                      " window, or 'pio device monitor'. Close it and retry.",
                      file=sys.stderr)
                stop.set()
                return
            time.sleep(1.0)
            continue

        with ser:
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b"s\n")        # ask for the firmware's alert configuration
            ser.write(b"c\n")        # then switch to the CSV stream
            ser.flush()
            print(f"[viewer] {'reading' if first_open else 'reconnected to'}"
                  f" {port} @ {baud}", file=sys.stderr)
            first_open = False
            last_sample = time.monotonic()
            last_handshake = last_sample
            while not stop.is_set():
                try:
                    while True:
                        ser.write(commands.get_nowait())
                        ser.flush()
                except queue.Empty:
                    pass
                except Exception:
                    break                            # unplugged while writing
                try:
                    raw = ser.readline().decode("utf-8", errors="replace")
                except Exception as exc:             # unplugged mid-run
                    print(f"[viewer] lost {port} ({exc.__class__.__name__});"
                          " waiting for it to come back...", file=sys.stderr)
                    break

                # Port alive but no CSV for 2 s: the board was reset (EN button,
                # a flash, a brown-out) and came back in the human view. Ask again.
                # 中文：端口还在但 2 秒没收到 CSV：板子被复位过（按 EN、刷固件、
                #       掉电），重启后回到了人类可读视图，重新发一次握手。
                now = time.monotonic()
                if now - last_sample > 2.0 and now - last_handshake > 2.0:
                    try:
                        ser.write(b"s\nc\n")
                        ser.flush()
                    except Exception:
                        break
                    last_handshake = now

                if not raw:
                    continue
                s = parse_line(raw)
                if s is None:
                    if "beep every" in raw:
                        parse_fw_config(raw)
                    elif "beep" in raw:
                        # "  beep    1100 cm   938 Hz   (t+0.44 s)"
                        parts = raw.split()
                        try:
                            buf.push_beep(float(parts[1]), int(parts[3]))
                        except (IndexError, ValueError):
                            buf.dropped += 1
                    elif raw.strip():
                        buf.dropped += 1
                else:
                    last_sample = time.monotonic()
                    buf.push(s)
        if not stop.is_set():
            time.sleep(1.0)


def load_mock(path: str) -> tuple[list[tuple[int, int, int]], dict[str, str]]:
    """Read a mock descent: t_ms,dist_cm,strength, plus `# key: value` metadata.

    The expectations travel with the data. What counts as a good run differs per
    scenario -- gaps are required in the hostile file and forbidden in the clean
    one -- so hard-coding a single notion of "correct" in the checker would make
    every new scenario a code change, and would quietly test the wrong thing.
    中文：期望值跟着数据走。什么算「跑得对」每个场景都不一样 ——
          断线在恶劣场景里是必须出现的，在完美场景里是不能出现的。
          把单一的「正确」写死在检查代码里，等于每加一个场景都要改代码，
          而且会悄悄地在测错的东西。
    """
    rows: list[tuple[int, int, int]] = []
    meta: dict[str, str] = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                body = line.lstrip("#").strip()
                if ":" in body:
                    k, v = body.split(":", 1)
                    k = k.strip()
                    if k.startswith("expect_") or k in ("scenario", "description"):
                        meta[k] = v.strip()
                continue
            if not line or line.startswith("t_ms"):
                continue
            t, d, s = line.split(",")
            rows.append((int(t), int(d), int(s)))
    return rows, meta


def custom_scenario(start_m: float, speed_ms: float, flare_m: float):
    """Build a descent from three numbers typed into the UI.

    Reuses the generator that produces the stored scenarios, so a hand-entered run
    and a canned one differ only in their parameters -- if they diverged, a bug
    could hide in whichever path was used less.
    中文：复用生成固定场景的那个函数，于是手输的运行和预置的运行只有参数不同。
          两条路径一旦不同，bug 就会藏在用得少的那一条里。
    """
    import make_mock_descent as mmd

    start_cm = max(50.0, start_m * 100.0)
    rows, _ = mmd.build("custom", start_cm=start_cm,
                        cruise_cm_s=max(20.0, speed_ms * 100.0),
                        flare_from_cm=max(50.0, flare_m * 100.0),
                        noise_cm=2.0, ground_dwell_s=4.0)

    # Expectations derived from the parameters, not copied from a preset. Stage
    # order is deliberately NOT asserted: whether the 'far' band is entered
    # depends on the start height sitting either side of 10 m, and the tilt
    # correction moves it -- the same trap that made the 10 m preset fail once.
    # 中文：期望值由参数推出来，不是从预置里抄的。**故意不断言档位顺序**：
    #       会不会经过 far 档取决于起始高度落在 10 m 的哪一侧，而倾角修正还会挪动它 ——
    #       10 m 那个预置场景就是这样错过一次的。
    expect = {
        "expect_flare": "yes",
        "expect_gaps": "no",
        "expect_peak_cm_min": str(int(start_cm * 0.85)),
        "expect_floor_cm_max": "60",
        "expect_flare_above_cm": "none",
        "expect_arm": "yes" if start_cm > 850 else "no",
    }
    meta = dict(expect)
    meta["scenario"] = "custom"
    meta["description"] = (f"{start_m:.1f} m at {speed_ms:.1f} m/s, "
                           f"changeover {flare_m:.2f} m")
    return rows, meta


def run_scenarios(items, commands, stop, buf, result, status=None):
    """Feed one or more descents to the board, checking each as it completes.

    This runs inside the viewer rather than as a separate script for a blunt
    reason: a serial port allows exactly one reader, so a second process feeding
    the board would have to take the port away from the display -- and then the
    graph everyone wants to watch would be dead for the whole run.

    中文说明（为什么喂数据的必须是查看器自己）：
      串口同一时刻只允许一个进程打开。另起脚本喂数据就得把端口从查看器手里抢走，
      那样整场回放里最想看的那张图恰好是黑的。所以走查看器已有的命令队列。
    """
    def say(msg):
        print(msg, file=sys.stderr)
        if status is not None:
            status(msg)

    failures = 0
    for index, (rows, meta, changeover_cm) in enumerate(items):
        if stop.is_set():
            break
        name = meta.get("scenario", "?")

        # Between scenarios, stop feeding long enough for the board to time out of
        # replay, disarm and return to live data. Without this pause the next run
        # starts from whatever state the previous one left behind -- exactly the
        # class of leftover-state bug that cost a beep earlier.
        # 中文：两个场景之间要停够久，让板子回放超时、撤防、回到实况数据。
        #       否则下一场会从上一场遗留的状态开始 ——
        #       而「残留状态」正是之前已经吃掉过一声 beep 的那类 bug。
        if index > 0:
            time.sleep(2.5)

        # Pin the changeover height so a run means the same thing every time. The
        # presets' expectations were written against 200 cm, and the setting is
        # persisted in NVS -- so without this, one custom run would silently
        # change what every later preset run is measuring.
        # 中文：每次运行前钉住转换高度，好让同一个场景每次含义相同。
        #       预置场景的期望值是按 200 cm 写的，而这个设置存在 NVS 里 ——
        #       不钉的话，跑过一次自定义之后，后面每个预置场景测的都是别的东西了。
        commands.put(b"Y%d\n" % int(changeover_cm))
        time.sleep(0.3)

        total_s = rows[-1][0] / 1000.0
        say(f"{name}: {len(rows)} frames, {total_s:.0f} s")
        if meta.get("description"):
            say(f"  {meta['description']}")

        # A run is paced from the file's own timestamps, so a slow descent from a
        # great height takes minutes of wall clock. Without progress the window
        # looks identical to a run that never started -- which is exactly how a
        # 107 s custom test got reported as "no different from the 9 s preset".
        # 中文：回放按数据自己的时间轴走，所以「高起点 + 慢下降」在真实世界里要跑几分钟。
        #       没有进度显示时，窗口和「根本没启动」长得一模一样 ——
        #       一次 107 秒的自定义测试就是这样被当成「和 9 秒的预置没区别」的。
        buf.start_recording()
        t0 = time.monotonic()
        next_report = 5.0
        for t_ms, dist_cm, strength in rows:
            if stop.is_set():
                break
            # Keep to the file's own timeline, so descent rate -- and therefore
            # the beep rate the whole design rests on -- is reproduced faithfully.
            elapsed = time.monotonic() - t0
            wait = (t_ms / 1000.0) - elapsed
            if wait > 0:
                time.sleep(wait)
            commands.put(b"X%d,%d\n" % (dist_cm, strength))
            if elapsed >= next_report:
                next_report += 5.0
                say(f"  {elapsed:5.0f}/{total_s:.0f} s   h={dist_cm/100.0:6.2f} m")

        # Check as soon as the data has been fed, not when the window is closed.
        # The window is there to be looked at for as long as the operator wants;
        # making pass/fail wait on that would mean the automated part of this test
        # only completes if a human remembers to click something.
        #
        # 0.3 s is long enough for the CSV rows still in flight and deliberately
        # SHORTER than the firmware's 500 ms staleness threshold: waiting longer
        # spans that window and then the 1 s replay timeout, after which live desk
        # data returns, sandwiching an artificial run of invalid samples between
        # two valid ones. That made every clean scenario report ~26 phantom
        # dropouts it had no source for.
        # 中文：0.3 秒只等在途的 CSV 行，而且**故意短于**固件 500 ms 的过期阈值。
        #       等更久会跨过过期窗口、再跨过 1 秒回放超时，桌面实况数据又回来，
        #       于是一段人造的无效样本被夹在两段有效数据中间 ——
        #       每个干净场景都因此报出约 26 个没有来源的「丢帧」。
        time.sleep(0.3)
        rc = check_replay(buf, meta, not stop.is_set())
        failures += rc
        say(f"[replay] {name}: {'PASS' if rc == 0 else 'FAIL'}")

    total = len(items)
    say(f"=== {total - failures}/{total} scenarios passed ===")
    result.append(1 if failures else 0)


def demo_reader(buf: SampleBuffer, stop: threading.Event):
    """Synthetic motion so the display can be exercised without hardware."""
    t0 = time.time()
    while not stop.is_set():
        t = time.time() - t0
        # Slow yaw sweep plus a gentle nod, roughly what a swinging foot does.
        yaw = math.radians(40.0 * t)
        pitch = math.radians(30.0 * math.sin(t * 1.5))
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        w, x, y, z = cy * cp, cy * sp, sy * cp, sy * sp
        r33 = w * w - x * x - y * y + z * z
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, r33))))
        # A descent from 5 m to touchdown, with a two-second dropout partway to
        # exercise the "no echo" path -- the case most likely to be mishandled.
        # 中文：模拟从 5 米下降到落地，中间故意插一段两秒的丢失回波，
        #       专门用来验证「没有读数」这条路径画出来是断开而不是掉到 0。
        d_true = max(20.0, 500.0 - 60.0 * (t % 12.0))
        dropout = 6.0 < (t % 12.0) < 8.0
        dist = float("nan") if dropout else d_true / max(0.2, math.cos(math.radians(tilt)))
        height = dist * math.cos(math.radians(tilt))
        buf.push(Sample(t, (w, x, y, z),
                        (math.degrees(pitch), 0.0, math.degrees(yaw)), tilt, 3,
                        dist, 0 if dropout else 1500, height, True,
                        0 if dropout else min(4, int(max(0, 500 - height) // 100))))
        time.sleep(1.0 / EXPECTED_HZ)


# ------------------------------------------------------------------ rendering

def build_figure():
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13.5, 8.0))
    fig.canvas.manager.set_window_title("Flare Altimeter - attitude + laser range")
    # The 3-D view spans both rows: it is the panel you actually look at while
    # waving the sensor around, the two histories are there to be glanced at.
    gs = fig.add_gridspec(2, 2, width_ratios=(1.15, 1.0), height_ratios=(1, 1))
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    axtilt = fig.add_subplot(gs[0, 1])
    axdist = fig.add_subplot(gs[1, 1])
    # bottom=0.24 reserves the control strip. The plots lose height for it, which
    # is the right trade: a test you can start without retyping a command line is
    # a test that gets run.
    # 中文：底部留出 0.24 给控制条。图变矮了，但这个取舍是对的 ——
    #       不用重新敲命令行就能启动的测试，才是真的会被跑的测试。
    fig.subplots_adjust(left=0.01, right=0.95, top=0.91, bottom=0.24,
                        wspace=0.20, hspace=0.42)
    return fig, ax3d, axtilt, axdist


def _range_ylimits(history: list[Sample]) -> tuple[str, float, float]:
    """Pick the y scale for the range panel.

    A descent spans 30 m to 10 cm, and the decisions all happen in the bottom two
    metres. A linear axis tall enough to show the top compresses the part that
    matters into a few pixels; one scaled to the bottom throws the top away. The
    two answers offered here are: grow linearly in fixed steps (readable, honest
    about proportion), or switch to log (both ends visible at once, at the cost
    of the eye no longer reading distance as distance).

    中文：一次下降跨越 30 米到 10 厘米，而所有决定都发生在最后两米。
      线性轴要装得下顶部，最要紧的那一段就被压成几个像素；按底部缩放又看不到顶部。
      这里给出两个答案：按固定档位线性放大（好读，比例是诚实的），
      或者切到对数轴（两端同时可见，代价是眼睛不再能把长度直接读成距离）。
    """
    mode = VIEW["yscale"]
    if mode == "log":
        peak = PLOT_MIN_CEILING_CM
        for s in history:
            for v in (s.dist, s.height):
                if math.isfinite(v):
                    peak = max(peak, v)
        return "log", PLOT_LOG_FLOOR_CM, max(peak * 1.3, 1000.0)

    if mode == "fixed" or not history:
        return "linear", 0.0, PLOT_MIN_CEILING_CM

    peak = 0.0
    for s in history:
        for v in (s.dist, s.height):
            if math.isfinite(v):
                peak = max(peak, v)
    for rung in PLOT_CEILING_LADDER_CM:
        if peak * 1.12 <= rung:
            return "linear", 0.0, rung
    # Past the top rung, fit rather than clip. Clipping silently hides data, and
    # a plot that omits the part you were looking for without saying so is the
    # worst possible failure mode for a diagnostic tool.
    # 中文：超出最高一档就按数据自适应，而不是裁掉。裁剪是无声的 ——
    #       一张把你正要看的那段省掉、却不告诉你的图，
    #       对一个诊断工具来说是最糟糕的失效方式。
    return "linear", 0.0, peak * 1.12


class LiveView:
    """Every artist on the figure, created once and then only updated.

    The first version of this viewer called ax.clear() on all three panels every
    frame and drew everything again: quivers, labels, grids, legend, reference
    lines. Measured 2026-10-03 at 137 ms per frame -- a ceiling of 7 fps, which
    is what made the 3-D view visibly step while the sensor was being moved.
    Almost all of that time was spent rebuilding things that never change. Now
    the static parts (limits, labels, grids, the gate line, the legend) are set
    up once, and a frame only moves the line data and changes the text.

    中文说明（为什么要改成「建一次、只更新」）：
      最早的写法每一帧都对三个面板 ax.clear()，然后从头再画一遍：箭头、文字、
      网格、图例、参考线全部重建。2026-10-03 实测每帧 137 ms，上限只有 7 fps ——
      这就是移动传感器时 3D 视图一顿一顿的原因。
      这些时间几乎全花在重建「根本不会变的东西」上。现在坐标范围、轴标签、
      网格、40° 判据线、图例这些静态部分只在开始时设置一次，
      每一帧只改线条的数据和文字内容。
    """

    AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")     # X red, Y green, Z blue
    AXIS_LABELS = ("+X", "+Y", "+Z")
    BEAM_COLOR = "#ff7f0e"

    def __init__(self, fig, ax3d, axtilt, axdist):
        self.fig, self.ax3d, self.axtilt, self.axdist = fig, ax3d, axtilt, axdist
        self._yscale_state = None
        self._setup_banner()
        self._setup_3d()
        self._setup_tilt()
        self._setup_range()

    # ---------------------------------------------------------------- setup

    def _setup_banner(self):
        """One big word for what the buzzer is doing right now."""
        self.banner = self.fig.text(0.72, 0.955, "alert: --", ha="center",
                                    va="center", fontsize=15, family="monospace",
                                    fontweight="bold", color="#8a8a8a")

    def _setup_3d(self):
        ax = self.ax3d
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("world X")
        ax.set_ylabel("world Y")
        ax.set_zlabel("world Z (up)")
        # World vertical: theta is measured against this.
        ax.plot([0, 0], [0, 0], [0, 1.15], color="0.6", linestyle=":", linewidth=1)

        # Body axes as plain 3-D lines with a dot at the tip. A quiver cannot have
        # its vectors moved after creation, so it would have to be destroyed and
        # rebuilt every frame -- exactly the cost this class exists to avoid.
        # 中文：机身三轴用普通 3D 线段加端点圆点。quiver 箭头创建后无法移动，
        #       只能每帧销毁重建 —— 那正是这个类要省掉的开销。
        self.axis_lines, self.axis_tips, self.axis_texts = [], [], []
        for color, label in zip(self.AXIS_COLORS, self.AXIS_LABELS):
            (line,) = ax.plot([0, 0], [0, 0], [0, 0], color=color, linewidth=2.5)
            (tip,) = ax.plot([0], [0], [0], marker="o", markersize=6, color=color,
                             linestyle="none")
            text = ax.text(0, 0, 0, label, color=color, fontsize=9, ha="center")
            self.axis_lines.append(line)
            self.axis_tips.append(tip)
            self.axis_texts.append(text)

        # The laser fires along body -Z, so the beam is the +Z axis mirrored. Drawn
        # at unit length regardless of the real range, which keeps it in frame and
        # is still geometrically honest: with |beam| = 1 the tip sits exactly
        # cos(theta) = h/d below the origin, so the dashed drop and the ground line
        # picture the correction itself, not a to-scale map.
        # 中文：激光沿 -Z 打出，光束就是 +Z 轴取反。一律画成单位长度：
        #       这时光束末端相对原点的垂直落差恰好等于 cos(theta) = h/d，
        #       那根竖直虚线和地面线画的就是「斜距换算成高度」本身。
        c = self.BEAM_COLOR
        (self.beam,) = ax.plot([0, 0], [0, 0], [0, 0], color=c, linewidth=2.0)
        (self.beam_tip,) = ax.plot([0], [0], [0], marker="v", markersize=7,
                                   color=c, linestyle="none")
        (self.beam_drop,) = ax.plot([0, 0], [0, 0], [0, 0], color=c,
                                    linestyle=":", linewidth=1.0)
        (self.ground,) = ax.plot([-1.1, 1.1], [0, 0], [0, 0], color="0.5",
                                 linestyle="--", linewidth=0.9)
        self.beam_text = ax.text(0, 0, 0, "beam", color=c, fontsize=9, ha="center")
        self.beam_parts = (self.beam, self.beam_tip, self.beam_drop, self.ground,
                           self.beam_text)
        self.title3d = ax.set_title("waiting for data...", fontsize=10,
                                    family="monospace")

    def _setup_tilt(self):
        ax = self.axtilt
        ax.set_ylim(0, 180)
        ax.set_xlim(-TRAIL_SECONDS, 0.5)
        ax.set_ylabel("theta from vertical (deg)")
        ax.grid(alpha=0.3)
        # Above this the laser points sideways and cos-correction is invalid.
        ax.axhline(TILT_GATE_DEG, color="#d62728", linestyle="--", linewidth=1)
        ax.text(0.01, TILT_GATE_DEG + 3, "40deg reject gate", color="#d62728",
                fontsize=8, transform=ax.get_yaxis_transform())
        (self.tilt_line,) = ax.plot([], [], color="#1f77b4", linewidth=1.4)
        ax.set_title("theta history -- feeds h = d x cos(theta)", fontsize=10)

    def _setup_range(self):
        ax = self.axdist
        ax.set_xlim(-TRAIL_SECONDS, 0.5)
        ax.set_xlabel("seconds")
        # The stage boundaries the firmware actually uses (alert_engine
        # stageForHeight). A reference line that is wrong is worse than none,
        # because it invites you to check against it.
        # 中文：固件真正在用的档位边界。画错的参考线比没有更糟，因为它会诱使你去核对。
        for edge in (1000.0, 500.0):
            ax.axhline(edge, color="0.75", linestyle=":", linewidth=0.8)

        # The changeover height is inferred from the data -- the highest height
        # at which the board reported FLARE NOW -- so it cannot go stale when the
        # setting is changed with '+' / '-' or the flare box. Hidden until seen.
        # 中文：转换高度从数据里推断（板子报出 FLARE NOW 的最高高度），
        #       改了设置之后这条线也不会变成谎话。没观测到之前隐藏。
        self.changeover = ax.axhline(0, color="#d62728", linestyle="--",
                                     linewidth=1.0, visible=False)
        self.changeover_text = ax.text(0.01, 0, " changeover", color="#d62728",
                                       fontsize=8, va="bottom", visible=False,
                                       transform=ax.get_yaxis_transform())

        # NaN gaps are the point: a dropped echo must read as absent, not as low.
        (self.d_line,) = ax.plot([], [], color="#7f7f7f", linewidth=1.0,
                                 label="d slant range")
        (self.h_line,) = ax.plot([], [], color=self.BEAM_COLOR, linewidth=1.8,
                                 label="h vertical")
        ax.legend(loc="upper left", fontsize=8, framealpha=0.85)
        self.range_title = ax.set_title("range: no data", fontsize=11,
                                        color="#d62728", family="monospace")
        self._apply_yscale(("linear", 0.0, PLOT_MIN_CEILING_CM))

    def _apply_yscale(self, state):
        # set_yscale rebuilds the axis' locators and is far from free, so it only
        # runs when the ladder actually moves to another rung.
        # 中文：set_yscale 会重建刻度定位器，开销不小，只在档位真的变化时才调用。
        if state == self._yscale_state:
            return
        scale, lo, hi = state
        ax = self.axdist
        if self._yscale_state is None or self._yscale_state[0] != scale:
            ax.set_yscale(scale)
            ax.set_ylabel("cm" + (" (log)" if scale == "log" else ""))
            ax.grid(False, which="both")
            ax.grid(alpha=0.3, which="both" if scale == "log" else "major")
        ax.set_ylim(lo, hi)
        self._yscale_state = state

    # --------------------------------------------------------------- update

    def update(self, latest: Sample | None, history: list[Sample]):
        self._update_banner(latest)
        self._update_3d(latest)
        self._update_tilt(history)
        self._update_range(latest, history)

    def _update_banner(self, latest):
        if latest is None:
            self.banner.set_text("alert: --")
            self.banner.set_color("#8a8a8a")
            return
        stage = max(0, min(len(STAGE_NAMES) - 1, latest.stage))
        self.banner.set_text(f"alert: {STAGE_NAMES[stage]}")
        self.banner.set_color(STAGE_COLORS[stage])

    def _update_3d(self, latest):
        if latest is None:
            self.title3d.set_text("waiting for data...")
            return
        R = quat_to_matrix(*latest.quat)
        for i in range(3):
            vx, vy, vz = R[:, i]
            self.axis_lines[i].set_data_3d([0, vx], [0, vy], [0, vz])
            self.axis_tips[i].set_data_3d([vx], [vy], [vz])
            self.axis_texts[i].set_position_3d((vx * 1.18, vy * 1.18, vz * 1.18))

        has_beam = math.isfinite(latest.dist)
        for part in self.beam_parts:
            part.set_visible(has_beam)
        if has_beam:
            bx, by, bz = -R[:, 2]
            drop = -math.cos(math.radians(latest.tilt))      # tip height = h/d
            self.beam.set_data_3d([0, bx], [0, by], [0, bz])
            self.beam_tip.set_data_3d([bx], [by], [bz])
            self.beam_drop.set_data_3d([bx, bx], [by, by], [bz, drop])
            self.ground.set_data_3d([-1.1, 1.1], [0, 0], [drop, drop])
            self.beam_text.set_position_3d((bx * 1.1, by * 1.1, bz * 1.1))

        roll, pitch, yaw = latest.rpy
        self.title3d.set_text(
            f"tilt theta = {latest.tilt:5.1f}deg    fusion: {ACC_NAMES[latest.acc & 3]}\n"
            f"roll {roll:+6.1f}  pitch {pitch:+6.1f}  yaw {yaw:+6.1f}")

    def _update_tilt(self, history):
        if not history:
            self.tilt_line.set_data([], [])
            return
        t_end = history[-1].t
        self.tilt_line.set_data([s.t - t_end for s in history],
                                [s.tilt for s in history])

    def _update_range(self, latest, history):
        self._apply_yscale(_range_ylimits(history))

        if history:
            t_end = history[-1].t
            ts = [s.t - t_end for s in history]
            self.d_line.set_data(ts, [s.dist for s in history])
            self.h_line.set_data(ts, [s.height for s in history])
        else:
            self.d_line.set_data([], [])
            self.h_line.set_data([], [])

        flare_heights = [s.height for s in history
                         if s.stage == 4 and math.isfinite(s.height)]
        seen = bool(flare_heights)
        self.changeover.set_visible(seen)
        self.changeover_text.set_visible(seen)
        if seen:
            top = max(flare_heights)
            self.changeover.set_ydata([top, top])
            self.changeover_text.set_y(top)

        if latest is None or not math.isfinite(latest.dist):
            # "Not talking to us" and "talking but sees nothing usable" are
            # different wiring problems with different fixes.
            # 中文：「激光没和我们说话」和「说话了但没看到可用回波」是两种不同的
            #       接线问题，修法也不同，必须分开显示。
            why = "no data" if latest is None else (
                "laser OFFLINE" if latest.strength == 0 else
                f"no echo (strength {latest.strength})")
            self.range_title.set_text(f"range: {why}")
            self.range_title.set_color("#d62728")
            return

        # With the correction off the firmware returns the raw slant range as
        # height, so h and d coincide -- which otherwise looks exactly like "the
        # unit happens to be level", the wrong conclusion. Say so in the title.
        # 中文：修正关掉时 h 和 d 完全重合，看起来和「设备正好水平」一模一样 ——
        #       这恰恰是最不该得出的结论，所以标题里必须写明。
        gated = latest.tilt > TILT_GATE_DEG and latest.corr
        if not latest.corr:
            note, color = "   [NO TILT CORR: h = d]", "#b34700"
        elif gated:
            note, color = "   [TILT GATED]", "#d62728"
        else:
            note, color = "", "#333333"
        self.range_title.set_text(
            f"h = {latest.height / 100.0:5.2f} m   (d = {latest.dist / 100.0:5.2f} m,"
            f" S = {latest.strength}){note}")
        self.range_title.set_color(color)


    def dynamic_artists(self) -> list:
        """Everything update() changes. The rest of the figure is static."""
        return [self.banner, self.title3d, *self.axis_lines, *self.axis_tips,
                *self.axis_texts, *self.beam_parts, self.tilt_line,
                self.changeover, self.changeover_text, self.d_line, self.h_line,
                self.range_title]


class Blitter:
    """Repaint only the moving artists over a cached picture of everything else.

    Rebuilding the artists instead of clearing the axes took a frame from 137 ms
    to 87 ms, still only 11 fps. Profiling showed why: each panel spends 20-33 ms
    re-rendering its own tick labels, grid and axis text, which do not change
    between frames either. So the figure is rendered in full once, that picture is
    kept, and a frame restores it and draws just the dynamic artists on top.

    A full redraw still happens whenever something static changes: the y-scale
    rung, a button label, the test-runner status text. The caller detects that and
    asks for one with draw_idle(); the draw_event it fires recaptures the
    background. Window resizes and mouse-rotating the 3-D view go through the same
    draw_event, so they are covered without any special case.

    中文说明（为什么还要再加一层 blit）：
      光是「建一次、只更新」只把每帧从 137 ms 降到 87 ms，还是只有 11 fps。
      剖析发现：每个面板每帧仍要花 20~33 ms 重新渲染自己的刻度数字、网格和轴标签，
      而这些东西帧与帧之间同样不变。所以先完整渲染一次，把这张图存下来，
      之后每帧只把它贴回去，再在上面画那几根会动的线和几行会变的字。
      静态部分真的变了（纵轴换档、按钮文字、测试状态栏）时，调用方用 draw_idle()
      要一次完整重绘，重绘触发的 draw_event 会重新截取背景。
      拖动窗口大小、用鼠标转 3D 视角也走同一个 draw_event，不需要单独处理。
    """

    def __init__(self, fig, artists):
        self.fig = fig
        self.canvas = fig.canvas
        self.artists = artists
        self.background = None
        for a in artists:
            a.set_animated(True)
        self._cid = self.canvas.mpl_connect("draw_event", self._on_draw)

    def _on_draw(self, _event):
        self.background = self.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_artists()

    def _draw_artists(self):
        for a in self.artists:
            self.fig.draw_artist(a)

    def blit(self):
        if self.background is None:
            self.canvas.draw_idle()        # first frame: nothing cached yet
            return
        self.canvas.restore_region(self.background)
        self._draw_artists()
        self.canvas.blit(self.fig.bbox)


def draw_frame(ax3d, axtilt, axdist, latest: Sample | None, history: list[Sample]):
    # The view is parked on the figure so every caller -- the live loop and the
    # headless --selftest -- shares one set of artists per figure.
    # 中文：LiveView 挂在 figure 上，实时窗口和无界面的 --selftest 共用同一套图元。
    fig = ax3d.figure
    view = getattr(fig, "_flare_view", None)
    if view is None:
        view = LiveView(fig, ax3d, axtilt, axdist)
        fig._flare_view = view
    view.update(latest, history)


def ensure_gui_backend() -> None:
    """Select an interactive backend, or explain exactly how to get one.

    The venv is built from PlatformIO's bundled Python, which ships without
    tkinter, so matplotlib silently falls back to the file-only 'agg' backend
    and plt.show() draws nothing. Fail loudly instead.
    """
    import matplotlib

    for backend in ("QtAgg", "TkAgg"):
        try:
            matplotlib.use(backend)
            return
        except Exception:
            continue

    raise SystemExit(
        "No interactive matplotlib backend available (current: "
        f"{matplotlib.get_backend()}).\n"
        "This Python has no tkinter, so install Qt into the venv:\n"
        '  & "$env:USERPROFILE\\.venvs\\flare-imu\\Scripts\\python.exe" '
        "-m pip install pyside6\n"
        "Or render a single frame to a file instead:  --selftest frame.png"
    )


def run_live(buf: SampleBuffer, stop: threading.Event, interval_ms: int,
             commands: "queue.Queue[bytes] | None",
             replay_items=None, replay_result=None):
    ensure_gui_backend()
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, TextBox

    fig, ax3d, axtilt, axdist = build_figure()

    # "Set level here" stores the pose the unit is in right now as its mounting
    # reference, in the board's flash. That is the install-time calibration: the
    # bracket, the boot and the ankle all add a fixed angle that nothing else can
    # measure. Put the unit in its real mounted pose before pressing -- the click
    # declares that pose to be level, and a crooked reference biases every later
    # height by cos(error), silently and forever.
    #
    # 中文说明：
    #   这个按钮把「此刻的姿态」存成安装基准，写进板子的闪存。这就是装机标定：
    #   支架、靴子、脚踝一共加了多少固定角度，没有别的办法能测出来，只能装好之后
    #   现场记一次。
    #   按之前必须让设备处于真实的安装姿态 —— 按下的那一刻你是在宣布「这个姿势是水平」。
    #   基准歪了，之后每一次高度都会乘上一个 cos(误差) 的系统性偏差，而且不会报错，
    #   看上去一切正常。
    def send(cmd: bytes, what: str):
        if commands is None:
            print(f"[viewer] demo mode: {what} not sent", file=sys.stderr)
            return
        commands.put(cmd)
        print(f"[viewer] {what}", file=sys.stderr)

    axreset = fig.add_axes((0.40, 0.145, 0.115, 0.05))
    btn_reset = Button(axreset, "Set level here", color="#e8e8e8",
                       hovercolor="#cfe3ff")
    btn_reset.label.set_fontsize(9)
    btn_reset.on_clicked(lambda _e: send(b"t\n", "mounting reference stored"))

    # The toggle shows the state the firmware reports, not a copy kept here, and
    # each click sends an explicit on or off rather than "flip". Both choices
    # exist so the button cannot drift out of step with the board -- after a
    # reboot, a dropped byte or a second viewer, a local flag would lie about
    # which formula is actually producing the height on screen.
    #
    # 中文说明：
    #   这个开关显示的是固件回报的状态，不是本地自己记的一份；每次点击发的是明确的
    #   「开」或「关」，而不是「翻转」。两个决定都是为了同一件事：让按钮不可能和板子
    #   失去同步。板子重启过、命令丢了一个字节、或者另开了一个查看器 —— 任何一种情况下，
    #   本地标志都会开始说谎，而它说的谎正好是「屏幕上这个高度是用哪个公式算出来的」。
    axcorr = fig.add_axes((0.40, 0.085, 0.115, 0.05))
    btn_corr = Button(axcorr, "tilt corr: ?", color="#e8e8e8", hovercolor="#cfe3ff")
    btn_corr.label.set_fontsize(9)
    corr_state: dict[str, bool | None] = {"on": None}   # as last reported by the board

    def on_corr(_event):
        if corr_state["on"] is None:
            print("[viewer] no data yet -- cannot tell which way to toggle",
                  file=sys.stderr)
            return
        want_on = not corr_state["on"]
        send(b"1\n" if want_on else b"0\n",
             f"tilt correction {'ON' if want_on else 'OFF'}")

    btn_corr.on_clicked(on_corr)

    # --- y scale ----------------------------------------------------------
    axyscale = fig.add_axes((0.40, 0.025, 0.115, 0.05))
    btn_yscale = Button(axyscale, f"y: {VIEW['yscale']}", color="#e8e8e8",
                        hovercolor="#cfe3ff")
    btn_yscale.label.set_fontsize(9)

    def on_yscale(_event):
        VIEW["yscale"] = YSCALE_MODES[
            (YSCALE_MODES.index(VIEW["yscale"]) + 1) % len(YSCALE_MODES)]
        btn_yscale.label.set_text(f"y: {VIEW['yscale']}")

    btn_yscale.on_clicked(on_yscale)

    # --- test runner ------------------------------------------------------
    # Being able to start a test from the window matters more than it looks: a
    # test that needs a command line retyped is a test that gets skipped when you
    # are halfway through moving a sensor around with both hands.
    # 中文：能从窗口里直接开跑，比看上去更重要 ——
    #       一个需要重新敲命令行的测试，在你两只手正忙着摆弄传感器的时候就会被跳过。
    scenario_names = ["perfect", "accel", "bad", "noarm", "custom"]
    axscen = fig.add_axes((0.02, 0.02, 0.10, 0.175))
    axscen.set_title("test case", fontsize=9, pad=4)
    radio = RadioButtons(axscen, scenario_names, active=0)
    for lbl in radio.labels:
        lbl.set_fontsize(8)

    def _mkbox(rect, label, initial):
        ax = fig.add_axes(rect)
        box = TextBox(ax, label, initial=initial, textalignment="center")
        box.label.set_fontsize(8)
        box.text_disp.set_fontsize(8)

        # matplotlib 3.11.1 wires TextBox._resize to 'resize_event', but _resize
        # carries the mouse-event decorator that reads event.inaxes -- which a
        # ResizeEvent does not have. Every window resize therefore prints a
        # traceback. matplotlib catches it and nothing breaks, but six tracebacks
        # at startup is visually indistinguishable from something being actually
        # wrong, and a log you learn to ignore is a log that hides the next real
        # failure. Its only job is repositioning the text cursor after a resize.
        #
        # Guarded by the cid count: if a future matplotlib changes what TextBox
        # connects, do nothing rather than disconnect the wrong handler and break
        # typing in a way that would be very confusing to trace back to here.
        #
        # 中文：matplotlib 3.11.1 把 TextBox._resize 挂到了 resize_event 上，
        #       但 _resize 带着读 event.inaxes 的鼠标事件装饰器，而 ResizeEvent
        #       没有这个属性。于是每次窗口尺寸变化都打一次 traceback。
        #       它被 matplotlib 捕获，功能没坏 —— 但一屏 traceback 和「真的出事了」
        #       在视觉上分不出来，而一个你学会忽略的日志，会藏住下一个真正的故障。
        #       那个回调的唯一职责是在缩放后重新摆放文本光标。
        #       用 cid 数量做保护：万一以后 matplotlib 改了 TextBox 注册的事件，
        #       就什么都不做，也不要断错回调 —— 那会让输入框失灵，
        #       而且极难追溯回这里。
        # The attribute is `cids` in older matplotlib and `_cids` in 3.11; accept
        # either, and if neither is there, leave the widget alone.
        cids = getattr(box, "cids", None)
        if cids is None:
            cids = getattr(box, "_cids", None)
        if cids is not None and len(cids) == 5:
            box.canvas.mpl_disconnect(cids.pop())
        return box

    box_start = _mkbox((0.215, 0.145, 0.045, 0.045), "start m ", "20")
    box_speed = _mkbox((0.215, 0.085, 0.045, 0.045), "speed m/s ", "4.5")
    box_flare = _mkbox((0.215, 0.025, 0.045, 0.045), "flare m ", "2.0")

    axrun = fig.add_axes((0.285, 0.085, 0.095, 0.06))
    btn_run = Button(axrun, "Run: perfect", color="#d9ead9", hovercolor="#b8dcb8")
    btn_run.label.set_fontsize(9)

    # Typing in a box selects `custom`. Without this the boxes sit there looking
    # editable while being ignored unless the radio happens to say custom, so
    # entering 10 m and pressing Run silently replays the 20 m preset -- the
    # numbers on screen and the test that runs disagree, and nothing says so.
    # A visible number that is not being used is worse than no number.
    # 中文：只要在输入框里打字就自动切到 custom。不这样的话，
    #       三个框看着可编辑却被忽略（除非单选正好选中 custom）——
    #       输入 10 m 再点 Run，跑的其实是 20 m 的预置场景，
    #       屏幕上的数字和实际运行的测试对不上，而且没有任何提示。
    #       一个显示出来却没被使用的数字，比不显示更糟。
    def _select_custom(_text):
        if radio.value_selected != "custom":
            radio.set_active(scenario_names.index("custom"))

    for box in (box_start, box_speed, box_flare):
        box.on_text_change(_select_custom)

    def _on_radio(label):
        btn_run.label.set_text(f"Run: {label}")

    radio.on_clicked(_on_radio)

    status_txt = fig.text(0.53, 0.185, "", fontsize=8.5, family="monospace",
                          va="top")
    status_lines: list[str] = []
    runner = {"thread": None}

    def set_status(msg: str):
        status_lines.append(msg)
        del status_lines[:-9]
        status_txt.set_text("\n".join(status_lines))

    def start_run(items, result):
        total_s = sum(rows[-1][0] / 1000.0 + 3.0 for rows, _, _ in items)
        runner["name"] = ", ".join(m.get("scenario", "?") for _, m, _ in items)
        runner["ends_at"] = time.monotonic() + total_s
        runner["thread"] = threading.Thread(
            target=run_scenarios,
            args=(items, commands, stop, buf, result, set_status),
            daemon=True)
        runner["thread"].start()

    def on_run(_event):
        if commands is None:
            print("[viewer] demo mode: no board to test", file=sys.stderr)
            return
        th = runner["thread"]
        if th is not None and th.is_alive():
            # Say how long is left. "Already running" on its own reads like a
            # refusal with no way forward, which is how a run that was going to
            # finish in four seconds got treated as the tool being stuck.
            # 中文：要说清楚还剩多久。光说「已经在跑了」像是一句没有下文的拒绝 ——
            #       一次本来四秒后就结束的运行，就是这样被当成工具卡住了的。
            left = runner.get("ends_at", 0.0) - time.monotonic()
            set_status(f"still running {runner.get('name', '?')}"
                       + (f", ~{left:.0f} s left" if left > 0 else ""))
            return
        name = radio.value_selected
        try:
            flare_cm = float(box_flare.text) * 100.0
        except ValueError:
            set_status("flare m: not a number")
            return
        try:
            if name == "custom":
                rows, meta = custom_scenario(float(box_start.text),
                                             float(box_speed.text),
                                             float(box_flare.text))
            else:
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "mock", f"{name}.csv")
                rows, meta = load_mock(path)
                # Presets carry expectations written against a 200 cm changeover;
                # running them at whatever the last custom test left behind would
                # measure something else while reporting the preset's name.
                # 中文：预置场景的期望值是按 200 cm 转换高度写的。
                #       用上一次自定义测试留下的值去跑，测的是别的东西，
                #       报出来的却还是这个预置场景的名字。
                flare_cm = 200.0
        except (ValueError, FileNotFoundError) as exc:
            set_status(f"cannot start: {exc}")
            return

        status_lines.clear()

        # Two limits worth saying out loud before a long run, because both make a
        # test look broken when it is behaving correctly.
        #
        # Above the firmware's pitch reference every beep is at the same lowest
        # pitch -- correct, but it sounds exactly like a stuck tone. A 100 m start
        # spends 174 of its 196 beeps there, i.e. 87 seconds of unchanging 700 Hz.
        #
        # And the TF02-Pro physically cannot see past ~40 m (13.5 m over grass),
        # so a test starting higher is exercising a regime that cannot occur.
        # 中文：长跑之前先把这两条限制说清楚，因为它们都会让一个**行为正确**的测试
        #       看起来像是坏了。
        #       高于固件的音高参考上沿时，每一声都是同一个最低音 —— 这是对的，
        #       但听起来就是卡住了。100 m 起跳的 196 声里有 174 声落在那里，
        #       也就是 87 秒毫无变化的 700 Hz。
        #       而 TF02-Pro 物理上看不到 40 m 以外（草地上 13.5 m），
        #       起点更高的测试跑的是一个不可能发生的区间。
        if name == "custom":
            start_cm = rows[0][1]
            top = FW.get("pitch_top_cm")
            if top and start_cm > top:
                flat = int((start_cm - top) / FW.get("beep_step_cm", 50.0))
                set_status(f"note: above {top/100:.0f} m the pitch is flat"
                           f" ({flat} beeps at {FW.get('pitch_low_hz', 700)} Hz)")
            if start_cm > SENSOR_MAX_CM:
                set_status(f"note: {start_cm/100:.0f} m is beyond the TF02-Pro"
                           f" ({SENSOR_MAX_CM/100:.0f} m max)")

        start_run([(rows, meta, flare_cm)], [])

    btn_run.on_clicked(on_run)

    # Both entry points -- the button and --replay -- go through here, so the
    # "already running" guard covers both. Two feeders would interleave their X
    # commands on the one queue and produce a descent that never happened, with
    # no error anywhere: the board would faithfully report the mixture.
    # 中文：按钮和 --replay 两条入口都走这里，「已经在跑」的判断才能同时管住两边。
    #       两个喂数据线程会把各自的 X 命令交错进同一个队列，
    #       合成出一次根本没发生过的下降 —— 而且哪里都不会报错，
    #       板子会忠实地把这锅混合物报出来。
    if replay_items:
        start_run(replay_items, replay_result if replay_result is not None else [])

    view = LiveView(fig, ax3d, axtilt, axdist)
    fig._flare_view = view
    blitter = Blitter(fig, view.dynamic_artists())

    # Everything static that can still change. When this differs from the last
    # frame, the cached background is out of date and a full redraw is due.
    # 中文：所有「静态但仍可能变」的东西。和上一帧不同时，缓存的背景已经过期，
    #       需要完整重绘一次。
    def static_signature():
        return (view._yscale_state, status_txt.get_text(),
                btn_corr.label.get_text(), btn_yscale.label.get_text(),
                btn_run.label.get_text())

    last_sig = {"sig": None}

    def tick():
        latest, history = buf.snapshot()
        view.update(latest, history)
        if latest is not None and latest.corr != corr_state["on"]:
            corr_state["on"] = latest.corr
            btn_corr.label.set_text(f"tilt corr: {'ON' if latest.corr else 'OFF'}")
            btn_corr.label.set_color("#1a7f37" if latest.corr else "#b34700")
        sig = static_signature()
        if sig != last_sig["sig"]:
            last_sig["sig"] = sig
            fig.canvas.draw_idle()
        else:
            blitter.blit()

    # A plain timer rather than FuncAnimation: without blit, FuncAnimation forces
    # a full draw every frame, which is the cost being removed; with blit=True it
    # only repaints inside each axes' box, which misses the titles above them.
    # Parked on the figure because a local would be garbage collected the moment
    # show() returns, and the timer would silently stop.
    # 中文：用普通定时器而不是 FuncAnimation：不开 blit 时它每帧强制完整重绘，
    #       正是要省掉的开销；开了 blit=True 它只重画每个 axes 方框以内，
    #       方框上方的标题就不更新了。
    #       定时器挂在 figure 上，否则 show() 一返回局部变量就被回收，定时器无声停止。
    timer = fig.canvas.new_timer(interval=interval_ms)
    timer.add_callback(tick)
    timer.start()
    fig._flare_timer = timer
    # Same reason as the animation: a widget with no strong reference is garbage
    # collected and its callbacks stop firing.
    fig._flare_btns = (btn_reset, btn_corr, btn_yscale, btn_run, radio,
                       box_start, box_speed, box_flare)
    try:
        plt.show()
    finally:
        stop.set()


def check_replay(buf: SampleBuffer, meta: dict, complete: bool) -> int:
    """Assert what the firmware actually did with one mock descent.

    Expectations come from the file's own header, not from here. What a good run
    looks like differs per scenario -- gaps are required in the hostile file and
    forbidden in the clean one -- so a single hard-coded notion of "correct" would
    make every new scenario a code change and would quietly test the wrong thing.

    The graph answers "does it look right", which is why the UI runs during a
    replay. But several of the claims that matter are *negatives*: no flare cue at
    altitude, no beeps at all in the never-arm case. Negative space is what a
    picture is worst at, so those are asserted rather than eyeballed.

    Chinese: expectations travel with the data; the negative claims are asserted
    because a graph cannot show that something did not happen.
    """
    name = meta.get("scenario", "?")
    history = buf.recording()
    beeps = buf.beeps()
    if not history:
        print("[%s] FAIL: no samples were received" % name, file=sys.stderr)
        return 1

    stages = [s.stage for s in history]
    heights = [s.height for s in history if math.isfinite(s.height)]

    # Count only gaps BETWEEN the first and last valid reading. A run always
    # begins and ends with invalid samples -- recording starts before the first
    # frame is injected, and the last injected frame goes stale 500 ms after
    # feeding stops while the checker is still waiting for CSV rows in flight.
    # Counting those would measure the harness's own edges, and did: every clean
    # scenario "failed" with ~26 gaps it had no dropouts to produce.
    # 中文：只统计第一个和最后一个有效读数**之间**的断线。
    #       每次运行的头尾必然是无效样本 —— 录制在第一帧注入之前就开始了，
    #       而喂完数据 500 ms 后最后一帧就过期，此时检查还在等在途的 CSV 行。
    #       把这些算进去，量的是测试框架自己的边缘 —— 事实也确实如此：
    #       每个干净场景都因为约 26 个它根本没有来源的「断线」而失败。
    valid_idx = [i for i, s in enumerate(history) if math.isfinite(s.height)]
    if valid_idx:
        interior = history[valid_idx[0]:valid_idx[-1] + 1]
        gaps = sum(1 for s in interior if not math.isfinite(s.height))
    else:
        gaps = 0
    seen_order = []
    for st in stages:
        if not seen_order or seen_order[-1] != st:
            seen_order.append(st)

    checks = []
    want = meta.get

    v = want("expect_arm")
    if v is not None:
        armed = any(s != 0 for s in stages) or bool(beeps)
        checks.append(("arms: %s" % v, armed == (v == "yes"),
                       "stages %s, %d beeps" % (sorted(set(stages)), len(beeps))))

    v = want("expect_flare")
    if v is not None:
        checks.append(("reaches FLARE NOW: %s" % v,
                       (4 in stages) == (v == "yes"), ""))

    v = want("expect_gaps")
    if v is not None:
        checks.append(("dropouts appear as gaps: %s" % v,
                       (gaps > 0) == (v == "yes"), "%d gap samples" % gaps))

    v = want("expect_peak_cm_min")
    if v is not None:
        ok = bool(heights) and max(heights) >= float(v)
        checks.append(("height peaks above %.1f m" % (float(v) / 100), ok,
                       "peak %.2f m" % (max(heights) / 100) if heights else "none"))

    v = want("expect_floor_cm_max")
    if v is not None:
        ok = bool(heights) and min(heights) <= float(v)
        checks.append(("height reaches below %.1f m" % (float(v) / 100), ok,
                       "floor %.2f m" % (min(heights) / 100) if heights else "none"))

    v = want("expect_flare_above_cm")
    if v == "none":
        # The regression that mattered: a dropout at altitude must never cue a
        # flare. This is the one claim a picture cannot make.
        bad = [s for s in history
               if s.stage == 4 and math.isfinite(s.height) and s.height > 400.0]
        detail = "worst %.2f m" % (max(b.height for b in bad) / 100) if bad else "none"
        checks.append(("no FLARE cue above 4 m", not bad, detail))

    v = want("expect_stage_order")
    if v is not None:
        expected = [int(x) for x in v.split(",")]
        checks.append(("stage order %s" % expected, seen_order == expected,
                       str([STAGE_NAMES[s] for s in seen_order])))

    v = want("expect_beeps_min")
    if v is not None:
        checks.append(("at least %s beeps" % v, len(beeps) >= int(v),
                       "%d beeps" % len(beeps)))

    v = want("expect_beeps_max")
    if v is not None:
        checks.append(("at most %s beeps" % v, len(beeps) <= int(v),
                       "%d beeps" % len(beeps)))

    if want("expect_beep_rate_rises") == "yes":
        # The core property of the whole alert model, measured rather than
        # assumed: one beep per fixed drop in height means the beep RATE is the
        # descent rate. If it ever degraded into a fixed cadence, every other
        # check here would still pass and only this one would notice.
        if len(beeps) >= 8:
            half = len(beeps) // 2
            span1 = beeps[half - 1][0] - beeps[0][0]
            span2 = beeps[-1][0] - beeps[half][0]
            r1 = (half - 1) / span1 if span1 > 0 else 0.0
            r2 = (len(beeps) - half - 1) / span2 if span2 > 0 else 0.0
            checks.append(("beep rate rises with descent rate", r2 > r1 * 1.3,
                           "%.1f -> %.1f beeps/s" % (r1, r2)))
        else:
            checks.append(("beep rate rises with descent rate", False,
                           "only %d beeps, cannot measure" % len(beeps)))

    print("\n=== replay check: %s ===" % name, file=sys.stderr)
    if meta.get("description"):
        print("  " + meta["description"], file=sys.stderr)
    failed = 0
    for label, ok, detail in checks:
        suffix = "   (%s)" % detail if detail else ""
        print("  [%s] %s%s" % ("PASS" if ok else "FAIL", label, suffix),
              file=sys.stderr)
        if not ok:
            failed += 1
    if not complete:
        print("  note: the window was closed before this scenario finished",
              file=sys.stderr)
    print("=== %s: %d/%d checks passed ===" % (name, len(checks) - failed,
                                               len(checks)), file=sys.stderr)
    return 1 if failed else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM7",
                    help="serial port (default COM7, the XIAO; the bench board is COM4)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true",
                    help="synthetic motion instead of real hardware")
    ap.add_argument("--fps", type=int, default=40,
                    help="redraw rate (default 40; a frame costs ~19 ms, so ~50 is the ceiling)")
    ap.add_argument("--selftest", metavar="PNG",
                    help="render a single frame headless and exit")
    ap.add_argument("--replay", metavar="CSV", nargs="+",
                    help="feed one or more mock descents to the board as laser"
                         " frames while displaying, then check each result")
    args = ap.parse_args()

    buf = SampleBuffer(maxlen=int(TRAIL_SECONDS * EXPECTED_HZ) + 50)
    stop = threading.Event()

    if args.selftest:
        import matplotlib
        matplotlib.use("Agg")
        demo = threading.Thread(target=demo_reader, args=(buf, stop), daemon=True)
        demo.start()
        time.sleep(1.5)
        stop.set()
        fig, ax3d, axtilt, axdist = build_figure()
        latest, history = buf.snapshot()
        draw_frame(ax3d, axtilt, axdist, latest, history)
        fig.savefig(args.selftest, dpi=110)
        print(f"selftest: {buf.received} synthetic samples -> {args.selftest}")
        return 0 if buf.received > 0 else 1

    # None in demo mode: there is no board on the other end, and the button says
    # so instead of silently doing nothing.
    commands: "queue.Queue[bytes] | None" = None
    if args.demo:
        reader = threading.Thread(target=demo_reader, args=(buf, stop), daemon=True)
    else:
        commands = queue.Queue()
        reader = threading.Thread(target=serial_reader,
                                  args=(args.port, args.baud, buf, stop, commands),
                                  daemon=True)
    reader.start()

    replay_result: list[int] = []
    replay_items: list = []
    if args.replay:
        if commands is None:
            print("--replay needs the board; it cannot run with --demo",
                  file=sys.stderr)
            return 2
        # Presets are pinned to a 200 cm changeover, the value their expectations
        # were written against.
        for path in args.replay:
            rows, meta = load_mock(path)
            replay_items.append((rows, meta, 200.0))

    run_live(buf, stop, interval_ms=max(10, 1000 // args.fps), commands=commands,
             replay_items=replay_items, replay_result=replay_result)
    print(f"[viewer] {buf.received} samples, {buf.dropped} non-CSV lines",
          file=sys.stderr)

    if args.replay:
        # The feeder reports as it goes; if the window was closed early it may
        # not have reported at all, and that is a failure, not a pass.
        if replay_result:
            return replay_result[0]
        print("[replay] window closed before any scenario completed",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
