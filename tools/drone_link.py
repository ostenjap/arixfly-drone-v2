"""Drone link for tools/live_flight.py: UDP protocol, video frames + frame acks, the network thread, flight
recording and the Wi-Fi join (split out of live_flight.py on 2026-11-10, code unchanged).
  Net        one thread, one socket: hello 10 Hz, stick packets 39 Hz with the app's ack records, video in,
             in-air watchdog; Frames: reassembly + ack every frame id (complete, given up, never seen, stall)
  Recorder   frame_NNNN.jpg + frames.csv + events.csv + control.csv per run
See tools/live_flight.py for usage and the findings behind each rule.
"""
import csv
import os
import queue
import select
import socket
import struct
import threading
import time

import drone_video as dv
from replay_test import DRONE_SSID, RATE_HZ, build, wait_for_ip, wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))
LAND, ESTOP = 0x02, 0x04


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


TAKEOFF, LAND, ESTOP, CALIBRATE = 0x01, 0x02, 0x04, 0x80  # byte 25 bits (app decompile, verified in flight)

PULSE_S, KILL_S, CAL_S, LAND_S = 1.0, 1.0, 1.0, 8.0

LIVE_S = 0.6               # feed counts as live if the newest frame is younger

WATCHDOG_S = 1.0           # in the air: window silent this long -> network thread lands

GIVE_UP_S = 0.2            # partial frame without a new fragment this long -> ack it anyway, drop it

ACK_REPEAT, ACK_PER_PKT = 3, 3

GAP_BACK = 8               # frame id N arrives -> also ack unseen ids below it (up to 8 back)

STALL_S, STALL_AHEAD = 0.4, 4   # no fragment for 0.4 s -> re-ack the last 4 ids and ack the next 4 (1x per s)

HDR = 56

CENTRED = [0x80] * 4

OUT_ROOT = os.path.join(HERE, "..", "captures", "flights")

# ------------------------------------------------------------------ video frames + acks (network thread only)
class Frames:
    """Reassembles `93 01` fragments per frame id and keeps the ack queue. Every frame id is acked:
    complete frames when they finish, partial ones when they are given up, and ids we never saw at all
    (gap below a newer id; 22:22 run: first frames were ids 3, 4 -> 1, 2 never acked -> drone froze).
    If no fragment comes for STALL_S, recent and next ids are acked again (lost acks / lost whole frames)."""

    def __init__(self):
        self.parts = {}    # id -> [meta (count, size, w, h, q), {idx: data}, t_last]
        self.acks = []     # [[id, count, times sent]]
        self.complete = self.dropped = self.frags = self.skipped = self.stalls = 0
        self.max_id, self.count, self.t_frag, self.t_stall = 0, 57, None, 0.0

    def feed(self, d, now):
        if len(d) < HDR or d[:2] != b"\x93\x01":
            return None
        self.frags += 1
        fid = struct.unpack_from("<I", d, 16)[0]
        idx, count, size = struct.unpack_from("<III", d, 32)
        w, h, q = struct.unpack_from("<HHB", d, 44)
        self.t_frag, self.count = now, count
        if fid > self.max_id:
            for m in range(max(self.max_id + 1, fid - GAP_BACK), fid):
                if m not in self.parts:
                    self.acks.append([m, count, 0])  # never saw it: tell the drone it's done
                    self.skipped += 1
            self.max_id = fid
        elif fid < self.max_id - 64:
            self.max_id = fid  # id counter restarted
        p = self.parts.get(fid)
        if p is None or (idx == 0 and p[1] and now - p[2] > GIVE_UP_S):  # new frame (ids repeat in hello mode)
            p = self.parts[fid] = [(count, size, w, h, q), {}, now]
        if idx < count:
            p[1][idx] = d[HDR:]
        p[2] = now
        count, size, w, h, q = p[0]
        if len(p[1]) < count:
            return None
        scan = b"".join(p[1][i] for i in range(count))[:size]
        del self.parts[fid]
        self.acks.append([fid, count, 0])
        self.complete += 1
        return fid, dv.jpeg_header(w, h, q or 50) + scan + b"\xff\xd9"

    def give_up(self, now):
        for fid in [f for f, p in self.parts.items() if now - p[2] > GIVE_UP_S]:
            self.acks.append([fid, self.parts.pop(fid)[0][0], 0])  # "complete" -> the drone moves on
            self.dropped += 1

    def unstall(self, now):
        """True if the stream stalled and acks were resent."""
        if self.t_frag is None or now - self.t_frag < STALL_S or now - self.t_stall < 1.0:
            return False
        self.t_stall = now
        self.stalls += 1
        for m in range(max(1, self.max_id - 3), self.max_id + STALL_AHEAD + 1):
            self.acks.append([m, self.count, 0])
        return True

    def take_acks(self):
        out = self.acks[:ACK_PER_PKT]
        for a in out:
            a[2] += 1
        self.acks = [a for a in self.acks if a[2] < ACK_REPEAT]
        return [(f, n) for f, n, _ in out]

# ------------------------------------------------------------------ network thread
class Net(threading.Thread):
    """Owns the socket. Sends hello 10 Hz always, sticks 39 Hz once the drone answered (until a kill),
    receives and reassembles video, acks every frame, runs the in-air watchdog."""

    def __init__(self, addr, bind_ip, recorder):
        super().__init__(daemon=True)
        self.addr, self.rec = addr, recorder
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, 0))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        self.sock.setblocking(False)
        self.lock = threading.Lock()
        self.running, self.control, self.killed = True, False, False
        self.button, self.pulse_end, self.kill_end = 0, 0.0, None
        self.flying, self.hb, self.state = False, time.monotonic(), "WAITING"
        self.axes = list(CENTRED)   # bytes 20..23, set by the window (approach), centred by watchdog / kill
        self.trim = {"roll": 0, "pitch": 0, "yaw": 0}   # added to bytes 22 / 23 / 21 in every stick packet
        self.frames, self.jpg, self.t_frame, self.fid, self.n = Frames(), None, None, None, 0
        self.rx = self.status_pkts = self.stick_pkts = self.ack_records = 0
        self.t_first_rx = self.t_first_frame = None
        self.error, self.events = "", []
        self.times = []    # recent frame times for fps
        self.status, self.status_raw, self.t_status_log = None, None, 0.0   # last decoded `93 04` status

    # --- called from the window thread
    def pulse(self, bit, secs):
        with self.lock:
            self.button, self.pulse_end = bit, time.monotonic() + secs
            trim, st = dict(self.trim), self.status
        if bit == TAKEOFF:
            # exact trims at every takeoff, in the format flight_review / drift_trend already read (no more
            # reconstructing them from tuning changes) + the drone's last status (battery) - added 2026-11-11
            for k in ("pitch", "roll", "yaw"):
                self.event(f"tune trim_{k} = {trim[k]} (at takeoff, logged by drone_link)")
            self.event("status at takeoff: " + (f"fly_state {st['fly_state']} battery {st['battery']} "
                                                f"flags 0x{st['flags']:02x}" if st else "none received yet"))

    def set_axes(self, axes):
        with self.lock:
            self.axes = list(axes)

    def kill(self):
        with self.lock:
            self.axes = list(CENTRED)
            self.button, self.pulse_end = ESTOP, time.monotonic() + KILL_S
            self.kill_end, self.flying = time.monotonic() + KILL_S, False
        self.event("KILL: 0x04 for 1 s, then no stick packets")

    def latest(self):
        with self.lock:
            return self.n, self.jpg, self.t_frame

    def fps(self):
        with self.lock:
            ts = [t for t in self.times if time.monotonic() - t < 2.0]
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0

    def live(self):
        return self.t_frame is not None and time.monotonic() - self.t_frame < LIVE_S

    def event(self, text):
        t = time.monotonic()
        self.events.append((t, text))
        self.rec.event(t, text)

    def stop(self):
        self.running = False
        self.join(timeout=2)
        self.sock.close()

    # --- the thread
    def run(self):
        next_hello = next_stick = time.monotonic()
        seq = 0
        try:
            while self.running:
                now = time.monotonic()
                if now >= next_hello:
                    self.sock.sendto(dv.HELLO, self.addr)
                    next_hello = now + 0.1
                with self.lock:
                    if self.button and now >= self.pulse_end:
                        self.button = 0
                    if self.kill_end and now >= self.kill_end:
                        self.kill_end, self.control, self.killed = None, False, True
                    watchdog = self.flying and now - self.hb > WATCHDOG_S
                    if watchdog:
                        self.button, self.pulse_end, self.flying = LAND, now + PULSE_S, False
                        self.axes = list(CENTRED)
                    button, axes = self.button, self.axes
                if watchdog:
                    self.event("WATCHDOG: window silent > 1 s in the air -> land 0x02")
                if self.control and now >= next_stick:
                    self.frames.give_up(now)
                    if self.frames.unstall(now):
                        self.event(f"video stall {now - self.frames.t_frag:.1f} s: re-acked ids up to "
                                   f"{self.frames.max_id + STALL_AHEAD}")
                    acks = self.frames.take_acks()
                    self.ack_records += len(acks)
                    out = [axes[0], clamp(axes[1] + self.trim["yaw"], 0, 255), clamp(axes[2] + self.trim["roll"], 0, 255),
                           clamp(axes[3] + self.trim["pitch"], 0, 255)]
                    self.sock.sendto(dv.add_acks(build(seq, axes=out, button=button), acks), self.addr)
                    seq += 1
                    self.stick_pkts += 1
                    next_stick = max(next_stick + 1 / RATE_HZ, now - 0.05)
                due = min(next_hello, next_stick if self.control else next_hello)
                select.select([self.sock], [], [], max(0.0, min(due - time.monotonic(), 0.01)))
                for _ in range(400):
                    try:
                        d = self.sock.recv(65535)
                    except (BlockingIOError, InterruptedError):
                        break
                    except ConnectionResetError:
                        continue
                    self._packet(d, time.monotonic())
        except OSError as e:
            self.error = f"network error: {e}"
            self.running = False

    def _packet(self, d, now):
        self.rx += 1
        if self.t_first_rx is None:
            self.t_first_rx = now
            self.event(f"first packet from the drone ({d[:2].hex()}) -> stick packets with acks start")
            if not self.killed:
                self.control = True
        if d[:2] == b"\x93\x04":
            self.status_pkts += 1
            st = decode_status(d)
            if d != self.status_raw or now - self.t_status_log >= STATUS_EVERY_S:
                self.status_raw, self.t_status_log = d, now
                with self.lock:
                    self.status = st
                self.rec.status(now, st, d)
            return
        got = self.frames.feed(d, now)
        if got:
            fid, jpg = got
            with self.lock:
                self.n += 1
                self.jpg, self.t_frame, self.fid = jpg, now, fid
                self.times = self.times[-30:] + [now]
                n, state, button = self.n, self.state, self.button
            if self.t_first_frame is None:
                self.t_first_frame = now
                self.event("first video frame")
            self.rec.frame(n, now, fid, state, button, jpg)

# ------------------------------------------------------------------ `93 04` status (battery)
STATUS_EVERY_S = 5.0   # status.csv: a row whenever the packet changes, and at least every 5 s


def decode_status(d):
    """`93 04` packet -> {fly_state, battery, flags, xor_ok} or None. The flight controller's frame inside it is
    `88 00 <fly-state> <battery> <flags> <xor of the 3> 55` (turbodrone research on a sibling app; matches all 153
    of our 2026-11-07 ground packets: fly-state 0, battery 0x5a = 90, flags 0). Battery scale NOT verified."""
    i = d.find(b"\x88\x00", 4)
    if i < 0 or len(d) < i + 7 or d[i + 6] != 0x55:
        return None
    fly, bat, flags, x = d[i + 2], d[i + 3], d[i + 4], d[i + 5]
    return {"fly_state": fly, "battery": bat, "flags": flags, "xor_ok": (fly ^ bat ^ flags) == x}


# ------------------------------------------------------------------ recording (own thread, never blocks the network)
class Recorder(threading.Thread):
    def __init__(self, out_dir, t0):
        super().__init__(daemon=True)
        self.dir, self.t0, self.q, self.rows, self.ctl = out_dir, t0, queue.Queue(), [], []
        os.makedirs(out_dir, exist_ok=True)
        self.ev = open(os.path.join(out_dir, "events.csv"), "w", newline="")
        self.evw = csv.writer(self.ev)
        self.evw.writerow(["t", "event"])
        self.st = self.stw = None   # status.csv, opened at the first `93 04` packet
        self.start()

    def frame(self, n, t, fid, state, button, jpg):
        self.q.put((n, round(t - self.t0, 3), fid, state, button, jpg))

    def event(self, t, text):
        self.q.put(("event", round(t - self.t0, 3), text))

    def status(self, t, st, raw):
        self.q.put(("status", round(t - self.t0, 3), st, raw.hex()))

    def control(self, t, state, sm, fresh, axes, button, heading="", pred=None, phase=""):
        self.ctl.append([round(t - self.t0, 3), state, int(fresh),
                         *(round(sm[k], 1) if sm else "" for k in ("dist", "bearing", "elev")), *axes, button,
                         round(heading, 1) if heading != "" else "",
                         *([round(v, 1) for v in pred[:2]] if pred else ["", ""]), phase])

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            if item[0] == "event":
                self.evw.writerow(item[1:])
                self.ev.flush()
                print(f"{item[1]:8.2f}s  {item[2]}", flush=True)
                continue
            if item[0] == "status":
                if self.st is None:
                    self.st = open(os.path.join(self.dir, "status.csv"), "w", newline="")
                    self.stw = csv.writer(self.st)
                    self.stw.writerow(["t", "fly_state", "battery", "flags", "xor_ok", "raw_hex"])
                _, t, st, raw = item
                self.stw.writerow([t, *((st["fly_state"], st["battery"], st["flags"], int(st["xor_ok"]))
                                        if st else ("", "", "", "")), raw])
                self.st.flush()
                continue
            n, t, fid, state, button, jpg = item
            with open(os.path.join(self.dir, f"frame_{n:04d}.jpg"), "wb") as f:
                f.write(jpg)
            self.rows.append([n, t, fid, state, button])

    def close(self):
        self.q.put(None)
        self.join(timeout=5)
        self.ev.close()
        if self.st:
            self.st.close()
        with open(os.path.join(self.dir, "frames.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "t", "frame_id", "state", "button"])
            w.writerows(self.rows)
        if self.ctl:
            with open(os.path.join(self.dir, "control.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "state", "fresh", "range_cm", "bearing_deg", "elev_deg",
                            "throttle", "yaw", "roll", "pitch", "button", "vo_heading_deg",
                            "pred_bearing_deg", "pred_elev_deg", "recover_phase"])
                w.writerows(self.ctl)

def summary(net, rec):
    lines = [f"saved {len(rec.rows)} frames to {os.path.normpath(rec.dir)}",
             f"drone packets {net.rx} (status {net.status_pkts}, video fragments {net.frames.frags}), "
             f"frames complete {net.frames.complete}, dropped+acked {net.frames.dropped}, never seen+acked "
             f"{net.frames.skipped}, stalls {net.frames.stalls}, "
             f"stick packets {net.stick_pkts}, ack records sent {net.ack_records}"]
    if net.t_first_rx:
        lines.append(f"first packet {net.t_first_rx - rec.t0:.1f} s, first frame "
                     f"{(net.t_first_frame - rec.t0) if net.t_first_frame else float('nan'):.1f} s after start")
    by = {}
    for n, t, fid, state, b in rec.rows:
        by.setdefault(state, []).append(t)
    for state, ts in by.items():
        span = ts[-1] - ts[0]
        gap = max((b - a for a, b in zip(ts, ts[1:])), default=0)
        lines.append(f"  {state:10} {len(ts):4} frames, {(len(ts) - 1) / span if span > 0 else 0:4.1f} fps, "
                     f"longest gap {gap:.2f} s")
    return "\n".join(lines)

# ------------------------------------------------------------------ Wi-Fi
def join_wifi(log):
    if os.path.exists(r"C:\Program Files\Mullvad VPN\resources\mullvad.exe"):
        import hud
        if hud.mullvad_blocks_lan():
            return None, "Mullvad blocks the LAN - disconnect it"
    for attempt in range(3):
        log(f"joining {DRONE_SSID} (try {attempt + 1}/3) ...")
        wifi_connect(DRONE_SSID)
        ip = wait_for_ip("192.168.169.", timeout=15)
        if ip:
            return ip, ""
    return None, "no 192.168.169.x address - drone on? phone off the drone Wi-Fi?"

