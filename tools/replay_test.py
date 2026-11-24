"""Send captured-format `ef 02` control packets from the laptop to the HS420 (props OFF!).

Joins the drone Wi-Fi, sends for at most --seconds (hard cap 10 s), logs any replies, then
reconnects to the home Wi-Fi. Default is a dry run that only prints the packets.

Usage:
  python tools/replay_test.py                       # dry run: print first packets, send nothing
  python tools/replay_test.py --send                # centred sticks only, 5 s
  python tools/replay_test.py --send --unlock       # + button byte 25 = 0x01 for 1 s at t=1 s (🔒 in app)
  python tools/replay_test.py --send --unlock --listen  # + laptop mic: motor on/off delays vs. packets
"""
import argparse
import os
import socket
import subprocess
import tempfile
import time

DRONE = ("192.168.169.1", 8800)
DRONE_SSID = "HolyStoneFPV_2BA841"
HOME_SSID = "TC-5C336"
RATE_HZ = 39          # measured app send rate
HARD_CAP_S = 10

# Real packet from captures/sticks.pcap: sticks centred (0x80), button 0, flags 0x04.
TEMPLATE = bytes.fromhex(
    "ef0258000202000100000000ea0400000b0088008080808000000404550000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000324b142d0000")
assert len(TEMPLATE) == TEMPLATE[2], "template length must match its length field"


def build(seq, axes=(0x80, 0x80, 0x80, 0x80), button=0x00, flags=0x04, speed=0):
    """speed = app speed button (eCtrlSpeedKey, 0 or 1), byte 24 bits 0-1 (HS420 app ba2.a(): s & 3)."""
    p = bytearray(TEMPLATE)
    p[12:14] = (seq & 0xFFFF).to_bytes(2, "little")
    p[20:24] = bytes(axes)
    p[24] = speed & 3
    p[25] = button
    p[26] = flags
    x = 0
    for b in p[20:27]:
        x ^= b
    p[27] = x
    return bytes(p)


def wifi_connect(ssid):
    if ssid == DRONE_SSID:  # open network needs a profile once
        xml = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
 <name>{ssid}</name><SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
 <connectionType>ESS</connectionType><connectionMode>manual</connectionMode>
 <MSM><security><authEncryption><authentication>open</authentication>
 <encryption>none</encryption><useOneX>false</useOneX></authEncryption></security></MSM>
</WLANProfile>"""
        path = os.path.join(tempfile.gettempdir(), "hs420_profile.xml")
        with open(path, "w") as f:
            f.write(xml)
        subprocess.run(["netsh", "wlan", "add", "profile", f"filename={path}"], capture_output=True)
    subprocess.run(["netsh", "wlan", "connect", f"name={ssid}"], capture_output=True)


def wait_for_ip(prefix, timeout=20):
    end = time.time() + timeout
    while time.time() < end:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(DRONE)
            ip = s.getsockname()[0]
            s.close()
            if ip.startswith(prefix):
                return ip
        except OSError:
            pass
        time.sleep(0.5)
    return None


def plan(seconds, unlock):
    """Yield (t, packet) for the whole run."""
    n = int(min(seconds, HARD_CAP_S) * RATE_HZ)
    for i in range(n):
        t = i / RATE_HZ
        button = 0x01 if unlock and 1.0 <= t < 2.0 else 0x00
        yield t, build(i, button=button)


def listen_report(lst, sent, unlock):
    """Print motor on/off (from the mic) relative to the first unlock packet and the last packet."""
    from motor_listen import report, save
    fr = lst.stop()
    runs = report(fr)
    print("mic data saved", save(fr, tag="replay_audio") + ".csv")
    if not sent:
        return
    first, last = sent[0] - lst.t0, sent[1] - lst.t0
    unlock_at = first + (1.0 if unlock else 0.0)
    print(f"packets: first {first:.2f}s, unlock {unlock_at:.2f}s, last {last:.2f}s (mic time)")
    for s, e, hz, _ in runs:
        print(f"  motors on {s - unlock_at:+.2f}s after unlock, off {e - last:+.2f}s after the last packet, ~{hz:.0f} Hz")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--send", action="store_true")
    ap.add_argument("--unlock", action="store_true")
    ap.add_argument("--listen", action="store_true", help="record motor sound with the laptop mic")
    ap.add_argument("--seconds", type=float, default=5)
    a = ap.parse_args()
    packets = list(plan(a.seconds, a.unlock))

    if not a.send:
        for t, p in packets[::RATE_HZ // 2][:12]:
            print(f"{t:5.2f}s  {p[:32].hex(' ')} ... ({len(p)} B)")
        print(f"dry run: {len(packets)} packets over {packets[-1][0]:.1f}s, nothing sent")
        return

    log, lst, sent = [], None, None
    if a.listen:
        from motor_listen import Listener
        lst = Listener()
        lst.start()  # starts before the Wi-Fi switch, so the first second is room noise
    try:
        wifi_connect(DRONE_SSID)
        ip = wait_for_ip("192.168.169.")
        if not ip:
            print("ABORT: laptop did not get a 192.168.169.x address")
            return
        print(f"on drone Wi-Fi as {ip}, sending {len(packets)} packets")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setblocking(False)
        t0 = time.perf_counter()
        for t, p in packets:
            while time.perf_counter() - t0 < t:
                try:
                    data, src = s.recvfrom(4096)
                    log.append((time.perf_counter() - t0, src, data))
                except (BlockingIOError, OSError):
                    time.sleep(0.002)
            s.sendto(p, DRONE)
        s.close()
        sent = (t0, time.perf_counter())
        print("done sending")
    finally:
        if lst:
            time.sleep(2)  # keep listening after the last packet to hear the motors stop
            listen_report(lst, sent, a.unlock)
        wifi_connect(HOME_SSID)
        print(f"replies received: {len(log)}")
        for t, src, data in log[:15]:
            print(f"  {t:5.2f}s {src[0]}:{src[1]} {len(data)} B  {data[:24].hex(' ')}")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out = os.path.join(os.path.dirname(__file__), "..", "captures", f"replay_replies_{stamp}.txt")
        with open(out, "w") as f:
            for t, src, data in log:
                f.write(f"{t:.3f} {src[0]}:{src[1]} {data.hex()}\n")
        print(f"replies saved to {os.path.normpath(out)}")


if __name__ == "__main__":
    main()
