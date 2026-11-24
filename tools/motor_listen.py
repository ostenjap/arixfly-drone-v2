"""Listen to the motors with the laptop microphone: when they run (on/off) and how fast (pitch).

Every 50 ms it measures loudness (dB) and the dominant frequency in the motor band. The first second
is taken as the room's noise floor; louder than floor + --on-db counts as "motors on". Only these
numbers are saved (captures/audio_<time>.csv), no audio, unless --keep-wav is given.

Usage:
  python tools/motor_listen.py --seconds 10            # record, print on/off timeline + pitch
  python tools/motor_listen.py --seconds 10 --keep-wav # also save the raw audio (records room sound!)
  from motor_listen import Listener                    # used by replay_test.py --listen
"""
import argparse
import os
import threading
import time
import wave

import numpy as np
import sounddevice as sd

RATE = 44100
WIN_S = 0.05
BAND_HZ = (150, 6000)  # micro motors whine in this range; below is room rumble
CAPTURES = os.path.join(os.path.dirname(__file__), "..", "captures")


class Listener:
    """Records the mic in the background. start() / stop() -> list of (t, dB, peak_hz)."""

    def __init__(self):
        self.chunks, self.t0, self.stream = [], None, None

    def start(self):
        self.chunks, self.t0 = [], time.perf_counter()
        self.stream = sd.InputStream(samplerate=RATE, channels=1, dtype="float32",
                                     callback=lambda d, n, t, s: self.chunks.append(d[:, 0].copy()))
        self.stream.start()

    def stop(self):
        self.stream.stop()
        self.stream.close()
        self.audio = np.concatenate(self.chunks) if self.chunks else np.zeros(0, np.float32)
        return frames(self.audio)


def frames(audio):
    n = int(RATE * WIN_S)
    win = np.hanning(n)
    freqs = np.fft.rfftfreq(n, 1 / RATE)
    band = (freqs >= BAND_HZ[0]) & (freqs <= BAND_HZ[1])
    out = []
    for i in range(0, len(audio) - n, n):
        x = audio[i:i + n]
        db = 20 * np.log10(np.sqrt(np.mean(x * x)) + 1e-9)
        spec = np.abs(np.fft.rfft(x * win))
        out.append((i / RATE, db, float(freqs[band][np.argmax(spec[band])])))
    return out


def timeline(fr, on_db=12.0, min_s=0.3):
    """Return noise floor and [(start, end, median_hz, median_db)] where the motors were on."""
    if not fr:
        return None, []
    floor = float(np.median([db for t, db, _ in fr if t < 1.0] or [fr[0][1]]))
    on = [db > floor + on_db for _, db, _ in fr]
    runs, start = [], None
    for (t, _, _), o in zip(fr + [(fr[-1][0] + WIN_S, 0, 0)], on + [False]):
        if o and start is None:
            start = t
        elif not o and start is not None:
            if t - start >= min_s:
                seg = [(db, hz) for tt, db, hz in fr if start <= tt < t]
                runs.append((start, t, float(np.median([h for _, h in seg])), float(np.median([d for d, _ in seg]))))
            start = None
    return floor, runs


def save(fr, audio=None, tag="audio"):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.normpath(os.path.join(CAPTURES, f"{tag}_{stamp}"))
    with open(base + ".csv", "w") as f:
        f.write("t_s,db,peak_hz\n")
        f.writelines(f"{t:.3f},{db:.1f},{hz:.0f}\n" for t, db, hz in fr)
    if audio is not None:
        with wave.open(base + ".wav", "wb") as w:
            w.setnchannels(1), w.setsampwidth(2), w.setframerate(RATE)
            w.writeframes((np.clip(audio, -1, 1) * 32767).astype(np.int16).tobytes())
    return base


def report(fr, on_db=12.0):
    floor, runs = timeline(fr, on_db)
    print(f"noise floor {floor:.1f} dB, 'on' above {floor + on_db:.1f} dB")
    if not runs:
        print("motors: no sound above the threshold")
    for s, e, hz, db in runs:
        print(f"motors ON  {s:6.2f}s -> {e:6.2f}s  ({e - s:4.2f}s)  pitch ~{hz:5.0f} Hz  {db:5.1f} dB")
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--on-db", type=float, default=12.0, help="dB above noise floor that counts as 'on'")
    ap.add_argument("--keep-wav", action="store_true")
    a = ap.parse_args()
    lst = Listener()
    print(f"listening {a.seconds:.0f} s ... (first second = room noise, keep the motors off then)")
    lst.start()
    time.sleep(a.seconds)
    fr = lst.stop()
    report(fr, a.on_db)
    print("saved", save(fr, lst.audio if a.keep_wav else None) + (".csv/.wav" if a.keep_wav else ".csv"))


if __name__ == "__main__":
    main()
