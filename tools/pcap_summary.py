"""Summarise flows in a classic pcap (raw-IP or Ethernet): per flow count, sizes, rate, sample payloads.

Usage:
  python tools/pcap_summary.py captures/idle.pcap            # flow table + 3 sample payloads per flow
  python tools/pcap_summary.py captures/idle.pcap --dump 20  # also hexdump first 20 payloads of small flows
"""
import argparse
import socket
import struct
from collections import defaultdict


def read_packets(path):
    data = open(path, "rb").read()
    e = "<" if data[:4] == b"\xd4\xc3\xb2\xa1" else ">"
    linktype = struct.unpack(e + "I", data[20:24])[0]
    off = 14 if linktype == 1 else 0
    pos = 24
    while pos + 16 <= len(data):
        sec, usec, incl, _ = struct.unpack(e + "IIII", data[pos:pos + 16])
        yield sec + usec / 1e6, data[pos + 16 + off:pos + 16 + incl]
        pos += 16 + incl


def parse(ip):
    if len(ip) < 20 or ip[0] >> 4 != 4:
        return None
    ihl = (ip[0] & 15) * 4
    proto = ip[9]
    src, dst = socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20])
    l4 = ip[ihl:]
    if proto == 17 and len(l4) >= 8:
        sp, dp = struct.unpack(">HH", l4[:4])
        return "UDP", src, sp, dst, dp, l4[8:]
    if proto == 6 and len(l4) >= 20:
        sp, dp = struct.unpack(">HH", l4[:4])
        return "TCP", src, sp, dst, dp, l4[(l4[12] >> 4) * 4:]
    return f"IP{proto}", src, 0, dst, 0, l4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--dump", type=int, default=0)
    a = ap.parse_args()
    flows = defaultdict(list)
    for t, ip in read_packets(a.pcap):
        p = parse(ip)
        if p:
            flows[p[:5]].append((t, p[5]))
    t0 = min(t for v in flows.values() for t, _ in v)
    for (proto, s, sp, d, dp), pk in sorted(flows.items(), key=lambda kv: -len(kv[1])):
        sizes = [len(x) for _, x in pk]
        dur = pk[-1][0] - pk[0][0]
        rate = (len(pk) - 1) / dur if dur > 0 else 0
        print(f"{proto} {s}:{sp} -> {d}:{dp}  n={len(pk)}  payload {min(sizes)}-{max(sizes)} B  "
              f"t={pk[0][0]-t0:.2f}..{pk[-1][0]-t0:.2f}s  ~{rate:.1f} pkt/s")
        for t, x in pk[:3]:
            print(f"    {t-t0:7.3f}s {len(x):5} B  {x[:32].hex(' ')}")
        if a.dump and max(sizes) <= 64:
            for t, x in pk[:a.dump]:
                print(f"    {t-t0:7.3f}s  {x.hex(' ')}")
        print()


if __name__ == "__main__":
    main()
