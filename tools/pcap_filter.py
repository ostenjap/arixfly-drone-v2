"""Keep only packets to/from given IPv4 addresses (classic pcap, raw-IP or Ethernet).

Only the IP header addresses are inspected; everything else is dropped unread.

Usage:
  python tools/pcap_filter.py IN.pcap OUT.pcap 192.168.169.1 192.168.100.1 ...
"""
import socket
import struct
import sys

LINKTYPE_ETHERNET, LINKTYPE_RAW = 1, 101


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    src, dst, ips = sys.argv[1], sys.argv[2], {socket.inet_aton(a) for a in sys.argv[3:]}
    data = open(src, "rb").read()
    magic = data[:4]
    if magic == b"\xd4\xc3\xb2\xa1":
        e = "<"
    elif magic == b"\xa1\xb2\xc3\xd4":
        e = ">"
    else:
        sys.exit("not a classic pcap file (pcapng not supported)")
    linktype = struct.unpack(e + "I", data[20:24])[0]
    if linktype not in (LINKTYPE_ETHERNET, LINKTYPE_RAW):
        sys.exit(f"unsupported linktype {linktype}")
    off_ip = 14 if linktype == LINKTYPE_ETHERNET else 0

    out, pos, total, kept = [data[:24]], 24, 0, 0
    while pos + 16 <= len(data):
        incl = struct.unpack(e + "I", data[pos + 8:pos + 12])[0]
        rec = data[pos:pos + 16 + incl]
        pkt = rec[16:]
        pos += 16 + incl
        total += 1
        ip = pkt[off_ip:]
        if len(ip) >= 20 and ip[0] >> 4 == 4 and (ip[12:16] in ips or ip[16:20] in ips):
            out.append(rec)
            kept += 1
    with open(dst, "wb") as f:
        f.write(b"".join(out))
    print(f"kept {kept} of {total} packets -> {dst}")


if __name__ == "__main__":
    main()
