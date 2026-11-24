"""Force a fresh Wi-Fi scan (Windows) and list visible SSIDs.

Usage:
  python tools/wifi_scan.py                 # print SSID, BSSID, channel, signal
  python tools/wifi_scan.py --save off.txt  # also save the SSID list for diffing
  python tools/wifi_scan.py --diff off.txt  # show networks not in the saved list
"""
import argparse
import ctypes
import re
import subprocess
import time
from ctypes import wintypes


class GUID(ctypes.Structure):
    _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD),
                ("Data3", wintypes.WORD), ("Data4", ctypes.c_ubyte * 8)]


class WLAN_INTERFACE_INFO(ctypes.Structure):
    _fields_ = [("InterfaceGuid", GUID), ("strInterfaceDescription", ctypes.c_wchar * 256),
                ("isState", ctypes.c_uint)]


class WLAN_INTERFACE_INFO_LIST(ctypes.Structure):
    _fields_ = [("dwNumberOfItems", wintypes.DWORD), ("dwIndex", wintypes.DWORD),
                ("InterfaceInfo", WLAN_INTERFACE_INFO * 1)]


def trigger_scan():
    wlan = ctypes.windll.wlanapi
    handle, ver = wintypes.HANDLE(), wintypes.DWORD()
    if wlan.WlanOpenHandle(2, None, ctypes.byref(ver), ctypes.byref(handle)):
        raise OSError("WlanOpenHandle failed")
    ifaces = ctypes.POINTER(WLAN_INTERFACE_INFO_LIST)()
    wlan.WlanEnumInterfaces(handle, None, ctypes.byref(ifaces))
    n = ifaces.contents.dwNumberOfItems
    arr = ctypes.cast(ifaces.contents.InterfaceInfo, ctypes.POINTER(WLAN_INTERFACE_INFO))
    for i in range(n):
        wlan.WlanScan(handle, ctypes.byref(arr[i].InterfaceGuid), None, None, None)
    wlan.WlanFreeMemory(ifaces)
    wlan.WlanCloseHandle(handle, None)


def list_networks():
    out = subprocess.run(["netsh", "wlan", "show", "networks", "mode=bssid"],
                         capture_output=True, text=True, errors="replace").stdout
    nets, ssid = [], None
    for line in out.splitlines():
        if m := re.match(r"^SSID \d+ : (.*)$", line.strip()):
            ssid = m.group(1) or "<hidden>"
        elif m := re.match(r"^BSSID \d+\s+: (.*)$", line.strip()):
            nets.append({"ssid": ssid, "bssid": m.group(1)})
        elif nets and (m := re.match(r"^(Signal|Channel|Band)\s+: (.*)$", line.strip())):
            nets[-1][m.group(1).lower()] = m.group(2).strip()
    return nets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save")
    ap.add_argument("--diff")
    a = ap.parse_args()
    trigger_scan()
    time.sleep(5)
    nets = list_networks()
    for n in nets:
        print(f"{n['ssid']:<32} {n['bssid']}  ch {n.get('channel', '?'):>3}  "
              f"{n.get('band', '?'):<8} {n.get('signal', '?')}")
    print(f"-- {len(nets)} BSSIDs")
    if a.save:
        with open(a.save, "w", encoding="utf-8") as f:
            f.write("\n".join(f"{n['ssid']}\t{n['bssid']}" for n in nets))
    if a.diff:
        with open(a.diff, encoding="utf-8") as f:
            old = {line.split("\t")[1] for line in f.read().splitlines() if "\t" in line}
        new = [n for n in nets if n["bssid"] not in old]
        print("\nNEW since baseline:" if new else "\nNo new networks since baseline.")
        for n in new:
            print(f"  {n['ssid']}  {n['bssid']}  ch {n.get('channel', '?')}  {n.get('signal', '?')}")


if __name__ == "__main__":
    main()
