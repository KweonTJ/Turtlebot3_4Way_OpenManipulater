#!/usr/bin/env python3
import argparse
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

def scan(port_name, baud, start=0, end=252):
    print(f"Scanning {port_name} @ {baud} ...", flush=True)
    port = PortHandler(port_name)
    if not port.openPort():  print(f"FAIL open {port_name}"); return 1
    if not port.setBaudRate(baud): print(f"FAIL baud {baud}"); return 1
    pkt = PacketHandler(2.0)
    cnt = 0
    for i in range(start, end+1):
        m,c,e = pkt.ping(port, i)
        if c == COMM_SUCCESS and e == 0:
            print(f"FOUND ID={i} MODEL={m}", flush=True); cnt += 1
    port.closePort()
    print(f"DONE. COUNT={cnt}", flush=True)
    return 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=252)
    args = ap.parse_args()
    raise SystemExit(scan(args.port, args.baud, args.start, args.end))

if __name__ == "__main__":
    main()
