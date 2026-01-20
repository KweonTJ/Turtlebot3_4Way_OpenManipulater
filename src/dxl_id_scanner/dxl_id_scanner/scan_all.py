#!/usr/bin/env python3
from dynamixel_sdk import *
import time

def main():
    port_name = "/dev/ttyACM0"
    baud = 1000000

    port = PortHandler(port_name)
    packet = PacketHandler(2.0)

    if not port.openPort():
        print(f"FAIL: cannot open {port_name}")
        return
    if not port.setBaudRate(baud):
        print(f"FAIL: cannot set baud {baud}")
        return

    print(f"\n=== Scanning {port_name} @ {baud} ===")
    found_ids = []
    for i in range(0, 253):
        model, comm, err = packet.ping(port, i)
        if comm == COMM_SUCCESS and err == 0:
            found_ids.append(i)
            print(f"FOUND ID={i} MODEL={model}")
    if not found_ids:
        print("No motors found.")
        port.closePort()
        return

    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_VELOCITY = 128
    LEN_4BYTES = 4

    print("\n=== Reading current data ===")
    for dxl_id in found_ids:
        pos, comm1, err1 = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        vel, comm2, err2 = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_VELOCITY)
        if comm1 == COMM_SUCCESS:
            print(f"ID={dxl_id:3d}  POS={pos:8d}  VEL={vel:8d}")
        else:
            print(f"ID={dxl_id:3d}  read error")

    port.closePort()

if __name__ == "__main__":
    main()
