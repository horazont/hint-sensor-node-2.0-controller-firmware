#!/usr/bin/python3
import math
import pathlib
import socket
import struct
import time

from datetime import datetime

PORT = 7284


RecordedFrameHeader = struct.Struct(
    ">"
    # length
    "H"
)


def format_bytes(byte_count):
    suffixes = ["", "Ki", "Mi", "Gi", "Ti", "Pi", "Ei", "Zi", "Yi"]
    if byte_count > 0:
        dimension = min(int(math.log(byte_count, 1024)), len(suffixes)-1)
    else:
        dimension = 0
    suffix = suffixes[dimension]+"B"
    if dimension == 0:
        return "{0:.0f} {1}".format(float(byte_count), suffix)
    else:
        value = byte_count / (1 << (10*dimension))
        return "{0:.2f} {1}".format(value, suffix)


def format_rate(byte_count, time):
    if time < 1:
        return "??? B/s"
    return format_bytes(byte_count / time) + "/s"


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "addr"
    )
    parser.add_argument(
        "basename"
    )

    args = parser.parse_args()

    filename = pathlib.Path(
        args.basename + "-" + datetime.utcnow().isoformat()
    )

    with filename.open("xb") as f:
        buf = bytearray(1024)
        s = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            0
        )

        s.bind(("0.0.0.0", PORT))
        s.connect((args.addr, PORT))

        total_packets = 0
        total_bytes = 0
        t0 = time.time()
        while True:
            t1 = time.time()
            print(
                "\x1b[Kreceived {} in {} packets ({})".format(
                    format_bytes(total_bytes),
                    total_packets,
                    format_rate(total_bytes, t1-t0),
                ),
                end="\r",
                flush=True
            )
            size = s.recv_into(buf)
            packet = bytes(memoryview(buf)[:size])
            f.write(RecordedFrameHeader.pack(len(packet)))
            f.write(packet)
            total_packets += 1
            total_bytes += len(packet)


if __name__ == "__main__":
    main()
