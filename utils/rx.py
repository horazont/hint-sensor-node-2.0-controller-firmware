#!/usr/bin/python3
import array
import contextlib
import pathlib
import serial
import struct

import numpy as np

from datetime import datetime


RXPacketHeader = struct.Struct(
    ">QHB"
)

TXPacketHeader = struct.Struct(
    ">BQHBB"
)

LightSensorSample = struct.Struct(
    "<"
    # time
    "H"
    # channels
    "HHHH"
)

DHT11SensorSample = struct.Struct(
    "<"
    # time
    "H"
    # relative humidity
    "H"
    # temperature
    "H"
)

StatusPacket = struct.Struct(
    "<"
    # rtc time
    "L"
    # uptime
    "H"
    # rx errors, overruns, checksum errors, unknown frames, most allocated
    "HHHHHH"
    # tx non acked, retries, most allocated
    "HHH"
    # undervoltage
    "B"
)


def decompress_single(packet):
    median, = struct.unpack("<h", packet[:2])
    values = [median]
    packet = packet[2:]

    remaining_payload_size = len(packet)

    bitmap = []
    while remaining_payload_size > 0:
        remaining_payload_size -= 1
        next_bitmap_part = packet[0]
        packet = packet[1:]

        for i in range(7, -1, -1):
            bit = (next_bitmap_part & (1 << i)) >> i
            bitmap.append(bit)
            if bit:
                remaining_payload_size -= 1
            else:
                remaining_payload_size -= 2
            if remaining_payload_size <= 0:
                if remaining_payload_size < 0:
                    print("median:", median)
                    print("bitmap so far:", bitmap)
                    print("remaining packet:", packet)
                    raise ValueError(
                        "codec error: remaining payload is negative!"
                    )
                break

    for compressed in bitmap:
        if compressed:
            raw, = struct.unpack("<b", packet[:1])
            packet = packet[1:]
            values.append(raw + median)
        else:
            raw, = struct.unpack("<h", packet[:2])
            packet = packet[2:]
            values.append(raw + median)

    assert not packet

    data = np.array(values, dtype=np.int16)

    return data


def read_api_frame(s):
    while True:
        ch = s.read(1)
        while ch != b'\x7e':
            ch = s.read(1)
        length, = struct.unpack(">H", s.read(2))
        if length > 0xff:
            print("warning: packet with length > 0xff discarded")
            # sounds unlikely
            continue
        payload = s.read(length)
        recvd_checksum = s.read(1)[0]
        break

    calcd_checksum = 0xff - (sum(payload) & 0xff)

    if calcd_checksum != recvd_checksum:
        print(
            "warning: checksum mismatch: 0x{:02x} != 0x{:02x}".format(
                recvd_checksum,
                calcd_checksum
            )
        )
        return None

    type_ = payload[0]
    if type_ == 0x8a:
        print("    < modem status =", "0x{:02x}".format(payload[1]))
        return None

    elif type_ == 0x90:
        payload = payload[1:]
        addr64, addr16, info = RXPacketHeader.unpack(
            payload[:RXPacketHeader.size]
        )
        payload = payload[RXPacketHeader.size:]
        return addr64, addr16, info, payload

    elif type_ == 0x10:
        payload = payload[1:]
        _, addr64, addr16, *_ = TXPacketHeader.unpack(
            payload[:TXPacketHeader.size]
        )
        payload = payload[TXPacketHeader.size:]
        return addr64, addr16, None, payload
    else:
        print("warning: unknown API frame received: 0x{:02x}".format(type_))
        return None


class IMUStream:
    def __init__(self, f):
        self.f = f
        self.seq_base = None
        self.prev_seq = None
        self.samples = 0

    def process(self, packet):
        seq, = struct.unpack("<H", packet[:2])
        packet = packet[2:]
        if self.seq_base is None:
            self.seq_base = seq
        elif seq <= self.prev_seq:
            self.seq_base -= 65536
        self.prev_seq = seq
        print(seq - (self.seq_base + self.samples))
        try:
            data = decompress_single(packet)
        except:
            raise
        data = array.array("h", data)
        self.samples += len(data)
        data.tofile(self.f)
        self.f.flush()


class LightStream:
    def __init__(self, fs):
        self.fs = fs

    def process(self, packet):
        for i in range(6):
            t, *chs = LightSensorSample.unpack(
                packet[:LightSensorSample.size]
            )
            packet = packet[LightSensorSample.size:]
            for f, ch in zip(self.fs, chs):
                f.write(struct.pack("<HH", t, ch))
        for f in self.fs:
            f.flush()


def dump_dht11_packet(packet):
    timestamp, humidity, temperature = DHT11SensorSample.unpack(
        packet[:DHT11SensorSample.size]
    )

    print("DHT11 readout: t={}  rh=0x{:04x} %  T=0x{:04x} °C".format(
        timestamp,
        humidity,
        temperature,
    ))


def dump_status_packet(packet):
    (rtc, uptime,
     rx_overruns, rx_errors, rx_checksum_errors, rx_unknown_frames,
     rx_skipped_bytes, rx_most_allocated,
     tx_non_acked, tx_retries, tx_most_allocated,
     undervoltage) = StatusPacket.unpack(
         packet[:StatusPacket.size]
     )

    print("--- BEGIN STATUS PACKET ---")
    print("time: rtc={}  uptime={}".format(rtc, uptime))
    print(
        "xbee rx: overruns={}  errors={}  chksum={}  unknown={}  skipped={}  mostalloc={}".format(
            rx_overruns,
            rx_errors,
            rx_checksum_errors,
            rx_unknown_frames,
            rx_skipped_bytes,
            rx_most_allocated,
        )
    )
    print(
        "xbee tx: non_acked={}  retries={}  mostalloc={}".format(
            tx_non_acked,
            tx_retries,
            tx_most_allocated,
        )
    )
    print("core: undervoltage={}".format(bool(undervoltage)))
    print("--- END STATUS PACKET ---")


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "tty",
    )
    parser.add_argument(
        "outdir",
        type=pathlib.Path
    )

    args = parser.parse_args()

    s = serial.Serial(
        args.tty,
        baudrate=115200,
    )

    now = datetime.utcnow()
    imu_streams = [
        IMUStream(
            (
                args.outdir / "ch{}_{}.bin".format(
                    i, now.timestamp()
                )
            ).open("wb")
        )
        for i in range(6)
    ]

    light_stream = LightStream([
        (
            args.outdir / "{}_{}.bin".format(
                ch, now.timestamp()
            )
        ).open("wb")
        for ch in ["r", "g", "b", "i"]
    ])

    with contextlib.ExitStack() as stack:
        for stream in imu_streams:
            stack.enter_context(stream.f)

        for f in light_stream.fs:
            stack.enter_context(f)

        while True:
            frame_info = read_api_frame(s)
            if frame_info is None:
                continue
            _, _, _, payload = frame_info
            type_ = payload[0]
            if 0xf8 <= type_ <= 0xfd:
                #stream_index = type_ - 0xf8
                #imu_streams[stream_index].process(payload[1:])
                print(type_)
            elif type_ == 0xf3:
                dump_dht11_packet(payload[1:])
            elif type_ == 0xf4:
                # light_stream.process(payload[1:])
                print(type_)
            elif type_ == 0x82:
                dump_status_packet(payload[1:])
            else:
                print(
                    "warning: unexpected packet type: {:02x} "
                    "(payload={})".format(
                        type_,
                        payload
                    )
                )


if __name__ == "__main__":
    main()
