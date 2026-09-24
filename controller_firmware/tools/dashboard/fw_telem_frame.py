"""
fw_telem_frame.py — codec for the firmware's framed telemetry (app/telem.h).

    0xAA 0x55 | TYPE(u8) | LEN(u16 LE) | PAYLOAD[LEN] | CRC16(u16 LE)

CRC16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect/xor-out) over
TYPE+LEN+PAYLOAD. Must stay bit-exact with crc16_ccitt()/telem_frame() in C.
"""
import struct

SYNC = b"\xaa\x55"
MAX_PAYLOAD = 128


def crc16_byte(crc, b):
    crc ^= b << 8
    for _ in range(8):
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16(data, crc=0xFFFF):
    for b in data:
        crc = crc16_byte(crc, b)
    return crc


def build_frame(mtype, payload=b""):
    body = struct.pack("<BH", mtype, len(payload)) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Incremental parser: feed() bytes, get back a list of (type, payload)."""

    def __init__(self):
        self.buf = b""
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            idx = self.buf.find(SYNC)
            if idx == -1:
                self.buf = self.buf[-1:] if self.buf[-1:] == SYNC[:1] else b""
                break
            self.buf = self.buf[idx:]
            if len(self.buf) < 5:
                break
            mtype = self.buf[2]
            (length,) = struct.unpack_from("<H", self.buf, 3)
            if length > MAX_PAYLOAD:
                self.buf = self.buf[2:]
                continue
            end = 5 + length + 2
            if len(self.buf) < end:
                break
            (crc_rx,) = struct.unpack_from("<H", self.buf, 5 + length)
            if crc16(self.buf[2:5 + length]) == crc_rx:
                frames.append((mtype, self.buf[5:5 + length]))
                self.buf = self.buf[end:]
            else:
                self.bad_crc += 1
                self.buf = self.buf[2:]
        return frames
