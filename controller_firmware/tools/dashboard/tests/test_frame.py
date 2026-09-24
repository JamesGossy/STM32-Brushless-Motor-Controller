import struct

import plot_motor as pm
from fw_telem_frame import FrameParser, build_frame, crc16


def test_crc_reference():
    assert crc16(b"123456789") == 0x29B1   # CRC-16/CCITT-FALSE check value, same as C unit test
    assert crc16(b"") == 0xFFFF


def test_build_and_parse_roundtrip():
    f = build_frame(0x10, b"hello")
    assert f[:2] == b"\xaa\x55" and f[2] == 0x10 and struct.unpack_from("<H", f, 3)[0] == 5
    p = FrameParser()
    assert p.feed(f) == [(0x10, b"hello")]


def test_parser_resyncs_and_handles_split_input():
    stream = b"\x01\x02\xaa" + build_frame(1, b"ab") + b"\xaa\x55\xff\xff" + build_frame(2, b"cd")
    p = FrameParser()
    out = []
    for i in range(len(stream)):
        out += p.feed(stream[i:i + 1])
    assert out == [(1, b"ab"), (2, b"cd")]


def test_bad_crc_dropped():
    f = bytearray(build_frame(3, b"xyz"))
    f[6] ^= 0xFF
    p = FrameParser()
    assert p.feed(bytes(f)) == []
    assert p.bad_crc == 1


def test_extract_frames_matches_parser():
    stream = build_frame(1, b"12") + b"junk" + build_frame(5, b"\x10\x00\x20\x00")
    frames, rest = pm._extract_frames(stream + b"\xaa")
    assert frames == [(1, b"12"), (5, b"\x10\x00\x20\x00")]
    assert rest == b"\xaa"


def test_dispatch_motor_and_faults():
    payload = struct.pack(pm._MOTOR_FMT, 3, 2, 0x0001 | 0x2000, 1.5, -0.5, 2.0, 0.0)
    d = pm._dispatch(pm._MSG_MOTOR, payload)
    assert d["state"] == 3 and d["enabled"] == 1 and d["fault"] == 1
    assert d["iq"] == 1.5 and d["id"] == -0.5
    assert pm.fault_names(d["__faults__"]) == ["overcurrent", "stall"]


def test_dispatch_other_types():
    assert pm._dispatch(pm._MSG_SPEED, struct.pack(pm._SPEED_FMT, 3000.0, 3100.0, 1.0))["speed_request_rpm"] == 3100.0
    v = pm._dispatch(pm._MSG_VOLT, struct.pack(pm._VOLT_FMT, 24.0, 1.0, 2.0, 0.4, 0.5, 0.6))
    assert v["vbus"] == 24.0 and abs(v["duty_c"] - 0.6) < 1e-6
    t = pm._dispatch(pm._MSG_TEMP, struct.pack(pm._TEMP_FMT, 4550, 2500))
    assert t == {"fet_temp_c": 45.5, "amb_temp_c": 25.0}
    assert pm._dispatch(pm._MSG_LOG, b"state=IDLE\r\n") == {"__log__": "state=IDLE"}
    assert pm._dispatch(pm._MSG_MOTOR, b"short") is None
    assert pm._dispatch(0x7F, b"") is None


def test_fields_cover_charts():
    for chart in pm.CHARTS:
        for f in chart[1]:
            assert f in pm.FIELDS
    page = pm.render_page(4.0)
    assert "__CHARTS__" not in page and "foc telemetry" in page
