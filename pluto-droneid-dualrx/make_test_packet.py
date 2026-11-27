#!/usr/bin/env python3
"""Build the synthetic DUML sample and 15.36/30.72 Ms/s CF32 waveforms."""

import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ENCODER = ROOT.parent / "droneid-encode"
sys.path.insert(0, str(ENCODER))
import droneid_encode  # noqa: E402

LAT = 51.5007
LON = -0.1246


def put_u16le(buf, offset, value):
    buf[offset:offset + 2] = struct.pack("<H", value)


def put_i16le(buf, offset, value):
    buf[offset:offset + 2] = struct.pack("<h", value)


def put_i32le(buf, offset, value):
    buf[offset:offset + 4] = struct.pack("<i", value)


def main():
    payload = bytearray(89)
    payload[0] = 0x5B  # DUML packet length
    payload[2] = 0x02  # protocol version used by the local receiver model
    put_u16le(payload, 3, 1)
    put_u16le(payload, 5, 0x1F13)
    payload[7:23] = b"TEST-DRONEID-01" + b"\0"
    put_i32le(payload, 23, round(LON * 174533.0))
    put_i32le(payload, 27, round(LAT * 174533.0))
    put_i16le(payload, 31, round(35.0 * 3.281))
    put_i16le(payload, 33, round(30.0 * 3.281))
    put_i16le(payload, 35, 0)
    put_i16le(payload, 37, 0)
    put_i16le(payload, 39, 0)
    put_i16le(payload, 41, 0)
    payload[43:51] = struct.pack("<Q", 0)
    put_i32le(payload, 51, round(LAT * 174533.0))
    put_i32le(payload, 55, round(LON * 174533.0))
    put_i32le(payload, 59, round(LON * 174533.0))
    put_i32le(payload, 63, round(LAT * 174533.0))
    payload[67] = 1
    payload[68] = 16
    payload[69:84] = b"TEST-DRONEID-01"
    full_payload = droneid_encode.normalize_payload(bytes(payload))
    examples = ROOT / "examples"
    examples.mkdir(exist_ok=True)
    (examples / "test-payload.bin").write_bytes(full_payload)
    for sample_rate, suffix in ((15360000, "15m36"), (30720000, "30m72")):
        wave = droneid_encode.make_waveform(full_payload, sample_rate, 100.0, 0.7)
        wave_path = examples / f"test-packet-{suffix}.cf32"
        metadata_path = examples / f"test-packet-{suffix}.json"
        wave.astype("<c8", copy=False).tofile(wave_path)
        metadata = {
            "format": "complex-float32-le-interleaved-iq",
            "sample_rate_hz": sample_rate,
            "sample_count": int(wave.size),
            "payload_crc16": full_payload[89:91].hex(),
            "latitude_deg": LAT,
            "longitude_deg": LON,
            "altitude_m": 35.0,
            "identifier": "TEST-DRONEID-01",
            "synthetic": True,
            "transmission": "explicit command only; one shot by default",
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
