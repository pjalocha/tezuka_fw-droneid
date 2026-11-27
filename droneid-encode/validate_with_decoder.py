#!/usr/bin/env python3
"""Decode an offline encoder output with scan2400's native locate library."""

from __future__ import annotations

import argparse
import ctypes
import json
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly


class Telemetry(ctypes.Structure):
    _fields_ = [
        ("pkt_len", ctypes.c_uint8), ("version", ctypes.c_uint8),
        ("sequence_number", ctypes.c_uint16), ("state_info", ctypes.c_uint16),
        ("serial_number", ctypes.c_char * 17),
        ("longitude", ctypes.c_double), ("latitude", ctypes.c_double),
        ("altitude_m", ctypes.c_double), ("height_m", ctypes.c_double),
        ("v_north", ctypes.c_int16), ("v_east", ctypes.c_int16),
        ("v_up", ctypes.c_int16), ("d_1_angle", ctypes.c_int16),
        ("gps_time", ctypes.c_uint64),
        ("app_lat", ctypes.c_double), ("app_lon", ctypes.c_double),
        ("longitude_home", ctypes.c_double), ("latitude_home", ctypes.c_double),
        ("device_type_id", ctypes.c_uint8), ("uuid", ctypes.c_char * 21),
        ("crc_valid", ctypes.c_bool),
    ]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path, help="raw interleaved little-endian CF32")
    p.add_argument("--rate", required=True, type=int,
                   choices=(15_360_000, 30_720_000))
    p.add_argument("--locate-lib", type=Path,
                   default=Path(__file__).resolve().parents[1] /
                   "scan2400/build/droneid-native/src/locate/libdroneid_locate.so")
    p.add_argument("--sync-dither", type=float, default=0.002,
                   help="deterministic complex RMS dither for CP peak picking; 0 disables")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args()

    iq = np.fromfile(args.input, dtype="<c8")
    if iq.size == 0:
        p.error("input file is empty")
    if args.rate == 30_720_000:
        iq = resample_poly(iq, 1, 2, window=("kaiser", 8.0)).astype(np.complex64)
    if args.sync_dither:
        rng = np.random.default_rng(args.seed)
        scale = args.sync_dither / np.sqrt(2.0)
        iq = (iq + scale * (rng.standard_normal(iq.size) +
                            1j * rng.standard_normal(iq.size))).astype(np.complex64)

    try:
        lib = ctypes.CDLL(str(args.locate_lib))
    except OSError as exc:
        p.error(f"cannot load decoder library {args.locate_lib}: {exc}")
    lib.locate_droneid.argtypes = [ctypes.c_void_p, ctypes.c_uint32,
                                   ctypes.c_uint32, ctypes.POINTER(Telemetry)]
    lib.locate_droneid.restype = ctypes.c_int32
    result = Telemetry()
    status = lib.locate_droneid(iq.ctypes.data, len(iq), 0, ctypes.byref(result))
    report = {
        "decoder_status": status,
        "crc_valid": bool(result.crc_valid),
        "sequence_number": int(result.sequence_number),
        "serial_number": bytes(result.serial_number).split(b"\0", 1)[0].decode(
            "ascii", errors="replace"),
        "sample_rate_hz": args.rate,
        "samples_at_decoder_rate": int(len(iq)),
        "sync_dither_rms": args.sync_dither,
    }
    print(json.dumps(report, indent=2))
    return 0 if status == 0 and result.crc_valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
