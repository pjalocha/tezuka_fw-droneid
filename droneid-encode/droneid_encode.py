#!/usr/bin/env python3
"""Offline encoder for the standard 9-symbol DJI DroneID OFDM frame.

This deliberately has no Pluto, libiio, or RF-device dependency. It accepts
the 89-byte DUML body, or a 91-byte body with its CRC field, and writes raw
little-endian complex float32 IQ samples.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.signal import resample_poly


FS_NATIVE = 15_360_000
NFFT = 1024
NCARRIERS = 601
DC_INDEX = 300
CP_LENGTHS = (80, 72, 72, 72, 72, 72, 72, 72, 80)
DATA_SYMBOLS = (0, 1, 2, 4, 6, 7, 8)
GOLD_SEED = 0x12345678
GOLD_NC = 1600
K = 1408
KPI = 1440
NROWS = 45
NCOLS = 32
NDUMMY = KPI - (K + 4)
TE = 7200
RM_PERM = (0, 16, 8, 24, 4, 20, 12, 28,
           2, 18, 10, 26, 6, 22, 14, 30,
           1, 17, 9, 25, 5, 21, 13, 29,
           3, 19, 11, 27, 7, 23, 15, 31)
QPSK_TO_BITS = (2, 3, 1, 0)


def crc16(data: bytes) -> int:
    """Reflected CRC-16 used by the 91-byte DUML payload."""
    crc = 0x3692
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xFFFF


def normalize_payload(data: bytes) -> bytes:
    if len(data) == 89:
        return data + crc16(data).to_bytes(2, "little")
    if len(data) == 91:
        expected = crc16(data[:89])
        actual = int.from_bytes(data[89:91], "little")
        if actual != expected:
            raise ValueError(f"payload CRC mismatch: got {actual:04x}, expected {expected:04x}")
        return data
    raise ValueError(f"payload must be 89 or 91 bytes, got {len(data)}")


def gold_sequence(length: int) -> np.ndarray:
    total = GOLD_NC + length + 31
    x1 = np.zeros(total, dtype=np.uint8)
    x2 = np.zeros(total, dtype=np.uint8)
    x1[0] = 1
    for n in range(32):
        x2[n] = (GOLD_SEED >> n) & 1
    for n in range(GOLD_NC + length):
        x1[n + 31] = (x1[n + 3] ^ x1[n]) & 1
        x2[n + 31] = (x2[n + 3] ^ x2[n + 2] ^ x2[n + 1] ^ x2[n]) & 1
    return x1[GOLD_NC:GOLD_NC + length] ^ x2[GOLD_NC:GOLD_NC + length]


def bits_from_bytes(data: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(data, dtype=np.uint8), bitorder="big")


def rsc_encode(bits: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """LTE constituent RSC encoder matching turbo_decode.c's trellis."""
    sys = np.empty(len(bits) + 3, dtype=np.uint8)
    parity = np.empty_like(sys)
    r1 = r2 = r3 = 0
    for i, bit in enumerate(bits):
        a = int(bit) ^ r2 ^ r3
        z = a ^ r1 ^ r3
        sys[i], parity[i] = bit, z
        r1, r2, r3 = a, r1, r2
    for i in range(3):
        u = r2 ^ r3
        a = u ^ r2 ^ r3
        z = a ^ r1 ^ r3
        sys[len(bits) + i], parity[len(bits) + i] = u, z
        r1, r2, r3 = a, r1, r2
    if (r1, r2, r3) != (0, 0, 0):
        raise AssertionError("RSC termination failed")
    return sys, parity


def turbo_encode(info: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(info) != K:
        raise ValueError(f"turbo input must be {K} bits")
    qpp = ((43 * np.arange(K, dtype=np.int64) +
            88 * np.arange(K, dtype=np.int64) ** 2) % K).astype(np.int32)
    s1, p1 = rsc_encode(info)
    s2, p2 = rsc_encode(info[qpp])

    # LTE tail-bit multiplexing, inverse of the receiver's tail demultiplexer.
    d0 = np.empty(K + 4, dtype=np.uint8)
    d1 = np.empty(K + 4, dtype=np.uint8)
    d2 = np.empty(K + 4, dtype=np.uint8)
    d0[:K], d1[:K], d2[:K] = s1[:K], p1[:K], p2[:K]
    d0[K] = s1[K]
    d2[K] = s1[K + 1]
    d1[K + 1] = s1[K + 2]
    d1[K] = p1[K]
    d0[K + 1] = p1[K + 1]
    d2[K + 1] = p1[K + 2]
    d0[K + 2] = s2[K]
    d2[K + 2] = s2[K + 1]
    d1[K + 3] = s2[K + 2]
    d1[K + 2] = p2[K]
    d0[K + 3] = p2[K + 1]
    d2[K + 3] = p2[K + 2]
    return d0, d1, d2


def rate_match(streams: tuple[np.ndarray, np.ndarray, np.ndarray]) -> np.ndarray:
    """LTE circular-buffer rate matching with DroneID's 4148-bit start."""
    d0, d1, d2 = streams

    def rm0(k: int) -> int:
        return (k % NROWS) * NCOLS + RM_PERM[k // NROWS]

    def rm2(k: int) -> int:
        return (RM_PERM[k // NROWS] + NCOLS * (k % NROWS) + 1) % KPI

    circular: list[tuple[int, int] | None] = []
    for w in range(KPI):
        idx = rm0(w)
        circular.append((0, idx - NDUMMY) if idx >= NDUMMY else None)
    for m in range(2 * KPI):
        if m % 2 == 0:
            idx = rm0(m // 2)
            circular.append((1, idx - NDUMMY) if idx >= NDUMMY else None)
        else:
            idx = rm2((m - 1) // 2)
            circular.append((2, idx - NDUMMY) if idx >= NDUMMY else None)

    valid = [item for item in circular if item is not None]
    first_systematic = next(i for i, item in enumerate(valid) if item[0] == 0)
    start = (first_systematic - 4148) % len(valid)
    return np.fromiter(
        (streams[s][idx] for j in range(TE)
         for s, idx in (valid[(start + j) % len(valid)],)),
        dtype=np.uint8, count=TE)


def qpsk(bits: np.ndarray) -> np.ndarray:
    if len(bits) % 2:
        raise ValueError("QPSK bit count must be even")
    values = bits[0::2] | (bits[1::2] << 1)
    inverse = np.empty(4, dtype=np.int8)
    for quadrant, value in enumerate(QPSK_TO_BITS):
        inverse[value] = quadrant
    quadrant = inverse[values]
    # Quadrants match get_symbol_bits() in the scanner decoder.
    re = np.where((quadrant == 0) | (quadrant == 1), 1.0, -1.0)
    im = np.where((quadrant == 0) | (quadrant == 3), 1.0, -1.0)
    return ((re + 1j * im) / math.sqrt(2.0)).astype(np.complex64)


def zc_carriers(root: int) -> np.ndarray:
    n = np.arange(NCARRIERS, dtype=np.float64)
    # The root sequence is mapped directly across the 601 OFDM carriers. The
    # decoder's FFT-of-ZC helper is for its channel estimate, not this mapping.
    active = np.exp(-1j * np.pi * root * n * (n + 1) / NCARRIERS)
    active[DC_INDEX] = 0
    return active.astype(np.complex64)


def ofdm_symbol(carriers: np.ndarray, cp: int) -> np.ndarray:
    if carriers.shape != (NCARRIERS,):
        raise ValueError("expected 601-bin carrier vector")
    freq = np.zeros(NFFT, dtype=np.complex64)
    freq[-300:] = carriers[:300]
    freq[:301] = carriers[300:]
    time = np.fft.ifft(freq).astype(np.complex64)
    return np.concatenate((time[-cp:], time))


def encode_frame(payload: bytes) -> np.ndarray:
    payload = normalize_payload(payload)
    # The decoder's LTE block is K=1408 bits (176 bytes); bytes following the
    # 91-byte DUML record are padding and do not alter the checked DUML CRC.
    block = payload + bytes(176 - len(payload))
    info = bits_from_bytes(block)
    coded = rate_match(turbo_encode(info))
    scrambled = coded ^ gold_sequence(TE)

    carriers_by_symbol: dict[int, np.ndarray] = {}
    pilot = qpsk(gold_sequence(1200))
    carriers_by_symbol[0] = np.zeros(NCARRIERS, dtype=np.complex64)
    carriers_by_symbol[0][:300] = pilot[:300]
    carriers_by_symbol[0][301:] = pilot[300:]

    for slot, sym in enumerate((1, 2, 4, 6, 7, 8)):
        symbols = qpsk(scrambled[slot * 1200:(slot + 1) * 1200])
        v = np.zeros(NCARRIERS, dtype=np.complex64)
        v[:300], v[301:] = symbols[:300], symbols[300:]
        carriers_by_symbol[sym] = v
    carriers_by_symbol[3] = zc_carriers(600)
    carriers_by_symbol[5] = zc_carriers(147)

    return np.concatenate([
        ofdm_symbol(carriers_by_symbol[i], CP_LENGTHS[i])
        for i in range(9)
    ]).astype(np.complex64)


def make_waveform(payload: bytes, sample_rate: int,
                  pad_us: float, peak: float) -> np.ndarray:
    if sample_rate not in (FS_NATIVE, 2 * FS_NATIVE):
        raise ValueError("sample rate must be 15.36 or 30.72 Ms/s")
    if not 0.0 < peak <= 1.0:
        raise ValueError("peak must be in (0, 1]")
    core = encode_frame(payload)
    if sample_rate == 2 * FS_NATIVE:
        core = resample_poly(core, 2, 1, window=("kaiser", 8.0)).astype(np.complex64)
    peak_now = float(np.max(np.abs(core)))
    if peak_now <= 0.0:
        raise ValueError("encoded waveform is all zero")
    core *= np.float32(peak / peak_now)
    padding = int(round(sample_rate * pad_us / 1e6))
    return np.pad(core, (padding, padding)).astype(np.complex64)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    payload_group = parser.add_mutually_exclusive_group(required=True)
    payload_group.add_argument("--payload-hex",
                               help="89-byte DUML body or complete 91-byte body with CRC")
    payload_group.add_argument("--payload-file", type=Path,
                               help="binary 89-byte DUML body or 91-byte body with CRC")
    parser.add_argument("--output", required=True, type=Path,
                        help="output raw interleaved little-endian CF32 file")
    parser.add_argument("--rate", type=int, choices=(FS_NATIVE, 2 * FS_NATIVE),
                        default=FS_NATIVE, help="output sample rate in samples/s")
    parser.add_argument("--pad-us", type=float, default=100.0,
                        help="zero padding before and after the frame")
    parser.add_argument("--peak", type=float, default=0.7,
                        help="normalize complex-sample magnitude peak (0..1]")
    parser.add_argument("--metadata", type=Path,
                        help="optional JSON sidecar output")
    args = parser.parse_args()

    try:
        payload = (bytes.fromhex(args.payload_hex) if args.payload_hex is not None
                   else args.payload_file.read_bytes())
        normalized = normalize_payload(payload)
        iq = make_waveform(normalized, args.rate, args.pad_us, args.peak)
    except (ValueError, OverflowError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    iq.astype("<c8", copy=False).tofile(args.output)
    meta = {
        "format": "complex-float32-le-interleaved-iq",
        "sample_rate_hz": args.rate,
        "sample_count": int(iq.size),
        "frame_sample_count": int(iq.size - 2 * round(args.rate * args.pad_us / 1e6)),
        "pad_us_each_side": args.pad_us,
        "peak_magnitude": float(np.max(np.abs(iq))),
        "payload_hex": normalized.hex(),
        "payload_crc16": normalized[89:91].hex(),
    }
    if args.metadata:
        args.metadata.write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
