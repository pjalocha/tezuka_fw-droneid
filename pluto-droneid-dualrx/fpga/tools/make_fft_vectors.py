#!/usr/bin/env python3
"""Create 64-bin complex FFT stimuli for the diagonal-trigger RTL bench."""

import argparse
from pathlib import Path

import numpy as np

FFT_SIZE = 64
HOP = 32


def pack_fft(iq: np.ndarray, out_path: Path) -> int:
    """Write frame count then bin-major signed-16 I/Q words as readmemh."""
    frames = 1 + (len(iq) - FFT_SIZE) // HOP if len(iq) >= FFT_SIZE else 0
    if not frames:
        raise ValueError("input must contain at least 64 complex samples")

    window = (np.sin(np.pi * np.arange(FFT_SIZE, dtype=np.float32) / FFT_SIZE) /
              np.sqrt(np.float32(FFT_SIZE)))
    matrix = np.lib.stride_tricks.sliding_window_view(iq, FFT_SIZE)[::HOP][:frames]
    spectrum = np.fft.fft(matrix * window[None, :], axis=1)
    fft_i = np.clip(np.rint(spectrum.real), -32768, 32767).astype(np.int16)
    fft_q = np.clip(np.rint(spectrum.imag), -32768, 32767).astype(np.int16)
    packed = (fft_q.astype(np.uint16).astype(np.uint32) << 16) | \
             fft_i.astype(np.uint16).astype(np.uint32)

    with out_path.open("w", encoding="ascii") as out:
        out.write(f"{frames:08x}\n")
        np.savetxt(out, packed.reshape(-1), fmt="%08x")
    return frames


def read_capture(path: Path, fmt: str) -> np.ndarray:
    dtype = np.int8 if fmt == "cs8" else np.dtype("<i2")
    raw = np.fromfile(path, dtype=dtype)
    if raw.size % 2:
        raise ValueError("capture has an unmatched I or Q value")
    return raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)


def pack_samples(path: Path, fmt: str, out_path: Path) -> int:
    """Write input samples scaled from CS8/CS16 into signed-16 I/Q words."""
    dtype = np.int8 if fmt == "cs8" else np.dtype("<i2")
    raw = np.fromfile(path, dtype=dtype)
    if raw.size % 2:
        raise ValueError("capture has an unmatched I or Q value")
    scale = 256 if fmt == "cs8" else 1
    i = np.clip(raw[0::2].astype(np.int32) * scale, -32768, 32767).astype(np.int16)
    q = np.clip(raw[1::2].astype(np.int32) * scale, -32768, 32767).astype(np.int16)
    packed = (q.astype(np.uint16).astype(np.uint32) << 16) | \
             i.astype(np.uint16).astype(np.uint32)
    with out_path.open("w", encoding="ascii") as out:
        out.write(f"{i.size:08x}\n")
        np.savetxt(out, packed, fmt="%08x")
    return int(i.size)


def make_noise(frames: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    i = np.rint(rng.normal(0.0, 250.0, size=(frames, FFT_SIZE))).astype(np.int16)
    q = np.rint(rng.normal(0.0, 250.0, size=(frames, FFT_SIZE))).astype(np.int16)
    # This mode directly synthesizes independent FFT-bin noise. pack_fft() is
    # intentionally bypassed: there is no time-domain FFT to perform.
    return np.stack((i, q), axis=-1).reshape(-1)


def pack_noise(iq: np.ndarray, frames: int, out_path: Path) -> None:
    i = iq[0::2].astype(np.int16).reshape(frames, FFT_SIZE)
    q = iq[1::2].astype(np.int16).reshape(frames, FFT_SIZE)
    packed = (q.astype(np.uint16).astype(np.uint32) << 16) | \
             i.astype(np.uint16).astype(np.uint32)
    with out_path.open("w", encoding="ascii") as out:
        out.write(f"{frames:08x}\n")
        np.savetxt(out, packed.reshape(-1), fmt="%08x")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture", type=Path)
    source.add_argument("--noise-frames", type=int)
    parser.add_argument("--format", choices=("cs8", "cs16"), default="cs8")
    parser.add_argument("--samples", action="store_true",
                        help="write time-domain signed-16 I/Q words instead of FFT bins")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.capture and args.samples:
        count = pack_samples(args.capture, args.format, args.output)
        print(f"wrote {count} time-domain samples to {args.output}")
        return 0
    elif args.capture:
        iq = read_capture(args.capture, args.format)
        frames = pack_fft(iq, args.output)
    else:
        if args.samples:
            parser.error("--samples requires --capture")
        if args.noise_frames <= 0:
            parser.error("--noise-frames must be positive")
        frames = args.noise_frames
        pack_noise(make_noise(frames, args.seed), frames, args.output)
    print(f"wrote {frames} FFT frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
