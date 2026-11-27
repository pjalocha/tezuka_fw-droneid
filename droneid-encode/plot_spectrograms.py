#!/usr/bin/env python3
"""Plot an encoded burst beside a crop from a recorded CF32 capture."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import stft


def load_crop(path: Path, rate: int, start_s: float, duration_s: float) -> np.ndarray:
    samples = np.fromfile(path, dtype="<c8")
    first = max(0, int(round(start_s * rate)))
    last = min(len(samples), first + int(round(duration_s * rate)))
    if last <= first:
        raise ValueError(f"empty crop from {path}")
    return samples[first:last]


def draw(ax, samples: np.ndarray, rate: int, title: str,
         nfft: int, frequency_offset_hz: float = 0.0) -> None:
    f, t, z = stft(samples, fs=rate, nperseg=nfft,
                   return_onesided=False,
                   noverlap=3 * nfft // 4, boundary=None, padded=False)
    f = np.fft.fftshift(f - frequency_offset_hz)
    z = np.fft.fftshift(z, axes=0)
    power = 20.0 * np.log10(np.maximum(np.abs(z), 1e-12))
    power -= np.max(power)
    ax.pcolormesh(t * 1e6, f / 1e6, power, shading="auto",
                  cmap="magma", vmin=-55, vmax=0)
    ax.set_title(title)
    ax.set_xlabel("Time within crop (µs)")
    ax.set_ylabel("Frequency relative to packet center (MHz)")
    ax.set_ylim(-8, 8)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("generated", type=Path)
    p.add_argument("--generated-rate", type=int, required=True)
    p.add_argument("capture", type=Path)
    p.add_argument("--capture-rate", type=int, default=50_000_000)
    p.add_argument("--capture-start-s", type=float, default=0.00008,
                   help="capture crop start; mini_50MHz first packet is near 80 us")
    p.add_argument("--capture-duration-us", type=float, default=900.0)
    p.add_argument("--capture-offset-hz", type=float, default=9.63e6,
                   help="offset of packet center from capture LO")
    p.add_argument("--nfft", type=int, default=512)
    p.add_argument("--output", type=Path, default=Path("droneid-spectrogram-comparison.png"))
    args = p.parse_args()

    generated = load_crop(args.generated, args.generated_rate, 0.0,
                          len(np.fromfile(args.generated, dtype="<c8")) /
                          args.generated_rate)
    captured = load_crop(args.capture, args.capture_rate, args.capture_start_s,
                         args.capture_duration_us * 1e-6)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    draw(axes[0], generated, args.generated_rate, "Generated offline waveform",
         args.nfft)
    draw(axes[1], captured, args.capture_rate, "Captured DroneID packet crop",
         args.nfft * max(1, round(args.capture_rate / args.generated_rate)),
         args.capture_offset_hz)
    fig.suptitle("DroneID OFDM burst spectrograms (each panel normalized to its peak)")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
