# DroneID offline encoder

This project generates a standard 9-symbol DJI DroneID OFDM waveform from an
89-byte DUML body (or a complete 91-byte body including CRC-16). It is a
reference encoder only: it writes raw complex float32 IQ and does not configure
or transmit through an SDR.

The first version mirrors the PHY decoded by `scan2400`:

- CRC-16 over DUML bytes 0–88, stored little-endian in bytes 89–90.
- 1408-bit LTE turbo input block: the 91-byte record followed by 85 zero pad
  bytes; QPP interleaving, constituent RSC encoders, LTE tail multiplexing,
  circular-buffer rate matching to 7200 bits.
- 3GPP Gold scrambling, QPSK, 1024-point OFDM, 601-carrier grid, direct
  Zadoff–Chu carrier sequences with roots 600 and 147, and the standard CP
  lengths.
- 15.36 Ms/s native waveform or 30.72 Ms/s by 2x polyphase interpolation.

The coding equations and PHY settings are based on the checked-in receiver
implementation. The extra 85 bytes in the turbo block are zero-filled because
the receiver validates the first 91 DUML bytes; this padding is an encoder
assumption to revisit if a transmitter capture proves a different block fill.

## Generate a waveform

Python 3, NumPy, and SciPy are required.

```sh
python3 droneid_encode.py \
  --payload-file payload.bin \
  --rate 15360000 \
  --output /tmp/droneid.cf32 \
  --metadata /tmp/droneid.json
```

`payload.bin` may contain 89 bytes; the encoder appends its CRC. A 91-byte
input is accepted only when its supplied CRC matches. `--peak` sets the output
complex magnitude peak, and `--pad-us` adds zeros around the frame.

## Check it with the scan2400 decoder

`validate_with_decoder.py` accepts either output rate. It uses the local
`droneid_locate` shared library; pass `--locate-lib` if it is not at the
default `../scan2400/build/droneid-native/src/locate/` path.

```sh
python3 validate_with_decoder.py /tmp/droneid.cf32 --rate 15360000
```

At 30.72 Ms/s the validator downsamples by two to the decoder's native rate.
By default it adds deterministic, low-level complex dither for CP peak picking.
The generated signal has an ideal noiseless CP, whose correlation is a flat
plateau; the scanner's peak picker expects a local peak as found in a real
capture. The dither is applied only in this validation step, not to the output
waveform. A successful result has `decoder_status: 0` and `crc_valid: true`.

Run the two-rate integration check with:

```sh
python3 tests/roundtrip.py --locate-lib ../scan2400/build/droneid-native/src/locate/libdroneid_locate.so
```

## Compare spectrograms

```sh
python3 plot_spectrograms.py /tmp/droneid.cf32 \
  --generated-rate 15360000 \
  ../scan2400/mini_50MHz.cf32 \
  --capture-rate 50000000 \
  --capture-start-s 0.00008 \
  --capture-offset-hz 9630000 \
  --output /tmp/droneid-compare.png
```

Each panel is normalized to its own peak and shown relative to the DroneID
channel center. This compares time/frequency shape, not absolute RF power,
phase, or a channel-calibrated EVM.

`examples/mini_50MHz_spectrogram_comparison.png` is an initial comparison
using the first decoded payload from `scan2400/mini_50MHz.cf32` and a capture
crop recentered by 9.63 MHz. The OFDM occupancy, symbol transitions, and the
two ZC-symbol features line up visually. This is a shape comparison; the
separate decoder round-trip is the CRC evidence.
