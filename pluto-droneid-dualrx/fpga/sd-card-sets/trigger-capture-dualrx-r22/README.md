# Pluto+ DroneID dual-RX trigger capture SD-card set

Copy every file and directory in this folder to the FAT boot partition of the
PlutoSDR+ SD card, replacing the previous files after making a backup. This is
a separate set; the standard and counter-only sets remain available in their
original folders.

`uEnv.txt` selects `droneid-capture-dualrx-r22_top.bin` and `mode=2r2t`. The
FPGA buffers 1 ms before and 1 ms after a detector or software trigger on each
RX chain, then sends the 30,720 complex samples per chain through the ADC DMA.

## One background/noise capture per second

After boot, log in as root and run:

```sh
mkdir -p /boot/noise
pluto-droneid-capture --channel-hz 2444500000 --noise-monitor \
  --output-dir /boot/noise
```

Stop with Ctrl-C. Each capture writes `*.rx0.cx16`, `*.rx1.cx16`, and a JSON
sidecar. CX16 is signed 16-bit little-endian interleaved I,Q at 15.36 MS/s. The
app needs exclusive use of `cf-ad9361-lpc`; it sets RX LO, 15.36 MS/s and 10 MHz
bandwidth and uses 30 dB manual gain by default. Captures occupy about 240 KiB
per event for both chains, so one per second needs roughly 850 MiB/hour; the files on `/boot`, which is the SD card boot partition on this image; check
free space before long runs.

The FPGA image routed and met timing in Vivado 2023.2. Hardware DMA ordering,
RX sample contents, and the noise-monitor loop have not yet been verified on a
physical PlutoSDR+. To return to the counter-only image, restore the prior
`uEnv.txt` or choose `double-channel-diagonal-r22`.
