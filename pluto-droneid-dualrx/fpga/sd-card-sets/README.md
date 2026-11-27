# Pluto+ SD test sets

Each directory is a complete FAT32 boot-file set: copy its contents to the
root of a separate FAT32 SD card. Each includes a `README.md` with that image's
RX tuning, detector-counter, and TX instructions. These are file sets, not raw
disk images.

- `standard/`: refreshed DroneID-enabled root filesystem with the original
  `system_top.bin` selected.
- `single-channel-diagonal-r22/`: the same files, selecting
  `droneid-maia-r22_top.bin` instead. Both sets retain both bitstream files so
  `uEnv.txt` is the only difference between these two sets.
- `double-channel-diagonal-r22/`: selects the routed shared-FFT dual-RX design,
  enables Pluto `2r2t` mode, and includes a `pluto-droneid-count` build that can
  read both counters.
- `trigger-capture-dualrx-r22/` and `trigger-capture-dualrx-r23-gpiofix/`:
  versioned two-chain I/Q capture images; r23 restores the PS GPIO route needed
  for AD9361 reset and device registration.
- `trigger-capture-dualrx-r24-ackfix/`: r23 plus the capture ACK/rearm fix for
  DMA transfers that finish while the FPGA remains in `ST_DRAIN`. Install it
  together with the matching rebuilt `pluto-droneid-capture` application.
  The standalone ACK simulation and Vivado routed build pass; physical rearm
  remains to be checked.

The default `system_top.bin` is preserved byte-for-byte from the prior Tezuka
Pluto+ output (SHA-256 `c49d150ab82a3db3ffc2e2d0b4b22a20054db20a09afa473eb1f259cef40aa87`).
The source XSA is also preserved; the DroneID XSA is a separately named
side-by-side candidate in the DroneID Tezuka clone.

The single- and double-channel sets passed RTL replay and Vivado routing but
have not been booted on hardware. The double-RX design means both Pluto receive
chains at one LO/sample clock, with independent counters. Check
`pluto-droneid-count --clear --both --watch-ms 250` after boot and confirm the
running frequency/sample-rate configuration before interpreting counts.
