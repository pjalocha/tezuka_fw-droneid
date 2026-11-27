# Maia R22 dual-RX DroneID detector candidate

This is the separately built Pluto+ FPGA image with one shared 64-point Maia
FFT scheduled across both AD9361 receive chains and independent diagonal
detectors/counters. It processes RX0 and RX1 at the same LO and sample rate; it
does not tune the two chains to different frequencies.

## Build and validation

- Vivado 2023.2 routed and generated the bitstream for the XC7Z010.
- Routed timing: WNS +0.314 ns, WHS +0.029 ns, zero failing endpoints; all
  supplied timing constraints are met.
- Placement utilization: 10,109 LUTs (57.44%), 16,748 registers (47.58%),
  7.5 BRAM tiles (12.50%), and 58 DSPs (72.50%).
- Vivado DRC: zero errors; 53 warnings and 22 advisories, mostly existing
  design checks. The detailed reports are in `reports/`.
- Icarus replay of the full reference capture detected all ten events with
  RX0 selected and all ten with RX1 selected. Both counters were independent.

This candidate has not been booted or measured on Pluto hardware. The SD file
set is in `../../sd-card-sets/double-channel-diagonal-r22/` and is separate from
the standard and single-channel sets.

## Artifacts

- `droneid-maia-r22-dualrx.xsa`: exported Maia hardware platform.
- `droneid-maia-r22-dualrx.bit`: routed FPGA bitstream.
- `source/`: exact custom RTL and integration script used for the build.
- `reports/`: routed timing, DRC, utilization and clock reports.
