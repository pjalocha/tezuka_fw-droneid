# Pluto+ DroneID dual-RX triggered-capture FPGA build

Built with Vivado 2023.2 for XC7Z010, from Maia Pluto+ IIO project with the
DroneID dual-RX detector, 1 ms pre-trigger ring, 1 ms post-trigger capture,
and ADC DMA mux for triggered IQ delivery to Linux.

Artifacts: `droneid-capture-dualrx-r22.bit` is the routed bitstream and
`droneid-capture-dualrx-r22.xsa` is its hardware platform. Vivado reports are
in `reports/`; the integration RTL and install script are in `source/`.

Placed utilization: 11,195 LUTs (63.61%), 18,084 registers (51.38%), 55.5/60
BRAM tiles (92.50%), and 58/80 DSPs (72.50%). Routed timing: WNS +0.595 ns,
WHS +0.020 ns; all specified timing constraints met. DRC completed with zero
errors; it reported warnings and advisories documented in the reports.

The bitstream has not been tested on a physical PlutoSDR+. Confirm IIO channel
ordering, DMA handoff, trigger capture data and RX/TX operation on hardware.
