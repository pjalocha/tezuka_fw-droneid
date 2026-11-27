# DroneID FFT and diagonal-trigger RTL prototype

This folder contains one fixed-point 64-point FFT front end and a diagonal
chirp trigger based on `../../dji-drone-rx/diagonal_chirp_detector.h`, plus
Icarus Verilog testbenches. It forms 64-sample windows with a 32-sample hop.

The current FFT is Maia SDR's generated 64-point radix-2² single-delay-feedback
core (`droneid_maia_fft64.v`), with 16-bit samples and twiddles and one-bit
truncation at each stage. The overlap adapter applies the Q15
`sin(pi*n/64)/8` window, forms 64-sample frames with a 32-sample hop, and
reorders the core's bit-reversed output into the detector's contiguous ROI.
It schedules frames at 2x the sample input cadence. CS8 test captures are
expanded by 256 to signed-16 input. The detector computes `I^2 + Q^2`, stores
the ROI and its precomputed local-peak flags in one history, then checks only
the center-offset diagonal. Its integer gates
implement the reference 6 dB ridge/background (approximated by a 4:1 power
ratio), 80% flank, and 1.05 temporal-peak checks.

It accepts one sample per FPGA clock and raises `overrun` if a new window would
reuse a bank before the prior FFT/output pass finishes. The detector asserts
backpressure while it scans the 32-row history; at one input sample every four
FPGA clocks, frame output and detector scan overlap the next window's input
pass.

The detector reports a zero-bin candidate offset; it does not estimate the
signal's actual frequency error. This center-only test preserved all ten
events in the current capture, despite the earlier multi-lane search preferring
offsets 1 or 2. The ARM stage can refine frequency and decode the candidate.

## Simulate

Requirements: Icarus Verilog (`iverilog`, `vvp`), Python 3, and NumPy.

```sh
sh pluto-droneid/fpga/sim/run_sim.sh
```

The script tests the diagonal block against software-generated FFT-bin stimuli,
runs a 256-row noise-only case, then sends the full reference capture through
the fixed-point FFT and detector. The reference capture produces ten triggers,
the noise-only test produces none, and the AXI-Lite counter test passes.
Simulation does not establish RF performance on PlutoSDR+.

The earlier handwritten FFT did not fit: the integrated design used 19,320 LUTs
against the XC7Z010's 17,600. The Maia streaming FFT alternative synthesizes the
detector IP to 3,202 LUTs, 4,830 FFs, 1.5 BRAM tiles, and 14 DSPs. The complete
Pluto+ design places at 9,068 LUTs (51.5%), 16,166 FFs (45.9%), 6 BRAM tiles
(10%), and 56 DSPs (70%). Detailed routing and bitstream generation succeeded.
Post-route timing reports WNS 0.595 ns, TNS 0, WHS 0.014 ns, and THS 0.
Vivado 2023.2 reports no DRC errors, with 41 warnings and 22 advisories. The
methodology report flags five scoped inter-clock max-delay constraints; DRC
also flags unpipelined inputs to detector DSP blocks. Timing closes under the
supplied constraints, but those warnings should be reviewed before treating the
design as hardware-validated. Reports are saved in `fpga/reports/`.
The generated XSA/bitstream are archived in `fpga/builds/plutoplus-maia-r22/`,
and the separate single-channel SD-card file set is in
`fpga/sd-card-sets/single-channel-diagonal-r22/`.

## Pluto+ block-design integration

`integrate_plutoplus.py` installs `droneid_pluto_axi_maia` into a generated Maia HDL
checkout's Pluto+ block design. It removes the Maia SDR spectrometer, its
HP1/HP2 DDR masters, the Maia-only RX decimation FIFOs, and the associated
clock generator, and enables both AD9361 receive chains. The detector taps RX0
I/Q in `axi_ad9361/l_clk`; RX1 is enabled but not currently processed. The
existing AD9361 TX DMA/unpacker/FIR chain, PS, and network design remain in
place.

The detector and its AXI-Lite register bank are separate from Maia's `maia_sdr`
core. The CPU map is at base `0x7c460000`: trigger count at `0x38`, and
enable/clear/overrun at `0x3c`. The register bank runs from the PS clock; count
and control cross to the AD9361 sample clock using Gray-count and two-flop
synchronizers. No full-I/Q DMA is added.
Linux continues to control center frequency and sample rate through the AD9361
IIO device.

Apply the integration to the extracted Maia HDL tree that the Tezuka Pluto+
Vivado build uses:

```sh
python3 pluto-droneid/fpga/integrate_plutoplus.py /path/to/maia-hdl
```

Run it after source extraction and before generating the Pluto+ Vivado project.
The detector expects the AD9361 stream at 15.36 MS/s; verify the raw sample
strobe cadence and configure the RX rate from Linux. The Maia DDC path is not
used by this integration.

On the running image, `/usr/bin/pluto-droneid-count` reads the AXI-Lite
registers through `/dev/mem`; `--clear` resets the counter and enables
detection, and `--watch-ms 250` polls it. Counter increments are candidate
events, not proof of successful packet decoding.

The detector simulation checks ten known capture bursts, rejects a noise-only
input, and exercises the AXI-Lite counter map. The integrated design routed and
produced an XSA and bitstream. A separately named SD-card file set is available
for the candidate; no Tezuka default files or physical SD card were changed.
On-device counter behavior and RF performance have not yet been checked.

The standalone vector generator can also be used with another CS8/CS16
capture:

```sh
python3 pluto-droneid/fpga/tools/make_fft_vectors.py \
  --capture capture.cs8 --format cs8 --output /tmp/capture_fft.mem
```

## Dual-RX shared-FFT experiment

This checkout keeps the dual-channel trial separate from the single-channel
baseline. `droneid_fft64_shared_2rx.sv` stores both AD9361 streams, snapshots
the waiting channel's next 64-sample window while the other channel is fed to
the Maia FFT, then alternates complete FFT frames through one core. The
snapshot is needed because each channel continues sampling while the shared
FFT processes the other channel. Each channel has its own reorder buffer and
diagonal detector, so both detector streams can drain at once.

At a 61.44 MHz AD9361 clock, each 15.36 MS/s stream supplies one sample every
four clocks. The shared FFT accepts one sample every clock and alternates
64-clock frames, giving each chain its 64-sample window with a 32-sample hop
every 128 clocks.

The dual integration script is `integrate_plutoplus_dualrx.py`. It enables
both AD9361 receive chains, adds RX1 I/Q slices, replaces an existing
single-channel integration if present, and installs the dual AXI wrapper. The
CPU map keeps RX0's trigger count at `0x7c460038`, adds RX1's count at
`0x7c460040`, and retains enable/clear/overrun at `0x7c46003c`. The count
reader accepts `--channel 0`, `--channel 1`, or `--both`.

The end-to-end dual testbench replays the known CS8 capture on one channel
while sending zeros on the other. RX0 and RX1 each independently report ten
triggers, zero on the other, with no scheduler overrun during live replay.
Vivado 2023.2 routed the full Pluto+ design and generated its XSA/bitstream:
10,109 LUTs, 16,748 registers, 7.5 BRAM tiles, and 58 DSPs; routed WNS is
+0.314 ns and WHS is +0.029 ns, with no failing constrained endpoints and no
DRC errors. The dual artifacts and reports are in
`fpga/builds/plutoplus-maia-r22-dualrx/`; its independent SD-card file set is
in `fpga/sd-card-sets/double-channel-diagonal-r22/`. The design still needs
on-device validation.

## Triggered dual-RX I/Q capture

`droneid_iq_capture_dual.sv` adds one 1 ms pre-trigger ring and one 1 ms
post-trigger buffer for both RX chains. At 15.36 MS/s, a capture is 30,720
simultaneous scans per chain. The completed window is replayed through the
existing AD9361 ADC DMA, so Linux receives a normal fixed-size `cf-ad9361-lpc`
IIO buffer rather than polling or copying the full-rate stream. Captures are
stored as separate signed-int16 little-endian I/Q files, one `.rx0.cx16` and
one `.rx1.cx16`, with a JSON sidecar containing sample rate, center frequency,
trigger index and the hardware/software trigger events recorded inside the
window. The single FPGA buffer is intentionally not a queue: triggers during
the two-ms capture window are recorded up to eight events; later triggers are
counted as dropped until Linux acknowledges and rearms the buffer.

The AXI-Lite capture registers are at base `0x7c460000`: capture control at
`0x44` (bit 0 enable, bit 1 software-trigger toggle, bit 2 acknowledge toggle),
status at `0x48`, trigger sample index at `0x4c`, dropped-trigger count at
`0x50`, event count at `0x54`, and event records at `0x80` through `0x9c`.
Control bits 1 and 2 are toggles: write a 1 once to pulse the corresponding
action. The Linux utility handles these registers and normal IIO DMA setup:

```sh
# Save one capture caused by the FPGA diagonal detector
pluto-droneid-capture --channel-hz 2444500000 --count 1 --output-dir /media/card/captures

# Save one capture caused by a software trigger (useful for background noise)
pluto-droneid-capture --software-trigger --count 1 --output-dir /media/card/noise

# Generate one background capture every 10 seconds until stopped
pluto-droneid-capture --software-trigger-interval-ms 10000 --output-dir /media/card/noise

# Save a background-noise I/Q window every second until stopped
pluto-droneid-capture --noise-monitor --output-dir /media/card/noise
```

Defaults are 15.36 MS/s, 10 MHz RX bandwidth, 30 dB manual RX gain, and a 1 ms
window on each side of the trigger. The utility currently requires 15.36 MS/s,
the detector's configured input rate. The frequency argument sets the RX LO.
The application needs root access for `/dev/mem` and exclusive ownership of
`cf-ad9361-lpc` while capturing.

The routed reference build is `builds/plutoplus-maia-r22-trigger-capture-dualrx/`;
its separate SD-card set is `sd-card-sets/trigger-capture-dualrx-r22/`. Vivado
2023.2 completed synthesis, placement, routing and bitstream generation for the
XC7Z010. The placed design uses 11,195 LUTs, 18,084 registers, 55.5 of 60 BRAM
tiles, and 58 DSPs. Routed timing meets all specified constraints (WNS +0.595
ns, WHS +0.020 ns). The image has 4.5 BRAM tiles of headroom, so check updated
utilization before adding more FPGA features.

The target Linux app compiled and was installed into the Buildroot target
filesystem. Hardware IIO channel ordering, DMA sample handoff, and actual
capture on a PlutoSDR+ still require on-device verification. Successful routing
does not by itself validate sample contents or the noise-monitor loop.

### r23 GPIO reset fix

The first physical boot of the r22 capture set showed only the XADC IIO device;
the kernel logged `Unsupported PRODUCT_ID 0xFF` for the AD9361. Inspection found
that Maia's Pluto block design had the PS GPIO output connection commented out.
The AD9361 RESETB pin is driven by PS GPIO 67, so this could hold the chip in
reset and prevent its IIO devices from registering. The integration script now
restores `gpio_o` to `sys_ps7/GPIO_O` before building.

The regenerated r23 image routes with WNS +0.829 ns and WHS +0.017 ns, with no
route or bitstream DRC errors. Its versioned build and bootable SD set are
`builds/plutoplus-maia-r23-trigger-capture-dualrx-gpiofix/` and
`sd-card-sets/trigger-capture-dualrx-r23-gpiofix/`; the zipped SD set is
`sd-card-sets/pluto-droneid-capture-dualrx-r23-gpiofix-sd.zip`. Device
registration and capture still need confirmation on the physical PlutoSDR+.

### r24 capture ACK/rearm fix

The first software-trigger run saved its two I/Q files but reported that the
FPGA did not acknowledge the capture (`status=0xe`). The fixed-size IIO transfer
can end while the replay FSM is in `ST_DRAIN`, after the DMAC has stopped
requesting words. The Linux application now ACKs after consuming the full
buffer, and the FPGA accepts that ACK in any completed state and returns to
pretrigger history. The standalone FSM testbench reproduces the stalled-DMA
case and passes. The rebuilt r24 dual-RX image meets routed timing (WNS
+0.987 ns, WHS +0.014 ns; zero failing endpoints) and uses 55.5/60 BRAM tiles.
Install the r24 SD set and matching capture application together, then verify
that a second software-trigger capture succeeds on hardware.

Artifacts: `builds/plutoplus-maia-r24-trigger-capture-dualrx-ackfix/`,
`sd-card-sets/trigger-capture-dualrx-r24-ackfix/`, and the zipped SD set
`sd-card-sets/pluto-droneid-capture-dualrx-r24-ackfix-sd.zip`.
