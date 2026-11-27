# Pluto DroneID transmitter

This is an experimental PlutoSDR+ transmitter for one locally generated DJI
DroneID OFDM frame. It is kept separate from the offline PHY reference in
`../droneid-encode` and from the receiver in `../scan2400`.

The bundled example is synthetic. Its aircraft, operator, and home positions
are all set to **51.5007 N, 0.1246 W** (a plausible test coordinate near
central London); the identifier is `TEST-DRONEID-01`. It is not a captured
aircraft identity. The payload and 15.36/30.72 Ms/s CF32 waveforms are generated
by `make_test_packet.py` using the independent Python encoder.

## Build the sample waveform

Requires Python 3, NumPy, and SciPy:

```sh
python3 make_test_packet.py
```

This writes `examples/test-payload.bin`, CF32 waveforms at both 15.36 and
30.72 Ms/s, and JSON sidecars. The generated packet has a valid DUML CRC and
PHY FEC. 15.36 Ms/s is the PHY's native rate; 30.72 Ms/s is a 2x oversampled
version.

## Build the Pluto app

From the root of this Tezuka fork (with the Pluto+ Buildroot configuration):

```sh
make -C buildroot \
  O="$PWD/output/plutoplus" \
  BR2_EXTERNAL="$PWD:$PWD/pluto-droneid-dualrx" \
  olddefconfig
```

Enable `BR2_PACKAGE_PLUTO_DRONEID` in that Buildroot configuration, then build
the package with the same paths:

```sh
make -C buildroot \
  O="$PWD/output/plutoplus" \
  BR2_EXTERNAL="$PWD:$PWD/pluto-droneid-dualrx" \
  pluto-droneid-rebuild
```

The package installs
`/usr/bin/pluto-droneid-tx`, `/usr/bin/pluto-droneid-count`,
`/usr/bin/pluto-droneid-capture`, and the reference waveform under
`/usr/share/pluto-droneid/`.

## Read the FPGA trigger counter

After building an FPGA image with the Maia counter integration, the Linux
utility reads the event counter at `0x7c460038`. It does not open the UIO
device or read/transfer ADC I/Q:

```sh
pluto-droneid-count --clear
pluto-droneid-count --watch-ms 250
```

`--clear` resets the event count and leaves detection enabled. `--watch-ms`
prints the total and newly detected candidate count each interval. These are
detector events, not CRC-valid packets; compare increments with a known
transmission and confirm packet decoding separately.

## Send one packet

The RF channel argument is the intended DroneID channel center. Sample rate,
RF filter bandwidth, and the offset from the TX LO are separate settings. At
15.36 Ms/s, use a small offset: the occupied packet is roughly 9 MHz wide, and
a 0.5 MHz upper-side shift puts it clear of DC while fitting within the
15.36 Ms/s Nyquist band and nominal 10 MHz RF filter. This leaves little filter
margin, so confirm it with a capture. The app sends one finite IIO DMA buffer
and exits; `--repeat-ms` explicitly enables repeated transmission.
In repeat mode the app reports successful IIO buffer pushes once per second.
This is a software submission count, not an RF power or over-the-air delivery
measurement. Ctrl-C or SIGTERM stops the loop and prints a final count.

```sh
pluto-droneid-tx --channel-hz 2437000000 --side upper \
  --sample-rate-hz 15360000 --offset-hz 500000 \
  --rf-bandwidth-hz 10000000 \
  --waveform /usr/share/pluto-droneid/test-packet-15m36.cf32
```

Use a conducted setup with suitable attenuation for initial RF validation.
This app does not start transmitting at boot. Its default TX hardware gain is
the minimum setting (-89.75 dB); set `--gain-db` deliberately when measuring.

The current first version does not yet provide live payload editing, packet
scheduling, channel scanning, or RF feedback. Validate the received burst and
decoded coordinates with the scanner before relying on any RF output.

## DJI DroneID packet transmitter

`dji-droneid-tx` builds the DUML payload, CRC, turbo-coded OFDM frame, and TX
samples on the Pluto. It uses local IIO and has no SoapySDR, Python, NumPy, or
SciPy runtime dependency. The waveform rate is 15.36 MS/s, matching the
channel scanner. The app reads the current TX sample-rate attribute and leaves
it untouched when already at 15.36 MS/s. If TX needs a rate change, it only
attempts that while the reported RX rate is also 15.36 MS/s, then verifies RX
rate remained unchanged. It never writes RX LO, RX gain, RX bandwidth, or RX
sample rate.

The positional `--hops` frequencies are the desired on-air channel centers.
By default, it offsets the TX LO 500 kHz below each channel and digitally
shifts the waveform up by 500 kHz to avoid placing the burst at DC. Select
`--side lower` for the corresponding lower-side placement. It transmits one
packet per period and cycles through the supplied channels. The app is not
started by the boot service; start it manually when needed.
Select the RF chain with `--tx-channel 0` (default) or `--tx-channel 1`; the
second setting uses the AD9361 TX2 I/Q pair and its independent gain control.
If the loaded FPGA/device-tree combination does not expose that pair, the app
reports that TX chain 1 is unavailable instead of silently using TX0.

```sh
dji-droneid-tx --hops 2444500000 --gain 0 --count 10 \
  --lat 51.5007 --lon -0.1246 --alt 35 --height 30

# Select the second TX chain for comparison
dji-droneid-tx --hops 2444500000 --gain 0 --tx-channel 1 --count 10
```

The default identity is `TEST-DRONEID-01`; set `--aircraft-id` and valid
coordinates before a controlled packet test. `--count 0` runs until Ctrl-C.
For initial RF checks, use a conducted setup with suitable attenuation.

## Capture I/Q around triggers

The dual-RX capture variant uses the FPGA diagonal detector or a Linux-issued
software trigger. It saves 1 ms before and 1 ms after the trigger from both RX
chains as `.rx0.cx16` and `.rx1.cx16` (signed little-endian int16 I/Q), plus a
JSON sidecar. The software trigger is useful for taking noise/background
samples without an RF event:

```sh
# One detector-triggered capture
pluto-droneid-capture --channel-hz 2444500000 --count 1 --output-dir /tmp/captures

# One software-triggered background capture
pluto-droneid-capture --software-trigger --count 1 --output-dir /tmp/background

# Repeat software-triggered captures every 10 seconds
pluto-droneid-capture --software-trigger-interval-ms 10000 --output-dir /tmp/background

# Noise-monitor mode: save a software-triggered I/Q window every second
pluto-droneid-capture --noise-monitor --output-dir /tmp/background
```

## Pluto channel scan and RF overview

`pluto-droneid-scan` hops the 2.4 GHz DroneID channel centers, tunes RX at
15.36 MS/s with 10 MHz RF bandwidth, and takes one FPGA software-triggered
2 ms IQ probe on each hop. The default channel list is `2399.5 + N*15 MHz`
for `N=0..5`. RX gain defaults to 30 dB and can be adjusted with `--gain-db`.
The FPGA detector remains enabled during each channel dwell. Its hardware
trigger counters are tallied per channel; the software probe has a separate
counter and is never included in the detector-trigger total.
Pass `--save-triggers DIR` to save every completed capture containing an FPGA
detector event. Pure software-triggered probe captures are excluded. Each
candidate is stored as RX0 and RX1 `.cx16` files plus a JSON sidecar containing
the tuned frequency, sample rate, trigger source, RX mask, and sample offset.
The scanner also runs the first offline PHY analysis on both chains: a 64-point
overlapping FFT chirp search reports the matched-ridge SNR, slope, duration and
chirp edges; its packet-activity fit then selects the active interval for a
long FFT edge/CFO fit. Per-chain results and explicit fit validity/rejection
reasons are added to the JSON. A chirp candidate is only a measurement; it is
not counted as a decoded DroneID packet.
Capture draining is kept separate from this analysis and file writing. The
scanner copies each completed DMA capture, acknowledges the FPGA, and queues
hardware candidates for a worker that prioritizes them over software probes.
The candidate queue holds 32 dual-RX captures (about 7.5 MiB); if it fills, the
capture thread waits for space instead of discarding an already-read candidate.
The software-probe queue holds two captures and drops probes under load because
they are monitoring data. `/status.json` reports candidate queue depth and
high-water mark, candidate enqueue waits, probe drops, save failures, and the
FPGA's cumulative dropped-trigger counter. A nonzero FPGA dropped-trigger
counter still means the hardware capture path missed events before Linux could
queue them; the worker cannot recover those samples.
The background service enables this option by default and writes to
`/boot/droneid-captures`, which is persistent SD storage in this Tezuka image.
The HTTP table's Saved candidates column counts successful file sets per
channel. Each dual-RX candidate uses about 240 KiB before its small JSON file.

```sh
pluto-droneid-scan --save-triggers /boot/droneid-captures
```

The HTTP page listens on port 8080 by default. Open `http://plutoplus.local:8080/`
from a computer on the same network. It shows channel frequency, separate RX0
and RX1 median FFT-bin noise in dBFS/bin across all bins and FFT slices in each
2 ms probe, and separate FPGA detector counts. Spectrogram columns use that
same capture-wide median as their intensity reference and the natural-log scale
used by `dji-droneid-rx`.
The status line includes available Linux thermal, hwmon and IIO temperature
sensors. A single combined spectrogram at `/spectrogram-all.bmp` places the
frequency label before each channel row, with RX0 and RX1 panels side by side.
Each table row has one Spectrum link; it opens a page with the two same-channel
histories next to each other. Live counters and sensor readings are at
`/status.json`. The page gain control sets both RX chains; changes take effect
at the next channel hop. `--gain-db` still sets the startup gain.

Useful options:

```sh
pluto-droneid-scan --gain-db 45 --dwell-ms 300 --http 8080
pluto-droneid-scan --hops 2414500000,2444500000,2474500000
```

Both RX chains are captured and analyzed on each software-probe capture. Each
spectrogram uses 64-point FFTs with 50% overlap. Its time axis concatenates the
probe windows collected on successive visits to each channel; it does not imply
continuous recording between hops.
The channel detector counters continue to count FPGA detector events during
the dwell and exclude the software-trigger probe.

## Background service and task configuration

The Tezuka image uses BusyBox init and `/etc/init.d`, so this package installs
`S70pluto-droneid`. It starts enabled tasks during boot and stops them during
shutdown. No separate procserv daemon is required. The `pluto-droneid` command
is the small task shellbox; it reads `/etc/pluto-droneid.conf` and manages each
task with BusyBox `start-stop-daemon`.

```sh
pluto-droneid status
pluto-droneid stop scan
pluto-droneid start scan
pluto-droneid start tx
pluto-droneid stop tx
pluto-droneid restart all
```

The default config enables the channel scanner with 30 dB gain. Edit the
`task|autostart|executable|arguments` rows in `/etc/pluto-droneid.conf` to
change its options or register another task. Set `autostart` to `no` to leave
a task available for manual starts without launching it on boot. Arguments in
the final field are whitespace-separated tokens; the config is data and is
not evaluated as a shell script. The transmitter is registered as `tx|no` by
default: start it explicitly with `pluto-droneid start tx` and stop it with
`pluto-droneid stop tx`. Change that row to `yes` only if it should transmit
automatically after every boot.

The app configures the RX LO, 15.36 MS/s sample rate, 10 MHz bandwidth and
manual gain (30 dB default). It needs root for `/dev/mem` register access and
exclusive use of the RX IIO DMA while running. The capture FPGA variant is
implemented in `fpga/rtl` and added by `fpga/integrate_plutoplus_dualrx.py`.
The existing routed `double-channel-diagonal-r22` image remains a counter-only
fallback. The capture variant is now routed with Vivado 2023.2 and meets timing;
see `fpga/builds/plutoplus-maia-r22-trigger-capture-dualrx/` and its separate
SD-card set. The placed design uses 55.5 of 60 BRAM tiles, 11,195 LUTs, and 58
DSPs. On-device DMA/channel-layout and capture-content checks are still needed.
