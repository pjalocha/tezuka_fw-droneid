# Pluto+ DroneID dual-RX trigger capture SD-card set (r23 GPIO fix)

This is the trigger-capture image with the Pluto PS GPIO output connection
restored in the FPGA block design. That route drives AD9361 reset (GPIO 67)
and EN_AGC (GPIO 66); without it Linux reported SPI product ID 0xFF and did
not create the AD9361 or RX IIO devices.

Copy the files and directories in this folder to the FAT boot partition of the
PlutoSDR+ SD card, replacing the previous files after making a backup.
`uEnv.txt` selects `droneid-capture-dualrx-r23-gpiofix_top.bin` and `mode=2r2t`. The selected file is Bootgen-converted raw FPGA data, as required by U-Boot; it is not the Vivado `.bit` container.
The FPGA captures 1 ms before and 1 ms after each detector/software trigger on
each RX chain and sends 30,720 complex samples per chain through ADC DMA.

After boot, verify the RF transceiver registered before running capture:

```sh
for d in /sys/bus/iio/devices/iio:device*; do
    [ -e "$d/name" ] && echo "$d: $(cat "$d/name")"
done
```

Expected devices include `ad9361-phy` and `cf-ad9361-lpc`. Then a one-second
background/noise capture can be run as root:

```sh
mkdir -p /boot/noise
pluto-droneid-capture --channel-hz 2444500000 --noise-monitor \
  --output-dir /boot/noise
```

Each capture writes `*.rx0.cx16`, `*.rx1.cx16`, and a JSON sidecar. CX16 is
signed 16-bit little-endian interleaved I,Q at 15.36 MS/s. The app needs
exclusive use of `cf-ad9361-lpc`; it sets RX LO, 15.36 MS/s, 10 MHz bandwidth,
and 30 dB manual gain. Hardware DMA sample ordering and capture contents still
need verification on the physical device.
