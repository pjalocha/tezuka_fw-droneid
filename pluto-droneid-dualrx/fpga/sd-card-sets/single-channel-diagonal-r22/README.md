# Pluto+ DroneID single-channel detector and TX notes

This image selects the Maia R22 FPGA design with the diagonal detector on RX0.
The detector reports chirp candidates; it does not decode or CRC-check packets.

## Tune RX0 and enable detection

Run as root on the Pluto. Replace the example frequency with the channel center
to monitor.

```sh
PHY=$(grep -l '^ad9361-phy$' /sys/bus/iio/devices/iio:device*/name | head -n1)
PHY=${PHY%/name}
echo 2444500000 > "$PHY/out_altvoltage0_RX_LO_frequency"
echo 15360000 > "$PHY/in_voltage_sampling_frequency"
echo 10000000 > "$PHY/in_voltage_rf_bandwidth"
cat "$PHY/out_altvoltage0_RX_LO_frequency"
pluto-droneid-count --clear --watch-ms 250
```

`--clear` clears the counter and enables detection. This single-channel image
counts RX0 only. The detector processes the AD936x RX stream continuously
while enabled; RX LO, sample rate, and filter bandwidth are configured through
the `ad9361-phy` IIO device.

## Transmit a test packet

Use a valid 2.4 GHz DroneID channel center (`2399.5 MHz + N * 15 MHz`). This
example sends one packet buffer and exits:

```sh
pluto-droneid-tx --channel-hz 2444500000 --side upper \
  --offset-hz 500000 --sample-rate-hz 15360000 \
  --rf-bandwidth-hz 10000000 --gain-db -20 \
  --waveform /usr/share/pluto-droneid/test-packet-15m36.cf32
```

For `upper`, TX LO is `channel-hz - offset-hz`; for `lower`, TX LO is
`channel-hz + offset-hz`. RX LO is separate and is not changed by the TX app.
Add `--repeat-ms 100` to submit packet buffers repeatedly. The TX LO can remain
programmed after the app stops, but the app disables its TX data channels, so
that does not mean continuous packet transmission. If RX is tuned to hear TX,
the detector may count the transmitted chirps; this has not been verified on
hardware, and TX leakage can affect RX.

IIO push counts are software submission counts, not proof of RF output or
successful packet decoding. Start at low TX power and use suitable attenuation
for conducted tests.
