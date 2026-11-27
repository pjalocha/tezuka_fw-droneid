# Pluto+ DroneID TX and RX notes

This is the standard FPGA image with the DroneID TX software in its root
filesystem. It does not contain the custom diagonal-trigger FPGA block, so
`pluto-droneid-count` will not report detector events with this image.

## Tune RX

Run these commands on the Pluto as root. Replace the example frequency with
the channel center to receive.

```sh
PHY=$(grep -l '^ad9361-phy$' /sys/bus/iio/devices/iio:device*/name | head -n1)
PHY=${PHY%/name}
echo 2444500000 > "$PHY/out_altvoltage0_RX_LO_frequency"
echo 15360000 > "$PHY/in_voltage_sampling_frequency"
echo 10000000 > "$PHY/in_voltage_rf_bandwidth"
cat "$PHY/out_altvoltage0_RX_LO_frequency"
```

RX LO, RX sample rate, and RX filter bandwidth are separate settings. The
DroneID FPGA detector builds expect 15.36 MS/s; this standard image does not
include that detector. The AD936x exposes separate RX and TX LOs:
`out_altvoltage0_RX_LO_frequency` and `out_altvoltage1_TX_LO_frequency`.

## Transmit a test packet

The TX app sends one finite waveform buffer and exits. Use a valid 2.4 GHz
DroneID channel center (`2399.5 MHz + N * 15 MHz`) for `--channel-hz`.

```sh
pluto-droneid-tx --channel-hz 2444500000 --side upper \
  --offset-hz 500000 --sample-rate-hz 15360000 \
  --rf-bandwidth-hz 10000000 --gain-db -20 \
  --waveform /usr/share/pluto-droneid/test-packet-15m36.cf32
```

`--side` chooses the sign of the offset. For `upper`, TX LO is
`channel-hz - offset-hz`; for `lower`, it is `channel-hz + offset-hz`. The
packet is shifted onto the requested channel center. To repeat, add
`--repeat-ms 100`; this submits another finite packet buffer after each pause.
The TX LO setting may remain after the command stops, but the app disables its
TX data channels, so the setting alone does not mean packets continue to be
sent. `--channel-hz` changes TX only; tune RX separately with the commands
above. IIO buffer submission messages do not verify over-the-air reception.

Start at low TX power and use appropriate attenuation for conducted tests.
