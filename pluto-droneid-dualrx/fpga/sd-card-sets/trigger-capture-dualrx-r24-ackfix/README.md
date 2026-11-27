# Pluto+ DroneID dual-RX trigger capture SD set (r24 ACK fix)

This set is based on the r23 GPIO-fixed Pluto+ image and updates the FPGA raw bitstream to fix capture rearming. Linux ACKs after consuming a full fixed-size IIO buffer; the FPGA accepts that ACK while in `ST_DRAIN`, where the ADC DMAC may have stopped requesting samples. The ACK clears the completed capture and returns to pretrigger history.

Copy the files and directories in this folder to the FAT boot partition of the PlutoSDR+ SD card, keeping a backup of the previous set. `uEnv.txt` selects `droneid-capture-dualrx-r24-ackfix_top.bin` and `mode=2r2t`. The selected FPGA file is Bootgen-converted raw data, as U-Boot expects. Keep the matching updated `pluto-droneid-capture` application installed; the app and bitstream contain the two halves of this ACK handshake.

The rebuilt ARM application is at `../../builds/plutoplus-maia-r24-trigger-capture-dualrx-ackfix/pluto-droneid-capture` relative to this directory. Copy it to the running Pluto as `/usr/bin/pluto-droneid-capture` after the new FPGA set is on the SD card; reboot once after installing both.

The FPGA captures 1 ms before and 1 ms after each detector/software trigger on each RX chain and sends 30,720 complex samples per chain through ADC DMA. After boot, verify `ad9361-phy` and `cf-ad9361-lpc` appear under `/sys/bus/iio/devices/`, then test one software trigger and a second capture to verify rearming.

Routed timing: WNS +0.987 ns, WHS +0.014 ns; all specified constraints met. The standalone ACK simulation passed. Physical boot/capture rearm has not yet been verified.
