# Pluto+ DroneID dual-RX trigger capture FPGA build (r23 GPIO fix)

Built with Vivado 2023.2 for XC7Z010 from the Maia Pluto+ IIO project. This
regenerates r22 with the PS GPIO output connection restored (`gpio_o` to
`sys_ps7/GPIO_O`), required for PS GPIO 67 to release AD9361 RESETB and GPIO 66
to control EN_AGC. The previous design left that connection commented; Linux
then read AD9361 SPI product ID `0xff` and failed to register its IIO devices.

The routed Vivado `.bit`, Bootgen-converted raw `.bit.bin` for U-Boot, and XSA
are saved here; reports and integration sources are included. The SD set uses
the Bootgen raw bitstream format, matching Tezuka `postimage-sd.sh`. Routed timing: WNS +0.829 ns, WHS +0.017 ns. Route and bitstream
DRC completed with zero errors. The design has not yet been boot-tested on the
physical PlutoSDR+; device registration is the next hardware verification.

The corresponding bootable file set is
`../../sd-card-sets/trigger-capture-dualrx-r23-gpiofix/`.
