#!/bin/sh
set -eu

SIM_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
FPGA_DIR=$(CDPATH= cd -- "$SIM_DIR/.." && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$FPGA_DIR/.." && pwd)
WORKSPACE_DIR=$(CDPATH= cd -- "$PROJECT_DIR/.." && pwd)
CAPTURE="$WORKSPACE_DIR/dji-drone-rx/reference/mini_15.36MHz_shift-9.5MHz.cs8"
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT HUP INT TERM

if ! command -v iverilog >/dev/null 2>&1 || ! command -v vvp >/dev/null 2>&1; then
    echo "iverilog and vvp are required" >&2
    exit 2
fi

iverilog -g2012 -Wall -s tb_diagonal_trigger \
    -o "$TMP_DIR/diagonal_trigger.vvp" \
    "$FPGA_DIR/rtl/droneid_diagonal_trigger.sv" \
    "$SIM_DIR/tb_diagonal_trigger.sv"

python3 "$FPGA_DIR/tools/make_fft_vectors.py" \
    --capture "$CAPTURE" --format cs8 --output "$TMP_DIR/capture.mem"
vvp "$TMP_DIR/diagonal_trigger.vvp" \
    +VECTORS="$TMP_DIR/capture.mem" +EXPECT=10

python3 "$FPGA_DIR/tools/make_fft_vectors.py" \
    --noise-frames 256 --seed 1 --output "$TMP_DIR/noise.mem"
vvp "$TMP_DIR/diagonal_trigger.vvp" \
    +VECTORS="$TMP_DIR/noise.mem" +EXPECT=0

iverilog -g2012 -s tb_fft_diagonal \
    -o "$TMP_DIR/fft_diagonal.vvp" \
    "$FPGA_DIR/rtl/droneid_maia_fft64.v" \
    "$FPGA_DIR/rtl/droneid_fft64_overlap_maia.sv" \
    "$FPGA_DIR/rtl/droneid_diagonal_trigger.sv" \
    "$FPGA_DIR/rtl/droneid_fft_diagonal_top.sv" \
    "$SIM_DIR/tb_fft_diagonal.sv"
python3 "$FPGA_DIR/tools/make_fft_vectors.py" \
    --capture "$CAPTURE" --format cs8 --samples --output "$TMP_DIR/samples.mem"
vvp "$TMP_DIR/fft_diagonal.vvp" \
    +SAMPLES="$TMP_DIR/samples.mem" +EXPECT=10

iverilog -g2012 -s tb_droneid_pluto_axi \
    -o "$TMP_DIR/droneid_pluto_axi.vvp" \
    "$FPGA_DIR/rtl/droneid_maia_fft64.v" \
    "$FPGA_DIR/rtl/droneid_fft64_overlap_maia.sv" \
    "$FPGA_DIR/rtl/droneid_diagonal_trigger.sv" \
    "$FPGA_DIR/rtl/droneid_fft_diagonal_top.sv" \
    "$FPGA_DIR/rtl/droneid_pluto_axi.v" \
    "$SIM_DIR/tb_droneid_pluto_axi.sv"
vvp "$TMP_DIR/droneid_pluto_axi.vvp"

iverilog -g2012 -Wall -s tb_iq_capture_ack \
    -o "$TMP_DIR/iq_capture_ack.vvp" \
    "$FPGA_DIR/rtl/droneid_iq_capture_dual.sv" \
    "$SIM_DIR/tb_iq_capture_ack.sv"
vvp "$TMP_DIR/iq_capture_ack.vvp"
