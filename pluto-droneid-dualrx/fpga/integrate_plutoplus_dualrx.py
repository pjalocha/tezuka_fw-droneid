#!/usr/bin/env python3
"""Install the dual-RX shared-FFT DroneID detector in a Maia HDL tree.

The detector replaces Maia's large spectrometer block. Both AD9361 receive
chains feed independent diagonal detectors through one time-shared Maia FFT.
This script only
edits the generated Maia HDL checkout passed on the command line; it does not
modify this repository's Tezuka checkout or XSA.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re


MARKER = "# PLUTO_DRONEID_FPGA_DUALRX_INTEGRATION"
TIMING_MARKER = "# PLUTO_DRONEID_FPGA_DUALRX_TIMING"
TIMING_FILENAME = "pluto_droneid_dualrx_timing.xdc"
RTL_FILES = (
    "droneid_fft64_shared_2rx.sv",
    "droneid_maia_fft64.v",
    "droneid_diagonal_trigger.sv",
    "droneid_fft_diagonal_dual_top.sv",
    "droneid_iq_capture_dual.sv",
    "droneid_pluto_axi_dualrx.v",
    "droneid_adc_dma_mux.v",
)


INTEGRATION = f"""
{MARKER}: begin
add_files -norecurse ../pluto/droneid_maia_fft64.v
add_files -norecurse ../pluto/droneid_fft64_shared_2rx.sv
add_files -norecurse ../pluto/droneid_diagonal_trigger.sv
add_files -norecurse ../pluto/droneid_fft_diagonal_dual_top.sv
add_files -norecurse ../pluto/droneid_iq_capture_dual.sv
add_files -norecurse ../pluto/droneid_pluto_axi_dualrx.v
add_files -norecurse ../pluto/droneid_adc_dma_mux.v

ad_ip_instance xlslice adc_i_slice1
ad_ip_parameter adc_i_slice1 CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_i_slice1 CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_i_slice1 CONFIG.DIN_FROM 11
ad_ip_instance xlslice adc_q_slice1
ad_ip_parameter adc_q_slice1 CONFIG.DIN_TO 0
ad_ip_parameter adc_q_slice1 CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_q_slice1 CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_q_slice1 CONFIG.DIN_FROM 11

create_bd_cell -type module -reference droneid_pluto_axi_dualrx droneid_rx
create_bd_cell -type module -reference droneid_adc_dma_mux droneid_dma_mux

# RX0 is processed in the AD9361 l_clk domain. The AXI-Lite bank is
# clocked from the PS 100 MHz domain; the wrapper synchronizes control/status
# signals and Gray-coded counters across this boundary.
ad_connect axi_ad9361/l_clk droneid_rx/rx_clk
ad_connect axi_ad9361/rst droneid_rx/rx_reset
ad_connect axi_ad9361/adc_valid_i0 droneid_rx/sample_valid0
ad_connect adc_i_slice/Dout droneid_rx/sample_i0
ad_connect adc_q_slice/Dout droneid_rx/sample_q0
ad_connect axi_ad9361/adc_valid_i1 droneid_rx/sample_valid1
ad_connect adc_i_slice1/Dout droneid_rx/sample_i1
ad_connect adc_q_slice1/Dout droneid_rx/sample_q1
ad_connect axi_ad9361/adc_data_i1 adc_i_slice1/Din
ad_connect axi_ad9361/adc_data_q1 adc_q_slice1/Din
ad_connect sys_cpu_clk droneid_rx/s_axi_aclk
ad_connect sys_cpu_resetn droneid_rx/s_axi_aresetn

# Reuse the normal IIO ADC DMA. Capture mode substitutes the frozen FPGA
# window for cpack's live stream, while preserving the DMA xfer request.
ad_connect droneid_rx/capture_mode droneid_dma_mux/capture_mode
ad_connect droneid_rx/capture_dma_data droneid_dma_mux/capture_data
ad_connect droneid_rx/capture_dma_valid droneid_dma_mux/capture_valid
{MARKER}: end
"""


DMA_WIRING = f"""
{MARKER}: DMA wiring begin
ad_connect cpack/packed_fifo_wr droneid_dma_mux/live_fifo
ad_connect droneid_dma_mux/dma_fifo axi_ad9361_adc_dma/fifo_wr
ad_connect droneid_dma_mux/dma_request droneid_rx/capture_dma_request
{MARKER}: DMA wiring end
"""


CPU_INTERCONNECT = f"""
{MARKER}: cpu interconnect begin
ad_cpu_interconnect 0x7C460000 droneid_rx
{MARKER}: cpu interconnect end
"""


TIMING_CONSTRAINTS = f"""
{TIMING_MARKER}: begin
# The AD9361 sample clock and PS FPGA clock are asynchronous. The wrapper
# synchronizes controls and compact status; raw sample memory remains in l_clk.
set_clock_groups -asynchronous \\
  -group [get_clocks rx_clk] \\
  -group [get_clocks clk_fpga_0]
{TIMING_MARKER}: end
"""


def remove_marked(text: str, marker: str, suffixes: tuple[str, ...]) -> str:
    for suffix in suffixes:
        begin_marker = f"{marker}: {suffix + ' ' if suffix else ''}begin\n"
        end_marker = f"{marker}: {suffix + ' ' if suffix else ''}end\n"
        while begin_marker in text:
            begin = text.index(begin_marker)
            end = text.index(end_marker, begin) + len(end_marker)
            text = text[:begin] + text[end:]
    return text


def remove_maia_spectrometer(text: str) -> str:
    """Remove the Maia SDR IP, its DDR masters, and its sweeper helper."""
    text, clocking_blocks = re.subn(
        r"(?ms)^# Maia SDR clocking\s*\n.*?(?=^# connections\s*$)",
        "",
        text,
        count=1,
    )
    if clocking_blocks != 1:
        # Reapplying the integration script is supported: the first pass has
        # already removed this clock block and every Maia spectrometer ref.
        if (clocking_blocks == 0 and "maia_sdr" not in text and
                "interclk_i" not in text and "interclk_q" not in text):
            return text
        raise SystemExit("could not isolate Maia clock-generator block")
    lines = text.splitlines(keepends=True)
    kept: list[str] = []
    skip_lines = 0
    remove_sweeper = False
    for index, line in enumerate(lines):
        if skip_lines:
            skip_lines -= 1
            continue
        if "###### SWEEPER" in line:
            remove_sweeper = True
            continue
        if remove_sweeper:
            if "logic_orgpio/Op2" in line:
                remove_sweeper = False
            continue
        if "create_bd_addr_seg" in line:
            end = index
            while end + 1 < len(lines) and lines[end].rstrip().endswith("\\"):
                end += 1
            if any("maia_sdr/" in segment or "HP1_DDR_LOWOCM" in segment
                   for segment in lines[index:end + 1]):
                skip_lines = end - index
                continue
        if "CONFIG.PCW_USE_S_AXI_HP1" in line or "S_AXI_HP1_ACLK" in line:
            continue
        if "maia_sdr_clk" in line:
            continue
        if "interclk_i" in line:
            if "m_axis_tdata mux_decim_i/data_in_1" in line:
                kept.append("\tad_connect axi_ad9361/adc_data_i0 mux_decim_i/data_in_1\n")
            elif "m_axis_tvalid mux_decim_i/valid_in_1" in line:
                kept.append("\tad_connect axi_ad9361/adc_valid_i0 mux_decim_i/valid_in_1\n")
            continue
        if "interclk_q" in line:
            if "m_axis_tdata mux_decim_q/data_in_1" in line:
                kept.append("\tad_connect axi_ad9361/adc_data_q0 mux_decim_q/data_in_1\n")
            elif "m_axis_tvalid mux_decim_q/valid_in_1" in line:
                kept.append("\tad_connect axi_ad9361/adc_valid_q0 mux_decim_q/valid_in_1\n")
            continue
        if ("maia_sdr/" in line or
                " maia_sdr\n" in line or
                " maia_sdr\r\n" in line):
            continue
        kept.append(line)
    return "".join(kept)


def select_dual_rf_chain(text: str) -> str:
    """Enable both AD9361 receive chains while retaining the TX chain."""
    parameter = re.compile(
        r"(?m)^([ \t]*ad_ip_parameter axi_ad9361 CONFIG\.MODE_1R1T) [01]([ \t]*)$"
    )
    text, count = parameter.subn(r"\g<1> 0\g<2>", text)
    if count == 0:
        if re.search(
                r"(?m)^[ \t]*ad_ip_parameter axi_ad9361 CONFIG\.MODE_1R1T 0[ \t]*$",
                text):
            return text
        raise SystemExit("could not find the AD9361 MODE_1R1T parameter")
    return text


def ensure_ps_gpio_output_wiring(text: str) -> str:
    """Keep PS GPIO outputs connected to the Pluto transceiver pins.

    GPIO 67 drives AD9361 RESETB and GPIO 66 controls EN_AGC. Without this
    connection the Linux driver sees 0xff from the AD9361 SPI product-ID read.
    Maia's xlslice LED connection only observes gpio_o; it does not replace the
    PS GPIO output driver.
    """
    active = re.compile(r"(?m)^ad_connect\s+gpio_o\s+sys_ps7/GPIO_O\s*$")
    if active.search(text):
        return text
    commented = re.compile(r"(?m)^#\s*ad_connect\s+gpio_o\s+sys_ps7/GPIO_O\s*$")
    text, count = commented.subn("ad_connect  gpio_o sys_ps7/GPIO_O", text)
    if count:
        return text
    anchor = "ad_connect  gpio_i sys_ps7/GPIO_I"
    if anchor not in text:
        raise SystemExit("could not find PS GPIO input connection in Pluto block design")
    return text.replace(anchor, anchor + "\nad_connect  gpio_o sys_ps7/GPIO_O", 1)


def install_timing_constraints(maia: Path) -> None:
    constraints = maia / "projects/pluto/system_constr.xdc"
    timing = maia / "projects/pluto" / TIMING_FILENAME
    project_script = maia / "projects/plutoplus_iio/system_project.tcl"
    if not constraints.is_file() or not project_script.is_file():
        raise SystemExit("Maia Pluto constraints or Pluto+ project script missing")

    constraint_text = remove_marked(
        constraints.read_text(), TIMING_MARKER, ("",))
    constraint_text = remove_marked(
        constraint_text, "# PLUTO_DRONEID_FPGA_TIMING", ("",))
    constraints.write_text(constraint_text.rstrip() + "\n")
    timing.write_text(TIMING_CONSTRAINTS.lstrip())

    project_text = project_script.read_text()
    source_line = '  "../plutoplus/system_constr.xdc" \\\n'
    timing_line = f'  "../pluto/{TIMING_FILENAME}" \\\n'
    old_timing_line = '  "../pluto/pluto_droneid_timing.xdc" \\\n'
    project_text = project_text.replace(old_timing_line, "")
    project_text = project_text.replace(
        "set_property used_in_synthesis false [get_files *pluto_droneid_timing.xdc]\n",
        "")
    if timing_line not in project_text:
        if source_line not in project_text:
            raise SystemExit(f"could not find XDC list in {project_script}")
        project_text = project_text.replace(source_line,
                                            source_line + timing_line, 1)
    prop = f"set_property used_in_synthesis false [get_files *{TIMING_FILENAME}]\n"
    if prop not in project_text:
        list_end = '  "$ad_hdl_dir/library/common/ad_iobuf.v"]\n'
        if list_end not in project_text:
            raise SystemExit(f"could not find project file list in {project_script}")
        project_text = project_text.replace(list_end, list_end + prop, 1)
    project_script.write_text(project_text)


def integrate(maia: Path) -> None:
    maia = maia.resolve()
    bd = maia / "projects/pluto/system_bd.tcl"
    project_script = maia / "projects/plutoplus_iio/system_project.tcl"
    if not bd.is_file() or not project_script.is_file():
        raise SystemExit(f"not a Maia HDL checkout: {maia}")

    source_dir = Path(__file__).resolve().parent / "rtl"
    destination_dir = maia / "projects/pluto"
    for filename in RTL_FILES:
        source = source_dir / filename
        if not source.is_file():
            raise SystemExit(f"missing RTL source: {source}")
        (destination_dir / filename).write_bytes(source.read_bytes())

    text = remove_marked(
        bd.read_text(), MARKER,
        ("", "cpu interconnect", "DMA wiring"))
    # This Maia worktree may already contain the single-channel integration.
    # Replace both its block cell and its CPU address segment before installing
    # the dual-channel variant.
    text = remove_marked(
        text, "# PLUTO_DRONEID_FPGA_INTEGRATION",
        ("", "cpu interconnect"))
    text = remove_maia_spectrometer(text)
    text = select_dual_rf_chain(text)
    text = ensure_ps_gpio_output_wiring(text)
    text = re.sub(
        r"(?m)^[ \t]*ad_connect\s+axi_ad9361_adc_dma/fifo_wr\s+"
        r"cpack/packed_fifo_wr[ \t]*\n", "", text)
    cpu_anchor = "ad_cpu_interconnect 0x79020000 axi_ad9361"
    clock_match = re.search(
        r"(?m)^[ \t]*ad_connect\s+axi_ad9361/l_clk\s+axi_ad9361/clk[^\n]*$",
        text)
    if clock_match is None or cpu_anchor not in text:
        raise SystemExit("Maia Pluto block-design anchors not found")
    insert_at = clock_match.end()
    text = text[:insert_at] + "\n" + INTEGRATION.strip() + text[insert_at:]
    text = text.replace(cpu_anchor,
                        CPU_INTERCONNECT.strip() + "\n" + cpu_anchor, 1)
    cpack_anchor = "\tad_ip_instance util_cpack2 cpack"
    if cpack_anchor not in text:
        raise SystemExit("Maia cpack instance anchor not found")
    text = text.replace(cpack_anchor,
                        cpack_anchor + "\n\n" + DMA_WIRING.strip(), 1)
    bd.write_text(text)
    install_timing_constraints(maia)
    print(f"removed Maia spectrometer, enabled both AD9361 RX chains, and installed "
          f"DroneID detector in {maia}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("maia_hdl", type=Path,
                        help="extracted Maia HDL checkout used by Vivado")
    args = parser.parse_args()
    integrate(args.maia_hdl)


if __name__ == "__main__":
    main()
