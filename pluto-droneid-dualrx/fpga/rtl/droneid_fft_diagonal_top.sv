// End-to-end single-channel FFT and diagonal trigger prototype.
module droneid_fft_diagonal_top (
    input  wire clk,
    input  wire reset,
    input  wire enable,
    input  wire counter_clear,
    input  wire sample_valid,
    input  wire signed [15:0] sample_i,
    input  wire signed [15:0] sample_q,
    output wire sample_ready,
    output wire overrun,
    output wire trigger_valid,
    output reg [31:0] trigger_count,
    output wire signed [4:0] trigger_offset_bins,
    output wire [31:0] trigger_start_frame,
    output wire [36:0] trigger_ridge_sum,
    output wire [36:0] trigger_background_sum
);
    wire fft_valid;
    wire fft_ready;
    wire [5:0] fft_bin_index;
    wire signed [15:0] fft_i;
    wire signed [15:0] fft_q;
    // The windowed, normalized 64-point FFT from 12-bit Pluto ADC samples
    // fits in signed 14-bit values. Keep the detector arithmetic at that
    // precision instead of carrying two redundant sign bits through every
    // square, history sum, and threshold comparison.
    wire signed [13:0] fft_i_detector = fft_i[13:0];
    wire signed [13:0] fft_q_detector = fft_q[13:0];

    droneid_fft64_overlap_maia fft (
        .clk(clk), .reset(reset),
        .sample_valid(sample_valid && enable), .sample_i(sample_i), .sample_q(sample_q),
        .sample_ready(sample_ready), .fft_valid(fft_valid), .fft_ready(fft_ready),
        .fft_bin_index(fft_bin_index), .fft_i(fft_i), .fft_q(fft_q),
        .overrun(overrun)
    );

    droneid_diagonal_trigger #(.INPUT_WIDTH(14)) detector (
        .clk(clk), .reset(reset),
        .fft_valid(fft_valid), .fft_ready(fft_ready),
        .fft_bin_index(fft_bin_index),
        .fft_i(fft_i_detector), .fft_q(fft_q_detector),
        .trigger_valid(trigger_valid),
        .trigger_offset_bins(trigger_offset_bins),
        .trigger_start_frame(trigger_start_frame),
        .trigger_ridge_sum(trigger_ridge_sum),
        .trigger_background_sum(trigger_background_sum)
    );

    always @(posedge clk) begin
        if (reset || counter_clear)
            trigger_count <= 32'd0;
        else if (trigger_valid)
            trigger_count <= trigger_count + 1'b1;
    end
endmodule
