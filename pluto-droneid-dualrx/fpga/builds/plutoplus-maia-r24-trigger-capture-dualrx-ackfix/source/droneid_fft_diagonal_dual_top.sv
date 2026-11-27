// Two independent diagonal detectors sharing one time-multiplexed Maia FFT.
module droneid_fft_diagonal_dual_top (
    input  wire clk,
    input  wire reset,
    input  wire enable,
    input  wire counter_clear,
    input  wire sample_valid0,
    input  wire signed [15:0] sample_i0,
    input  wire signed [15:0] sample_q0,
    input  wire sample_valid1,
    input  wire signed [15:0] sample_i1,
    input  wire signed [15:0] sample_q1,
    output wire sample_ready0,
    output wire sample_ready1,
    output wire overrun,
    output wire trigger_valid0,
    output reg [31:0] trigger_count0,
    output wire trigger_valid1,
    output reg [31:0] trigger_count1
);
    wire fft_valid0, fft_valid1;
    wire fft_ready0;
    wire fft_ready1;
    wire [5:0] fft_bin_index0, fft_bin_index1;
    wire signed [15:0] fft_i0, fft_q0, fft_i1, fft_q1;
    wire signed [13:0] fft_i_detector0 = fft_i0[13:0];
    wire signed [13:0] fft_q_detector0 = fft_q0[13:0];
    wire signed [13:0] fft_i_detector1 = fft_i1[13:0];
    wire signed [13:0] fft_q_detector1 = fft_q1[13:0];

    droneid_fft64_shared_2rx fft_shared (
        .clk(clk), .reset(reset),
        .sample_valid0(sample_valid0 && enable),
        .sample_i0(sample_i0), .sample_q0(sample_q0),
        .sample_valid1(sample_valid1 && enable),
        .sample_i1(sample_i1), .sample_q1(sample_q1),
        .sample_ready0(sample_ready0), .sample_ready1(sample_ready1),
        .fft_valid0(fft_valid0), .fft_ready0(fft_ready0),
        .fft_bin_index0(fft_bin_index0), .fft_i0(fft_i0), .fft_q0(fft_q0),
        .fft_valid1(fft_valid1), .fft_ready1(fft_ready1),
        .fft_bin_index1(fft_bin_index1), .fft_i1(fft_i1), .fft_q1(fft_q1),
        .overrun(overrun)
    );

    droneid_diagonal_trigger #(.INPUT_WIDTH(14)) detector_rx0 (
        .clk(clk), .reset(reset),
        .fft_valid(fft_valid0), .fft_ready(fft_ready0),
        .fft_bin_index(fft_bin_index0),
        .fft_i(fft_i_detector0), .fft_q(fft_q_detector0),
        .trigger_valid(trigger_valid0),
        .trigger_offset_bins(), .trigger_start_frame(),
        .trigger_ridge_sum(), .trigger_background_sum()
    );

    droneid_diagonal_trigger #(.INPUT_WIDTH(14)) detector_rx1 (
        .clk(clk), .reset(reset),
        .fft_valid(fft_valid1), .fft_ready(fft_ready1),
        .fft_bin_index(fft_bin_index1),
        .fft_i(fft_i_detector1), .fft_q(fft_q_detector1),
        .trigger_valid(trigger_valid1),
        .trigger_offset_bins(), .trigger_start_frame(),
        .trigger_ridge_sum(), .trigger_background_sum()
    );

    always @(posedge clk) begin
        if (reset || counter_clear) begin
            trigger_count0 <= 32'd0;
            trigger_count1 <= 32'd0;
        end else begin
            if (trigger_valid0)
                trigger_count0 <= trigger_count0 + 1'b1;
            if (trigger_valid1)
                trigger_count1 <= trigger_count1 + 1'b1;
        end
    end
endmodule
