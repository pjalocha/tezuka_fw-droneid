// 64-point 50%-overlap FFT adapter for Maia SDR's radix-2^2 SDF core.
// The input ring stores the 64 newest samples. Each 64-sample frame is fed to
// the core at 30.72 Ms/s, while Pluto's 15.36 Ms/s ADC stream continues to
// arrive at 4x the sample clock. The output is reordered from bit-reversed
// order and rotated so the detector sees its established contiguous ROI.
module droneid_fft64_overlap_maia (
    input  wire clk,
    input  wire reset,
    input  wire sample_valid,
    input  wire signed [15:0] sample_i,
    input  wire signed [15:0] sample_q,
    output wire sample_ready,
    output wire fft_valid,
    input  wire fft_ready,
    output wire [5:0] fft_bin_index,
    output wire signed [15:0] fft_i,
    output wire signed [15:0] fft_q,
    output reg overrun
);
    reg signed [15:0] sample_i_mem [0:63];
    reg signed [15:0] sample_q_mem [0:63];
    reg signed [15:0] output_i_mem [0:1][0:63];
    reg signed [15:0] output_q_mem [0:1][0:63];

    reg [5:0] write_ptr;
    reg [6:0] sample_count;
    reg [4:0] hop_count;
    reg [5:0] frame_start;
    reg feed_active;
    reg feed_phase;
    reg [5:0] feed_index;

    reg [1:0] startup_markers;
    reg [5:0] raw_index;
    reg capture_bank;
    reg [1:0] output_ready;
    reg emit_active;
    reg emit_bank;
    reg [5:0] emit_index;

    wire core_clken = feed_active && !feed_phase;
    wire signed [15:0] core_re_in;
    wire signed [15:0] core_im_in;
    wire signed [15:0] core_re_out;
    wire signed [15:0] core_im_out;
    wire core_out_last;
    wire [5:0] feed_addr = frame_start + feed_index;
    wire signed [15:0] feed_window = window_q15(feed_index);
    wire signed [31:0] feed_prod_i = $signed(sample_i_mem[feed_addr]) * feed_window;
    wire signed [31:0] feed_prod_q = $signed(sample_q_mem[feed_addr]) * feed_window;

    assign core_re_in = round_q15({feed_prod_i[31], feed_prod_i});
    assign core_im_in = round_q15({feed_prod_q[31], feed_prod_q});
    assign sample_ready = 1'b1;
    assign fft_valid = emit_active;
    assign fft_bin_index = emit_index;
    assign fft_i = output_i_mem[emit_bank][(emit_index + 6'd45) & 6'd63];
    assign fft_q = output_q_mem[emit_bank][(emit_index + 6'd45) & 6'd63];

    droneid_maia_fft64 fft_core (
        .clk(clk), .rst(reset), .clken(core_clken),
        .re_in(core_re_in), .im_in(core_im_in),
        .re_out(core_re_out), .im_out(core_im_out), .out_last(core_out_last)
    );

    function [5:0] bit_reverse6;
        input [5:0] value;
        begin
            bit_reverse6 = {value[0], value[1], value[2],
                            value[3], value[4], value[5]};
        end
    endfunction

    function signed [15:0] round_q15;
        input signed [32:0] value;
        reg signed [33:0] extended;
        begin
            extended = {{1{value[32]}}, value};
            if (extended >= 0)
                round_q15 = (extended + 34'sd16384) >>> 15;
            else
                round_q15 = -(((-extended + 34'sd16384) >>> 15));
        end
    endfunction

    function signed [15:0] window_q15;
        input integer index;
        begin
            case (index)
                0: window_q15 = 16'sd0;
                1: window_q15 = 16'sd201;
                2: window_q15 = 16'sd401;
                3: window_q15 = 16'sd601;
                4: window_q15 = 16'sd799;
                5: window_q15 = 16'sd995;
                6: window_q15 = 16'sd1189;
                7: window_q15 = 16'sd1380;
                8: window_q15 = 16'sd1567;
                9: window_q15 = 16'sd1751;
                10: window_q15 = 16'sd1931;
                11: window_q15 = 16'sd2106;
                12: window_q15 = 16'sd2276;
                13: window_q15 = 16'sd2440;
                14: window_q15 = 16'sd2598;
                15: window_q15 = 16'sd2751;
                16: window_q15 = 16'sd2896;
                17: window_q15 = 16'sd3035;
                18: window_q15 = 16'sd3166;
                19: window_q15 = 16'sd3290;
                20: window_q15 = 16'sd3406;
                21: window_q15 = 16'sd3513;
                22: window_q15 = 16'sd3612;
                23: window_q15 = 16'sd3703;
                24: window_q15 = 16'sd3784;
                25: window_q15 = 16'sd3856;
                26: window_q15 = 16'sd3920;
                27: window_q15 = 16'sd3973;
                28: window_q15 = 16'sd4017;
                29: window_q15 = 16'sd4052;
                30: window_q15 = 16'sd4076;
                31: window_q15 = 16'sd4091;
                32: window_q15 = 16'sd4096;
                33: window_q15 = 16'sd4091;
                34: window_q15 = 16'sd4076;
                35: window_q15 = 16'sd4052;
                36: window_q15 = 16'sd4017;
                37: window_q15 = 16'sd3973;
                38: window_q15 = 16'sd3920;
                39: window_q15 = 16'sd3856;
                40: window_q15 = 16'sd3784;
                41: window_q15 = 16'sd3703;
                42: window_q15 = 16'sd3612;
                43: window_q15 = 16'sd3513;
                44: window_q15 = 16'sd3406;
                45: window_q15 = 16'sd3290;
                46: window_q15 = 16'sd3166;
                47: window_q15 = 16'sd3035;
                48: window_q15 = 16'sd2896;
                49: window_q15 = 16'sd2751;
                50: window_q15 = 16'sd2598;
                51: window_q15 = 16'sd2440;
                52: window_q15 = 16'sd2276;
                53: window_q15 = 16'sd2106;
                54: window_q15 = 16'sd1931;
                55: window_q15 = 16'sd1751;
                56: window_q15 = 16'sd1567;
                57: window_q15 = 16'sd1380;
                58: window_q15 = 16'sd1189;
                59: window_q15 = 16'sd995;
                60: window_q15 = 16'sd799;
                61: window_q15 = 16'sd601;
                62: window_q15 = 16'sd401;
                63: window_q15 = 16'sd201;

                default: window_q15 = 16'sd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            write_ptr <= 6'd0;
            sample_count <= 7'd0;
            hop_count <= 5'd0;
            frame_start <= 6'd0;
            feed_active <= 1'b0;
            feed_phase <= 1'b0;
            feed_index <= 6'd0;
            startup_markers <= 2'd0;
            raw_index <= 6'd0;
            capture_bank <= 1'b0;
            output_ready <= 2'b00;
            emit_active <= 1'b0;
            emit_bank <= 1'b0;
            emit_index <= 6'd0;
            overrun <= 1'b0;
        end else begin
            overrun <= 1'b0;

            if (sample_valid && sample_ready) begin
                sample_i_mem[write_ptr] <= sample_i;
                sample_q_mem[write_ptr] <= sample_q;
                write_ptr <= write_ptr + 1'b1;

                if (sample_count < 7'd64) begin
                    sample_count <= sample_count + 1'b1;
                    if (sample_count == 7'd63) begin
                        feed_active <= 1'b1;
                        feed_phase <= 1'b0;
                        feed_index <= 6'd0;
                    end
                end else if (hop_count == 5'd31) begin
                    hop_count <= 5'd0;
                    frame_start <= frame_start + 6'd32;
                    if (feed_active) begin
                        overrun <= 1'b1;
                    end else begin
                        feed_active <= 1'b1;
                        feed_phase <= 1'b0;
                        feed_index <= 6'd0;
                    end
                end else begin
                    hop_count <= hop_count + 1'b1;
                end
            end

            if (feed_active) begin
                if (!feed_phase) begin
                    feed_phase <= 1'b1;
                    if (feed_index == 6'd63)
                        feed_active <= 1'b0;
                    else
                        feed_index <= feed_index + 1'b1;
                end else begin
                    feed_phase <= 1'b0;
                end
            end

            if (core_clken) begin
                if (core_out_last) begin
                    raw_index <= 6'd0;
                    if (startup_markers < 2'd2) begin
                        startup_markers <= startup_markers + 1'b1;
                    end else begin
                        output_i_mem[capture_bank][bit_reverse6(raw_index)] <= core_re_out;
                        output_q_mem[capture_bank][bit_reverse6(raw_index)] <= core_im_out;
                        if (output_ready[capture_bank] ||
                            (emit_active && (emit_bank == capture_bank))) begin
                            overrun <= 1'b1;
                        end else begin
                            output_ready[capture_bank] <= 1'b1;
                            capture_bank <= ~capture_bank;
                        end
                    end
                end else if (startup_markers == 2'd2) begin
                    output_i_mem[capture_bank][bit_reverse6(raw_index)] <= core_re_out;
                    output_q_mem[capture_bank][bit_reverse6(raw_index)] <= core_im_out;
                    raw_index <= raw_index + 1'b1;
                end else if (startup_markers != 2'd0) begin
                    raw_index <= raw_index + 1'b1;
                end
            end

            if (!emit_active) begin
                if (output_ready[0]) begin
                    emit_active <= 1'b1;
                    emit_bank <= 1'b0;
                    emit_index <= 6'd0;
                end else if (output_ready[1]) begin
                    emit_active <= 1'b1;
                    emit_bank <= 1'b1;
                    emit_index <= 6'd0;
                end
            end else if (fft_ready) begin
                if (emit_index == 6'd63) begin
                    emit_active <= 1'b0;
                    output_ready[emit_bank] <= 1'b0;
                end else begin
                    emit_index <= emit_index + 1'b1;
                end
            end
        end
    end

    // Maia SDR FFT RTL is generated with Amaranth; this adapter keeps its
    // bit-reversed transform order hidden from the existing detector stream.
endmodule
