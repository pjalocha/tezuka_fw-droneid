// 64-point, 50%-overlap fixed-point FFT front end.
// One input sample per accepted clock; 16-bit I/Q input. Window coefficients
// use Q15 sin(pi*n/64)/8. Each radix-2 stage divides by two (FFT/64).
// Windowing, FFT butterflies, and output are scheduled in bounded cycles.
// Output is rotated by 45 bins so the detector's 32-bin ROI and its +/-3
// local-peak neighborhood arrive as one contiguous span in each frame.
module droneid_fft64_overlap (
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
    // Two butterflies per clock give 16 cycles per stage (96 total). The
    // window pass for the next frame overlaps the current FFT in the other
    // ping-pong bank, so FFT starts remain 128 clocks apart at 15.36 MS/s on
    // Maia's 61.44 MHz sample clock.
    localparam integer FFT_PARALLEL=2;
    reg signed [15:0] sample_i_mem [0:63];
    reg signed [15:0] sample_q_mem [0:63];
    reg signed [15:0] fft_i_mem [0:1][0:63];
    reg signed [15:0] fft_q_mem [0:1][0:63];
    reg [5:0] write_ptr;
    reg [6:0] sample_count;
    reg [4:0] hop_count;
    reg [2:0] stage;
    reg [3:0] calc_cycle;
    reg [5:0] output_index;
    reg window_active;
    reg fft_active;
    reg stream_active;
    reg [5:0] window_index;
    reg [5:0] window_start;
    reg window_bank;
    reg fft_bank;
    reg stream_bank;

    integer source_index;
    integer lane;
    integer butterfly_number;
    integer group_index;
    integer offset_index;
    integer half_span;
    integer span;
    integer butterfly_index;
    integer twiddle_index;
    reg signed [15:0] source_i;
    reg signed [15:0] source_q;
    reg signed [15:0] win;
    reg signed [31:0] window_product_i;
    reg signed [31:0] window_product_q;
    reg signed [31:0] tw_re;
    reg signed [31:0] tw_im;
    reg signed [31:0] prod_rr;
    reg signed [31:0] prod_ii;
    reg signed [31:0] prod_ri;
    reg signed [31:0] prod_ir;
    reg signed [32:0] rotated_i;
    reg signed [32:0] rotated_q;
    reg signed [17:0] a_i;
    reg signed [17:0] a_q;
    reg signed [17:0] b_i;
    reg signed [17:0] b_q;
    reg signed [18:0] sum_i;
    reg signed [18:0] sum_q;
    reg signed [18:0] diff_i;
    reg signed [18:0] diff_q;

    assign sample_ready = 1'b1;
    assign fft_valid = stream_active;
    assign fft_bin_index = output_index;
    assign fft_i = fft_i_mem[stream_bank][(output_index + 6'd45) & 6'd63];
    assign fft_q = fft_q_mem[stream_bank][(output_index + 6'd45) & 6'd63];

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

    function signed [15:0] round_half;
        input signed [18:0] value;
        begin
            if (value >= 0)
                round_half = (value + 19'sd1) >>> 1;
            else
                round_half = (value - 19'sd1) >>> 1;
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

    function signed [15:0] twiddle_real_q15;
        input integer index;
        begin
            case (index)
                0: twiddle_real_q15 = 16'sd32767;
                1: twiddle_real_q15 = 16'sd32609;
                2: twiddle_real_q15 = 16'sd32137;
                3: twiddle_real_q15 = 16'sd31356;
                4: twiddle_real_q15 = 16'sd30273;
                5: twiddle_real_q15 = 16'sd28898;
                6: twiddle_real_q15 = 16'sd27245;
                7: twiddle_real_q15 = 16'sd25329;
                8: twiddle_real_q15 = 16'sd23170;
                9: twiddle_real_q15 = 16'sd20787;
                10: twiddle_real_q15 = 16'sd18204;
                11: twiddle_real_q15 = 16'sd15446;
                12: twiddle_real_q15 = 16'sd12539;
                13: twiddle_real_q15 = 16'sd9512;
                14: twiddle_real_q15 = 16'sd6393;
                15: twiddle_real_q15 = 16'sd3212;
                16: twiddle_real_q15 = 16'sd0;
                17: twiddle_real_q15 = -16'sd3212;
                18: twiddle_real_q15 = -16'sd6393;
                19: twiddle_real_q15 = -16'sd9512;
                20: twiddle_real_q15 = -16'sd12539;
                21: twiddle_real_q15 = -16'sd15446;
                22: twiddle_real_q15 = -16'sd18204;
                23: twiddle_real_q15 = -16'sd20787;
                24: twiddle_real_q15 = -16'sd23170;
                25: twiddle_real_q15 = -16'sd25329;
                26: twiddle_real_q15 = -16'sd27245;
                27: twiddle_real_q15 = -16'sd28898;
                28: twiddle_real_q15 = -16'sd30273;
                29: twiddle_real_q15 = -16'sd31356;
                30: twiddle_real_q15 = -16'sd32137;
                31: twiddle_real_q15 = -16'sd32609;
                32: twiddle_real_q15 = -16'sd32767;
                33: twiddle_real_q15 = -16'sd32609;
                34: twiddle_real_q15 = -16'sd32137;
                35: twiddle_real_q15 = -16'sd31356;
                36: twiddle_real_q15 = -16'sd30273;
                37: twiddle_real_q15 = -16'sd28898;
                38: twiddle_real_q15 = -16'sd27245;
                39: twiddle_real_q15 = -16'sd25329;
                40: twiddle_real_q15 = -16'sd23170;
                41: twiddle_real_q15 = -16'sd20787;
                42: twiddle_real_q15 = -16'sd18204;
                43: twiddle_real_q15 = -16'sd15446;
                44: twiddle_real_q15 = -16'sd12539;
                45: twiddle_real_q15 = -16'sd9512;
                46: twiddle_real_q15 = -16'sd6393;
                47: twiddle_real_q15 = -16'sd3212;
                48: twiddle_real_q15 = 16'sd0;
                49: twiddle_real_q15 = 16'sd3212;
                50: twiddle_real_q15 = 16'sd6393;
                51: twiddle_real_q15 = 16'sd9512;
                52: twiddle_real_q15 = 16'sd12539;
                53: twiddle_real_q15 = 16'sd15446;
                54: twiddle_real_q15 = 16'sd18204;
                55: twiddle_real_q15 = 16'sd20787;
                56: twiddle_real_q15 = 16'sd23170;
                57: twiddle_real_q15 = 16'sd25329;
                58: twiddle_real_q15 = 16'sd27245;
                59: twiddle_real_q15 = 16'sd28898;
                60: twiddle_real_q15 = 16'sd30273;
                61: twiddle_real_q15 = 16'sd31356;
                62: twiddle_real_q15 = 16'sd32137;
                63: twiddle_real_q15 = 16'sd32609;

                default: twiddle_real_q15 = 16'sd0;
            endcase
        end
    endfunction

    function signed [15:0] twiddle_imag_q15;
        input integer index;
        begin
            case (index)
                0: twiddle_imag_q15 = 16'sd0;
                1: twiddle_imag_q15 = -16'sd3212;
                2: twiddle_imag_q15 = -16'sd6393;
                3: twiddle_imag_q15 = -16'sd9512;
                4: twiddle_imag_q15 = -16'sd12539;
                5: twiddle_imag_q15 = -16'sd15446;
                6: twiddle_imag_q15 = -16'sd18204;
                7: twiddle_imag_q15 = -16'sd20787;
                8: twiddle_imag_q15 = -16'sd23170;
                9: twiddle_imag_q15 = -16'sd25329;
                10: twiddle_imag_q15 = -16'sd27245;
                11: twiddle_imag_q15 = -16'sd28898;
                12: twiddle_imag_q15 = -16'sd30273;
                13: twiddle_imag_q15 = -16'sd31356;
                14: twiddle_imag_q15 = -16'sd32137;
                15: twiddle_imag_q15 = -16'sd32609;
                16: twiddle_imag_q15 = -16'sd32767;
                17: twiddle_imag_q15 = -16'sd32609;
                18: twiddle_imag_q15 = -16'sd32137;
                19: twiddle_imag_q15 = -16'sd31356;
                20: twiddle_imag_q15 = -16'sd30273;
                21: twiddle_imag_q15 = -16'sd28898;
                22: twiddle_imag_q15 = -16'sd27245;
                23: twiddle_imag_q15 = -16'sd25329;
                24: twiddle_imag_q15 = -16'sd23170;
                25: twiddle_imag_q15 = -16'sd20787;
                26: twiddle_imag_q15 = -16'sd18204;
                27: twiddle_imag_q15 = -16'sd15446;
                28: twiddle_imag_q15 = -16'sd12539;
                29: twiddle_imag_q15 = -16'sd9512;
                30: twiddle_imag_q15 = -16'sd6393;
                31: twiddle_imag_q15 = -16'sd3212;
                32: twiddle_imag_q15 = 16'sd0;
                33: twiddle_imag_q15 = 16'sd3212;
                34: twiddle_imag_q15 = 16'sd6393;
                35: twiddle_imag_q15 = 16'sd9512;
                36: twiddle_imag_q15 = 16'sd12539;
                37: twiddle_imag_q15 = 16'sd15446;
                38: twiddle_imag_q15 = 16'sd18204;
                39: twiddle_imag_q15 = 16'sd20787;
                40: twiddle_imag_q15 = 16'sd23170;
                41: twiddle_imag_q15 = 16'sd25329;
                42: twiddle_imag_q15 = 16'sd27245;
                43: twiddle_imag_q15 = 16'sd28898;
                44: twiddle_imag_q15 = 16'sd30273;
                45: twiddle_imag_q15 = 16'sd31356;
                46: twiddle_imag_q15 = 16'sd32137;
                47: twiddle_imag_q15 = 16'sd32609;
                48: twiddle_imag_q15 = 16'sd32767;
                49: twiddle_imag_q15 = 16'sd32609;
                50: twiddle_imag_q15 = 16'sd32137;
                51: twiddle_imag_q15 = 16'sd31356;
                52: twiddle_imag_q15 = 16'sd30273;
                53: twiddle_imag_q15 = 16'sd28898;
                54: twiddle_imag_q15 = 16'sd27245;
                55: twiddle_imag_q15 = 16'sd25329;
                56: twiddle_imag_q15 = 16'sd23170;
                57: twiddle_imag_q15 = 16'sd20787;
                58: twiddle_imag_q15 = 16'sd18204;
                59: twiddle_imag_q15 = 16'sd15446;
                60: twiddle_imag_q15 = 16'sd12539;
                61: twiddle_imag_q15 = 16'sd9512;
                62: twiddle_imag_q15 = 16'sd6393;
                63: twiddle_imag_q15 = 16'sd3212;

                default: twiddle_imag_q15 = 16'sd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            write_ptr <= 6'd0;
            sample_count <= 7'd0;
            hop_count <= 5'd0;
            stage <= 3'd0;
            calc_cycle <= 4'd0;
            output_index <= 6'd0;
            window_active <= 1'b0;
            fft_active <= 1'b0;
            stream_active <= 1'b0;
            window_index <= 6'd0;
            window_start <= 6'd0;
            window_bank <= 1'b0;
            fft_bank <= 1'b0;
            stream_bank <= 1'b0;
            overrun <= 1'b0;
        end else begin
            overrun <= 1'b0;
            if (sample_valid && sample_ready) begin
                sample_i_mem[write_ptr] <= sample_i;
                sample_q_mem[write_ptr] <= sample_q;
                if (write_ptr == 6'd63) write_ptr <= 6'd0;
                else write_ptr <= write_ptr + 1'b1;

                // First window after 64 samples; then one every 32 samples.
                if (sample_count < 7'd64) begin
                    sample_count <= sample_count + 1'b1;
                end else if (hop_count == 5'd31) begin
                    hop_count <= 5'd0;
                end else begin
                    hop_count <= hop_count + 1'b1;
                end
                if ((sample_count == 7'd63) ||
                    ((sample_count == 7'd64) && (hop_count == 5'd31))) begin
                    // A frame can be windowed while the previous frame is
                    // transformed, as long as it uses the other bank. The
                    // output stream may likewise run beside an FFT in the
                    // opposite bank.
                    if (window_active ||
                        (fft_active && (fft_bank == ~window_bank)) ||
                        (stream_active && (stream_bank == ~window_bank))) begin
                        overrun <= 1'b1;
                    end else begin
                        window_active <= 1'b1;
                        window_index <= 6'd0;
                        window_start <= write_ptr + 1'b1;
                        window_bank <= ~window_bank;
                    end
                end
            end

            // One I and one Q window multiply per clock; this pass completes
            // well before the next 32-sample hop at the intended input rate.
            if (window_active) begin
                source_index = (window_start + window_index) & 63;
                source_i = sample_i_mem[source_index];
                source_q = sample_q_mem[source_index];
                win = window_q15(window_index);
                window_product_i = source_i * win;
                window_product_q = source_q * win;
                fft_i_mem[window_bank][bit_reverse6(window_index)] <=
                    round_q15(window_product_i);
                fft_q_mem[window_bank][bit_reverse6(window_index)] <=
                    round_q15(window_product_q);

                if (window_index == 6'd63) begin
                    window_active <= 1'b0;
                    fft_active <= 1'b1;
                    fft_bank <= window_bank;
                    stage <= 3'd0;
                    calc_cycle <= 4'd0;
                end else begin
                    window_index <= window_index + 1'b1;
                end
            end

            // There are 32 butterflies per radix-2 stage. Two run in
            // parallel for sixteen cycles. Windowing the next frame may
            // proceed concurrently in the other bank.
            if (fft_active) begin
                half_span = 1 << stage;
                span = half_span << 1;
                for (lane=0; lane<FFT_PARALLEL; lane=lane+1) begin
                    butterfly_number = (calc_cycle * FFT_PARALLEL) + lane;
                    if (butterfly_number < 32) begin
                        group_index = (butterfly_number >> stage) << (stage + 1);
                        offset_index = butterfly_number & (half_span - 1);
                        butterfly_index = offset_index;
                        twiddle_index = offset_index << (5 - stage);
                        tw_re = twiddle_real_q15(twiddle_index);
                        tw_im = twiddle_imag_q15(twiddle_index);
                        prod_rr = fft_i_mem[fft_bank][group_index+butterfly_index+half_span] * tw_re;
                        prod_ii = fft_q_mem[fft_bank][group_index+butterfly_index+half_span] * tw_im;
                        prod_ri = fft_i_mem[fft_bank][group_index+butterfly_index+half_span] * tw_im;
                        prod_ir = fft_q_mem[fft_bank][group_index+butterfly_index+half_span] * tw_re;
                        rotated_i = prod_rr - prod_ii;
                        rotated_q = prod_ri + prod_ir;
                        b_i = round_q15(rotated_i);
                        b_q = round_q15(rotated_q);
                        a_i = fft_i_mem[fft_bank][group_index+butterfly_index];
                        a_q = fft_q_mem[fft_bank][group_index+butterfly_index];
                        sum_i = a_i + b_i;
                        sum_q = a_q + b_q;
                        diff_i = a_i - b_i;
                        diff_q = a_q - b_q;
                        fft_i_mem[fft_bank][group_index+butterfly_index] <= round_half(sum_i);
                        fft_q_mem[fft_bank][group_index+butterfly_index] <= round_half(sum_q);
                        fft_i_mem[fft_bank][group_index+butterfly_index+half_span] <= round_half(diff_i);
                        fft_q_mem[fft_bank][group_index+butterfly_index+half_span] <= round_half(diff_q);
                    end
                end

                if (calc_cycle == 4'd15) begin
                    calc_cycle <= 4'd0;
                    if (stage == 3'd5) begin
                        fft_active <= 1'b0;
                        stream_active <= 1'b1;
                        stream_bank <= fft_bank;
                        output_index <= 6'd0;
                    end else begin
                        stage <= stage + 1'b1;
                    end
                end else begin
                    calc_cycle <= calc_cycle + 1'b1;
                end
            end

            if (stream_active && fft_ready) begin
                if (output_index == 6'd63) begin
                    stream_active <= 1'b0;
                end else begin
                    output_index <= output_index + 1'b1;
                end
            end

        end
    end
endmodule

