// Two independent 15.36 Ms/s streams, serialized through one 64-point Maia
// SDF FFT. Each channel uses a 64-sample frame and 32-sample hop. The core
// accepts one frame sample per rx_clk and alternates complete frames RX0/RX1.
module droneid_fft64_shared_2rx (
    input  wire clk,
    input  wire reset,
    input  wire sample_valid0,
    input  wire signed [15:0] sample_i0,
    input  wire signed [15:0] sample_q0,
    input  wire sample_valid1,
    input  wire signed [15:0] sample_i1,
    input  wire signed [15:0] sample_q1,
    output wire sample_ready0,
    output wire sample_ready1,
    output wire fft_valid0,
    input  wire fft_ready0,
    input  wire fft_ready1,
    output wire [5:0] fft_bin_index0,
    output wire signed [15:0] fft_i0,
    output wire signed [15:0] fft_q0,
    output wire fft_valid1,
    output wire [5:0] fft_bin_index1,
    output wire signed [15:0] fft_i1,
    output wire signed [15:0] fft_q1,
    output reg overrun
);
    reg signed [15:0] sample_i_mem0 [0:63];
    reg signed [15:0] sample_q_mem0 [0:63];
    reg signed [15:0] sample_i_mem1 [0:63];
    reg signed [15:0] sample_q_mem1 [0:63];
    reg signed [15:0] sample_i_shadow0 [0:63];
    reg signed [15:0] sample_q_shadow0 [0:63];
    reg signed [15:0] sample_i_shadow1 [0:63];
    reg signed [15:0] sample_q_shadow1 [0:63];
    reg signed [15:0] output_i_mem [0:1][0:63];
    reg signed [15:0] output_q_mem [0:1][0:63];

    reg [5:0] write_ptr0;
    reg [5:0] write_ptr1;
    reg [6:0] sample_count0;
    reg [6:0] sample_count1;
    reg [4:0] hop_count0;
    reg [4:0] hop_count1;
    reg [5:0] frame_start0;
    reg [5:0] frame_start1;
    reg [1:0] shadow_valid;
    reg [1:0] use_shadow;
    reg copy_active;
    reg copy_channel;
    reg [5:0] copy_index;
    reg frame_ready0;
    reg frame_ready1;
    reg stream_started;
    reg feed_channel;
    reg [5:0] feed_index;

    reg [1:0] startup_markers;
    reg [5:0] raw_index;
    reg [1:0] output_ready;
    reg [1:0] emit_active;
    reg [5:0] emit_index0;
    reg [5:0] emit_index1;

    reg channel_tag_mem [0:3];
    reg [1:0] channel_tag_wr;
    reg [1:0] channel_tag_rd;
    reg [2:0] channel_tag_count;

    wire sample0_initial = sample_valid0 && (sample_count0 == 7'd63);
    wire sample1_initial = sample_valid1 && (sample_count1 == 7'd63);
    wire sample0_hop = sample_valid0 && (sample_count0 >= 7'd64) &&
                       (hop_count0 == 5'd31);
    wire sample1_hop = sample_valid1 && (sample_count1 >= 7'd64) &&
                       (hop_count1 == 5'd31);
    wire start_stream = !stream_started && frame_ready0 && frame_ready1;
    wire frame_boundary = stream_started && (feed_index == 6'd63);
    wire next_channel = ~feed_channel;
    wire consume_ready0 = start_stream || (frame_boundary && (next_channel == 1'b0));
    wire consume_ready1 = frame_boundary && (next_channel == 1'b1);
    wire copy_finishes = copy_active && (copy_index == 6'd63);
    wire capture_channel = channel_tag_mem[channel_tag_rd];
    wire selected_channel_ready0 = fft_ready0;
    wire selected_channel_ready1 = fft_ready1;
    wire capture_completes = stream_started && core_out_last &&
                             (startup_markers == 2'd2);
    wire emit_completes0 = emit_active[0] && selected_channel_ready0 &&
                           (emit_index0 == 6'd63);
    wire emit_completes1 = emit_active[1] && selected_channel_ready1 &&
                           (emit_index1 == 6'd63);
    wire same_channel_turnover = capture_completes &&
        (capture_channel ? emit_completes1 : emit_completes0);
    wire capture_completes0 = capture_completes && !capture_channel;
    wire capture_completes1 = capture_completes && capture_channel;

    wire signed [15:0] selected_sample_i = feed_channel ?
        (use_shadow[1] ? sample_i_shadow1[feed_index] :
                         sample_i_mem1[frame_start1 + feed_index]) :
        (use_shadow[0] ? sample_i_shadow0[feed_index] :
                         sample_i_mem0[frame_start0 + feed_index]);
    wire signed [15:0] selected_sample_q = feed_channel ?
        (use_shadow[1] ? sample_q_shadow1[feed_index] :
                         sample_q_mem1[frame_start1 + feed_index]) :
        (use_shadow[0] ? sample_q_shadow0[feed_index] :
                         sample_q_mem0[frame_start0 + feed_index]);
    wire signed [15:0] feed_window = window_q15(feed_index);
    wire signed [31:0] feed_prod_i = $signed(selected_sample_i) * feed_window;
    wire signed [31:0] feed_prod_q = $signed(selected_sample_q) * feed_window;
    wire signed [15:0] core_re_in = round_q15({feed_prod_i[31], feed_prod_i});
    wire signed [15:0] core_im_in = round_q15({feed_prod_q[31], feed_prod_q});
    wire signed [15:0] core_re_out;
    wire signed [15:0] core_im_out;
    wire core_out_last;

    assign sample_ready0 = 1'b1;
    assign sample_ready1 = 1'b1;
    assign fft_valid0 = emit_active[0];
    assign fft_valid1 = emit_active[1];
    assign fft_bin_index0 = emit_index0;
    assign fft_bin_index1 = emit_index1;
    assign fft_i0 = output_i_mem[0][(emit_index0 + 6'd45) & 6'd63];
    assign fft_q0 = output_q_mem[0][(emit_index0 + 6'd45) & 6'd63];
    assign fft_i1 = output_i_mem[1][(emit_index1 + 6'd45) & 6'd63];
    assign fft_q1 = output_q_mem[1][(emit_index1 + 6'd45) & 6'd63];

    droneid_maia_fft64 fft_core (
        .clk(clk), .rst(reset), .clken(stream_started),
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
            write_ptr0 <= 6'd0;
            write_ptr1 <= 6'd0;
            sample_count0 <= 7'd0;
            sample_count1 <= 7'd0;
            hop_count0 <= 5'd0;
            hop_count1 <= 5'd0;
            frame_start0 <= 6'd0;
            frame_start1 <= 6'd0;
            shadow_valid <= 2'b00;
            use_shadow <= 2'b00;
            copy_active <= 1'b0;
            copy_channel <= 1'b0;
            copy_index <= 6'd0;
            frame_ready0 <= 1'b0;
            frame_ready1 <= 1'b0;
            stream_started <= 1'b0;
            feed_channel <= 1'b0;
            feed_index <= 6'd0;
            startup_markers <= 2'd0;
            raw_index <= 6'd0;
            output_ready <= 2'b00;
            emit_active <= 2'b00;
            emit_index0 <= 6'd0;
            emit_index1 <= 6'd0;
            channel_tag_wr <= 2'd0;
            channel_tag_rd <= 2'd0;
            channel_tag_count <= 3'd0;
            overrun <= 1'b0;
        end else begin
            overrun <= 1'b0;

            if (sample_valid0) begin
                sample_i_mem0[write_ptr0] <= sample_i0;
                sample_q_mem0[write_ptr0] <= sample_q0;
                write_ptr0 <= write_ptr0 + 1'b1;
                if (sample_count0 < 7'd64) begin
                    sample_count0 <= sample_count0 + 1'b1;
                    if (sample0_initial) begin
                        frame_ready0 <= 1'b1;
                    end
                end else if (sample0_hop) begin
                    hop_count0 <= 5'd0;
                    if (frame_ready0 && !consume_ready0) begin
                        overrun <= 1'b1;
                    end
                    frame_ready0 <= 1'b1;
                end else begin
                    hop_count0 <= hop_count0 + 1'b1;
                end
            end

            if (sample_valid1) begin
                sample_i_mem1[write_ptr1] <= sample_i1;
                sample_q_mem1[write_ptr1] <= sample_q1;
                write_ptr1 <= write_ptr1 + 1'b1;
                if (sample_count1 < 7'd64) begin
                    sample_count1 <= sample_count1 + 1'b1;
                    if (sample1_initial) begin
                        frame_ready1 <= 1'b1;
                    end
                end else if (sample1_hop) begin
                    hop_count1 <= 5'd0;
                    if (frame_ready1 && !consume_ready1) begin
                        overrun <= 1'b1;
                    end
                    frame_ready1 <= 1'b1;
                end else begin
                    hop_count1 <= hop_count1 + 1'b1;
                end
            end

            if (copy_active) begin
                if (copy_channel) begin
                    sample_i_shadow1[copy_index] <=
                        sample_i_mem1[frame_start1 + copy_index];
                    sample_q_shadow1[copy_index] <=
                        sample_q_mem1[frame_start1 + copy_index];
                end else begin
                    sample_i_shadow0[copy_index] <=
                        sample_i_mem0[frame_start0 + copy_index];
                    sample_q_shadow0[copy_index] <=
                        sample_q_mem0[frame_start0 + copy_index];
                end
                if (copy_finishes) begin
                    shadow_valid[copy_channel] <= 1'b1;
                    copy_active <= 1'b0;
                end else begin
                    copy_index <= copy_index + 1'b1;
                end
            end

            if (start_stream) begin
                stream_started <= 1'b1;
                feed_channel <= 1'b0;
                feed_index <= 6'd0;
                frame_ready0 <= 1'b0;
                copy_active <= 1'b1;
                copy_channel <= 1'b1;
                copy_index <= 6'd0;
                channel_tag_mem[channel_tag_wr] <= 1'b0;
                channel_tag_wr <= channel_tag_wr + 1'b1;
            end else if (stream_started) begin
                if (feed_index == 6'd63) begin
                    feed_index <= 6'd0;
                    feed_channel <= next_channel;
                    if (feed_channel == 1'b0)
                        frame_start0 <= frame_start0 + 6'd32;
                    else
                        frame_start1 <= frame_start1 + 6'd32;

                    if (next_channel == 1'b0) begin
                        if (!(frame_ready0 || sample0_hop)) begin
                            overrun <= 1'b1;
                        end
                        if (!(shadow_valid[0] ||
                              (copy_finishes && !copy_channel)))
                            overrun <= 1'b1;
                        use_shadow[0] <= 1'b1;
                        shadow_valid[0] <= 1'b0;
                        frame_ready0 <= 1'b0;
                    end else begin
                        if (!(frame_ready1 || sample1_hop)) begin
                            overrun <= 1'b1;
                        end
                        if (!(shadow_valid[1] ||
                              (copy_finishes && copy_channel)))
                            overrun <= 1'b1;
                        use_shadow[1] <= 1'b1;
                        shadow_valid[1] <= 1'b0;
                        frame_ready1 <= 1'b0;
                    end
                    copy_active <= 1'b1;
                    copy_channel <= feed_channel;
                    copy_index <= 6'd0;
                    channel_tag_mem[channel_tag_wr] <= next_channel;
                    channel_tag_wr <= channel_tag_wr + 1'b1;
                end else begin
                    feed_index <= feed_index + 1'b1;
                end
            end

            if (stream_started && core_out_last) begin
                if (channel_tag_count == 0) begin
                    overrun <= 1'b1;
                end
                channel_tag_rd <= channel_tag_rd + 1'b1;
                if (startup_markers < 2'd2) begin
                    startup_markers <= startup_markers + 1'b1;
                    raw_index <= 6'd0;
                end else begin
                    output_i_mem[capture_channel][bit_reverse6(raw_index)] <= core_re_out;
                    output_q_mem[capture_channel][bit_reverse6(raw_index)] <= core_im_out;
                    if (capture_channel ?
                        (output_ready[1] && !same_channel_turnover) ||
                        (emit_active[1] && !same_channel_turnover) :
                        (output_ready[0] && !same_channel_turnover) ||
                        (emit_active[0] && !same_channel_turnover)) begin
                        overrun <= 1'b1;
                    end else begin
                        if (capture_channel)
                            output_ready[1] <= 1'b1;
                        else
                            output_ready[0] <= 1'b1;
                    end
                    raw_index <= 6'd0;
                end
            end else if (stream_started && startup_markers == 2'd2) begin
                if (raw_index == 0 &&
                    (capture_channel ? (output_ready[1] || emit_active[1]) :
                                       (output_ready[0] || emit_active[0])))
                    overrun <= 1'b1;
                output_i_mem[capture_channel][bit_reverse6(raw_index)] <= core_re_out;
                output_q_mem[capture_channel][bit_reverse6(raw_index)] <= core_im_out;
                raw_index <= raw_index + 1'b1;
            end else if (stream_started && startup_markers != 2'd0) begin
                raw_index <= raw_index + 1'b1;
            end

            if (!emit_active[0]) begin
                if ((output_ready[0] || capture_completes0) && fft_ready0) begin
                    emit_active[0] <= 1'b1;
                    emit_index0 <= 6'd0;
                    output_ready[0] <= 1'b0;
                end
            end else if (fft_ready0) begin
                if (emit_index0 == 6'd63) begin
                    emit_active[0] <= 1'b0;
                    output_ready[0] <= capture_completes0;
                end else begin
                    emit_index0 <= emit_index0 + 1'b1;
                end
            end

            if (!emit_active[1]) begin
                if ((output_ready[1] || capture_completes1) && fft_ready1) begin
                    emit_active[1] <= 1'b1;
                    emit_index1 <= 6'd0;
                    output_ready[1] <= 1'b0;
                end
            end else if (fft_ready1) begin
                if (emit_index1 == 6'd63) begin
                    emit_active[1] <= 1'b0;
                    output_ready[1] <= capture_completes1;
                end else begin
                    emit_index1 <= emit_index1 + 1'b1;
                end
            end

            case ({(start_stream || (stream_started && feed_index == 6'd63)),
                   (stream_started && core_out_last)})
                2'b10: channel_tag_count <= channel_tag_count + 1'b1;
                2'b01: if (channel_tag_count != 0)
                           channel_tag_count <= channel_tag_count - 1'b1;
                default: channel_tag_count <= channel_tag_count;
            endcase
        end
    end
endmodule
